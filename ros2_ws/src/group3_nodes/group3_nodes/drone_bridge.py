"""ROS2 <-> cf_server translator: the drone's version of arduino_bridge. It decides nothing.

cf_server (agri_loop, runs in ~/agri_venv with cflib) owns the radio. This node runs in
the system Python with ROS2 and only uses the standard library, so cflib and its numpy
never meet ROS2's.

    /group3/cf/cmd (String) --> this node --UDP 127.0.0.1:5010--> cf_server
    /group3/cf/pose (PoseStamped), /group3/cf/state (String) <-- this node <--UDP 5011--

Install: copy into ~/ros2_ws/src/group3_nodes/group3_nodes/, add
    "drone_bridge = group3_nodes.drone_bridge:main",
to console_scripts in setup.py, then colcon build (a new node needs a rebuild).
"""
import math
import socket

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

SERVER = ("127.0.0.1", 5010)  # cf_server listens here
LISTEN = ("127.0.0.1", 5011)  # cf_server sends STATE here


def parse_state(text):
    """'FLYING,1.4,0.6,0.8,2.1,3.95' -> ("FLYING", x, y, z, yaw_deg, vbat), or None if broken."""
    parts = text.strip().split(",")
    if len(parts) != 6:
        return None
    try:
        return (parts[0],) + tuple(float(p) for p in parts[1:])
    except ValueError:
        return None


class DroneBridge(Node):
    def __init__(self):
        super().__init__("drone_bridge")
        self.command = "Status"  # only ask, never order, until the coordinator sends a real command

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(LISTEN)
        self.sock.setblocking(False)

        self.pose_pub = self.create_publisher(PoseStamped, "/group3/cf/pose", 10)
        self.state_pub = self.create_publisher(String, "/group3/cf/state", 10)
        self.create_subscription(String, "/group3/cf/cmd", self.on_cmd, 10)
        self.create_timer(0.2, self.send)       # resend the command 5 times per second, like arduino_bridge
        self.create_timer(0.02, self.receive)   # drain STATE often, so pose is not delayed or bunched up

    def on_cmd(self, msg):
        self.command = msg.data
        self.get_logger().info(f"New command: {self.command}")

    def send(self):
        self.sock.sendto(self.command.encode(), SERVER)

    def receive(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(128)
            except BlockingIOError:
                break
            except ConnectionResetError:  # only on Windows, after sending to a closed port
                continue
            state = parse_state(data.decode(errors="replace"))
            if state is None:
                continue  # ignore broken messages
            name, x, y, z, yaw_deg, vbat = state

            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()  # receive time, same Pi clock as cf_server
            pose.header.frame_id = "world"  # Lighthouse frame: origin at the green patch, x along the long side
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            yaw = math.radians(yaw_deg)  # rotation about z only; roll and pitch are not needed for the mission
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
            self.pose_pub.publish(pose)
            self.state_pub.publish(String(data=f"{name},{vbat:.2f}"))


def main():
    rclpy.init()
    rclpy.spin(DroneBridge())


if __name__ == "__main__":
    main()
