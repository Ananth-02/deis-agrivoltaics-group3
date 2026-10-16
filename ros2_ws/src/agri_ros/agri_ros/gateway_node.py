"""ROS2 wrapper for agri_loop.link.gateway.Gateway.  UNTESTED: written without a ROS2 install.

Topics: sub /v2x/cam (agri_msgs/DroneBeacon, best effort), sub /panel/state (PanelState),
        pub /v2x/denm (PanelEvent, reliable), pub /v2x/heartbeat (Heartbeat).
Needs `pip install -e <agri_loop>` in the same Python environment as ROS2.
"""
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from agri_msgs.msg import DroneBeacon as RBeacon, Heartbeat as RHeartbeat, PanelEvent as REvent, PanelState as RState

from agri_loop.link.eventlog import EventLog
from agri_loop.link.gateway import Gateway
from agri_loop.link.messages import DroneBeacon, Heartbeat, PanelEvent, PanelState


class GatewayNode(Node):
    def __init__(self):
        super().__init__('gateway')
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.log = EventLog()
        self.pub_event = self.create_publisher(REvent, '/v2x/denm', reliable)
        self.pub_hb = self.create_publisher(RHeartbeat, '/v2x/heartbeat', best_effort)
        self.gw = Gateway(self._send, self.log)
        self.create_subscription(RBeacon, '/v2x/cam', self._on_beacon, best_effort)
        self.create_subscription(RState, '/panel/state', self._on_state, best_effort)
        self.create_timer(0.01, lambda: self.gw.tick(self._now()))
        self.declare_parameter('log_path', 'gateway_log.jsonl')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _send(self, m) -> None:
        if isinstance(m, PanelEvent):
            r = REvent(seq=m.seq, t_send=m.t_send, panel_id=m.panel_id, event_type=m.event_type,
                       mission_type=m.mission_type, target_angle_deg=m.target_angle_deg)
            self.pub_event.publish(r)
        elif isinstance(m, Heartbeat):
            self.pub_hb.publish(RHeartbeat(seq=m.seq, t_send=m.t_send))

    def _on_beacon(self, r: RBeacon) -> None:
        now = self._now()
        self.log.add('recv', now, msg='DroneBeacon', seq=int(r.seq))
        self.gw.on_beacon(DroneBeacon(r.seq, r.t_send, r.station_id, r.x, r.y, r.z, r.vx, r.vy, r.vz,
                                      r.heading_deg, r.mission_type), now)

    def _on_state(self, r: RState) -> None:
        now = self._now()
        self.log.add('recv', now, msg='PanelState', seq=int(r.seq))
        self.gw.on_state(PanelState(r.seq, r.t_send, r.panel_id, r.fsm_state, r.angle_deg, r.target_deg,
                                    r.current_a, r.ack_seq), now)

    def destroy_node(self):
        self.log.save(self.get_parameter('log_path').value)
        super().destroy_node()


def main():
    rclpy.init()
    node = GatewayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
