"""ROS2 wrapper for agri_loop.link.panel_sim.PanelSim.  UNTESTED: written without a ROS2 install.

Stands in for the Arduino until the real panel is ready, and is the reference for its behaviour.
Topics: sub /v2x/denm (PanelEvent), sub /v2x/heartbeat (Heartbeat), pub /panel/state (PanelState).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from agri_msgs.msg import Heartbeat as RHeartbeat, PanelEvent as REvent, PanelState as RState

from agri_loop.link.eventlog import EventLog
from agri_loop.link.messages import Heartbeat, PanelEvent, PanelState
from agri_loop.link.panel_sim import PanelSim


class PanelSimNode(Node):
    def __init__(self):
        super().__init__('panel_sim')
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.log = EventLog()
        self.pub_state = self.create_publisher(RState, '/panel/state', best_effort)
        self.panel = PanelSim(self._send, self.log)
        self.create_subscription(REvent, '/v2x/denm', self._on_event, reliable)
        self.create_subscription(RHeartbeat, '/v2x/heartbeat', self._on_hb, best_effort)
        self.create_timer(0.01, lambda: self.panel.tick(self._now()))
        self.declare_parameter('log_path', 'panel_log.jsonl')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _send(self, m: PanelState) -> None:
        self.pub_state.publish(RState(seq=m.seq, t_send=m.t_send, panel_id=m.panel_id, fsm_state=m.fsm_state,
                                      angle_deg=float(m.angle_deg), target_deg=float(m.target_deg),
                                      current_a=float(m.current_a), ack_seq=m.ack_seq))

    def _on_event(self, r: REvent) -> None:
        now = self._now()
        self.log.add('recv', now, msg='PanelEvent', seq=int(r.seq))
        self.panel.on_event(PanelEvent(r.seq, r.t_send, r.panel_id, r.event_type, r.mission_type,
                                       r.target_angle_deg), now)

    def _on_hb(self, r: RHeartbeat) -> None:
        now = self._now()
        self.log.add('recv', now, msg='Heartbeat', seq=int(r.seq))
        self.panel.on_heartbeat(Heartbeat(r.seq, r.t_send), now)

    def destroy_node(self):
        self.log.save(self.get_parameter('log_path').value)
        super().destroy_node()


def main():
    rclpy.init()
    node = PanelSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
