import os
import subprocess
import rclpy
from rclpy.node import Node
from enum import Enum
from rclpy.duration import Duration

from autoware_auto_system_msgs.msg import AutowareState
from tier4_external_api_msgs.srv import Engage
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty


class OrchestratorState(Enum):
    Initializing = 1
    ReadyToPlan = 2
    ReadyToEngage = 3
    Driving = 4
    Done = 5


class OrchestratorStateMachine:
    def __init__(self):
        self._state = OrchestratorState.Initializing
    
    def update(self, symbol: AutowareState) -> bool:
        def _raise():
            raise ValueError(f"Symbol {symbol} has no valid transition in state {self._state}")

        if self._state == OrchestratorState.Initializing:
                if symbol == AutowareState.WAITING_FOR_ROUTE:
                    self._state = OrchestratorState.ReadyToPlan
                    return True
                elif symbol == AutowareState.INITIALIZING:
                    pass
                else:
                    _raise()
        elif self._state == OrchestratorState.ReadyToPlan:
                if symbol == AutowareState.WAITING_FOR_ENGAGE:
                    self._state = OrchestratorState.ReadyToEngage
                    return True
                elif symbol == AutowareState.PLANNING or symbol ==  AutowareState.WAITING_FOR_ROUTE:
                    pass
                else:
                    _raise()
        elif self._state == OrchestratorState.ReadyToEngage:
                if symbol == AutowareState.DRIVING:
                    self._state = OrchestratorState.Driving
                    return True
                elif symbol == AutowareState.WAITING_FOR_ENGAGE:
                    pass
                else:
                    _raise()
        elif self._state == OrchestratorState.Driving:
                if symbol == AutowareState.ARRIVED_GOAL or symbol ==  AutowareState.WAITING_FOR_ROUTE:
                    self._state = OrchestratorState.Done
                    return True
                elif symbol == AutowareState.DRIVING:
                    pass
                else:
                    _raise()
        else:
            pass
        
        return False


class AwOrchestrator(Node):
    def __init__(self):
        super().__init__('aw_orchestrator')
        self.aw_state_sub = self.create_subscription(
            AutowareState,
            '/autoware/state',
            self.aw_state_callback,
            10)

        self.shutdown_sub = self.create_subscription(Empty, "/ma_awsim_scenario_runner/shutdown", self.shutdown_callback, 10)

        self.goal_publisher = self.create_publisher(PoseStamped, '/planning/mission_planning/goal', 1)
        self.engage_client = self.create_client(Engage, '/api/external/set/engage')
        self.disengage_timer = self.create_timer(.1, self.disengage_check)
        self.engaged_time = None
        self.disengage_delay = Duration(seconds=60)  # seconds after engaging
        
        self.state_machine = OrchestratorStateMachine()

    def aw_state_callback(self, msg):
        old_state = self.state_machine._state
        has_state_changed = self.state_machine.update(msg.state)

        if has_state_changed:
            self.get_logger().info(f'Orchestrator state changed: {old_state.name} -> {self.state_machine._state.name} through AW state {msg.state}')
            self.state_change_callback(self.state_machine._state)
    
    def shutdown_callback(self, _):
        self.destroy_node()

    def state_change_callback(self, state: OrchestratorState):
        if state == OrchestratorState.ReadyToPlan:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"

            msg.pose.position.x = 81542.3515625
            msg.pose.position.y = 50296.1875
            msg.pose.position.z = 0.0

            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.7615288806503434
            msg.pose.orientation.w = 0.6481309774539673
            self.goal_publisher.publish(msg)
            self.get_logger().info("Published goal message")
        elif state == OrchestratorState.ReadyToEngage:
            req = Engage.Request()
            req.engage =  True
            self.engage_client.call(req)
            self.get_logger().info("Engage service called (engage)")
            self.engaged_time = self.get_clock().now()
        elif state == OrchestratorState.Done:
            pass
    
    def disengage_check(self):
        if self.engaged_time is None:
            return

        now = self.get_clock().now()
        if now >= self.engaged_time + self.disengage_delay:
            req = Engage.Request()
            req.engage =  False
            self.engage_client.call(req)
            self.get_logger().info("Engage service called (disengage)")
            self.engaged_time = None

def main(args=None):
    rclpy.init(args=args)

    orchestrator = AwOrchestrator()

    rclpy.spin(orchestrator)
    orchestrator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
