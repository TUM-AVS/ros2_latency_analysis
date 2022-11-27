import os
import subprocess
import rclpy
from rclpy.node import Node
from enum import Enum

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

        match self._state:
            case OrchestratorState.Initializing:
                match symbol:
                    case AutowareState.WAITING_FOR_ROUTE:
                        self._state = OrchestratorState.ReadyToPlan
                        return True
                    case AutowareState.INITIALIZING_VEHICLE:
                        pass
                    case _:
                        _raise()
            case OrchestratorState.ReadyToPlan:
                match symbol:
                    case AutowareState.WAITING_FOR_ENGAGE:
                        self._state = OrchestratorState.ReadyToEngage
                        return True
                    case AutowareState.PLANNING | AutowareState.WAITING_FOR_ROUTE:
                        pass
                    case _:
                        _raise()
            case OrchestratorState.ReadyToEngage:
                match symbol:
                    case AutowareState.DRIVING:
                        self._state = OrchestratorState.Driving
                        return True
                    case AutowareState.WAITING_FOR_ENGAGE:
                        pass
                    case _:
                        _raise()
            case OrchestratorState.Driving:
                match symbol:
                    case AutowareState.ARRIVAL_GOAL | AutowareState.WAITING_FOR_ROUTE:
                        self._state = OrchestratorState.Done
                        return True
                    case AutowareState.DRIVING:
                        pass
                    case _:
                        _raise()
            case _:
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

        self.shutdown_sub = self.create_subscription(Empty, "/ma_awsim_scenario_runner/shutdown", self.shutdown_callback)

        self.goal_publisher = self.create_publisher(PoseStamped, '/planning/mission_planning/goal', 1)
        self.engage_client = self.create_client(Engage, '/api/external/set/engage')
        
        self.state_machine = OrchestratorStateMachine()
        self.launch_aw()

    def aw_state_callback(self, msg):
        old_state = self.state_machine._state
        has_state_changed = self.state_machine.update(msg.state)

        if has_state_changed:
            self.get_logger().info(f'Orchestrator state changed: {old_state.name} -> {self.state_machine._state.name} through AW state {msg.state}')
            self.state_change_callback(self.state_machine._state)
    
    def shutdown_callback(self, _):
        self.destroy_node()
        rclpy.shutdown()

    def state_change_callback(self, state: OrchestratorState):
        match state:
            case OrchestratorState.ReadyToPlan:
                msg = PoseStamped()
                msg.header.stamp = rclpy.Time.now()
                msg.header.frame_id = "map"

                msg.pose.position.x = 81542.3515625
                msg.pose.position.y = 50296.1875
                msg.pose.position.z = 0.0

                msg.pose.orientation.x = 0.0
                msg.pose.orientation.y = 0.0
                msg.pose.orientation.z = 0.7615288806503434
                msg.pose.orientation.w = 0.6481309774539673
                self.goal_publisher.publish()
                self.get_logger().info("Published goal message")
            case OrchestratorState.ReadyToEngage:
                req = Engage.Request()
                req.engage =  True
                self.engage_client.call(req)
                self.get_logger().info("Engage service called")
            case OrchestratorState.Done:
                pass

    def launch_aw(self):
        workdir = os.path.expanduser(f"~/Max_MA/autoware")
        self.aw_proc = subprocess.Popen(["ros2"], cwd=workdir)

def main(args=None):
    rclpy.init(args=args)

    orchestrator = AwOrchestrator()

    rclpy.spin(orchestrator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    orchestrator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
