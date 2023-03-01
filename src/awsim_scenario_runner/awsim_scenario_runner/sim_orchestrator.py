import os
import signal
import subprocess
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class SimOrchestrator(Node):
    def __init__(self):
        super().__init__('sim_orchestrator')
        
        self.shutdown_sub = self.create_subscription(Empty, "/ma_awsim_scenario_runner/shutdown", self.shutdown_callback, 10)
        self.launch_sim()

    def launch_sim(self):
        self.sim_proc = subprocess.Popen([f"{os.environ['RUNNER_DIR']}/AWSIM/AWSIM.headless.x86_64"])

    def shutdown_callback(self, _):
        self.get_logger().info(f"[Orchestrator] Shutting down")
        return_code = self.sim_proc.poll()
        if return_code is not None:
            self.get_logger().error(f"[Orchestrator] Simulator exited prematurely with code {return_code}")
        else:
            self.sim_proc.send_signal(signal.SIGINT)
            try:
                return_code = self.sim_proc.wait(10)
                if return_code != 0:
                    self.get_logger().warning(f"[Orchestrator] Simulator exited with code {return_code}")
            except subprocess.TimeoutExpired:
                self.sim_proc.send_signal(signal.SIGTERM)
        
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)

    orchestrator = SimOrchestrator()

    rclpy.spin(orchestrator)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
