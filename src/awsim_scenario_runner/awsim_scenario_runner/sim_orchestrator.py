import os
import signal
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class SimOrchestrator(Node):
    def __init__(self):
        super().__init__('sim_orchestrator')
        
        self.shutdown_sub = self.create_subscription(Empty, "/ma_awsim_scenario_runner/shutdown", self.shutdown_callback, 1)
        self.launch_sim()

    def launch_sim(self):
        self.sim_proc = subprocess.Popen([f"/home/{os.environ['USER']}/Max_MA/AWSIM/AWSIM.headless.x86_64"])

    def shutdown_callback(self, _):
        return_code = self.sim_proc.poll()
        if return_code is not None:
            self.get_logger().error(f"Simulator exited prematurely with code {return_code}")
        else:
            self.sim_proc.send_signal(signal.SIGINT)
            return_code = self.sim_proc.wait(10)
            if return_code != 0:
                self.get_logger().warning(f"Simulator exited with code {return_code}")
        
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    orchestrator = SimOrchestrator()

    rclpy.spin(orchestrator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    orchestrator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
