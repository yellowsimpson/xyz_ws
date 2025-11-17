import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import DrlStart
import DR_init
from dsr_example.simple.gripper_drl_controller import GripperController
from std_msgs.msg import Float32   ## Isaac용 stroke 토픽
grip_on = 440
grip_off = 250
def main(args=None):
    rclpy.init(args=args)
    ROBOT_ID = "dsr01"
    ROBOT_MODEL = "e0509"
    VEL = 50
    ACC = 50
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    node = rclpy.create_node('gripper_example_py', namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    ## :white_check_mark: Isaac 심 쪽으로 stroke 보내는 퍼블리셔
    pub_gripper = node.create_publisher(Float32, '/fuel/gripper_move', 10)
    from DSR_ROBOT2 import set_robot_mode, ROBOT_MODE_AUTONOMOUS, posj, movej, wait
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    def command_gripper(stroke: float, wait_sec: float = 3.0):
        ## 실 로봇 + Isaac 동시에 갱신
        node.get_logger().info(f"[TEST] gripper.move({stroke})")
        gripper.move(int(stroke))
        msg = Float32()
        msg.data = float(stroke)
        pub_gripper.publish(msg)
        wait(wait_sec)
    gripper = None
    try:
        gripper = GripperController(node=node, namespace=ROBOT_ID)
        if not gripper.initialize():
            node.get_logger().error("Gripper initialization failed. Exiting.")
            return
        node.get_logger().info("Sending a priming command to wake up the gripper...")
        wait(5)
        ## 여기부터는 command_gripper만 호출
        command_gripper(0)
        command_gripper(200)
        command_gripper(500)
        command_gripper(700)
        command_gripper(400)
        command_gripper(0)
    except Exception as e:
        node.get_logger().error(f"An error occurred: {e}")
    finally:
        if gripper:
            gripper.terminate()
        rclpy.shutdown()
        node.get_logger().info("Shutdown complete.")
if __name__ == '__main__':
    main()