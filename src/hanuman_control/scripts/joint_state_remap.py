# Save as scripts/joint_state_remap.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Map hardware motor names to URDF joint names
MOTOR_TO_URDF = {
    "motor_1": "JL_hip_y",
    "motor_2": "JL_hip_r",
    "motor_3": "JL_hip_p",
    "motor_4": "JL_knee",
    "motor_5": "JL_ankle_p",
    "motor_6": "JL_ankle_r",
    "motor_11": "JR_hip_y",
    "motor_12": "JR_hip_r",
    "motor_13": "JR_hip_p",
    "motor_14": "JR_knee",
    "motor_15": "JR_ankle_p",
    "motor_16": "JR_ankle_r",
}

class JointStateRemap(Node):
    def __init__(self):
        super().__init__('joint_state_remap')
        self.sub = self.create_subscription(
            JointState, '/dynamixel_joint_states', self.callback, 10)
        self.pub = self.create_publisher(
            JointState, '/joint_states', 10)

    def callback(self, msg):
        remapped = JointState()
        remapped.header = msg.header
        remapped.name = [MOTOR_TO_URDF.get(n, n) for n in msg.name]
        remapped.position = msg.position
        remapped.velocity = msg.velocity
        remapped.effort = msg.effort
        self.pub.publish(remapped)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateRemap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()