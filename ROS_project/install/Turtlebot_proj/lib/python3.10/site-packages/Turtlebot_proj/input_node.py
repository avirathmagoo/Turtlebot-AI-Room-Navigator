import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class InputNode(Node):
    """
    This node simulates incoming tasks for the robot.
    It publishes (Time of Day, Task Type, Room status) as a comma-separated string
    on the 'task_conditions' topic.
    """

    def __init__(self):
        super().__init__("input_node")

        self.publisher = self.create_publisher(String, "task_conditions", 10)
        self.timer = self.create_timer(5.0, self.publish_task)

        # A few representative samples based on your dataset distribution
        self.samples = [
            ("morning",  "delivery", "medium"),
            ("evening",  "delivery", "high"),
            ("evening",  "charging", "low"),
            ("night",    "delivery", "high"),
            ("afternoon","charging", "medium"),
        ]

        self.get_logger().info("InputNode started, publishing task_conditions.")

    def publish_task(self):
        time_of_day, task_type, room_status = random.choice(self.samples)
        msg = String()
        msg.data = f"{time_of_day},{task_type},{room_status}"
        self.publisher.publish(msg)
        self.get_logger().info(f"Published task_conditions: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = InputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
