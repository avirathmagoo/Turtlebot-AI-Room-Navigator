import os

import joblib
import pandas as pd
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DecisionNode(Node):
    """
    Loads the trained ML model and subscribes to 'task_conditions'.
    Predicts 'Target Room' and publishes it on 'target_room'.
    """

    def __init__(self):
        super().__init__("decision_node")

        # Absolute path to the trained model
        model_path = "/home/avirath/ROS_project/src/Turtlebot_proj/Turtlebot_proj/room_decision_tree.pkl"

        self.get_logger().info(f"Loading model from: {model_path}")
        if not os.path.exists(model_path):
            self.get_logger().error(f"Model file does not exist: {model_path}")
            raise FileNotFoundError(model_path)

        try:
            self.model = joblib.load(model_path)
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            raise

        # Subscribe to task_conditions
        self.subscription = self.create_subscription(
            String,
            "task_conditions",
            self.task_callback,
            10,
        )

        # Publish predicted room
        self.publisher = self.create_publisher(String, "target_room", 10)

        self.get_logger().info("DecisionNode started, waiting for task_conditions...")

    def task_callback(self, msg: String):
        # msg.data: "morning,delivery,medium"
        try:
            time_of_day, task_type, room_status = msg.data.split(",")
        except ValueError:
            self.get_logger().error(f"Invalid task_conditions format: '{msg.data}'")
            return

        # Build a 2D table (1 row, 3 columns) as pandas DataFrame
        X = pd.DataFrame(
            {
                "Time of Day": [time_of_day],
                "Task Type": [task_type],
                "Room status": [room_status],
            }
        )

        try:
            predicted_room = self.model.predict(X)[0]
        except Exception as e:
            self.get_logger().error(f"Model prediction failed: {e}")
            return

        out_msg = String()
        out_msg.data = predicted_room
        self.publisher.publish(out_msg)

        self.get_logger().info(
            f"Conditions ({time_of_day}, {task_type}, {room_status}) "
            f"-> Target Room: {predicted_room}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
