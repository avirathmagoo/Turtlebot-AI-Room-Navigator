import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

class NavigatorNode(Node):
    """
    This node subscribes to 'target_room' and uses Nav2 Simple Commander
    to navigate the TurtleBot3 to predefined poses (one per room).
    """

    def __init__(self):
        super().__init__("navigator_node")

        self.navigator = BasicNavigator()

        self.get_logger().info("Waiting for Nav2 to become active...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active.")

        # Poses from your /amcl_pose readings (map frame)
        self.room_goals = {
            "livingroom": self.make_pose(
                -0.6156096514406907,
                1.7873801931689135,
                0.8032932020708177,
                0.595583773710141,
            ),
            "kitchen": self.make_pose(
                6.471326109568462,
                1.6170783991742583,
                0.5453443049120837,
                0.8382121384828284,
            ),
            "bedroom": self.make_pose(
                -4.3374912042926725,
                -1.9005635249192459,
                0.5418511218413837,
                0.8404744860846365,
            ),
        }

        self.subscription = self.create_subscription(
            String,
            "target_room",
            self.room_callback,
            10,
        )

        self.get_logger().info("NavigatorNode started, waiting for target_room...")

    def make_pose(self, x, y, oz, ow):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = oz
        pose.pose.orientation.w = ow
        return pose

    def room_callback(self, msg: String):
        room_raw = msg.data.strip()
        room = room_raw.lower()

        if room not in self.room_goals:
            self.get_logger().warn(
                f"Unknown room '{room_raw}'. Known rooms: {list(self.room_goals.keys())}"
            )
            return

        goal = self.room_goals[room]
        goal.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(
            f"Navigating to {room} at "
            f"({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})"
        )

        self.navigator.goToPose(goal)

        # Wait until navigation is done
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback is not None:
                self.get_logger().info(
                    f"Distance remaining: {feedback.distance_remaining:.2f} m"
                )

        result = self.navigator.getResult()
        self.get_logger().info(f"Navigation result: {result}")

def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
