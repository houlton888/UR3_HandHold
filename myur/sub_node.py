import rclpy
from std_msgs.msg import String

class Sub_Node(rclpy.node.Node):
    """
    Node for subscribing to a given topic and retrieiving data.
    """

    def __init__(self,topic):
        """
        Initialize the generic subscriber node.
        """
        super().__init__("subscriber_node")
        self.subscription = self.create_subscription(String, topic, self.listener_callback, 10)
        self.data = None
        self.done = False

    def listener_callback(self, msg):
        """
        Callback for when data is received.

        Args:
            msg (String): The data published to topic.
        """
        self.message = msg.data
        self.done = True

    def get_data(self):
        """
        Get the current topic data.

        Returns:
            String: The most recent .
        """
        self.wait(self)
        self.done = False
        return self.data

    def wait(self, client):
        """
        Wait for the joint states to be updated.

        Args:
            client (Node): The node to wait for.
        """
        rclpy.spin_once(client)
        while not client.done:
            rclpy.spin_once(client)
            self.get_logger().debug(f"Waiting for joint_states_client")