import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import openai
import base64
import cv2
from cv_bridge import CvBridge


class OpenAIAPINode(Node):
    def __init__(self):
        super().__init__('openai_api_node')
        
        # Initialize OpenAI client
        self.openai_client = openai.OpenAI()
        
        # Initialize CV bridge for image conversion
        self.bridge = CvBridge()
        
        # Subscribe to camera topic
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_color',
            self.image_callback,
            10
        )
        
        # Publisher for robot movement commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        # Timer to process images periodically (every 2 seconds)
        self.timer = self.create_timer(2.0, self.process_image)
        
        # Store the latest image
        self.latest_image = None
        self.image_received = False
        
        self.get_logger().info('OpenAI API Node started')
    
        self.get_logger().info('Subscribed to camera topic: /camera/image_color')
    
    def image_callback(self, msg):
        """Callback function to receive camera images"""
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image
            self.image_received = True
            self.get_logger().debug('Image received successfully')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def process_image(self):
        """Process the latest image and send to OpenAI API"""
        if not self.image_received or self.latest_image is None:
            self.get_logger().debug('No image available for processing')
            return
        
        try:
            # Encode image to base64
            self.get_logger().info(f'Processing image with shape: {self.latest_image.shape}')
            _, buffer = cv2.imencode('.jpg', self.latest_image)
            image_base64 = base64.b64encode(buffer).decode("utf-8")
            
            # Send to OpenAI API
            response = self.openai_client.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image_base64}"}},
                            {"type": "text", "text": "What do you see? You are the small robot. You can GO forward or STOP. You must get close to yellow duck but continue to see it entirely. Explain you decision and add to you answer '<GO>' or '<STOP>'"}
                        ]
                    }
                ],
                max_tokens=300
            )
            
            # Get the response text
            response_text = response.choices[0].message.content
            self.get_logger().info(f'OpenAI Response: {response_text}')
            
            # Parse response and control robot
            self.control_robot(response_text)
            
        except Exception as e:
            self.get_logger().error(f'Error calling OpenAI API: {str(e)}')
    
    def control_robot(self, response_text):
        """Control robot based on OpenAI response"""
        command = Twist()
        
        if '<GO>' in response_text:
            # Move forward
            command.linear.x = 0.05
            command.angular.z = 0.0
            self.get_logger().info('Moving forward - GO signal received')
        elif '<STOP>' in response_text:
            # Stop
            command.linear.x = 0.0
            command.angular.z = 0.0
            self.get_logger().info('Stopping - STOP signal received')
        else:
            # Default behavior - move slowly
            command.linear.x = 0.01
            command.angular.z = 0.0
            self.get_logger().info('Default behavior - no clear GO/STOP signal')
        
        # Publish the command
        self.cmd_vel_publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = OpenAIAPINode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()