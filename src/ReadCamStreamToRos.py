import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import requests
import io
import time
from cv_bridge import CvBridge
import cv2
import numpy as np
from builtin_interfaces.msg import Time
import piexif
from PIL import Image as PILImage

class CameraStreamPublisher(Node):

    def __init__(self):
        super().__init__('camera_stream_publisher')
        # Publisher for Image
        self.image_publisher = self.create_publisher(Image, 'camera_image', 10)

        # Bridge to convert OpenCV images to ROS Image messages
        self.bridge = CvBridge()

        # Stream URL (Modify this to your Flask stream URL)
        self.stream_url = 'http://192.168.1.167:5000/video_feed'
        
        self.get_logger().info("Starting the camera stream subscriber...")
        self.timer = self.create_timer(0.033, self.timer_callback)  # ~30 FPS (33ms interval)

    def timer_callback(self):
        try:
            # Fetch frame data from Flask server (Image + Timestamp)
            response = requests.get(self.stream_url, stream=True)
            # Iterate over each frame in the response stream
            for chunk in response.iter_content(chunk_size=1024):
                if b'--frame' in chunk:  # Detect the start of a frame
                    # Find where the image data starts (after boundary)
                    start_index = chunk.find(b'\r\n\r\n') + 4  # Skip past the boundary headers
                    image_data = chunk[start_index:]

                    # Convert image data to OpenCV format (use cv2 for OpenCV-based processing)
                    nparr = np.frombuffer(image_data, np.uint8)
                    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    timestamp = self.read_timestamp_from_exif(image_data)
                    if frame is not None:
                        # Create the ROS Image message
                        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                        
                        # If timestamp is available, embed it in the ROS Image header
                        if timestamp:
                            # Set the timestamp into the ROS message header
                            # Convert timestamp string to ROS time
                            ros_timestamp = Time()
                            ros_timestamp.sec, ros_timestamp.nanosec = self.convert_timestamp_to_ros(timestamp)

                            ros_image.header.stamp = ros_timestamp  # Set timestamp in ROS message header
                            ros_image.header.frame_id = "/world"

                        # Publish the image message
                        self.image_publisher.publish(ros_image)
                        return
                        
                    
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Error fetching stream: {e}")

    def convert_timestamp_to_ros(self, timestamp):
        # Ensure the timestamp is a string (if it's in bytes)
        if isinstance(timestamp, bytes):
            timestamp = timestamp.decode('utf-8')  # Decode bytes to string

        # Convert timestamp (in string form) to ROS time (seconds and nanoseconds)
        # Timestamp format: 'YYYY-MM-DD HH:MM:SS.ssssss'
        # t = time.strptime(timestamp, "%Y:%m:%d %H:%M:%S")
        # seconds = int(time.mktime(t))  # Convert time to seconds
        # nanoseconds = int((timestamp.split('.')[1] if '.' in timestamp else '0')[:9])  # Convert fractional seconds to nanoseconds
        seconds = timestamp.split(':')[0]
        nanoseconds = timestamp.split(':')[1]
        return int(seconds), int(nanoseconds)

    
    def read_timestamp_from_exif(self, image_data):
        # Open the image with Pillow
        img = PILImage.open(io.BytesIO(image_data))

        # Get the EXIF data (if exists)
        exif_data = img.info.get("exif", b"")

        if not exif_data:
            return None  # No EXIF data found

        # Load the EXIF data using piexif
        exif_dict = piexif.load(exif_data)

        # Check if 'DateTimeOriginal' (0x9003) exists in the EXIF data
        if piexif.ImageIFD.DateTime in exif_dict["0th"]:
            timestamp_str = exif_dict["0th"][piexif.ImageIFD.DateTime]
            return timestamp_str
        else:
            return None  # DateTimeOriginal tag is not present in the EXIF data

def main(args=None):
    rclpy.init(args=args)
    camera_stream_publisher = CameraStreamPublisher()

    try:
        rclpy.spin(camera_stream_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_stream_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
