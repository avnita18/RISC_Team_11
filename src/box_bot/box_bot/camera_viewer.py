
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.forward_speed = 0.30
        self.curve_speed = 0.012
        self.Kp = 0.003
        self.max_turn = 0.15
        self.threshold_value = 100
        self.min_contour_area = 100

        self.roi_top_ratio = 0.55
        self.roi_bottom_ratio = 0.95

        self.get_logger().info(
            'Contour-based camera line follower started'
        )

    def publish_velocity(self, linear=0.0, angular=0.0):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_pub.publish(cmd)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as exc:
            self.get_logger().error(f'Image conversion failed: {exc}')
            self.publish_velocity()
            return

        height, width = frame.shape[:2]
        print("Frame mean brightness:", frame.mean())
        print("Frame timestamp:", msg.header.stamp.sec, msg.header.stamp.nanosec)
        center_x = width / 2.0
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(
            gray,
            self.threshold_value,
            255,
            cv2.THRESH_BINARY_INV
        )

        y_start = int(height * self.roi_top_ratio)
        y_end = int(height * self.roi_bottom_ratio)
        roi = mask[y_start:y_end, :].copy()
        contours, _ = cv2.findContours(
            roi,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        valid_contours = [
            contour for contour in contours
            if cv2.contourArea(contour) >= self.min_contour_area
        ]

        output = frame.copy()
        cv2.rectangle(
            output,
            (0, y_start),
            (width - 1, y_end),
            (0, 0, 255),
            2
        )

        cv2.line(
            output,
            (int(center_x), 0),
            (int(center_x), height - 1),
            (255, 0, 0),
            2
        )

        if not valid_contours:
            self.publish_velocity()

            cv2.putText(
                output,
                'LINE LOST - STOPPED',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        else:
            contour = max(valid_contours, key=cv2.contourArea)
            area = cv2.contourArea(contour)

            moments = cv2.moments(contour)

            if moments['m00'] == 0:
                self.publish_velocity()
            else:
                local_x = moments['m10'] / moments['m00']
                local_y = moments['m01'] / moments['m00']
                line_x = int(local_x)
                line_y = int(local_y + y_start)

                error = line_x - center_x

                angular = max(
                    -self.max_turn,
                    min(self.max_turn, -self.Kp * error)
                )

                if abs(error) > width * 0.12:
                    speed = self.curve_speed
                else:
                    speed = self.forward_speed

                self.publish_velocity(speed, angular)

                shifted_contour = contour.copy()
                shifted_contour[:, 0, 1] += y_start

                cv2.drawContours(
                    output,
                    [shifted_contour],
                    -1,
                    (0, 255, 255),
                    2
                )
                cv2.circle(
                    output,
                    (line_x, line_y),
                    7,
                    (0, 0, 255),
                    -1
                )
                cv2.putText(
                    output,
                    f'Error: {error:.1f}  Area: {area:.0f}',
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 180, 0),
                    2
                )
                cv2.putText(
                    output,
                    f'Angular: {angular:.3f}  Speed: {speed:.3f}',
                    (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 180, 0),
                    2
                )

        cv2.imshow('Gazebo Camera', output)
        # cv2.imshow('Threshold Mask', mask)
        # cv2.imshow('Raw Camera Feed', frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_velocity()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

