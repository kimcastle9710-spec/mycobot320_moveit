#!/usr/bin/env python3
import os
import sys
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class AstraCaptureNode(Node):
    def __init__(self, output_dir: str, prefix: str = "astra"):
        super().__init__('astra_capture_node')

        self.output_dir = output_dir
        self.prefix = prefix
        os.makedirs(self.output_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.latest_frame = None
        self.img_count = 0

        # Astra 컬러 이미지 구독
        self.sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        # 30Hz 타이머로 화면 업데이트 & 키입력 체크
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

        self.get_logger().info(f"AstraCaptureNode started. Saving to: {self.output_dir}")
        self.get_logger().info("Press 's' to save image, 'q' to quit.")

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_frame = frame
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def timer_callback(self):
        if self.latest_frame is None:
            return

        # 미리보기
        cv2.imshow("Astra Color (press 's' to save, 'q' to quit)", self.latest_frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):
            self.save_image()
        elif key == ord('q'):
            self.get_logger().info("Quit requested by user.")
            rclpy.shutdown()

    def save_image(self):
        filename = f"{self.prefix}_{self.img_count:04d}.jpg"
        filepath = os.path.join(self.output_dir, filename)
        ok = cv2.imwrite(filepath, self.latest_frame)
        if ok:
            self.get_logger().info(f"Saved: {filepath}")
            self.img_count += 1
        else:
            self.get_logger().error(f"Failed to save: {filepath}")


def main():
    # 간단한 인자 처리: python3 astra_capture.py <output_dir> [prefix]
    if len(sys.argv) < 2:
        print("Usage: python3 astra_capture.py <output_dir> [prefix]")
        sys.exit(1)

    output_dir = sys.argv[1]
    prefix = sys.argv[2] if len(sys.argv) > 2 else "astra"

    rclpy.init(args=None)
    node = AstraCaptureNode(output_dir=output_dir, prefix=prefix)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

