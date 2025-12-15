#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import PoseStamped
import time

class CubePicker(Node):
    def __init__(self):
        super().__init__('cube_picker')
        
        # 1. TF 리스너 (좌표 변환 듣기)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 주기적으로 큐브 위치 확인 (1초에 1번)
        self.timer = self.create_timer(1.0, self.check_cube_position)
        
        self.get_logger().info("Cube Picker Node Started. Waiting for 'cube_target' TF...")

    def check_cube_position(self):
        try:
            target_frame = 'base' 
            source_frame = 'cube_target'
            
            # [수정된 부분] Time(seconds=0)을 넣어야 최신 데이터를 바로 가져옵니다.
            trans = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(seconds=0), 
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z

            # 로그를 WARN으로 찍어서 잘 보이게 함
            self.get_logger().warn(
                f"\n[MOVEIT TARGET]\n"
                f"Target: {target_frame} -> {source_frame}\n"
                f"Pos   : X={x:.3f}, Y={y:.3f}, Z={z:.3f}\n"
            )

        except (LookupException, ConnectivityException, ExtrapolationException):
            # 너무 자주 뜨면 시끄러우니까 2초에 한번만 경고
            # self.get_logger().warn("Waiting for YOLO detection...")
            pass


def main(args=None):
    rclpy.init(args=args)
    node = CubePicker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
