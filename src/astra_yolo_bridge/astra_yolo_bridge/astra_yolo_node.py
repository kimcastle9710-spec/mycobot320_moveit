#!/usr/bin/env python3
import math
from typing import Optional, Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data  # <--- [중요] QoS 추가
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge

import cv2
import numpy as np
import serial

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


class AstraYoloNode(Node):
    def __init__(self):
        super().__init__('astra_yolo_node')

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        # 파라미터 (기존과 동일)
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('yolo_model_path', '/home/han/robot/models/best2.pt')
        self.declare_parameter('yolo_conf_thres', 0.5)
        self.declare_parameter('target_class_id', 0)

        color_topic = self.get_parameter('color_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        cam_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        serial_baud = self.get_parameter('serial_baud').get_parameter_value().integer_value
        yolo_model_path = self.get_parameter('yolo_model_path').get_parameter_value().string_value
        self.yolo_conf_thres = self.get_parameter('yolo_conf_thres').get_parameter_value().double_value
        self.target_class_id = self.get_parameter('target_class_id').get_parameter_value().integer_value
        # [추가] 로그 타이머 변수
        self.last_log_time = 0.0
        self.camera_intrinsics = None
        self.latest_depth = None
        self.depth_encoding = None

        # YOLO 로드
        self.detector = None
        if YOLO_AVAILABLE:
            try:
                self.get_logger().info(f'Loading YOLO model: {yolo_model_path}')
                self.detector = YOLO(yolo_model_path)
            except Exception as e:
                self.get_logger().error(f'Failed to load YOLO model: {e}')

        # Serial
        self.ser = None
        try:
            self.ser = serial.Serial(serial_port, serial_baud, timeout=0.1)
        except Exception:
            pass

        # ---------- Subscribers (QoS 수정됨) ----------
        # 여기서 qos_profile_sensor_data를 써야 카메라 데이터를 안 놓칩니다.
        self.sub_color = self.create_subscription(
            Image, color_topic, self.color_callback, 10
        )
        self.sub_depth = self.create_subscription(
            Image, depth_topic, self.depth_callback, 10
        )
        self.sub_info = self.create_subscription(
            CameraInfo, cam_info_topic, self.info_callback, 10
        )
        
        self.debug_pub = self.create_publisher(Image, 'astra_yolo/debug_image', 10)

        self.get_logger().info('AstraYoloNode + QoS Fix started.')

    def info_callback(self, msg: CameraInfo):
        if self.camera_intrinsics is None:
            self.camera_intrinsics = {
                'fx': msg.k[0], 'fy': msg.k[4], 'cx': msg.k[2], 'cy': msg.k[5]
            }
            self.get_logger().info(f"Intrinsics Loaded: {self.camera_intrinsics}")

    def depth_callback(self, msg: Image):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_encoding = msg.encoding
        except Exception:
            pass

    def color_callback(self, msg: Image):
        # [디버그] 콜백이 들어오는지 무조건 찍어봄 (너무 빠르면 주석 처리)
        # self.get_logger().info('Color callback triggered!') 
        # print("!! IMAGE RECEIVED !!")
        try:
            color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image convert error: {e}")
            return

        debug_img = color_image.copy()

        # Depth 확인
        if self.latest_depth is None:
            # Depth가 없어서 리턴되는지 확인
            if str(self.get_clock().now().nanoseconds)[-2:] == "00": # 가끔 로그 찍기
                 self.get_logger().warn("WAITING FOR DEPTH... (Check depth topic)")
            
            cv2.putText(debug_img, "WAITING FOR DEPTH...", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255), 2)
            self.publish_debug(debug_img, msg.header)
            return

        # YOLO 실행
        det = None
        if self.detector is not None:
            det = self.run_yolo(color_image)

        if det is None:
            # 찾고 있다는 로그를 찍음
            # self.get_logger().info("Searching... No cube found.")
            
            cv2.putText(debug_img, "Searching...", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 165, 255), 2)
            self.publish_debug(debug_img, msg.header)
            return

        # 인식 성공 처리
        x1, y1, x2, y2 = det['bbox']
        u, v = int((x1 + x2)/2), int((y1 + y2)/2)

        cv2.rectangle(debug_img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 4)
        
        # Depth 조회
        depth_img = self.latest_depth
        if v < 0 or v >= depth_img.shape[0] or u < 0 or u >= depth_img.shape[1]:
            self.publish_debug(debug_img, msg.header)
            return
            
        d_val = depth_img[v, u]
        Z_m = float(d_val) / 1000.0 if self.depth_encoding in ['16UC1', 'mono16'] else float(d_val)

        if Z_m <= 0.0:
            self.publish_debug(debug_img, msg.header)
            return

        # TF 및 로그
        if self.camera_intrinsics:
            fx, fy = self.camera_intrinsics['fx'], self.camera_intrinsics['fy']
            cx, cy = self.camera_intrinsics['cx'], self.camera_intrinsics['cy']

            X_m = (u - cx) * Z_m / fx
            Y_m = (v - cy) * Z_m / fy
            
            # --- 로그 출력 (인식 되면 무조건 출력) ---
#            self.get_logger().info(f"Cube Found! X={X_m:.2f}, Y={Y_m:.2f}, Z={Z_m:.2f}")
            # --- 로그 출력 (속도 조절: 2초에 1번만) ---
            now = self.get_clock().now().seconds_nanoseconds()[0]
            if now - self.last_log_time > 2.0:
                self.get_logger().info(f"Cube Found! X={X_m:.2f}, Y={Y_m:.2f}, Z={Z_m:.2f}")
                self.last_log_time = now
# 카메라 설치 위치 (런치 파일에 적었던 값)
            CAM_X_OFFSET = -0.35
            CAM_Y_OFFSET = 0.0
            CAM_Z_OFFSET = 0.58            

            # t = TransformStamped()
            # t.header.stamp = self.get_clock().now().to_msg()
            # t.header.frame_id = 'base'
            # t.child_frame_id = "cube_target"
            # t.transform.translation.x = X_m
            # t.transform.translation.y = Y_m
            # t.transform.translation.z = Z_m
            # t.transform.rotation.w = 1.0
            # self.tf_broadcaster.sendTransform(t)
# --- [수정] 수직 아래(Top-down) 보기 좌표 변환 ---
            
            # 카메라 설치 위치 (님께서 알려주신 값)
            CAM_X_OFFSET = -0.35 # 로봇 중심에서 앞쪽으로 35cm
            CAM_Y_OFFSET = 0.0   # 좌우 중심
            CAM_Z_OFFSET = 0.60  # 바닥에서 높이 60cm
            
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            
            # 1. 부모는 무조건 'base'로 고정 (트리 끊김 방지)
            t.header.frame_id = 'base'
            t.child_frame_id = "cube_target"
            
            # 2. 좌표 변환 공식 (Top-down View)
            # - 카메라 Z(Depth)는 바닥까지의 거리 -> 로봇 Z축과 반대 방향
            # - 카메라 Y(이미지 아래쪽)는 로봇의 앞쪽 방향 -> 로봇 X축에 더함
            # - 카메라 X(이미지 우측)는 로봇의 오른쪽 -> 로봇 Y축의 반대(-Y)
            
            # X: (카메라 위치) + (이미지 상의 상하 위치 Y_m)
            t.transform.translation.x = CAM_X_OFFSET - Y_m
            
            # Y: (카메라 위치) - (이미지 상의 좌우 위치 X_m)
            t.transform.translation.y = CAM_Y_OFFSET - X_m
            
            # Z: (카메라 높이) - (카메라가 잰 거리 Z_m)
            # 예: 높이 60cm - 거리 55cm = 바닥 위 5cm
            t.transform.translation.z = CAM_Z_OFFSET - Z_m

            # 회전: 큐브 좌표계도 바닥과 평행하게 눕혀줍니다 (안 그러면 MoveIt이 헷갈려함)
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform(t)
            
            # 로그로 확인 (이제 X가 0.35 근처로 나와야 정상!)
            if now - self.last_log_time > 2.0:
                self.get_logger().warn(f"Top-Down Pos -> X={t.transform.translation.x:.2f}, Y={t.transform.translation.y:.2f}, Z={t.transform.translation.z:.2f}")
                self.last_log_time = now
        self.publish_debug(debug_img, msg.header)    
    def publish_debug(self, img, header):
        try:
            msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            msg.header = header
            self.debug_pub.publish(msg)
        except Exception:
            pass

    def run_yolo(self, bgr_image):
        if self.detector is None: return None
        rgb = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        try: results = self.detector.predict(source=rgb, verbose=False, conf=self.yolo_conf_thres)
        except: return None
        if not results or not results[0].boxes: return None
        
        for box in results[0].boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            if conf >= self.yolo_conf_thres and cls == self.target_class_id:
                return {'bbox': box.xyxy[0].tolist(), 'conf': conf}
        return None

    def send_serial(self, cmd):
        if self.ser and self.ser.is_open:
            try: self.ser.write(cmd.encode())
            except: pass

def main(args=None):
    rclpy.init(args=args)
    node = AstraYoloNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
