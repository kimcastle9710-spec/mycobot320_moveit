#!/usr/bin/env python3
from ultralytics import YOLO
import cv2

# 🔧 YOLO 모델 경로
MODEL_PATH = "/home/han/robot/models/best2.pt"

def main():
    # 1) YOLO 모델 로드
    model = YOLO(MODEL_PATH)

    # 2) 카메라 열기 (0: 기본 웹캠 / 1: 두 번째 카메라)
    cap = cv2.VideoCapture(7)

    if not cap.isOpened():
        print("[ERR] 카메라를 열 수 없음")
        return

    print("[INFO] 카메라 오픈 완료 — 'q' 누르면 종료")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[ERR] 프레임 읽기 실패")
            break

        # 3) YOLO 추론
        results = model(frame, conf=0.5)
        annotated = results[0].plot()  # bounding box 그리기

        # 4) 화면 표시
        cv2.imshow("YOLO Camera Test", annotated)

        # q 로 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

