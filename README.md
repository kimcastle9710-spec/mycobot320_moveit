# myCobot320 ROS MoveIt 시스템 통합 패키지
## Vision · Planning · Execution을 연결하는 로봇 미들웨어 아키텍처

본 리포지토리는 Elephant Robotics사의 6축 협동 로봇 **myCobot 320**을 대상으로  
**ROS 기반 MoveIt 모션 플래닝을 실제 하드웨어 실행까지 연결하는 시스템 통합(System Integration) 패키지**입니다.

이 프로젝트는 단순한 로봇 제어 예제가 아니라,  
로봇의 **기구학(Kinematics), 충돌 감지(Collision Detection), 경로 계획(Motion Planning),  
하드웨어 제어(Hardware Interface), 비전 입력(Vision)** 을 하나의 구조로 연결하는  
**로봇 미들웨어 아키텍처 구현**에 초점을 두고 있습니다.

---

## 📌 프로젝트 개요 (Executive Summary)

본 패키지는 다음과 같은 목표로 설계되었습니다.

- MoveIt 기반 **지능형 경로 계획(Motion Planning)** 을 myCobot 320에 적용
- RViz 시뮬레이션에서 생성된 계획을 **실물 로봇 실행(Sim-to-Real)** 으로 연결
- M5Stack / Raspberry Pi 기반 **서로 다른 하드웨어 구조를 단일 제어 인터페이스로 추상화**
- 실물 로봇 환경에서 발생하는 **제어·통신·인터페이스 문제를 소프트웨어적으로 해결**
- Astra Depth Camera를 활용한 **Vision 기반 로봇 응용 확장 구조 제공**

즉, MoveIt이라는 범용 로봇 플래닝 프레임워크를  
**myCobot 320이라는 실제 하드웨어 환경에 맞게 ‘현실화’하는 것이 핵심 가치**입니다.

---

## 🤖 대상 하드웨어: myCobot 320

- 제조사: Elephant Robotics
- 자유도: 6-DOF 협동 로봇
- 작업 반경: 약 350mm
- 최대 가반하중: 약 1kg

이러한 물리적 특성은 관절 제한, 속도, 가속도 설정에 직접적인 영향을 미치며,  
본 패키지는 이를 **MoveIt 구성 파일(joint_limits.yaml, SRDF 등)** 로 명시적으로 관리합니다.

---

## 🧠 ROS / MoveIt의 역할

MoveIt은 로봇에게 다음 질문에 답하는 역할을 합니다.

- 목표 위치(x, y, z)에 도달하기 위한 **관절 각도 계산 (역기구학, IK)**
- 이동 중 장애물을 회피하는 **충돌 없는 경로 탐색**
- 부드럽고 안전한 **궤적(Trajectory) 생성**

본 리포지토리는 MoveIt이  
**myCobot 320의 구조적·물리적 특성을 정확히 이해하도록 돕는 구성 데이터와 실행 구조를 제공**합니다.

---

## 🧩 전체 시스템 아키텍처

```text
[Vision Layer]
Astra Depth Camera / Vision Node
        ↓
[Planning Layer]
MoveIt (IK, Collision Checking, OMPL Planning)
        ↓
[Middleware Layer]
ROS Topics / Actions
        ↓
[Execution Layer]
ros2_control / Custom Hardware Interface
        ↓
[pymycobot SDK - Serial Protocol]
        ↓
[myCobot320 Physical Robot]
핵심 포인트
시뮬레이션 결과가 RViz에만 머물지 않고 실물 로봇으로 실행

비전 입력 → 경로 계획 → 하드웨어 제어까지 완결된 로봇 파이프라인

🔀 Simulation / Real 분리 설계 (Branch Strategy)
본 저장소는 하드웨어 유무에 따라 실행 경로를 명확히 분리하여 설계되었습니다.

🧪 fake_sym 브랜치 (Simulation Only)
하드웨어 없이 MoveIt 시뮬레이션 전용

FakeSystem 기반

경로 계획, 충돌 회피, 컨트롤러 설정 검증 목적

🤖 main 브랜치 (Real Hardware)
실물 myCobot 320 제어

Serial 기반 Hardware Interface 사용

실환경 테스트 및 안정화 목적
이 구조를 통해:
소프트웨어 설정 이슈
하드웨어 통신 및 제어 이슈
를 분리하여 디버깅 효율을 크게 향상시켰습니다.

🔀 하드웨어 아키텍처 추상화 (M5 / Raspberry Pi)
M5Stack 버전
ESP32 기반 M5Stack 내장

PC ↔ USB Serial 통신
/dev/ttyUSB0, /dev/ttyACM0


GPIO UART 통신
/dev/ttyAMA0

Launch 파일과 파라미터를 통해
동일한 MoveIt 제어 로직을 서로 다른 통신 구조로 추상화합니다.

📷 비전 확장 구성: Astra Depth Camera
본 리포지토리에는 Astra Depth Camera 기반 비전 입력 노드가 포함되어 있으며,
이는 필수 구성 요소가 아닌 확장 가능한 Vision 계층으로 설계되었습니다.

Astra 구성의 역할
MoveIt을 대체하지 않음

로봇 제어를 위한 입력 정보(목표 좌표, 객체 위치) 제공

Vision → Planning → Execution 구조 완성

관련 구성 요소
ros2_astra_camera/

Astra Depth Camera ROS 노드 (RGB / Depth 스트림)

astra_capture.py

카메라 데이터 캡처 및 테스트 유틸리티

astra_yolo_bridge/

비전 인식 결과를 로봇 제어로 연결하기 위한 브리지 노드

이를 통해 본 패키지는
Vision 기반 Pick & Place, 환경 인식 로봇 응용으로 확장 가능한 구조를 갖습니다.

📂 주요 디렉터리 구조
text
코드 복사
config/     → 기구학, 충돌, 관절 제한 설정
launch/     → 시스템 실행 진입점
scripts/    → Sim-to-Real 동기화 로직
src/        → 하드웨어 인터페이스 및 제어 코드
docs/       → 실물 로봇 트러블슈팅 및 분석 문서
📘 실물 하드웨어 문제 해결 기록
→ docs/troubleshooting.md

🚨 한계 및 고려 사항
Serial 기반 통신 구조로 인해 실시간 제어에는 한계 존재

고정밀 궤적 추종보다는 Pick & Place, 연구·교육 목적에 적합

특이점(Singularity) 근처에서 IK 실패 가능성 존재

📌 핵심 요약 (Key Takeaway)
이 프로젝트는 단순한 로봇 제어 예제가 아니라,
ROS MoveIt 기반 로봇 시스템을 실물 하드웨어와 비전 입력까지 연결한
시스템 통합 아키텍처 구현 프로젝트입니다.

지능형 경로 계획 ✔

하드웨어 추상화 ✔

Sim-to-Real 실행 ✔

Vision 확장 구조 ✔

실환경 트러블슈팅 ✔
