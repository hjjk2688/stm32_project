# 🚗 RTOS 기반 제스처 인식 RC카 컨트롤 시스템

[![STM32](https://img.shields.io/badge/STM32-03234B?style=for-the-badge&logo=stmicroelectronics&logoColor=white)](https://www.st.com/en/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus.html)
[![FreeRTOS](https://img.shields.io/badge/FreeRTOS-00979D?style=for-the-badge&logo=freertos&logoColor=white)](https://www.freertos.org/)
[![C](https://img.shields.io/badge/C-00599C?style=for-the-badge&logo=c&logoColor=white)](https://en.wikipedia.org/wiki/C_(programming_language))

IMU 센서를 이용한 제스처 제어와 초음파 센서 기반 장애물 감지 기능을 갖춘 양방향 블루투스 무선 RC카 시스템

## 📋 목차

- [프로젝트 소개](#-프로젝트-소개)
- [주요 기능](#-주요-기능)
- [시스템 아키텍처](#-시스템-아키텍처)
- [하드웨어 구성](#-하드웨어-구성)
- [소프트웨어 설계](#-소프트웨어-설계)
- [통신 프로토콜](#-통신-프로토콜)
- [시작하기](#-시작하기)
- [문제 해결 과정](#-문제-해결-과정)
- [시연 결과](#-시연-결과)
- [향후 계획](#-향후-계획)
- [연락처](#-연락처)

## 🎯 프로젝트 소개

본 프로젝트는 두 개의 STM32 보드를 Master-Slave 구조로 연결하여, IMU 센서 기반의 직관적인 제스처 컨트롤과 실시간 장애물 감지 기능을 구현한 임베디드 시스템입니다.

### 개발 기간
2025.10.13 ~ 2025.10.31 (19일)

### 핵심 특징

- ✨ **직관적인 제스처 제어**: MPU6050 IMU 센서를 활용한 손목 기울기 기반 RC카 조종
- 🔄 **양방향 무선 통신**: 블루투스 기반 커스텀 패킷 프로토콜로 안정적인 제어 및 상태 전송
- 🚨 **실시간 장애물 감지**: 전/후방 초음파 센서로 장애물 거리 측정 및 경고
- 📊 **시각적 피드백**: LCD 디스플레이를 통한 실시간 거리 정보 표시
- ⚡ **멀티태스킹**: FreeRTOS를 활용한 안정적인 동시 처리
- 🛡️ **안전 기능**: 긴급 정지 버튼 및 안전 재개 메커니즘

## ✨ 주요 기능

### Controller (Master)
- **IMU 제스처 인식**: Roll/Pitch 각도 계산 및 제어 신호 생성
- **실시간 거리 표시**: RC카로부터 수신한 전/후방 거리 데이터 LCD 출력
- **위험 경고 시스템**: 30cm 이내 장애물 감지 시 LCD 붉은색 점멸
- **긴급 정지**: 하드웨어 버튼을 통한 즉각적인 정지 명령
- **Non-blocking Superloop**: 효율적인 순차 처리 구조

### Vehicle (Slave)
- **모터 제어**: 전진/후진/좌회전/우회전/정지 명령 처리
- **장애물 감지**: HC-SR04 초음파 센서로 전/후방 거리 측정
- **청각 경고**: 장애물 근접 시 부저를 통한 경고음 발생
- **FreeRTOS 기반**: 4개의 태스크를 통한 안정적인 멀티태스킹
  - Motor Task (High Priority)
  - Bluetooth Task (Above Normal Priority)
  - Multi Task (Normal Priority)
  - Ultrasonic Task (Low Priority)

## 🏗️ 시스템 아키텍처

### 전체 구조
```
┌─────────────────────┐                    ┌─────────────────────┐
│   Controller        │                    │   RC Car (Vehicle)  │
│   (Master)          │                    │   (Slave)           │
│                     │                    │                     │
│  ┌──────────────┐   │                    │  ┌──────────────┐   │
│  │ IMU Sensor   │   │   Bluetooth        │  │ DC Motors    │   │
│  │ (MPU6050)    │───┼──────────────────► │  │              │   │
│  └──────────────┘   │   Control Signal   │  └──────────────┘   │
│                     │   (1-Byte)         │                     │
│  ┌──────────────┐   │                    │  ┌──────────────┐   │
│  │ LCD Display  │   │   Distance Data    │  │ Ultrasonic   │   │
│  │ (ST7735)     │ ◄─┼────────────────────┤  │ (HC-SR04)    │   │
│  └──────────────┘   │   (6-Byte Packet)  │  └──────────────┘   │
│                     │                    │                     │
│  ┌──────────────┐   │                    │  ┌──────────────┐   │
│  │ Stop Button  │   │                    │  │ Buzzer       │   │
│  └──────────────┘   │                    │  └──────────────┘   │
│                     │                    │                     │
│  Non-blocking       │                    │  FreeRTOS           │
│  Super Loop         │                    │  Multi-tasking      │
└─────────────────────┘                    └─────────────────────┘
```

### 아키텍처 비교

| 구분 | Master (Super-loop) | Slave (RTOS) |
|------|---------------------|--------------|
| **구조** | 단일 루프 순차 처리 | 멀티태스킹 병렬 처리 |
| **장점** | 구조가 단순하고 직관적<br>오버헤드가 적어 단일 목적에 유리 | 기능별 역할 분리가 명확<br>복잡한 동시 작업에 안정적 |
| **단점** | 기능이 복잡해지면 코드 관리 어려움 | 초기 설정이 복잡하고 약간의 오버헤드 발생 |
| **선택 이유** | IMU 값 계산 및 송수신이라는 명확한 단일 목표 | 모터, 센서, 통신 등 여러 장치의 동시 독립 구동 필요 |

## 🔧 하드웨어 구성

### MCU Boards
- **Controller**: NUCLEO-L412KB
- **Vehicle**: NUCLEO-F103RB

### 모듈 사양

| 모듈 | 모델 | 통신 인터페이스 | 전압 |
|------|------|----------------|------|
| IMU | MPU6050 (6축) | I2C | 3.3V |
| LCD | ST7735 Driver | SPI | 3.3V |
| Bluetooth | BoT-nLE523D | UART | 3.3V |
| Ultrasonic | HC-SR04 | GPIO (40KHz) | 3.3V |
| DC Motor | 120rpm, 2.2kgf.cm | PWM | 5V |
| Buzzer | PWM | PWM | 3.3V |

### 통신 인터페이스
- GPIO: 초음파 센서 트리거/에코, 버튼 입력
- UART: 블루투스 통신
- I2C: IMU 센서 데이터 수신
- SPI: LCD 디스플레이 제어
- PWM: 모터 및 부저 제어

## 💻 소프트웨어 설계

### Controller 구현

#### IMU 센서 처리 (MPU6050)
```c
// 상보 필터 적용: 자이로스코프(90%) + 가속도계(10%)
// Full-Scale Range: ±1000°/s (FS_SEL=2)
// Sensitivity Scale Factor: 32.8 (Gyro), 16,384 (Accel)
final_pitch = gyro_pitch * 0.9 + accel_pitch * 0.1;
final_roll = gyro_roll * 0.9 + accel_roll * 0.1;
```

#### 제스처 인식 로직
- **전진 (w)**: Pitch > 임계값
- **후진 (s)**: Pitch < -임계값
- **좌회전 (a)**: Roll < -임계값
- **우회전 (d)**: Roll > 임계값
- **정지 (f)**: 모든 각도가 임계값 이내

#### 인터럽트 우선순위
```c
Priority 0: EXTI line1 (Stop Button) - 최우선
Priority 1: I2C Event & Error (IMU Communication)
Priority 2: TIM2 (IMU Measurement Timing Control)
Priority 3: USART1 (Bluetooth Data Reception)
```

### Vehicle 구현 (FreeRTOS)

#### Task 구조
```c
// Task 우선순위 및 역할
Motor_Task      (High):         제어 명령 처리 및 모터 구동
Bluetooth_Task  (Above Normal): 제어 명령 수신 및 거리 데이터 송신
Multi_Task      (Normal):       부저 제어 및 경고 신호
Ultrasonic_Task (Low):          거리 측정 및 패킷 생성
```

#### Message Queue 기반 통신
```c
// Bluetooth Task → Motor Task
xQueueSend(motorQueue, &command, portMAX_DELAY);

// Motor Task에서 수신 대기
xQueueReceive(motorQueue, &received_cmd, portMAX_DELAY);
```

## 📡 통신 프로토콜

### 6-Byte 거리 데이터 패킷 (Vehicle → Controller)
```
┌─────┬─────┬──────────┬──────────┬──────────┬─────┐
│ SOP │ CMD │ Front(1B)│ Rear(1B) │ Checksum │ EOP │
├─────┼─────┼──────────┼──────────┼──────────┼─────┤
│0x02 │ 'D' │  0-255   │  0-255   │  XOR     │0x03 │
└─────┴─────┴──────────┴──────────┴──────────┴─────┘
```

**설계 의도**:
- 전/후방 거리를 하나의 패킷으로 전송하여 효율성 향상
- Checksum(XOR)을 통한 데이터 무결성 검증
- SOP/EOP를 통한 패킷 경계 명확화

### 1-Byte 제어 명령 (Controller → Vehicle)
```
┌─────┬──────────────────────┐
│ CMD │      Description     │
├─────┼──────────────────────┤
│ 'w' │ Forward              │
│ 's' │ Backward             │
│ 'a' │ Turn Left            │
│ 'd' │ Turn Right           │
│ 'f' │ Stop                 │
└─────┴──────────────────────┘
```

**설계 의도**:
- 최소 크기(1-Byte)로 응답 속도 최대화
- 파싱 없이 즉시 실행 가능한 직관적 설계

## 🚀 시작하기

### 필수 요구사항
- STM32CubeIDE
- STM32CubeMX
- ST-Link Driver
- Git

### 설치 및 빌드

1. **저장소 클론**
```bash
git clone https://github.com/hjjk2688/stm32_project.git
cd stm32_project
```

2. **Controller 프로젝트 빌드**
```bash
cd master_L412KB
# STM32CubeIDE에서 프로젝트 임포트
# Build Project (Ctrl+B)
# Flash to NUCLEO-L412KB
```

3. **Vehicle 프로젝트 빌드**
```bash
cd ../rc_car_slave
# STM32CubeIDE에서 프로젝트 임포트
# Build Project (Ctrl+B)
# Flash to NUCLEO-F103RB
```

### 하드웨어 연결

#### Controller (NUCLEO-L412KB)
- **IMU (MPU6050)**: I2C (SCL: PB6, SDA: PB7)
- **LCD (ST7735)**: SPI (SCK: PA5, MOSI: PA7, CS: PA6, DC: PA4, RST: PA3)
- **Bluetooth**: UART (TX: PA9, RX: PA10)
- **Stop Button**: GPIO (PA11)

#### Vehicle (NUCLEO-F103RB)
- **DC Motors**: PWM (PA0, PA1, PA2, PA3)
- **Bluetooth**: UART (TX: PA9, RX: PA10)
- **Ultrasonic Front**: GPIO (Trig: PB10, Echo: PB11)
- **Ultrasonic Rear**: GPIO (Trig: PB12, Echo: PB13)
- **Buzzer**: PWM (PA8)

### 블루투스 페어링

1. Vehicle의 Bluetooth 모듈(Slave) 전원 인가
2. Controller의 Bluetooth 모듈(Master) 전원 인가
3. 자동 페어링 대기 (약 5초)
4. 페어링 성공 시 LED 점등

## 🔍 문제 해결 과정

### 1. 급격한 제스처 시 측정 오류

**문제**: Controller를 급격하게 꺾으면 IMU 측정 실패

**원인**: 자이로스코프의 측정 범위 초과로 인한 센서 포화 상태

**해결**:
- Full-Scale Range: ±250°/s → ±1000°/s 증가
- 측정 범위 확대로 민감도 저하
- 상보 필터의 가속도 비율: 2% → 10% 증가
- 결과: 정확도 향상

### 2. LCD 업데이트 시 시스템 지연

**문제**: 실시간 LCD 업데이트 시 다른 동작 멈춤

**원인**: Display 함수의 긴 실행 시간으로 인한 블로킹

**해결**:
- 전체 화면 업데이트 → 변경된 영역만 부분 업데이트
- 디바운싱(1000ms) 적용으로 과도한 업데이트 방지
- 결과: 부드러운 동작

### 3. 과도한 IMU 데이터 전송

**문제**: I2C 통신이 지속적으로 발생하여 불필요한 데이터 과다

**원인**: 측정 주기 제어 부재

**해결**:
- Timer 인터럽트(20ms 주기) 도입
- 플래그 기반 I2C 활성화
- 결과: 효율적인 데이터 전송

### 4. RTOS Heap Size 부족

**문제**: Task가 제대로 동작하지 않음

**원인**: 최소 Stack Size(128) 고정으로 인한 메모리 낭비

**해결**:
- 디버깅용 printf() 함수 제거
- Task별 Stack Size 최적화
- 결과: 안정적인 동작

## 📸 시연 결과

### 주요 기능 시연

| 기능 | 설명 |
|------|------|
| 제스처 제어 | IMU 센서 기울기에 따른 RC카 제어 |
| 긴급 정지 | Stop 버튼을 통한 즉시 정지 |
| 장애물 감지 | 30cm 이내 근접 시 LCD 붉은색 점멸 |
| 거리 표시 | 전/후방 거리 실시간 LCD 출력 |
| 청각 경고 | 장애물 근접 시 부저 경고음 |

### 성능 지표
- **제어 응답 속도**: < 50ms
- **거리 측정 주기**: 100ms
- **IMU 샘플링 주기**: 20ms
- **블루투스 전송 속도**: 9600 bps
- **안정적인 제어 거리**: 최대 10m

## 🎓 개발 후기

### 주요 성과

1. **분산 시스템 구축**: 2-Board 무선 시스템 설계 및 구현
2. **실시간 시스템 경험**: FreeRTOS를 활용한 멀티태스킹 시스템 구현
3. **통신 프로토콜 설계**: 커스텀 패킷 프로토콜 정의 및 검증
4. **임베디드 통합 역량**: H/W 및 S/W 설계부터 디버깅까지 전 과정 경험

### 핵심 학습 내용

- **아키텍처 선택**: 용도에 맞는 Super Loop vs RTOS 적용
- **센서 융합**: 상보 필터를 통한 IMU 데이터 정확도 향상
- **메모리 최적화**: RTOS Heap 및 Stack 관리 기법
- **통신 안정성**: 패킷 검증 및 에러 핸들링

## 🔮 향후 계획

### Hardware 개선
- **9축 IMU 도입**: 지자기 센서 추가로 Yaw 드리프트 보정
  - 절대 방위 확보로 장거리 주행 안정성 향상
  - 가속도/자이로/자력계의 상보 보정으로 오차 감소

### Software 개선
- **칼만 필터 적용**: 노이즈 억제 및 정확한 제스처 인식
- **RTOS 메모리 최적화**: 스택 오버플로우 분석 및 동적 할당 개선
- **PID 제어 구현**: 모터 속도 및 방향 제어 정밀도 향상
- **전력 관리**: Sleep 모드 및 저전력 동작 모드 추가

## 📄 라이선스

이 프로젝트는 개인 포트폴리오 목적으로 작성되었습니다.

## 📞 연락처

**백현준 (Baek Hyun Jun)**

- 📧 Email: Hjjk2688@gmail.com
- 📱 Phone: +82 10-8820-2688
- 💻 GitHub: [hjjk2688](https://github.com/hjjk2688)

---

<div align="center">

**⭐ 이 프로젝트가 도움이 되었다면 Star를 눌러주세요! ⭐**

</div>
