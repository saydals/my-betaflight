# Betaflight Custom Firmware for Fixed-Wing

본 프로젝트는 [Betaflight](https://github.com/betaflight/betaflight) 4.5.3을 기반으로 **RC 고정익(비행기)** 에 최적화된 개조 펌웨어입니다.

---

## 1. 주요 패치 (Key Features)

| #   | 패치 내용                           | 설명                                                                  |
| --- | ------------------------------- | ------------------------------------------------------------------- |
| 1   | **Wing Rescue (GPS Rescue)**    | PID Profile 3 전용 매핑 + 12단계 상태 머신으로 고정익 자율 복귀/착륙 (Flyaway 방지)        |
| 2   | **OSD 레스큐 좌표/고도**               | 레스큐 작동 중 목표 좌표(Home/A/B) 및 실시간 목표 고도 OSD 표시                         |
| 3   | **단독 셔틀 모드**                    | 레스큐 실행 Aux 키를 1500 미만으로 실행시 두 지점을 왕복 무한 비행                          |
| 4   | **I-term 회전 최적화**               | 소각도 근사 대신 로드리게스 회전 변환(Rodrigues' Rotation) 적용, 빠른 회전 비행 중 자세 정확도 향상 |
| 5   | **Dshot 모터 버저**                 | 기체가 Arming 상태가 되면 모터 비콘음 알림 (RX_SET 옵션 활성 시)                        |
| 6   | **비행기 GUI 상시 표시**               | 믹서 종류 무관, Configurator 메인화면에 3D 비행기 모델 항상 표시                        |
| 7   | **예약 짐벌 서보 제거**                 | 강제 짐벌 서보 할당 코드 삭제 → 서보 0번부터 자유 할당 가능                                |
| 8   | **OSD Ready 필드**                | 레스큐 세부 단계(Phase) 실시간 표시                                             |
| 9   | **Aux Value → 서보 모니터**          | 서보 0~3 출력값(us)을 4자리 숫자로 한 줄 표시                                      |
| 10  | **GPS LED**                     | 사용자 설정 최소 위성 수 완전 만족 시에만 녹색 점등                                      |
| 11  | **Servo Range Extension**       | 서보 min/max 범위 확장 시 비례 스케일링 자동 적용 (비대칭/중립 변경 지원, AUX 포워딩 보호)         |
| 12  | **Board Alignment Tuning Mode** | 비행 중 스틱으로 보드 정렬값(align_board_roll/pitch/yaw) 실시간 조절                 |
| 13  | **Ready-to-Arm Wiggle**         | 에일러론 타면을 주기적으로 흔들어 아밍 가능 상태를 시각적으로 알림                               |
| 14  | **Servo Trim Mode**             | 조종기 스틱 입력으로 서보 물리 중립(middle) 실시간 조정 (자이로 기체에서도 사용 가능)               |

| 15  | **PID Profile AUX Switching**   | AUX 스위치로 비행 중 PID 프로파일 즉시 전환 (Configurator 미지원, CLI 전용 설정)          |
|

### 1.1 I-term Rotation DAMP 제어

`rotateVector()`는 회전 중 I-term(및 axisError)에 로드리게스 회전 변환(Rodrigues' Rotation)을 적용합니다.  
여기에 추가로 **DAMP** 계수를 곱해 I-term을 선택적으로 감쇠시키며, 감쇠 강도는 CLI 파라미터 `horizon_level_strength` (`pid[PID_LEVEL].I`, 0~100)로 제어합니다.

| 값       | DAMP 계수                 | 회전 특성         | 셋업 에러 보정             |
| ------- | ----------------------- | ------------- | -------------------- |
| **100** | 1.000000 (순수 Rodrigues) | 깔끔한 회전, 감쇠 없음 | 보정 최소 (에러 누적 가능)     |
| **50**  | 0.999950                | 중간            | 중간                   |
| **0**   | 0.999900 (최대 감쇠)        | 회전 시 꿀렁임 가능   | 보정 최대 (셋업 에러 빠르게 수렴) |

**설정:** `set horizon_level_strength = <0-100>` (current profile)

- **100에 가까울수록** 순수 로드리게스 변환에 가까워져 회전이 깔끔하지만, 기체 셋업 에러(트림/IMU 정렬 오차) 보정은 줄어듦

- **0에 가까울수록** 회전 중 I-term이 더 많이 감쇠되어 셋업 에러 보정량은 커지나, 회전 시 꿀렁임이 발생할 수 있음

- 실제 테스트 시 이론과 다르게 모두 비슷하게 반응. 원본 소각도 근사값 보다는 훨씬 정확한 회전시 자세 유지를 보임
  
  ### 1.2 튜닝 가이드: Board Alignment + DAMP 연계 세팅

I-term Rotation과 Board Alignment는 **회전 시 꿀렁임**이라는 동일한 증상을 공유하므로,
두 기능을 분리하여 단계별로 튜닝해야 최적의 결과를 얻을 수 있습니다.

**원칙:** 물리적 오차(보드 정렬)를 먼저 제거한 후, 남은 소프트웨어적 셋업 에러를 DAMP로 보정합니다.

#### 1단계 — 보드 정렬 (물리적 오차 제거)

1. `horizon_level_strength = 100` (순수 Rodrigues, DAMP 없음)으로 설정
2. 호버링 비행 중 롤링 → 보드 정렬 튜닝 모드(BOARD ALIGN)로 정렬 보정
3. 회전이 가장 깔끔한 지점에서 정렬 완료 (DAMP가 오차를 가리지 않으므로 정확한 진단 가능)

#### 2단계 — DAMP 튜닝 (잔여 셋업 에러 보정)

1. 보드 정렬 완료 후 `horizon_level_strength`를 70~90 범위에서 조정
2. 수평 비행 중 나이프엣지, 스핀, 급뱅크 기동 수행
3. 꿀렁임이 있으면 값을 낮추고, I-term 회복(복귀 후 자세 유지력)이 느리면 값을 높임
4. 최종 값 확정

#### DAMP가 보정하는 잔여 셋업 에러

| 에러 유형            | 설명                                     | DAMP 영향                          |
| ---------------- | -------------------------------------- | -------------------------------- |
| **무게 중심(CG) 오차** | 전방/후방 무게 쏠림 → I-term이 지속적으로 피치 트림 보정 중 | 기동마다 I-term이 교란되므로 DAMP가 빠르게 재수렴 |
| **타면 트림 오차**     | 에일러론/엘리베이터 기계적 중립 불일치                  | CG 오차와 동일한 패턴, DAMP로 잡음          |
| **IMU 정렬 잔여 오차** | 1단계에서 완전히 제거되지 않은 미세 오차                | 보통 낮은 DAMP로도 충분                  |

---

## 2. Wing Rescue (GPS Rescue) — 고정익 자율 복귀 시스템

본 시스템은 Betaflight 고정익(Fixed-wing) 항공기의 비상 상황 시 자동으로 안전하게 홈 포인트로 복귀하고 착륙시키는 자율 비행 시스템입니다.  
특히 **PID Profile 3를 레스큐 제어 파라미터로 전용 매핑**하여, 사용자가 CLI/Configurator GUI를 통해 실시간으로 응답성을 튜닝할 수 있는 것이 가장 큰 특징입니다.

### 2.1 동작 로직: 12단계 상태 머신 (Phase Machine)

GPS Rescue는 고도 확보부터 착륙까지 총 12단계의 상태 머신(State Machine)으로 정밀하게 제어됩니다.

| Phase                | 설명                                                                                                               |
| -------------------- | ---------------------------------------------------------------------------------------------------------------- |
| **INITIALIZE**       | 초기화 및 GPS 데이터 수신 대기. GPS fix만 있고 home fix가 없을 때 AUX<1500이면 헤딩 기반 SHUTTLE_INFINITE로 직접 진입                         |
| **ATTAIN_ALT**       | 설정 피치각 고정, 직선 상승하여 목표 고도 확보                                                                                      |
| **FLY_HOME**         | A포인트 타겟으로 복귀 순항                                                                                                  |
| **POINT_A**          | `descentAlt` 짝수=이륙방향, 홀수=복귀방향에 A포인트 매핑                                                                           |
| **SHUTTLE**          | A↔B포인트 왕복 셔틀 모드                                                                                                  |
| **SHUTTLE_INFINITE** | 무한 셔틀 모드 — AUX 스위치로 ON/OFF 제어. Home fix가 있으면 현재 위치 기준 홈 방향으로 A-B 생성. Home fix가 없으면 기체 헤딩 기준 좌(A)/우(B)에 포인트 자동 생성 |
| **SHUTTLE_DESCENT**  | 셔틀 완료 후 하강 시작                                                                                                    |
| **DESCENT**          | 하강 단계 진입                                                                                                         |
| **FINAL_DESCENT**    | 홈 반경 30m 진입까지 속도/고도 추종                                                                                           |
| **LANDING**          | 30m 이내에서 landingAlt+landingSpeed 조건 충족 시 활공 스로틀 3초 → 스로틀 1000 차단                                                 |
| **LANDED**           | 착륙 완료 — 충격 감지(disarmOnImpact)로 지면 접촉 즉시 모터 정지                                                                    |
| **RESCUE_FLYAWAY**   | 비정상 상황 감지 시 안전 중단 (Flyaway 방지)                                                                                   |

#### 상승 및 귀환

INITIALIZE 후 **ATTAIN_ALT**(고도 확보) 단계를 거쳐 **FLY_HOME**(홈 귀환) 단계로 진입합니다.  
설정 피치각으로 3초간 직선 상승 후 목표 고도에 도달하면 귀환을 시작합니다.

#### 셔틀 모드 (특화 기능)

단순히 귀환하는 것을 넘어, 특정 지점(A↔B)을 왕복하는 **SHUTTLE** 모드를 지원합니다.  
`ShuttleCount` 값에 따라 왕복 횟수가 결정되며, 무한 반복(SHUTTLE_INFINITE) 설정이 가능하여 정찰이나 특정 구간 순찰 시 유용합니다.  
셔틀 모드가 활성화되면 A포인트~B포인트를 CPA(Closest Point of Approach) 알고리즘으로 판정하여 전환점을 자동 생성합니다.

##### SHUTTLE_INFINITE (무한 셔틀) — Home Fix 유/무에 따른 동작 차이

**Home Fix가 있는 경우 (기존 동작):**

- 현재 위치를 기준(C 포인트)으로 홈 방향으로 B포인트가 생성됩니다.
- A(C) ↔ B를 왕복하며 AUX 스위치로 ON/OFF를 제어합니다.
- Rescue 전 구간에서 AUX < 1500 시 무한셔틀 진입 가능합니다.

**Home Fix가 없고 GPS Fix만 있는 경우 (신규):**

- `gpsRescueIsDisabled()`가 false를 반환하므로 Rescue 모드 진입이 가능합니다.
- `checkGPSRescueIsAvailable()`가 true를 반환하므로 Rescue가 활성화됩니다.
- INITIALIZE 단계에서 AUX < 1500이면 기체의 **현재 헤딩 방향**을 기준으로:
  - **A 포인트**: 헤딩 기준 **좌측 90°** 방향으로 `ShuttleDistance / 2` 거리에 생성
  - **B 포인트**: 헤딩 기준 **우측 90°** 방향으로 `ShuttleDistance / 2` 거리에 생성
  - A-B 간 거리 = ShuttleDistance (Profile 3 Roll F 값)
- 헤딩 기반 셔틀은 홈 방향이 없으므로 자체 A-B 축을 따라 CPA로 전환점을 판정하며 왕복합니다.
- AUX >= 1500 시 RESCUE_INITIALIZE로 복귀 → Home Fix가 없으므로 `RESCUE_NO_HOME_POINT` 실패 처리

#### 착륙 과정

**DESCENT**(하강) 단계에서 홈 근처에 도달하면 **FINAL_DESCENT**로 전환됩니다.  
**LANDING** 단계에서는 `landingAlt` + `landingSpeed` 조건이 충족되면 활공용 스로틀로 3초간 비행 후 스로틀 1000으로 차단합니다.  
착륙 시 충격 감지 기능(disarmOnImpact)이 내장되어 있어 지면 접촉 즉시 모터를 정지시켜 기체를 보호합니다.

### 2.2 핵심 제어 기술

#### 스마트 헤딩(Smart Heading) & 뱅크턴

기체의 현재 위치와 홈 방향의 오차에 따라 **요(Yaw) 제어**와 **조화선회(Bank-turn)** 모드를 지능적으로 전환합니다.

- **오차가 작을 때**: 요(Yaw) PI 제어로 정밀하게 헤딩 보정
- **오차가 클 때**: 뱅크를 활용한 선회로 기동성 확보

선회 시 발생할 수 있는 고도 손실은 `BANK_BOOST_FLYHOME` 계수를 통해 쓰로틀을 보상합니다.

#### PID Profile 3 전용 매핑

레스큐 제어 파라미터가 **PID Profile 3의 P/I/D 변수를 전용으로 대체(Repurpose)**하여 사용합니다.

| PID 축       | 할당 기능                     |
| ----------- | ------------------------- |
| **Roll P**  | 뱅크각(Bank Angle) 게인        |
| **Roll I**  | 뱅크 유지 및 피치 보정 적분          |
| **Roll D**  | 조화선회 요(Yaw) 제어 감쇠         |
| **Pitch P** | 상승/착륙 시 피치 각도 게인          |
| **Pitch I** | 고도 유지 적분 게인 (AltholdGain) |
| **Pitch D** | 착륙 피치 각도 (LandingPitch)   |
| **Yaw P**   | 셔틀 전용 뱅크 게인 (SBankGain)   |
| **Yaw I**   | 요 헤딩 유지 적분                |
| **Yaw D**   | 하강 종료 고도 (descentAlt)     |

이를 통해 복잡한 코드 수정 없이도 사용자는 익숙한 PID 튜닝 방식으로 레스큐의 비행 스타일을 변경할 수 있습니다.

### 2.3 안전 및 장애 대응 (Sanity Check)

시스템은 비행 중 지속적으로 안전성을 진단합니다.

- **Flyaway 방지**: 30초간 속도 부족이나 GPS 신호 상실 등 비정상 상황 발생 시 `RESCUE_FLYAWAY` 상태로 진입하여 레스큐를 안전하게 중단하거나 강제 착륙을 시도합니다.
- **데이터 필터링**: GPS 속도 데이터는 **3차 LPF(Low Pass Filter)**를 적용하여 노이즈를 제거하고, 정밀한 속도 제어가 가능하도록 업샘플링합니다.
- **속도 PI 제어**: `VELOCITY_KP`, `VELOCITY_KI`를 통해 목표 지상 속도에 도달하도록 정밀 제어합니다.

### 2.4 핵심 파라미터 (Profile 3 전용)

| 축         | Proportional      | Integral                 | D Max               | Derivative         | Feedforward            |
| --------- | ----------------- | ------------------------ | ------------------- | ------------------ | ---------------------- |
| **ROLL**  | BankGain     = 30 | BankPitchGain = 30       | BankYawGain = 30    | ShuttleCount = 1   | ShuttleDistance = 80   |
| **PITCH** | AscendPitch = 100 | MidPitch           = 100 | LandingPitch  = 100 | LandingSpeed = 4   | Alt Hold Gain     = 25 |
| **YAW**   | S-Bank  Gain = 15 | SBankPitchGain = 15      | DescentAlt    = 30  | Landing Alt    = 2 | HeadingYawGain = 30    |

### 2.5 튜닝 가이드

쉬운 세팅 요령.

끝이 Gain으로 끝나는 범위는 대체로 10-30 사이입니다.

끝이 Pitch로 끝나는 범위는 사용법을 익히기전에는 100으로 놓으시면 됩니다.

BankGain과 BankPitchGain은 뱅크턴시 강도이며 비 대칭 입력시 상승 또는 하강합니다.

예) 10-30 : 작은 뱅크각에 큰 피치는 상승합니다. 30-10 처럼 하면 하강합니다.

주) S-는 셔틀모드의 뱅크턴시 강도이며 작은값은 원형궤도 큰값은 직선궤도를 그립니다.

Yaw가 없는 기체의 경우에도 작동하며 2가지 YawGain을 사용하면 뱅크턴시 빨리 회전합니다.

그 외의 값은 사용자의 설계대로 지정입니다.

ShuttleDistance의 경우 너무 작은값을 주면 정상적인 궤도 비행이 어렵습니다. 50 이상 추천.

DescentAlt 는 GPS 레스큐 파라미터에 있는 descent distance 거리에서의 목표 고도입니다.

Landing 거리는 홈에서 30미터로 하드코딩 되어 있어 수정 불가합니다.

홈에서 30미터 도착할때의 목표 고도와 목표 속도를 지정할 수 있습니다.

홈에서 30미터에 도착했을때 Landing Pitch의 자세로 쓰로틀을 줄이며 활공 착륙합니다.

**OSD 확인**: OSD Element를 통해 현재 Phase(gpsRescueGetPhase)와 타겟까지의 거리, 셔틀 횟수 등을 실시간 모니터링할 수 있습니다.

### ⚠️ 중요 - PID Profile 3 경고

레스큐 제어 파라미터가 **Profile 3 변수를 전용으로 대체(Repurpose)**하여 사용합니다.  
**수동 비행 시 Profile 3 절대 선택 금지** - Profile 1 또는 2를 사용하세요.

Profile 1으로 바꾸려면 탭에서 선택한 후 내용을 하나라도 바꾸고 저장해야 profile 1으로

비행합니다.  D gain이 50 이라면 49 로 바꾸고 저장을 눌러야 활성 프로파일로 저장됩니다.

### 

## 3. Servo Range Extension (서보 범위 확장)

### 개요

서보의 `min/max/middle`를 기본값(1000-1500-2000)에서 확장(예: 500-1500-2500)할 때,  
**원래 deviation(편차) 비율을 유지한 채 새 범위에 맞게 자동 비례 스케일링**합니다.

### 수정 파일

`src/main/flight/servos.h`, `src/main/flight/servos.c`, `src/main/cli/settings.c`

---

## 4. 믹서 설정 (Mixer)

### 3.1 Airplane Mixer

| 출력핀    | 기능      | 설명         |
| ------ | ------- | ---------- |
| S1, S2 | **모터**  | Dshot 지원   |
| S3, S4 | 서보 0, 1 | 에일러론 자동 할당 |
| S5     | 서보 2    | 엘리베이터 할당   |
| S6     | 서보 3    | 러더 할당      |

### 3.2 Custom Airplane Mixer

S1, S2는 모터 고정 (S2를 서보로 대체 불가). S3~S6 완전 커스텀 할당 가능.

**CLI 예시:**

```
mixer CUSTOMAIRPLANE
mmix reset
mmix 0  1.000  0.000  0.000  0.300
mmix 1  1.000  0.000  0.000 -0.300

smix reset
smix 0 0 0 100 0 0 100 0
smix 1 1 0 100 0 0 100 0
smix 2 2 1 100 0 0 100 0
smix 3 3 2 100 0 0 100 0
```

### 서보 입력 소스 ID

| ID  | 입력 소스               | ID     | 입력 소스         |
| --- | ------------------- | ------ | ------------- |
| 0   | Stabilized ROLL     | 7      | RC THROTTLE   |
| 1   | Stabilized PITCH    | 8~11   | RC AUX 1~4    |
| 2   | Stabilized YAW      | 12     | GIMBAL PITCH  |
| 3   | Stabilized THROTTLE | 13     | GIMBAL ROLL   |
| 4~6 | RC ROLL/PITCH/YAW   | **14** | **Bird Flap** |

**Elevon(전비익) 예시:**

```
smix 0 0 0 50 0 0 100 0     # 우측: 롤 50%
smix 1 0 1 50 0 0 100 0     # 우측: 피치 50%
smix 2 1 0 -50 0 0 100 0    # 좌측: 롤 -50%
smix 3 1 1  50 0 0 100 0    # 좌측: 피치 50%
```

---

## 5. Bird Flap (새 날개짓)

새의 날갯짓을 모방한 서보 플랩 기능. smix id **14** 사용.

### 활성화

smix 룰에 `INPUT_BIRD_FLAP(14)`을 사용하는 서보를 지정하면 자동으로 활성화됩니다.

```
smix 0 0 14 100 0 0 100 0   # 서보 0번에 Bird Flap 할당
```

동작 ON/OFF는 **BOXUSER1** 모드로 제어합니다. Configurator → 모드 탭 → BOXUSER1에 원하는 스위치를 배정하세요.

### CLI 파라미터

| 파라미터                       | 기본값  | 범위       | 설명                               |
| -------------------------- | ---- | -------- | -------------------------------- |
| `bird_flap_max_freq_x10`   | 20   | 5~40     | 최대 플랩 주파수 (×0.1 Hz, 기본 2.0Hz)    |
| `bird_flap_min_freq_x10`   | 3    | 1~10     | 최소 플랩 주파수 (×0.1 Hz, 기본 0.3Hz)    |
| `bird_flap_max_amplitude`  | 200  | 0~500    | 상향 스트로크 진폭 (PWM 단위)              |
| `bird_flap_down_amplitude` | 300  | 0~500    | 하향 스트로크 진폭 (PWM 단위)              |
| `bird_flap_up_ratio`       | 40   | 30~80    | 상행 시간비율 (기본 40=Up 40%, Down 60%) |
| `bird_flap_servo_speed`    | 500  | 10~500   | 서보 각속도 제한 (deg/s)                |
| `bird_flap_soft_start_ms`  | 800  | 100~3000 | 활성화 시 부드러운 시작 시간 (ms)            |
| `bird_flap_soft_stop_ms`   | 600  | 100~3000 | 비활성화 시 부드러운 정지 시간 (ms)           |
| `bird_flap_freq_tau_ms`    | 100  | 10~1000  | 주파수 LPF 타우 (ms)                  |
| `bird_flap_slew_rate`      | 3000 | 0~10000  | 출력 슬루 제한 (units/s, 0=비활성)        |

### 동작 흐름

```
OFF → [BOXUSER1 ON] → STARTING → [진폭 100%] → ACTIVE → [BOXUSER1 OFF] → STOPPING → [진폭 0%] → OFF
```

AUX 채널 PWM 값(강도)에 따라 주파수가 minFreq ~ maxFreq로 선형 변화합니다.  
AUX에 스로틀을 50% 정도 믹스해두면 스로틀에 따라 날개짓 속도가 변하여 자연스럽습니다.

### 

## 6. Ready-to-Arm Wiggle (아밍 준비 완료 알림)

에일러론 타면을 주기적으로 흔들어 **시동(Arm)이 가능한 상태**임을 사용자에게 시각적으로 알리는 기능입니다.

부저나 LED가 없는 고정익 기체에서 유용하며, 방치 시에도 타면이 움직여 배터리 연결 상태를 시각적으로 경고합니다.

### CLI 파라미터

| 파라미터                     | 범위    | 기본값 | 설명                                 |
| ------------------------ | ----- | --- | ---------------------------------- |
| `ready_to_arm_wiggle_hz` | 0 ~ 6 | 4   | **0** = OFF, **1~6** = 왕복 주파수 (Hz) |

`ready_to_arm_wiggle_hz = 0`으로 설정하면 기능이 완전히 비활성화됩니다.

### 동작 흐름

```
배터리 ON
  └─ [부팅 5초 유예] 자이로 안정화 대기
       └─ [isArmingDisabled() == false]
            └─ ★ 1초간 타면 파닥 (설정된 Hz × sin_approx)
                 └─ 10초 대기
                      └─ ★ 재파닥 (Disarm 상태 유지 시 무한 반복)
                           └─ [ARM] → 즉시 중단, 순정 조종 신호로 완전 복귀
                                └─ [Disarm] → 즉시 재개
```

### 시나리오별 동작

| 시나리오           | 설명                                                         |
| -------------- | ---------------------------------------------------------- |
| **GPS 없는 비행기** | 자이로 보정 완료 → `isArmingDisabled()` = false → 즉시 첫 파닥         |
| **GPS + 아밍 락** | 위성 확보 전까지 `isArmingDisabled()` = true → 무동작 → 위성 확보 후 첫 파닥 |
| **방치 경고**      | Disarm 상태로 기체 방치 시 10초마다 지속적 타면 움직임 → 배터리 과방전 방지           |

### 기술 상세

| 항목           | 값                                                              |
| ------------ | -------------------------------------------------------------- |
| **알림 지속 시간** | 1초                                                             |
| **반복 주기**    | 10초 (알림 종료 후 9초 대기)                                            |
| **진폭**       | ±250 (중립 기준, input[INPUT_STABILIZED_ROLL] 오프셋)                 |
| **파형**       | `sin_approx()` 기반 사인파                                          |
| **적용 방식**    | `input[INPUT_STABILIZED_ROLL]`에 오프셋 주입 → 믹서가 자동 분배             |
| **트리거 조건**   | `!isArmingDisabled()` (`ARMING_FLAG(ARMED)`가 아니면서 모든 아밍 조건 충족) |
| **주입 위치**    | `servoMixer()` - input[] 완성 직후, 믹서 루프 진입 전                     |

### 적용 기체

- **Airplane** (FLAPPERON_1, FLAPPERON_2): 각각 +100%, +100%로 동일 방향 → 좌우 함께 흔들림
- **Flying Wing** (Elevon): FLAPPERON_1(+100%), FLAPPERON_2(-100%) → 반대 방향으로 흔들려 롤 효과 발생
- **Custom Mixer**: `INPUT_STABILIZED_ROLL`을 사용하는 모든 룰에 자동 적용

### 수정 파일

`src/main/flight/servos.h`, `src/main/flight/servos.c`, `src/main/cli/settings.c`

---

## 7. Board Alignment Tuning Mode (보드 정렬 튜닝 모드)

비행 중에 스틱으로 보드 정렬값(`align_board_roll`, `align_board_pitch`, `align_board_yaw`)을 직접 실시간 조절하는 기능입니다.

보드가 기체에 미세하게 비뚤게 장착된 경우, 롤 기동 시 피치가 같이 흔들리는 **축 간 간섭(cross-axis coupling)** 현상이 발생합니다. 이 기능을 사용하면 기체를 분해하지 않고, 공중에서 스틱 조작만으로 정렬값을 보정할 수 있습니다.

### 준비 (최초 1회)

송신기의 빈 스위치 하나를 `BOARD ALIGN` 모드에 배정합니다.

```
Configurator → 모드 탭 → BOARD ALIGN → 원하는 스위치 배정
```

### 사용 조건

- **반드시 Acro 모드**에서만 동작합니다.
- Angle / Horizon 모드에서는 스위치를 켜도 아무 반응이 없습니다.

### 비행 중 사용법

#### 1단계 — 문제 확인

이륙 후 Acro 모드로 전환합니다. 롤 기동을 해보고 피치가 같이 흔들리는지 확인합니다.

| 증상                        | 의심 항목               |
| ------------------------- | ------------------- |
| 롤 기동 시 피치가 **한쪽 방향으로 밀림** | `align_board_yaw`   |
| 롤 기동 시 피치가 **앞뒤로 진동**     | `align_board_pitch` |

실제로는 두 가지가 섞여 있는 경우도 많습니다. **Yaw → Pitch** 순서로 조절하는 것을 권장합니다.

#### 2단계 — 모드 진입

롤 스틱을 원하는 방향으로 밀어 롤 회전을 시작한 상태에서 스위치를 켭니다.

모드 진입 시 스틱 입력 처리:

| 스틱           | 모드 진입 후 동작                       |
| ------------ | -------------------------------- |
| **Roll**     | 진입 순간의 롤 레이트 고정 유지 (손을 떼도 계속 회전) |
| **Pitch**    | 강제 중립 (스틱 입력 무시)                 |
| **Yaw**      | 강제 중립 (스틱 입력 무시)                 |
| **Throttle** | 평소와 동일 (변화 없음)                   |

롤 회전이 자동으로 유지되므로 파일럿은 손을 떼고 기체 반응만 관찰하면 됩니다. 피치나 Yaw 흔들림이 있다면 순수하게 보드 정렬 오차 때문입니다 (스틱 조작 오염 없음).

#### 3단계 — 스틱으로 조절

롤이 계속 도는 상태에서 스틱으로 정렬값을 조절합니다.

| 조절 항목        | 스틱 방향       | 효과                      |
| ------------ | ----------- | ----------------------- |
| **Roll 정렬**  | Roll 스틱 오른쪽 | `align_board_roll` +1°  |
| **Roll 정렬**  | Roll 스틱 왼쪽  | `align_board_roll` -1°  |
| **Pitch 정렬** | Pitch 스틱 앞  | `align_board_pitch` +1° |
| **Pitch 정렬** | Pitch 스틱 뒤  | `align_board_pitch` -1° |
| **Yaw 정렬**   | Yaw 스틱 오른쪽  | `align_board_yaw` +1°   |
| **Yaw 정렬**   | Yaw 스틱 왼쪽   | `align_board_yaw` -1°   |

- 값이 바뀔 때마다 **삐 소리(BEEPER_RX_SET)** 가 1회 울립니다.
- 스틱을 계속 밀고 있어도 **1°씩만** 변경됩니다 (래치 방식). 한 번 떼고 다시 밀어야 다음 1°가 조정됩니다.

#### 4단계 — 확인 및 종료

피치/Yaw 흔들림이 사라지는 지점을 찾았으면 조절을 멈춥니다.

> ⚠️ **스위치를 끄기 전에 롤 스틱을 중립으로 복귀시킨 후 스위치를 끄세요.**
> 
> 스위치를 끄는 순간 고정됐던 롤 커맨드가 즉시 해제됩니다.
> 롤 스틱이 중립이 아닌 상태에서 끄면 기체가 갑자기 반응할 수 있습니다.

#### 5단계 — 저장

착륙 후 CLI에서 저장합니다. 저장하지 않으면 전원을 끄는 순간 조절값이 사라집니다.

또는 스틱커맨드로 저장할 수도 있습니다 ( disarm상태 + 모드2 왼손 좌하 오른손 우하 위치.)



### 조절 범위 제한

안전을 위해 과도한 변경을 방지하는 상한/하한이 적용됩니다.

| 축         | 최대 범위 |
| --------- | ----- |
| **Roll**  | ±10°  |
| **Pitch** | ±10°  |
| **Yaw**   | ±15°  |

한계에 도달하면 더 이상 값이 바뀌지 않으며 비프음도 울리지 않습니다.

### 현재 값 확인

CLI에서 언제든지 확인할 수 있습니다.

```
get align_board_roll
get align_board_pitch
get align_board_yaw
```

### 구현 상세

- **모드 박스 ID**: `BOXUSER2` (permanentId = 41) — Configurator에서 "BOARD ALIGN"으로 표시됨
- **동작 위치**: `src/main/fc/rc.c` — `updateRcCommands()` 함수 내
- **보드 정렬 행렬 갱신**: `src/main/sensors/boardalignment.c` — `updateBoardAlignmentMatrix()`
- **스틱 임계값**: HIGH > 1750, LOW < 1250 (래치 해제: HIGH < 1600, LOW > 1400)
- **수정 파일**: `src/main/fc/rc.c`, `src/main/fc/rc_modes.h`, `src/main/msp/msp_box.c`, `src/main/sensors/boardalignment.c`, `src/main/sensors/boardalignment.h`

## 8. Servo Trim Mode (서보 트림 모드)

### 개요

조종기 트림 버튼은 입력값(rcData)의 중립만 바꿀 뿐, 실제 서보 물리 중립(PWM middle)은 바꾸지 않습니다.  이 기능은 트림 버튼을 사용하지 않고 스틱으로 서보의 mid 값을 실시간 바꿉니다.

**Configurator → Modes 탭** → **"SERVO TRIM"** (permanentId = 6) 에 AUX 스위치를 할당합니다.

### 사용 조건

- **반드시 Acro 모드**에서만 동작합니다.
- Angle / Horizon 모드에서는 모드가 작동하지 않습니다.
- Board Align(BOXUSER2) 모드와 **상호배타** — 동시에 켤 수 없습니다.

### 모드 활성 시 입력 처리

| 스틱           | 모드 활성 시 동작                  |
| ------------ | --------------------------- |
| **Roll**     | 모드 진입 시점의 스틱 위치로 고정 (0이 아님) |
| **Pitch**    | 강제 0 고정 (스틱 입력 무시)          |
| **Yaw**      | 강제 0 고정 (스틱 입력 무시)          |
| **Throttle** | 평소와 동일 (변화 없음)              |

### 트림 조정 방법

1. AUX 스위치를 **ON** → OSD에 **"TRIM"** 표시
2. 조정할 축의 스틱을 **끝점**까지 움직입니다:
   - **증가(+)** : 스틱을 1750 이상으로 이동
   - **감소(-)** : 스틱을 1250 미만으로 이동
3. 한 번 움직일 때마다 `servo_trim_step` 값만큼 해당 서보들의 `middle`이 변경됩니다.
4. 같은 끝점에 계속 머물러도 **한 번만** 적용됩니다 (래치/히스테리시스).
5. 다음 스텝을 적용하려면 스틱을 **중립 근처**(1600 미만 / 1400 초과)로 복귀한 후 다시 끝점으로 움직입니다.
6. 변경 시 **Beeper**가 한 번 울립니다 (RX_SET).

### CLI 설정 파라미터

| 파라미터              | 타입     | 범위                  | 기본값    | 설명                             |
| ----------------- | ------ | ------------------- | ------ | ------------------------------ |
| `servo_trim_step` | uint8  | 1~20                | 5      | 스틱 끝점 1회당 변경되는 PWM 값           |
| `trim_aileron`    | lookup | OFF/NORMAL/REVERSED | NORMAL | 에일러론 트림 (0=사용안함, 1=자동계산, 2=반전) |
| `trim_elevator`   | lookup | OFF/NORMAL/REVERSED | NORMAL | 엘리베이터 트림                       |
| `trim_rudder`     | lookup | OFF/NORMAL/REVERSED | NORMAL | 러더 트림                          |

### 트림 방향 자동 계산

트림 방향은 다음 **3요소**를 곱하여 자동으로 계산됩니다:

```
effectiveDir = servoDirection() × sign(rule.rate) × sign(servoParams.rate)
```

- **servoDirection()**: `reversedSources` 비트필드에 저장된 반전 설정
- **rule.rate 부호**: 믹서 규칙의 rate 값 부호
- **servoParams.rate 부호**: 서보 설정의 rate 값 부호

방향이 반대일 때는 CLI에서 해당 축의 `trim_*` 파라미터를 `REVERSED`로 변경하면 즉시 반전됩니다 (코드 재컴파일 불필요).

### 안전장치

1. **트림 한계값 (하드 리밋)**: 모드 진입 시점 middle 기준 ±100 PWM
   - 모드를 껐다가 다시 켜면(재진입) 새로운 기준점이 설정됩니다.
2. **ANGLE / HORIZON 모드 가드**: 자이로 보조 모드에서는 작동하지 않습니다.
3. **Board Align 모드와 상호배타**: 두 모드는 동시에 사용할 수 없습니다.
4. **forwardFromChannel 서보 자동 제외**: `forwardFromChannel`이 설정된 서보는 트림 대상에서 자동으로 제외됩니다.

### Flying Wing (플라잉윙) 특이사항

플라잉윙에서는 두 개의 플래퍼론 서보가 ROLL 규칙과 PITCH 규칙을 동시에 받습니다.

- **에일러론 트림**: 두 플래퍼론 서보의 middle을 반대 방향으로 조정

- **엘리베이터 트림**: 두 플래퍼론 서보의 middle을 같은 방향으로 조정

- **동시 적용 시**: 두 트림 값이 같은 서보에 **누적(가산)** 됩니다 (엘리본 방식, 의도된 동작).
  
     예) 왼쪽 서보: +step(에일러론) + +step(엘리베이터) = +2step
  
         오른쪽 서보: -step(에일러론) + +step(엘리베이터) = 0

### OSD 표시

Servo Trim Mode 활성 시 OSD에 **"TRIM"** 문구가 표시됩니다 (FLIGHT MODE 표시 영역).

### 변경사항 영구 저장

트림 조정으로 변경된 `middle` 값은 **RAM에만** 저장됩니다. 재부팅 후에도 유지하려면:

```
# CLI 접속
save
```

또는 Configurator에서 **Save** 버튼을 클릭합니다. 또는 스틱커맨드로 저장.

### 구현 상세

- **모드 박스 ID**: `BOXHEADFREE` (permanentId = 6) — Configurator에서 **"SERVO TRIM"**으로 표시됨
- **동작 위치**: `src/main/fc/rc.c` — `updateRcCommands()` 함수 내 Servo Trim 블록
- **CLI 파라미터**: `src/main/cli/settings.c` — `servo_trim_step`, `trim_aileron`, `trim_elevator`, `trim_rudder`
- **접근자 함수**: `src/main/flight/servos.c` — `getActiveServoRuleCount()`, `getCurrentServoMixer()`
- **구현 상세 위치**: `src/main/fc/rc.c`, `src/main/flight/servos.h`, `src/main/flight/servos.c`, `src/main/cli/settings.c`, `src/main/cli/settings.h`, `src/main/msp/msp_box.c`, `src/main/fc/core.c`, `src/main/osd/osd_elements.c`

### 부록: Servo Trim Mode 없이 물리 중립 잡는 법 (forwardFromChannel 트릭)

Servo Trim Mode와는 별개로 존재하는 **기존 기법**입니다. AUX 채널이 여유 있고, 조종기가 트림 버튼 재할당을 지원하는 경우에 사용할 수 있습니다. 가능하다면 Trim Mode 보다는 아래 방법을

더 추천합니다.

**설정 방법:**

1. 조종기에서 원하는 조종 채널(예: 에일러론)을 남는 AUX 채널(예: AUX3)에 **복사**하도록 설정한다.

2. Betaflight CLI에서 해당 서보의 `forwardFromChannel`을 그 AUX 채널로 지정한다.
   
   ```
   servo 0 1000 1500 2000 100 -1    ← forwardFromChannel = -1 (비활성)
   servo 0 1000 1500 2000 100 2     ← forwardFromChannel = AUX3 (채널 2)
   ```

3. 조종기에서 원래 채널(에일러론)에 할당되어 있던 트림 버튼 기능은 **삭제**하고, 복사된 AUX 채널 쪽에 트림 버튼을 **재할당**한다.

4. 이제 조종기 트림 버튼으로 서보의 물리 중립(PWM)을 직접 바꿀 수 있다.

**제약 조건:**

- AUX 채널이 여유 있어야 한다.
- 조종기가 트림 버튼의 삭제/재할당을 **지원**해야 한다 (일부 조종기 기종은 지원하지 않음).
- 이 방법이 되는 조종기라면 Servo Trim Mode보다 간단할 수 있지만, 모든 조종기에서 쓸 수 있는 방법은 아니다.

**Servo Trim Mode와의 관계:**

`forwardFromChannel`이 설정된 서보는 Servo Trim Mode의 트림 대상에서 **자동으로 제외**됩니다. 따라서 두 방식은 서로 간섭하지 않으며, 기체에 따라 **혼용**할 수도 있습니다 (예: forwardFromChannel 트릭으로 에일러론만 조정 + Servo Trim Mode로 엘리베이터/러더 조정).

Servo Trim Mode의 주된 목적은 **수신기 추가채널 불필요  트림버튼 재할당기능이 필요없어 모든 조종기에서 Trim을 쓸 수 있게 하는 것**입니다.


---

## 15. PID Profile AUX Switching

AUX 스위치로 비행 중 PID 프로파일(1/2/3)을 즉시 전환할 수 있습니다. OSD 메뉴나 Configurator 연결 없이 상황에 맞는 PID 셋업으로 즉시 변경 가능합니다.

**⚠ Configurator Adjustments 탭에서는 이 기능을 선택할 수 없습니다.** 반드시 **CLI**로 설정해야 합니다.

### CLI 설정 방법

```
# 1. CLI 접속 후 아래 명령어 입력
adjrange 0 0 4 900 2100 31 5 900 2100

# 2. 저장
save

# 3. 확인
diff
```

### 파라미터 설명

```
adjrange <슬롯> <unused> <모니터채널> <시작> <끝> <기능> <스위치채널> <center> <scale>
  ─────── ──────── ───────────── ────── ───── ────── ───────────── ──────── ──────
    0        0          4         900   2100   31        5          900     2100
```

| 파라미터 | 값 | 의미 |
|----------|-----|------|
| 슬롯(index) | 0 | adjrange 슬롯 번호 (0~29) |
| unused | 0 | 항상 0 (과거 호환용) |
| 모니터채널(auxChannelIndex) | 4 = AUX5 | 범위 활성화를 감시할 채널 |
| 시작/끝(range) | 900~2100 | 풀레인지 (항상 활성) |
| **기능(adjustmentFunction)** | **31** | **`ADJUSTMENT_PID_PROFILE`** (enum 값 35가 아님!) |
| 스위치채널(auxSwitchChannelIndex) | 5 = AUX6 | 3포지션 스위치가 연결된 채널 |
| center/scale | 900/2100 | SELECT 모드에서는 사용되지 않음 |

### 스위치 채널 인덱스

| 리시버 채널 | 인덱스 |
|:----------:|:------:|
| AUX1 | 0 |
| AUX2 | 1 |
| AUX3 | 2 |
| AUX4 | 3 |
| AUX5 | 4 |
| **AUX6 (추천)** | **5** |
| AUX7 | 6 |
| AUX8 | 7 |

### OSD 표시 활성화 (선택)

```
set osd_pidrate_profile_pos = 2450   # 원하는 OSD 위치 (0이면 숨김)
save
```

OSD에 `"1-1"`, `"2-1"`, `"3-1"` 형식으로 **PID 프로파일 번호 - Rate 프로파일 번호**가 표시됩니다.

### 동작 확인 방법

| 단계 | 확인할 내용 | 정상 반응 |
|:----:|-----------|---------|
| 1 | 3포지션 스위치 조작 | LOW→MID→HIGH 순서대로 **비프음 1회/2회/3회** |
| 2 | 동일 위치 유지 | 추가 비프음 없음 (재호출 방지) |
| 3 | OSD 확인 | 설정한 위치에 프로파일 번호 실시간 갱신 |
| 4 | `profile` CLI 명령어 | 전환된 인덱스와 일치하는지 확인 |

### 자주 하는 실수

| 증상 | 원인 | 해결 |
|------|------|------|
| 스위치 조작해도 아무 반응 없음 (비프X, OSD 변화X) | `function` 값에 `35`(enum 값)를 넣음 | **`31`**로 수정 |
| OSD 변화 없음 (비프는 정상) | `osd_pidrate_profile_pos = 0` | `set osd_pidrate_profile_pos = 2450` |
| 비프 패턴이 이상함 (4회 등) | `PID_PROFILE_COUNT`가 3이 아닌 타겟 | `get` 명령어로 현재 값 확인 |

### 구현 상세

- **Adjustment Function**: `ADJUSTMENT_PID_PROFILE` (enum 값 35, `defaultAdjustmentConfigs[]` 위치 1-based 31)
- **파일**: `src/main/fc/rc_adjustments.h`, `src/main/fc/rc_adjustments.c`
- **프로파일 전환 함수**: `changePidProfile()` (`config/config.c`)
- **OSD 요소**: `OSD_PIDRATE_PROFILE` (기존 요소, 코드 수정 불필요)
- **블랙박스 로깅**: `FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT` 자동 기록

## License

GNU General Public License v3.0
