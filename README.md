# Betaflight Custom Firmware for Fixed-Wing

본 프로젝트는 [Betaflight](https://github.com/betaflight/betaflight) 4.5.3을 기반으로 **RC 고정익(비행기)** 에 최적화된 개조 펌웨어입니다.

---

## 1. 주요 패치 (Key Features)

| #   | 패치 내용                           | 설명                                                                    |
| --- | ------------------------------- | --------------------------------------------------------------------- |
| 1   | **I-term 회전 최적화**               | 소각도 근사 대신 로드리게스 회전 변환(Rodrigues' Rotation) 적용, 급회전 비행 중 자세 계산 정확도 향상    |
| 2   | **Dshot 모터 버저**                 | 기체가 Arming 상태가 되면 모터 비콘음 알림 (RX_SET 옵션 활성 시)                   |
| 3   | **비행기 GUI 상시 표시**               | 믹서 종류 무관, Configurator 메인화면에 3D 비행기 모델 항상 표시                          |
| 4   | **OSD 레스큐 좌표/고도**               | 레스큐 작동 중 목표 좌표(Home/A/B) 및 실시간 목표 고도 OSD 표시                           |
| 5   | **예약 짐벌 서보 제거**                 | 강제 짐벌 서보 할당 코드 삭제 → 서보 0번부터 자유 할당 가능                                  |
| 6   | **OSD Ready 필드**                | 레스큐 세부 단계(Phase) 실시간 표시                                               |
| 7   | **Aux Value → 서보 모니터**          | 서보 0~3 출력값(us)을 4자리 숫자로 한 줄 표시                                        |
| 8   | **단독 셔틀 모드**                    | 레스큐 실행 Aux 키를 1500 미만으로 실행시 두 지점을 왕복 무한 비행                                             |
| 9   | **GPS LED**                     | 사용자 설정 최소 위성 수 완전 만족 시에만 녹색 점등                                        |
| 10  | **Servo/Smix Rate 125% 확장**     | CLI에서 servo/smix rate를 -125~+125%까지 설정 가능 (Configurator GUI는 100% 한계) |
| 11  | **Board Alignment Tuning Mode** | 비행 중 스틱으로 보드 정렬값(align_board_roll/pitch/yaw) 실시간 조절                   |
| 12  | **Ready-to-Arm Wiggle**  |   비행 준비시 에일러론 서보를 좌우로 흔들어 사용자에게 알려줌                                             |

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

## 2. Servo / Smix Rate 125% 확장

### 개요

`cli.c` 검증 로직 변경을 통해 **servo rate**와 **smix rate**의 허용 범위를 기존 `-100~+100%`에서 **`-125~+125%`**로 확장했습니다.

### 중요: Configurator GUI(서보탭) vs CLI 차이

| 구분                       | servo rate          | smix rate                |
| ------------------------ | ------------------- | ------------------------ |
| **Configurator 서보탭 GUI** | ❌ **100%까지**만 표시/적용 | ❌ 적용 불가 (smix 전용 GUI 없음) |
| **CLI (`servo` cmd)**    | ✅ **125%** 입력 및 작동  | -                        |
| **CLI (`smix` cmd)**     | -                   | ✅ **125%** 입력 및 작동       |

- **서보탭(Servos 탭)** 의 rate 슬라이더는 여전히 100%가 최대입니다.  
  GUI에서 100% 초과 값을 설정하려면 직접 CLI로 입력해야 합니다.
- **smix rate 125%는 CLI로만 설정 가능**하며, 실제 서보 출력에 정상 반영됩니다.

### CLI 사용법

**Servo rate 125% (최종 출력 증폭):**

```
servo 0 1000 1500 2000 125 -1
```

- 모든 믹서 출력을 합산한 최종 결과물을 125%로 증폭
- 예: 중립 1500 기준 최대 편차가 ±500 → ±625 (최대 2125 또는 875)까지 확장

**Smix rate 125% (개별 입력 기여도 증폭):**

```
smix 2 2 2 125 0 0 100 0
```

- 특정 입력 채널이 서보에 기여하는 비율을 125%로
- 여러 smix 규칙 중 원하는 규칙의 rate(4번째 인자)만 변경

### 실제 출력 계산

예: `servo 0 1000 1500 2000` + `smix rate 125%` + 입력 최대값 →  
`1500 + (2000-1500) × 125/100 = 1500 + 625 = 2125`

출력 한도(`servo N min mid max`) 자체를 `servo 0 1000 1500 2500` 식으로 늘리면 2500까지 도달 가능합니다.

### 수정 파일

- `src/main/cli/cli.c`: 2곳의 rate 검증 조건문 수정 (`100` → `125`)

---

## 3. 믹서 설정 (Mixer)

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

## 4. GPS Rescue — 고정익 자율 복귀 시스템

본 시스템은 Betaflight 고정익(Fixed-wing) 항공기의 비상 상황 시 자동으로 안전하게 홈 포인트로 복귀하고 착륙시키는 자율 비행 시스템입니다.  
특히 **PID Profile 3를 레스큐 제어 파라미터로 전용 매핑**하여, 사용자가 CLI/Configurator GUI를 통해 실시간으로 응답성을 튜닝할 수 있는 것이 가장 큰 특징입니다.

### 4.1 동작 로직: 12단계 상태 머신 (Phase Machine)

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

### 4.2 핵심 제어 기술

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

### 4.3 안전 및 장애 대응 (Sanity Check)

시스템은 비행 중 지속적으로 안전성을 진단합니다.

- **Flyaway 방지**: 30초간 속도 부족이나 GPS 신호 상실 등 비정상 상황 발생 시 `RESCUE_FLYAWAY` 상태로 진입하여 레스큐를 안전하게 중단하거나 강제 착륙을 시도합니다.
- **데이터 필터링**: GPS 속도 데이터는 **3차 LPF(Low Pass Filter)**를 적용하여 노이즈를 제거하고, 정밀한 속도 제어가 가능하도록 업샘플링합니다.
- **속도 PI 제어**: `VELOCITY_KP`, `VELOCITY_KI`를 통해 목표 지상 속도에 도달하도록 정밀 제어합니다.

### 4.4 핵심 파라미터 (Profile 3 전용)

| 축         | Proportional      | Integral                 | D Max              | Derivative         | Feedforward            |
| --------- | ----------------- | ------------------------ | ------------------ | ------------------ | ---------------------- |
| **ROLL**  | BankGain     = 30 | BankPitchGain = 30       | BankYawGain = 30   | ShuttleCount = 1   | ShuttleDistance = 80   |
| **PITCH** | AscendPitch = 100 | MidPitch           = 100 | LadnigPitch  = 100 | LandingSpeed = 4   | Alt Hold Gain     = 25 |
| **YAW**   | S-Bank  Gain = 15 | SBankPitchGain = 15      | DescentAlt    = 30 | Landing Alt    = 2 | HeadingYawGain = 30    |



### 4.5 튜닝 가이드

Profile 3 의 PID로 레스큐 작동시 비행 특성을 조절합니다.

목적지로 뱅크턴 비행을 할때 BankGain ( 적정범위10-30 )이 크면 뱅크각을 크게합니다.

뱅크턴중 회전강도는 BankPitch (적정범위 10-30) 로 합니다.

빠른 회전을 원하면 30-30 을 사용하며 셔틀 비행 중 거의 직진 왕복을 합니다.

느린 회전을 원하면 10-10 을 사용하며 셔틀 비행 중 거의 원 운동을 합니다.

만약 뱅크각이 작고 뱅크피치를 크게 사용할경우 10-30 처럼 쓰면 회전중 원치않는 상승을 유발

만약 뱅크각이 크고 뱅크피치가 작을 경우 30-10 처럼 쓰면 회전중 원치않는 하강을 유발

S-Bank  Gain은 S 가 셔틀을 의미하며 A B 두 지점을 왕복할때의 뱅크턴 형태를 의미합니다.

위의 예시에서는 셔틀 모드에서는 거의 타원 운동이며 그 이외에서는 빠른 뱅크턴을 나타냅니다.

BankYaw HeadingYaw는 Yaw가 있는 비행기에서 작동하며 느린 회전을 원할때 값을 낮춤.

BankYaw HeadingYaw는 0-30 범위가 적당합니다.

Alt Hold Gain 는 고도 유지를 위한 피치 사용 강도를 나타냅니다.





**OSD 확인**: OSD Element를 통해 현재 Phase(gpsRescueGetPhase)와 타겟까지의 거리, 셔틀 횟수 등을 실시간 모니터링할 수 있습니다.

### ⚠️ 중요 - PID Profile 3 경고

레스큐 제어 파라미터가 **Profile 3 변수를 전용으로 대체(Repurpose)**하여 사용합니다.  
**수동 비행 시 Profile 3 절대 선택 금지** - Profile 1 또는 2를 사용하세요.

### 무한 셔틀 스탠바이

레스큐 ON 시 AUX < 1500 → 착륙 없이 무한 셔틀 홀딩 (InitShuttlePoints 실행)  
AUX >= 1500 → 정상 착륙 모드로 복귀 (RESCUE_INITIALIZE 재진입)  
(어느 단계든 Failsafe 수신 시 강제 안전 절차 우선)

**참고**: Home Fix가 없으면 AUX < 1500 조건에서만 무한셔틀 진입 가능하며, AUX >= 1500으로 전환 시 RESCUE_INITIALIZE를 거쳐 Home Fix 부재로 NO_HOME_POINT 실패 처리됩니다.

### 스로틀 제한

```
throttleHover - 150 <= 자동 스로틀 <= throttleHover + 250
```

---

## 5. Bird Flap (새 날개짓)

새의 날갯짓을 모방한 서보 플랩 기능. smix id **14** 사용.

### 활성화

```
smix 0 0 14 100 0 0 100 0   # 서보 0번에 Bird Flap 할당
```

Mode 탭 → **Bird Flap** 항목에 AUX 할당 → ON/OFF 제어

### CLI 파라미터

| 파라미터                      | 기본값  | 범위       | 설명                                     |
| ------------------------- | ---- | -------- | -------------------------------------- |
| `bird_flap_max_freq_10x`  | 20   | 5~40     | 최대 플랩 주파수 (×0.1 Hz, 기본 2.0Hz)          |
| `bird_flap_min_freq_10x`  | 3    | 1~10     | 최소 플랩 주파수 (×0.1 Hz, 기본 0.3Hz)          |
| `bird_flap_max_amplitude` | 1000 | 0~1000   | 최대 스트로크 진폭 (PWM 단위)                    |
| `bird_flap_servo_speed`   | 500  | 10~500   | 서보 각속도 제한 (deg/s)                      |
| `bird_flap_up_ratio_100x` | 40   | 30~50    | Upstroke 시간비율 (기본 40=Up 40%, Down 60%) |
| `bird_flap_soft_start_ms` | 800  | 100~3000 | 활성화 시 부드러운 시작 시간 (ms)                  |
| `bird_flap_soft_stop_ms`  | 600  | 100~3000 | 비활성화 시 부드러운 정지 시간 (ms)                 |

### 동작 흐름

```
OFF → [AUX ON] → STARTING → [진폭 100%] → ACTIVE → [AUX OFF] → STOPPING → [진폭 0%] → OFF
```

AUX 채널 PWM 값(강도)에 따라 주파수가 minFreq ~ maxFreq로 선형 변화합니다.  
AUX에 스로틀을 50% 정도 믹스해두면 스로틀에 따라 날개짓 속도가 변하여 자연스럽습니다.

---

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

```
save
```

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

## 8. Rescue Profile 3 PID 튜닝

| 축         | Proportional      | Integral                 | D Max              | Derivative         | Feedforward            |
| --------- | ----------------- | ------------------------ | ------------------ | ------------------ | ---------------------- |
| **ROLL**  | BankGain     = 30 | BankPitchGain = 30       | BankYawGain = 30   | ShuttleCount = 1   | ShuttleDistance = 80   |
| **PITCH** | AscendPitch = 100 | MidPitch           = 100 | LadnigPitch  = 100 | LandingSpeed = 4   | Alt Hold Gain     = 25 |
| **YAW**   | S-Bank  Gain = 15 | SBankPitchGain = 15      | DescentAlt    = 30 | Landing Alt    = 2 | HeadingYawGain = 30    |

Profile 3 의 PID로 레스큐 작동시 비행 특성을 조절합니다.

목적지로 뱅크턴 비행을 할때 BankGain ( 적정범위10-30 )이 크면 뱅크각을 크게합니다.

뱅크턴중 회전강도는 BankPitch (적정범위 10-30) 로 합니다.

빠른 회전을 원하면 30-30 을 사용하며 셔틀 비행 중 거의 직진 왕복을 합니다.

느린 회전을 원하면 10-10 을 사용하며 셔틀 비행 중 거의 원 운동을 합니다.

만약 뱅크각이 작고 뱅크피치를 크게 사용할경우 10-30 처럼 쓰면 회전중 원치않는 상승을 유발

만약 뱅크각이 크고 뱅크피치가 작을 경우 30-10 처럼 쓰면 회전중 원치않는 하강을 유발

S-Bank  Gain은 S 가 셔틀을 의미하며 A B 두 지점을 왕복할때의 뱅크턴 형태를 의미합니다.

위의 예시에서는 셔틀 모드에서는 거의 타원 운동이며 그 이외에서는 빠른 뱅크턴을 나타냅니다.

BankYaw HeadingYaw는 Yaw가 있는 비행기에서 작동하며 느린  회전을 원할때 값을 낮춤.

BankYaw HeadingYaw는  0-30 범위가 적당합니다.

Alt Hold Gain 는 고도 유지를 위한 피치 사용 강도를 나타냅니다.

---

---

## License

GNU General Public License v3.0
