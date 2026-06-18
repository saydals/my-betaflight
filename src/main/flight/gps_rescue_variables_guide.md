# GPS Rescue 실행 시 사용되는 변수 설명서

> **대상 파일**: `src/main/flight/gps_rescue.c` (1477줄)  
> **개요**: Betaflight 고정익 전용 GPS Rescue 및 셔틀 착륙 시스템의 모든 변수, 상수, 구조체, Phase 머신 정의서  
> **특이사항**: PID Profile 3의 파라미터를 레스큐 제어 파라미터로 전용 매핑하여 실시간 튜닝 지원

---

## 1. Phase 상태 머신 (rescuePhase_e)

레스큐 실행은 아래 12개 단계(Phase)로 구성된 상태 머신으로 동작한다.

| Phase 열거값 | 설명 | 진입 조건 | 다음 Phase 전환 조건 |
|---|---|---|---|
| `RESCUE_IDLE` | 대기 상태. 레스큐 비활성 | 기본 상태, 또는 `rescueStop()` 호출 | GPS_RESCUE_MODE 활성화 → `RESCUE_INITIALIZE` |
| `RESCUE_INITIALIZE` | 초기화. 변수 리셋 및 Phase 결정 | `rescueStart()` 호출 | 고도/거리 조건에 따라 `ATTAIN_ALT` / `FLY_HOME` / `SHUTTLE_INFINITE` / `DO_NOTHING` / `ABORT` |
| `RESCUE_ATTAIN_ALT` | 상승 단계 (고도 확보) | 현재고도 < 목표고도 | 고도 도달 또는 3초 타임아웃 → `FLY_HOME` |
| `RESCUE_FLY_HOME` | 귀환 단계 (A포인트 또는 홈 방향 비행) | 고도 확보 완료 | A포인트 CPA 터치 → `DESCENT`(shuttleCount=0) 또는 `SHUTTLE`(shuttleCount>0) |
| `RESCUE_SHUTTLE` | 셔틀 단계 (A↔B 왕복, shuttleCount 만큼) | FLY_HOME에서 A포인트 통과 후 (shuttleCount>0) | 셔틀 왕복 완료 → `SHUTTLE_DESCENT` |
| `RESCUE_SHUTTLE_INFINITE` | 무한 셔틀 모드 (AUX 스위치 연동) | INITIALIZE/ATTAIN_ALT/FLY_HOME/... 에서 AUX<1500 | AUX≥1500 → `INITIALIZE` 복귀 |
| `RESCUE_SHUTTLE_DESCENT` | 셔틀 하강 단계 (셔틀 유지하며 고도 낮춤) | 셔틀 왕복 완료 (`shuttleCount` 도달) | A포인트 도착 + 하강고도 도달 → `DESCENT` |
| `RESCUE_DESCENT` | 하강 단계 (홈 방향으로 하강) | 셔틀완료 또는 A통과(shuttleCount=0) | 홈 30m 이내 + 착륙고도 이하 → `LANDING` |
| `RESCUE_LANDING` | 최종 착륙 (throttleMin → 3초 후 PWM 1000) | 하강 조건 만족 | 충격 감지(`disarmOnImpact`) 또는 3초 후 모터정지 |
| `RESCUE_DO_NOTHING` | 아무것도 하지 않음 (실패/비정상 상황) | 위성부족/GPS손실/이륙실패/홈30m이내A없음 | 지속 (수동 복구) |
| `RESCUE_ABORT` | 중단 (착륙고도 이하에서 레스큐 발동) | INITIALIZE에서 5m이내 + 착륙고도 이하 | 즉시 `DO_NOTHING` |
| `RESCUE_COMPLETE` | 완료 | (예비) Phase 전환 없음 | `rescueStop()` → `IDLE` |

### Phase 전이 다이어그램 (ASCII)

```
IDLE ──[GPS_RESCUE_MODE ON]──→ INITIALIZE
                                  │
                          ┌───────┼───────────┐
                          │       │           │
                     ATTAIN_ALT  │    SHUTTLE_INFINITE
                          │       │           │
                          ▼       ▼           │
                      FLY_HOME  (A통과)       │
                          │       │           │
                     ┌────┘       │           │
                     ▼            ▼           │
              (shuttleCount>0) SHUTTLE ───────┘
                     │            │
                     └── COUNT완료┘
                          │
                          ▼
                   SHUTTLE_DESCENT
                          │
                     ┌────┘
                     ▼
                  DESCENT ──[홈30m + 착륙고도]──→ LANDING
                                                     │
                                              [충격감지/3초]
                                                     │
                                                     ▼
                                              (모터정지)
```

---

## 2. 제어 상수 (Magic Number 정리)

### 2.1 속도 및 요 제어

| 상수명 | 값 | 단위 | 설명 |
|---|---|---|---|
| `GPS_RESCUE_MAX_YAW_RATE` | 180 | 도/초 | 최대 요(yaw) 회전 속도 |
| `VELOCITY_KP` | 4.00 | - | 속도 PI 제어 비례 게인 |
| `VELOCITY_KI` | 1.00 | - | 속도 PI 제어 적분 게인 |
| `YAW_I_LIMIT` | `GPS_RESCUE_MAX_YAW_RATE * 0.3f` | 도/초 | 요 적분항 리미터 (54도/초) |

### 2.2 고도 제어

| 상수명 | 값 | 단위 | 설명 |
|---|---|---|---|
| `ALT_DEADBAND_M` | 0.3 | m | 고도 데드밴드 (이 범위 내에선 적분 업데이트 안 함) |
| `ALT_I_LIMIT` | 20.0 | 도 | 고도 피치 적분 최대 누적치 (최대 20도) |
| `ALT_ERR_LIMIT_SHUTTLE_M` | 10.0 | m | 셔틀 단계 고도 오차 제한 |
| `ALT_ERR_LIMIT_FLYHOME_M` | 5.0 | m | 귀환 단계 고도 오차 제한 |
| `BANK_FF_MAX_DEG` | -36.0 | 도 | 뱅크각 피드포워드 피치 보상 최대치 (기수 올림) |
| `ATTAIN_ALT_TIMEOUT_US` | 3000000 | μs (3초) | 상승 단계 타임아웃 |

### 2.3 헤딩 제어 (뱅크턴 vs 요 PI 전환)

| 상수명 | 값 | 단위 | 설명 |
|---|---|---|---|
| `HEADING_HYST_LOW_DEG` | 30.0 | 도 | 요(Yaw) PI 제어 모드 상한 (오차 ≤ 30° → 정밀 요 제어) |
| `HEADING_HYST_HIGH_DEG` | 45.0 | 도 | 뱅크턴 강제 모드 하한 (오차 ≥ 45° → 조화선회) |
| `HEADING_LATCH_ON_DEG` | 160.0 | 도 | 스마트 헤딩 래치 활성화 임계값 (150~170 권장) |
| `HEADING_LATCH_OFF_DEG` | 90.0 | 도 | 스마트 헤딩 래치 해제 임계값 |

### 2.4 최대 각도 제한

| 상수명 | 값 | 단위 | 설명 |
|---|---|---|---|
| `MAX_PITCH_SHUTTLE_DEG` | 15.0 | 도 | 셔틀 단계 최대 피치각 |
| `MAX_PITCH_FLYHOME_DEG` | 20.0 | 도 | 귀환 단계 최대 피치각 |
| `MAX_ROLL_DEG` | 75.0 | 도 | 최대 뱅크(롤) 각도 |

### 2.5 터치 판정 (CPA: Closest Point of Approach)

| 상수명 | 값 | 단위 | 설명 |
|---|---|---|---|
| `GPS_RESCUE_TOUCH_ACTIVATION_CM` | 2000.0 | cm (20m) | CPA 거리 미분 감시 시작 거리 |
| `GPS_RESCUE_TOUCH_PROXIMITY_CM` | 500.0 | cm (5m) | 즉시 터치 판정 근접 거리 (폴백) |

### 2.6 뱅크 보상

| 상수명 | 값 | 단위 | 설명 |
|---|---|---|---|
| `BANK_BOOST_FLYHOME` | 4.0 | - | 귀환/상승 단계에서 뱅크각에 비례한 쓰로틀 보상 계수 |

---

## 3. 데이터 구조체

### 3.1 `rescueFailureState_e`

레스큐 실패/중단 원인을 나타내는 열거형.

| 값 | 설명 |
|---|---|
| `RESCUE_HEALTHY` | 정상 |
| `RESCUE_FLYAWAY` | 귀환 불가 (30초간 속도 부족) |
| `RESCUE_GPSLOST` | GPS 신호 손실 |
| `RESCUE_LOWSATS` | 위성 수 부족 (10초 연속) |
| `RESCUE_CRASH_FLIP_DETECTED` | 충돌/전복 감지 |
| `RESCUE_STALLED` | 실속 |
| `RESCUE_TOO_CLOSE` | 너무 가까움 |
| `RESCUE_NO_HOME_POINT` | 홈 포인트 미설정 |

### 3.2 `rescueIntent_s` (의도/목표값 구조체)

| 필드 | 타입 | 초기값 | 설명 |
|---|---|---|---|
| `maxAltitudeCm` | float | 0 (또는 max 누적) | 비행 중 최대 도달 고도 (누적 갱신) |
| `returnAltitudeCm` | float | 설정값 | 귀환 목표 고도 (cm) |
| `targetAltitudeCm` | float | 현재고도 | 현재 Phase 목표 고도 (cm) |
| `targetLandingAltitudeCm` | float | `landingAlt * 100` | 최종 착륙 목표 고도 = `100.0f * landingAlt` |
| `targetVelocityCmS` | float | `gpsRescueConfig()->groundSpeedCmS` | 목표 지상 속도 (cm/s) |
| `descentDistanceM` | float | `gpsRescueConfig()->descentDistanceM` | 하강 시작 거리 (m) |
| `secondsFailing` | int8_t | 0 | 연속 실패 카운터 (초) |
| `yawAttenuator` | float | 0 (→ INIT에서 1.0) | 요 감쇠기 (일반모드=1.0, 무한셔틀=1.0) |
| `disarmThreshold` | float | `gpsRescueConfig()->disarmThreshold * 0.1` | 충격 감지 임계값 |
| `distanceToTargetCm` | uint32_t | 계산값 | 현재 타겟까지의 거리 (cm, OSD용) |
| `directionToTargetCd` | int32_t | 계산값 | 타겟 방향 (0.01도 단위, OSD용) |

### 3.3 `rescueSensorData_s` (센서 입력값 구조체)

| 필드 | 타입 | 설명 |
|---|---|---|
| `currentAltitudeCm` | float | 현재 고도 (cm), `getAltitude()` |
| `distanceToHomeCm` | float | 홈까지 거리 (cm) |
| `distanceToHomeM` | float | 홈까지 거리 (m) |
| `groundSpeedCmS` | uint16_t | 현재 지상 속도 (cm/s), 업샘플링 필터 통과 |
| `directionToHome` | int16_t | 홈 방향 (0.1도 단위) |
| `accMagnitude` | float | 가속도 크기 (랜딩 충격 감지용) |
| `healthy` | bool | GPS 건강 상태 |
| `errorAngle` | float | 헤딩 에러 각도 (스마트 래치 적용) |
| `absErrorAngle` | float | 헤딩 에러 절댓값 |
| `gpsDataIntervalSeconds` | float | GPS 데이터 갱신 간격 (초) |
| `altitudeDataIntervalSeconds` | float | 고도 센서 데이터 갱신 간격 (초) |
| `gpsRescueTaskIntervalSeconds` | float | 레스큐 태스크 간격 (초), `HZ_TO_INTERVAL(TASK_GPS_RESCUE_RATE_HZ)` |
| `velocityToHomeCmS` | float | 홈 방향 속도 성분 (cm/s) = 거리 미분 |
| `imuYawCogGain` | float | IMU Yaw → COG 변환 가변 게인 (바람 편류 보정) |

### 3.4 `rescueState_s` (통합 상태 구조체)

| 필드 | 타입 | 설명 |
|---|---|---|
| `phase` | `rescuePhase_e` | 현재 단계 |
| `failure` | `rescueFailureState_e` | 실패 원인 |
| `sensor` | `rescueSensorData_s` | 센서 데이터 |
| `intent` | `rescueIntent_s` | 목표 데이터 |
| `isAvailable` | bool | 레스큐 사용 가능 여부 |

---

## 4. Profile 3 PID 파라미터 매핑 테이블

`updateRescueParams()` 함수에서 PID Profile 3의 각 축 PID 값을 레스큐 전용 변수로 변환하여 사용한다.  
이를 통해 사용자는 CLI/GUI에서 Profile 3를 튜닝하여 레스큐 응답성을 실시간 조정할 수 있다.

### 4.1 Roll 축 매핑

| PID Profile 3 필드 | 매핑 변수 | 변환식 | 설명 |
|---|---|---|---|
| `pid[PID_ROLL].P` | `bankGain` | `P * 0.01f` | 헤딩 에러 → 뱅크각 게인 |
| `pid[PID_ROLL].I` | `bankPitchGain` | `I / 100.0f` | 뱅크 시 피치 보정 게인 |
| `pid[PID_ROLL].D` | `bankYawGain` | `constrainf(D / 50.0f, 0.0f, 5.0f)` | 조화선회 Yaw 게인 |
| `pid[PID_ROLL].F` | `shuttleDistance` | `F` (그대로) | 셔틀 A-B 포인트 간 거리 (m) |
| `d_min[PID_ROLL]` | `shuttleCount` | `d_min` (그대로) | 셔틀 왕복 횟수 |

### 4.2 Pitch 축 매핑

| PID Profile 3 필드 | 매핑 변수 | 변환식 | 설명 |
|---|---|---|---|
| `pid[PID_PITCH].P` | `ascendPitch` | `convertPidToPitchDeg(P)` | 상승 단계 피치각 (0~250 → -75~+60도) |
| `pid[PID_PITCH].I` | `midPitch` | `convertPidToPitchDeg(I)` | 기체 수평 기준 피치 (트림) |
| `pid[PID_PITCH].D` | `landingPitch` | `convertPidToPitchDeg(D)` | 착륙 단계 피치각 |
| `pid[PID_PITCH].F` | `altHoldGain` | `constrainf(F / 10.0f, 0.0f, 25.0f)` | 고도 유지 피치 게인 |
| `d_min[PID_PITCH]` | `landingSpeed` | `d_min * 100.0f` | 최종 착륙 시도 속도 (1입력=1m/s=100cm/s) |

### 4.3 Yaw 축 매핑

| PID Profile 3 필드 | 매핑 변수 | 변환식 | 설명 |
|---|---|---|---|
| `pid[PID_YAW].P` | `sbankGain` | `P * 0.01f` | 셔틀 전용 뱅크각 게인 |
| `pid[PID_YAW].I` | `sbankPitchGain` | `I / 100.0f` | 셔틀 전용 피치 보정 게인 |
| `pid[PID_YAW].D` | `descentAlt` | `D` (그대로) | 셔틀하강 종료 및 홈하강 시작 고도 (m) |
| `pid[PID_YAW].F` | `headingYawGain` | `constrainf(F / 100.0f, 0.0f, 2.5f)` | 헤딩 추적 요 게인 |
| `d_min[PID_YAW]` | `landingAlt` | `d_min` (그대로) | 최종 착륙 시도 고도 (m) |

### 4.4 `convertPidToPitchDeg()` 변환표

```
입력 PID값 (0~250) → 출력 피치각도
  0    → +100° → +60° (constrain 적용)
  50   →  +50°
  80   →  +20°
  90   →  +10°
 100   →    0° (수평)
 110   →  -10° (기수 약간 올림)
 150   →  -50°
 200   → -100° → -75° (constrain 적용)
 250   → -150° → -75° (constrain 적용)

공식: pitchDeg = constrainf(100 - pidValue, -75, 60)
양수(+) = 하강 (기수 내림)
음수(-) = 상승 (기수 올림)
```

---

## 5. 전역 변수 목록

### 5.1 제어 출력 변수

| 변수명 | 타입 | 초기값 | 설명 | 사용 Phase |
|---|---|---|---|---|
| `rescueThrottle` | `static float` | - | 최종 계산된 쓰로틀 명령 | 모든 활성 Phase |
| `rescueYaw` | `static float` | 0 | 최종 요(yaw) 명령 (도/초) | 모든 활성 Phase (+ LPF 적용) |
| `lastRescueYaw` | `static float` | 0 | 이전 프레임 요값 (LPF 재귀용) | 모든 활성 Phase |
| `gpsRescueAngle[AI_ROLL]` | `float` (전역) | 0 | 레스큐 롤 명령 (0.01도) | 모든 활성 Phase |
| `gpsRescueAngle[AI_PITCH]` | `float` (전역) | 0 | 레스큐 피치 명령 (0.01도) | 모든 활성 Phase |
| `magForceDisable` | `bool` (전역) | false | 자력계 강제 비활성화 플래그 | FLYAWAY 대응 |

### 5.2 PID 적분기 변수

| 변수명 | 타입 | 초기값 | 설명 | 초기화 시점 |
|---|---|---|---|---|
| `velocityIterm` | `static float` | 0.0 | 속도 PI 제어 적분항 | INIT, Phase 전환 |
| `altitudePitchIterm` | `static float` | 0.0 | 고도 피치 제어 적분항 | INIT, Phase 전환 |
| `yawHeadingIterm` | `static float` | 0.0 | 요 헤딩 PI 제어 적분항 | INIT, Phase 전환, 터치 시 리셋 |
| `prevDistanceToHomeCm` | `static float` | 0.0 | 이전 프레임 홈 거리 (속도 미분용) | Phase 전환 |
| `prevAltM` | `static float` | 0.0 | 이전 고도 (상승률 계산용) | Phase 전환 시 `prevAltMInitialized = false` |
| `prevAltMInitialized` | `static bool` | false | `prevAltM` 초기화 여부 | Phase 전환 시 false |

### 5.3 피치 스무딩

| 변수명 | 타입 | 초기값 | 설명 |
|---|---|---|---|
| `smoothedPitchNeedsReset` | `static bool` | false | Phase 전환 시 피치 LPF 강제 초기화 플래그 |
| `smoothedPitch` | `static float` (로컬) | 0 | 2.0Hz LPF 적용된 피치 명령 |
| `smoothedPitchInitialized` | `static bool` (로컬) | false | 스무딩 초기화 여부 |

### 5.4 GPS 좌표 포인트

| 변수명 | 타입 | 초기값 | 설명 |
|---|---|---|---|
| `rescuePointA` | `gpsLocation_t` | - | 이륙 방향 하강거리 위치에 생성되는 A포인트 (디스암까지 유효) |
| `shuttlePointA` | `gpsLocation_t` | - | 셔틀 단계 시작점 (하강고도 홀수면 현재위치, 짝수면 rescuePointA) |
| `shuttlePointB` | `gpsLocation_t` | - | 셔틀 반환점 (A→Home 방향으로 shuttleDistance만큼 떨어진 지점) |
| `rescuePointC` | `gpsLocation_t` | - | 무한 셔틀 발동 위치 (C-B를 왕복) |
| `currentVCLat` | `static int32_t` | 0 | OSD 표시용 현재 타겟 위도 (1e-7도) |
| `currentVCLon` | `static int32_t` | 0 | OSD 표시용 현재 타겟 경도 (1e-7도) |

### 5.5 셔틀 상태 플래그

| 변수명 | 타입 | 초기값 | 설명 |
|---|---|---|---|
| `takeoffVectorCaptured` | `static bool` | false | 이륙 방향 벡터 캡쳐 완료 여부 (짝수 descentAlt 전용) |
| `aPointValid` | `static bool` | false | `rescuePointA`가 정상 설정되었는지 여부 |
| `shuttleTargetB` | `static bool` | false | 현재 목적지 = B(true) / A(false) |
| `currentShuttleTrips` | `static float` | 0.0 | 현재까지 완료한 왕복 횟수 |
| `shuttleInfinite` | `static bool` | false | 무한 셔틀 모드 (AUX 스위치 연동) |
| `shuttleHeadingToA` | `static bool` | false | B 통과 후 A로 복귀 중인지 여부 (하강 단계 탈출 조건) |
| `descentAltReached` | `static bool` | false | 하강 고도 도달 여부 래치 (전 구간 감지용) |
| `turnDirectionSign` | `static int8_t` | 0 | 선회 방향 고정: 0=자유, 1=우회전, -1=좌회전 (스마트 헤딩 래치) |
| `isDescentFalling` | `static bool` | false | 급하강 모드 활성화 (DESCENT 진입 시 고도 높을 때) |
| `descentFallAligned` | `static bool` | false | 급하강 전 헤딩 정렬 완료 여부 |

### 5.6 CPA (터치 판정) 변수

| 변수명 | 타입 | 초기값 | 설명 |
|---|---|---|---|
| `cpaDistToTargetCm` | `static float` | -1.0 | 이전 프레임 타겟 거리 (cm). -1 = 미초기화 |
| `cpaWasClosing` | `static bool` | false | 이전 프레임에서 거리가 감소 중이었는지 |
| `abVecLat` | `static float` | 0.0 | A→B 단위 벡터 위도 성분 (CPA Along-Track 기준) |
| `abVecLon` | `static float` | 0.0 | A→B 단위 벡터 경도 성분 |

### 5.7 기타 전역/파일스코프 변수

| 변수명 | 타입 | 초기값 | 설명 |
|---|---|---|---|
| `rescueState` | `rescueState_s` (전역) | - | 통합 상태 구조체 |
| `attainAltStartTime` | `static timeUs_t` | 0 | 상승 단계 시작 시간 (타임아웃 측정) |
| `newGPSData` | `static bool` | false | 새로운 GPS 데이터 도착 플래그 (1회성) |
| `throttleDLpf` | `pt2Filter_t` (static) | - | 쓰로틀 명령 2차 LPF |
| `velocityDLpf` | `pt1Filter_t` (static) | - | 속도 오차 1차 LPF |
| `velocityUpsampleLpf` | `pt3Filter_t` (static) | - | GPS 속도 업샘플링 3차 LPF |

---

## 6. OSD / 외부 인터페이스 함수

레스큐 내부 변수를 외부(OSD, LED, Blackbox)에서 읽을 수 있도록 제공되는 Getter 함수.

| 함수 | 반환 타입 | 설명 | 내부 참조 변수 |
|---|---|---|---|
| `gpsRescueGetPhase()` | `rescuePhase_e` | 현재 레스큐 단계 반환 | `rescueState.phase` |
| `gpsRescueGetThrottle()` | `float` | 정규화된 쓰로틀 명령 (0.0~1.0) | `rescueThrottle` (scale + constrain) |
| `gpsRescueGetYawRate()` | `float` | 현재 요 명령 (도/초) | `rescueYaw` |
| `gpsRescueGetImuYawCogGain()` | `float` | IMU→COG 변환 게인 | `rescueState.sensor.imuYawCogGain` |
| `gpsRescueGetTargetAltitude()` | `float` | 현재 Phase 목표 고도 (cm) | `rescueState.intent.targetAltitudeCm` |
| `gpsRescueGetTargetVelocity()` | `float` | 목표 속도 (cm/s) | `rescueState.intent.targetVelocityCmS` |
| `gpsRescueGetTargetLat()` | `int32_t` | 현재 타겟 위도 | `currentVCLat` |
| `gpsRescueGetTargetLon()` | `int32_t` | 현재 타겟 경도 | `currentVCLon` |
| `gpsRescueGetTargetDistance()` | `uint32_t` | 타겟까지 거리 (cm) | `rescueState.intent.distanceToTargetCm` |
| `gpsRescueGetTargetDirection()` | `int32_t` | 타겟 방향 (0.01도) | `rescueState.intent.directionToTargetCd` |
| `gpsRescueGetTargetLabel()` | `char` | OSD용 타겟 레이블 ('A'/'B'/'H') | Phase + `aPointValid` + `shuttleTargetB` |
| `gpsRescueGetCurrentShuttleTrips()` | `uint16_t` | 현재 셔틀 왕복 횟수 | `currentShuttleTrips` |
| `gpsRescueIsConfigured()` | `bool` | 레스큐 설정 여부 | failsafe procedure 또는 BOXGPSRESCUE 존재 |
| `gpsRescueIsAvailable()` | `bool` | 레스큐 사용 가능 여부 | `rescueState.isAvailable` |
| `gpsRescueIsDisabled()` | `bool` | GPS 홈 미설정 여부 | `!STATE(GPS_FIX_HOME)` |
| `gpsRescueDisableMag()` | `bool` (USE_MAG) | 자력계 비활성화 필요 여부 | Phase + `useMag` + `magForceDisable` |

---

## 7. 설정 파라미터 (gpsRescueConfig)

`gpsRescueConfig()`를 통해 읽어오는 CLI/GUI 설정값들 (변수명은 `settings.c`에서 정의).

| 설정 키 | 타입 | 설명 | 사용처 |
|---|---|---|---|
| `throttleHover` | uint16_t | 호버 쓰로틀 값 (= 크루즈 쓰로틀) | `calculateVelocityThrottle()` |
| `throttleMax` | uint16_t | 최대 쓰로틀 제한 | `calculateVelocityThrottle()` |
| `throttleMin` | uint16_t | 최소 쓰로틀 (하강/랜딩) | `calculateVelocityThrottle()`, 핸들러 |
| `groundSpeedCmS` | uint16_t | 목표 지상 속도 (cm/s) | 의도값, 속도 PI, 바람보정 |
| `descentDistanceM` | uint16_t | 하강 및 A포인트 거리 (m) | Phase 전환 조건 |
| `initialClimbM` | uint16_t | 초기 상승 여유 고도 (m) | 고도 모드 계산 |
| `returnAltitudeM` | uint16_t | 고정 귀환 고도 (m) | `ALT_MODE_FIXED` 사용 시 |
| `altitudeMode` | uint8_t | 고도 모드 (0=FIXED, 1=CURRENT, 2=MAX) | `setReturnAltitude()` |
| `yawP` | uint8_t | 요 PI 제어 P게인 | `handleFlyHomePhase()`, `handleShuttleProgress()` |
| `pitchCutoffHz` | uint8_t | 속도 LPF 차단 주파수 (/100 Hz) | 레스큐 Init |
| `useMag` | uint8_t | 자력계 사용 여부 | FLYAWAY fallback |
| `maxRescueAngle` | uint8_t | 최대 레스큐 피치각 | 급하강 단계 |
| `disarmThreshold` | uint8_t | 충격 감지 임계값 (x0.1 G) | 착륙 충격 감지 |

---

## 8. 참고: 주요 함수 흐름

### `gpsRescueUpdate()` (메인 루프, 1초에 약 10~50Hz 호출)

```
gpsRescueUpdate()
  ├─ 1) GPS_RESCUE_MODE 체크 → IDLE/START 전환
  ├─ 2) sensorUpdate()       → 센서 데이터 갱신
  ├─ 3) Phase 전환 감지       → 적분기/플래그 리셋
  ├─ 4) Phase 처리 switch      → Phase별 핸들러 호출
  ├─ 5) performSanityChecks() → 안전 진단 (GPS 손실, Flyaway 감지)
  └─ 6) rescueAttainPosition()→ 각 Phase 핸들러 디스패치 + Yaw LPF
       ├── IDLE          : 피치/롤=0, 쓰로틀=rcCommand
       ├── ATTAIN_ALT    : 상승 피치 고정, 쓰로틀=속도PI
       ├── FLY_HOME      : 헤딩→뱅크턴, 고도유지 피치, 속도PI
       ├── SHUTTLE/SHUTTLE_INFINITE   : A↔B 셔틀진행
       ├── SHUTTLE_DESCENT : 셔틀진행 + 고도 낮춤
       ├── DESCENT       : 홈 방향 하강 (급하강/정상하강)
       ├── LANDING       : 피치고정, 쓰로틀↓ → 3초 후 모터정지
       ├── DO_NOTHING    : 피치고정, 쓰로틀최소
       └── 모든 활성 Phase 공통: Yaw = lastRescueYaw * 0.7 + rescueYaw * 0.3
```

---

> **작성일**: 2026-06-15  
> **대상 커밋**: `8457a0f32954fbd3695fa4d84afb425e2e5e2066`  
> **문서 버전**: v1.0