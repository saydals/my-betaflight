# Betaflight 서보/모드 코드 심층 분석 보고서

> **분석 일자:** 2026-07-01
> **분석 대상:** /home/betaflight/betaflight/src/main/
> **목적:** Servo Trim 모드 구현에 앞선 기존 코드의 부작용 및 참조 패턴 분석

---

## 목차

1. [명령어 1: 서보 출력 체인의 부호 전파 완전 분석](#1-서보-출력-체인의-부호-전파-완전-분석)
2. [명령어 2: HEADFREE_MODE 플래그 잔존 코드 전수 조사](#2-headfree_mode-플래그-잔존-코드-전수-조사)
3. [명령어 3: getBoxIdState() 함수의 전방 호출 그래프 분석](#3-getboxidstate-함수의-전방-호출-그래프-분석)
4. [명령어 4: forwardFromChannel과 servo.middle의 상호작용](#4-forwardfromchannel과-servomiddle의-상호작용)
5. [명령어 5: Board Align 모드의 부저 및 중복 처리 패턴](#5-board-align-모드의-부저-및-중복-처리-패턴)
6. [명령어 6: currentServoMixer 정적 변수의 접근 가능성](#6-currentservomixer-정적-변수의-접근-가능성)


---

## 1. 서보 출력 체인의 부호 전파 완전 분석

### 1.1. 목적

`servoParams(i)->rate`(음수 설정 가능)가 서보 출력 배열(`servo[]`)에 최종 적용되는 정확한 시점과, 이 부호가 `servo.middle` 증감 방향에도 영향을 줘야 하는지 확인합니다.

### 1.2. 탐색 대상

- **파일:** `src/main/flight/servos.c`
- **핵심 함수:** `servoMixer()` -> `servoTable()` 공통 후처리 루틴

### 1.3. 실제 코드

#### 1.3.1. 공통 후처리 루틴 (servos.c:878-884)

```c
// Common post-processing: apply rate and add middle/forward offset
static void servoTable(void)
{
    // ... mixer-specific code ...

    // 881-884: 모든 서보에 공통 적용
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo[i] = ((int32_t)servoParams(i)->rate * servo[i]) / 100L;  // [1] rate *
        servo[i] += determineServoMiddleOrForwardFromChannel(i);        // [2] middle +
    }
}
```

#### 1.3.2. determineServoMiddleOrForwardFromChannel() (servos.c:466-475)

```c
int16_t determineServoMiddleOrForwardFromChannel(servoIndex_e servoIndex)
{
    const uint8_t channelToForwardFrom = servoParams(servoIndex)->forwardFromChannel;

    if (channelToForwardFrom != CHANNEL_FORWARDING_DISABLED
        && channelToForwardFrom < rxRuntimeState.channelCount) {
        // forwarding 활성: middle 무시, 원시 채널값 반환
        return scaleRangef(..., min, max);
    }

    return servoParams(servoIndex)->middle;  // forwarding 비활성: middle 반환
}
```

### 1.4. 데이터 흐름도

```
servoMixer() / servoTable()
  |
  +-- (1) mixer-specific: servo[i] = input x rate / 100
  |
  +-- (2) 공통 후처리 (servos.c:881-884)
          |
          +-- 882: servo[i] = rate x servo[i] / 100  <- rate 부호 곱해짐
          |
          +-- 883: servo[i] += determineServoMiddleOrForwardFromChannel(i)
          |         +-- forwarding 활성 -> RC 원시값 (middle 무시)
          |         +-- forwarding 비활성 -> servo.middle (순수 양수)
          |
          +-- 937: scaleServoOutput(i)
                    +-- forwarding 활성 -> constrain만
                    +-- forwarding 비활성 -> deviation = servo[i] - middle
```

### 1.5. 분석 결과

| 연산 순서 | 코드 | rate 음수 시 영향 |
|---|---|---|
| (1) rate 곱셈 | servo[i] = rate x servo[i] / 100 | servo[i] 값의 **부호가 반전됨** |
| (2) middle 가산 | servo[i] += middle | **middle은 항상 양수**로 가산, rate 부호 무시 |
| (3) scaleServoOutput | deviation = servo[i] - middle | deviation 값도 rate 반영 후 middle 차감 |

**경고 결론:**

rate가 음수로 설정된 서보에서:
1. servo[i]의 부호가 먼저 반전됨 (882번 줄)
2. middle이 **부호 반전 없이** 항상 같은 방향(양수)으로 더해짐 (883번 줄)
3. 결과적으로 rate 음수 시 중립점(middle) 이동 방향과 실제 서보 출력 방향이 **불일치**

**새 Servo Trim 모드 구현 시, rate 부호가 음수이면 middle 증감 방향을 반전시키는 추가 로직이 필요합니다.**


---

## 2. HEADFREE_MODE 플래그 잔존 코드 전수 조사

### 2.1. 목적

`core.c`에서 제거한 `HEADFREE_MODE` 설정 로직 외에, 이 플래그를 읽거나 쓰는 모든 잔존 코드를 찾아 Servo Trim 모드와 충돌 여부를 확인합니다.

### 2.2. 정의부

**`src/main/fc/runtime_config.h:87`**
```c
HEADFREE_MODE = (1 << 6),
```

**`src/main/fc/runtime_config.h:107`** (BOXID -> FlightMode 매핑)
```c
[BOXHEADFREE] = LOG2(HEADFREE_MODE),
```

### 2.3. HEADFREE_MODE 설정/해제 (제거 대상)

**`src/main/fc/core.c:1019-1025`**
```c
if (IS_RC_MODE_ACTIVE(BOXHEADFREE) && !FLIGHT_MODE(GPS_RESCUE_MODE)) {
    if (!FLIGHT_MODE(HEADFREE_MODE)) {
        ENABLE_FLIGHT_MODE(HEADFREE_MODE);
    }
} else {
    DISABLE_FLIGHT_MODE(HEADFREE_MODE);
}
```

**`src/main/fc/core.c:1041`** - 비행기 모드 강제 비활성화
```c
if (mixerConfig()->mixerMode == MIXER_FLYING_WING || mixerConfig()->mixerMode == MIXER_AIRPLANE) {
    DISABLE_FLIGHT_MODE(HEADFREE_MODE);
}
```

### 2.4. HEADFREE_MODE 읽기 (잔존 참조, 총 9곳)

| # | 파일 | 라인 | 용도 |
|---|---|---|---|
| 1 | fc/rc.c | 675 | FPV Cam Angle Mix 조건 가드 |
| 2 | fc/rc.c | 748 | rcCommand 헤드프리 쿼터니언 변환 |
| 3 | flight/imu.c | 309 | 헤드프리 오일러 각도 계산 분기 |
| 4 | osd/osd_elements.c | 1072 | OSD 모드 텍스트 "HEAD" 표시 |
| 5 | osd/osd_warnings.c | 252 | OSD 경고 "HEADFREE" 블링킹 표시 |
| 6 | io/ledstrip.c | 491 | LED_MODE_HEADFREE 패턴 매핑 |
| 7 | telemetry/smartport.c | 788 | FrSky +4000 값 전송 |
| 8 | telemetry/ibus_shared.c | 263 | IBUS flightMode=4 |
| 9 | telemetry/ltm.c | 173 | LTM lt_flightmode=4 |

### 2.5. 분석 결과

HEADFREE_MODE 설정 로직 제거 시:
- OSD, LED, 텔레메트리의 `FLIGHT_MODE(HEADFREE_MODE)`는 **절대 true가 될 수 없음**
- 해당 코드는 자연스럽게 dead code가 되어 아무 영향도 미치지 않음
- `runtime_config.h`의 `HEADFREE_MODE` 정의와 BOXID 매핑은 컴파일 에러 방지를 위해 유지 필요
- Servo Trim이 HEADFREE_MODE와 **다른 BOX ID**(예: BOXUSER3/4)를 사용한다면 직접 충돌 없음

**위험도: 낮음**


---

## 3. getBoxIdState() 함수의 전방 호출 그래프 분석

### 3.1. 목적

`getBoxIdState(BOXHEADFREE)`에 특별 케이스를 추가했을 때, OSD, LED, VTX, MSP 등 다른 모듈이 이 함수의 반환값을 어떻게 사용하는지 파악하여 부작용이 없는지 확인합니다.

### 3.2. 함수 정의 (msp_box.c:360-374)

```c
bool getBoxIdState(boxId_e boxid)
{
    const uint8_t boxIdToFlightModeMap[] = BOXID_TO_FLIGHT_MODE_MAP_INITIALIZER;

    if (boxid == BOXARM) {
        return ARMING_FLAG(ARMED);
    } else if (boxid <= BOXID_FLIGHTMODE_LAST) {
        return FLIGHT_MODE(1 << boxIdToFlightModeMap[boxid]);  // flightModeFlags 검사
    } else {
        return IS_RC_MODE_ACTIVE(boxid);  // rcModeActivationMask 검사
    }
}
```

**BOXHEADFREE 경로:**
```
getBoxIdState(BOXHEADFREE)
-> boxid <= BOXID_FLIGHTMODE_LAST
-> FLIGHT_MODE(1 << LOG2(HEADFREE_MODE))
-> FLIGHT_MODE(HEADFREE_MODE)
-> flightModeFlags & (1 << 6)
```

### 3.3. 호출 그래프

```
getBoxIdState(boxId_e)
  |
  +-- msp_box.c:387  <- packFlightModeFlags()
  |     |
  |     +-- MSP 프로토콜 응답 직렬화 (모든 활성 BOX 상태 비트맵)
  |         -> GCS/Configurator가 BOX 상태 표시
  |
  +-- io/piniobox.c:69  <- pinioBoxUpdate()
        |
        +-- PINIO 하드웨어 GPIO 출력
            -> 외부 장치 제어 (LED, 릴레이 등)
```

### 3.4. 분석 결과

OSD, LED, 텔레메트리 모듈은 `getBoxIdState()`를 직접 호출하지 않고 각자의 방식으로 `FLIGHT_MODE(HEADFREE_MODE)` 또는 `IS_RC_MODE_ACTIVE(BOXHEADFREE)`를 직접 사용합니다.

`getBoxIdState()`에 BOXHEADFREE 특별 케이스를 추가할 필요가 없습니다. HEADFREE_MODE 설정 로직 자체가 제거되면 모든 참조가 자연히 dead code가 됩니다.

**위험도: 낮음**


---

## 4. forwardFromChannel과 servo.middle의 상호작용

### 4.1. 목적

`forwardFromChannel`이 활성화된 서보에서 `servoParams()->middle`이 실제 스케일링/출력 과정에서 완전히 무시되는지 확인합니다.

### 4.2. 실제 코드

#### 4.2.1. determineServoMiddleOrForwardFromChannel() (servos.c:466-475)

```c
int16_t determineServoMiddleOrForwardFromChannel(servoIndex_e servoIndex)
{
    const uint8_t channelToForwardFrom = servoParams(servoIndex)->forwardFromChannel;

    if (channelToForwardFrom != CHANNEL_FORWARDING_DISABLED) {
        // middle을 완전히 무시하고 원시 채널값 반환
        return scaleRangef(rcData[ch], PWM_RANGE_MIN, PWM_RANGE_MAX, min, max);
    }

    return servoParams(servoIndex)->middle;  // forwarding 비활성 시에만 middle 사용
}
```

#### 4.2.2. scaleServoOutput() (servos.c:500-505) - forwarding 분기

```c
const uint8_t channelToForwardFrom = servoParams(i)->forwardFromChannel;
if (channelToForwardFrom != CHANNEL_FORWARDING_DISABLED) {
    servo[i] = constrain(servo[i], min, max);  // 단순 clamp만, middle 스킵
    return;
}
// (이하 forwarding 비활성 시에만 middle 기반 스케일링)
```

### 4.3. 처리 비교

| 단계 | forwarding 비활성 | forwarding 활성 |
|---|---|---|
| 883 middle 결정 | `return servo.middle` | `return scaleRangef(RC값, min, max)` |
| scaleServoOutput | `deviation = servo[i] - middle` 후 비례 스케일링 | **constrain만 수행, middle 스킵** |
| middle 참조 여부 | 두 곳에서 참조 | **완전히 무시됨** |

### 4.4. 분석 결과

forwardFromChannel이 활성화된 서보에서 servo.middle은:
1. `determineServoMiddleOrForwardFromChannel()`에서 **무시됨** (RC 원시값 반환)
2. `scaleServoOutput()`에서 **스킵됨** (constrain만 수행)
3. 전체 출력 체인에서 **단 한 번도 참조되지 않음**

**결론: forwarding 활성 서보는 trim 대상에서 제외해도 완전히 안전합니다.**

---

## 5. Board Align 모드의 부저 및 중복 처리 패턴

### 5.1. 목적

기존 rc.c의 Board Align 블록에서 beeper 호출 패턴과 중복 축 처리를 분석하여 새 Servo Trim 코드의 참고 자료로 삼습니다.

### 5.2. Board Align 블록 (rc.c:766-848)

```c
{
    static bool boardAlignModeActive = false;
    static float boardAlignBaseline[3];
    static bool latchHi[3];
    static bool latchLo[3];

    if (IS_RC_MODE_ACTIVE(BOXUSER2) && !ANGLE && !HORIZON) {
        if (!boardAlignModeActive) {
            boardAlignModeActive = true;
            boardAlignBaseline[ROLL] = rcCommand[ROLL];
            boardAlignBaseline[PITCH] = 0;
            boardAlignBaseline[YAW] = 0;
            for (int i = 0; i < 3; i++) {
                latchHi[i] = latchLo[i] = false;
            }
        }

        // rcCommand 고정
        rcCommand[ROLL] = boardAlignBaseline[ROLL];
        rcCommand[PITCH] = 0;
        rcCommand[YAW] = 0;

        bool changed = false;
        for (int i = 0; i < 3; i++) {
            float raw = rcData[axes[i]];
            int32_t oldVal = *alignDeg[i];
            int32_t newVal = oldVal;

            // Positive increment (stick > 1750)
            if (!latchHi[i] && raw > 1750.0f) {
                newVal = constrain(oldVal + 1, -limits[i], limits[i]);
                latchHi[i] = true;
            } else if (latchHi[i] && raw < 1600.0f) {
                latchHi[i] = false;
            }

            // Negative decrement (stick < 1250)
            if (!latchLo[i] && raw < 1250.0f) {
                newVal = constrain(oldVal - 1, -limits[i], limits[i]);
                latchLo[i] = true;
            } else if (latchLo[i] && raw > 1400.0f) {
                latchLo[i] = false;
            }

            if (newVal != oldVal) {
                *alignDeg[i] = newVal;
                changed = true;
            }
        }

        // beeper는 루프 밖에서 단 1회 호출
        if (changed) {
            updateBoardAlignmentMatrix(boardAlignment());
            beeper(BEEPER_RX_SET);
        }
    } else {
        if (boardAlignModeActive) {
            boardAlignModeActive = false;
        }
    }
}
```

### 5.3. 분석 결과: 핵심 패턴 요약

#### 5.3.1. Beeper 호출 패턴

| 특성 | Board Align | Servo Trim 적용 여부 |
|---|---|---|
| 호출 위치 | for 루프 밖 (3축 처리 완료 후) | 동일 패턴 적용 가능 |
| 호출 조건 | changed 플래그가 true일 때 | 동일 패턴 적용 가능 |
| 호출 횟수 | **1회** (BEEPER_RX_SET) | 동일 패턴 적용 가능 |
| 축별 호출 여부 | 하지 않음 | 따라야 함 |

#### 5.3.2. rcCommand 고정 패턴

| 구성 요소 | Board Align 방식 |
|---|---|
| 고정 대상 | rcCommand[ROLL] = entry값, rcCommand[PITCH/YAW] = 0 |
| 적용 시점 | 매 루프마다 고정 (모드 활성 중 지속) |
| Servo Trim 참고 | Trim은 rcCommand가 아닌 servo.middle 직접 변경하므로 **그대로 사용 불가** |

#### 5.3.3. 래치 변수(Latch) 패턴

```c
static bool latchHi[3];  // 각 축별 high 방향 래치
static bool latchLo[3];  // 각 축별 low 방향 래치
```

| 스틱 위치 | latchHi 동작 | latchLo 동작 |
|---|---|---|
| raw > 1750 | latchHi[i]=true, 값 +1 | 유지 |
| raw < 1600 | latchHi[i]=false (해제) | 유지 |
| raw < 1250 | 유지 | latchLo[i]=true, 값 -1 |
| raw > 1400 | 유지 | latchLo[i]=false (해제) |

특징: 히스테리시스 150us (1600-1750, 1250-1400)로 채터링 방지

### 5.4. Servo Trim 적용 권장 패턴

```c
// Board Align 패턴 기반 권장 구조
{
    static bool servoTrimModeActive = false;
    static bool latchHi[MAX_SUPPORTED_SERVOS];
    static bool latchLo[MAX_SUPPORTED_SERVOS];

    if (IS_RC_MODE_ACTIVE(BOXUSER3) && 조건) {
        if (!servoTrimModeActive) {
            servoTrimModeActive = true;
            // 래치 초기화
        }

        // rcCommand는 건드리지 않음
        bool changed = false;
        for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            if (forwardFromChannel 활성) continue;  // forwarding 제외
            // 래치 패턴으로 servoParamsMutable(i)->middle 증감
            // rate 음수 시 부호 반전 고려
        }

        if (changed) {
            beeper(BEEPER_RX_SET);  // 1회만
        }
    } else {
        if (servoTrimModeActive) {
            servoTrimModeActive = false;
        }
    }
}
```

---

## 6. currentServoMixer 정적 변수의 접근 가능성

### 6.1. 목적

rc.c에서 currentServoMixer[] 배열과 servoRuleCount에 접근하기 위해 계획된 접근자 함수(getActiveServoRuleCount 등)의 구현 가능성을 검증합니다.

### 6.2. 변수 선언부 (servos.c:107-111)

```c
int16_t servo[MAX_SUPPORTED_SERVOS];          // 전역 (extern, servos.h:171)

static uint8_t servoRuleCount = 0;            // static -> 외부 접근 불가
static servoMixer_t currentServoMixer[MAX_SERVO_RULES];  // static -> 외부 접근 불가
static int useServo;                           // static -> 우회 함수 존재
```

### 6.3. 기존 접근자 함수 현황

| 함수 | servos.h 선언 | 반환값 |
|---|---|---|
| isMixerUsingServos() | 있음 | useServo (static) 반환 |
| getActiveServoRuleCount() | **없음** | - |
| getCurrentServoMixer() | **없음** | - |
| getServoMixer() | **없음** | - |

### 6.4. 이름 충돌 검사

| 함수명 | servos.c 내 존재? | servos.h 내 선언? | 충돌 위험 |
|---|---|---|---|
| getActiveServoRuleCount | 없음 | 없음 | **없음** |
| getCurrentServoMixer | 없음 | 없음 | **없음** |
| getServoMixer | 없음 | 없음 | **없음** |
| getServoRuleCount | 없음 | 없음 | **없음** |

### 6.5. 추천 접근자 함수 구현

**servos.c** (547번 줄, loadCustomServoMixer 함수 뒤에 추가):
```c
uint8_t getActiveServoRuleCount(void)
{
    return servoRuleCount;
}

const servoMixer_t *getCurrentServoMixer(void)
{
    return currentServoMixer;
}
```

**servos.h** (servoMixer 함수 선언 뒤에 추가):
```c
uint8_t getActiveServoRuleCount(void);
const servoMixer_t *getCurrentServoMixer(void);
```

### 6.6. 분석 결과

| 변수 | static? | 외부 접근 | 필요 조치 |
|---|---|---|---|
| servoRuleCount | 예 | 불가능 | **접근자 함수 필수** |
| currentServoMixer | 예 | 불가능 | **접근자 함수 필수** |
| useServo | 예 | isMixerUsingServos() 우회 가능 | 불필요 |
| servo[] | 아니오 | extern 직접 접근 가능 | 불필요 |


---

## 부록: 전체 파일 경로 요약

| 명령어 | 대상 파일 | 라인 범위 |
|---|---|---|
| 1. 부호 전파 | `src/main/flight/servos.c` | 466-520, 878-939 |
| 2. HEADFREE 잔존 | 다수 | - |
| 3. getBoxIdState | `src/main/msp/msp_box.c` | 360-393 |
| 4. forwardFromChannel | `src/main/flight/servos.c` | 466-520 |
| 5. Board Align | `src/main/fc/rc.c` | 766-848 |
| 6. currentServoMixer | `src/main/flight/servos.c` | 107-111, 532-567 |

## 부록: 상호 영향도 매트릭스

| 명령어 | 주요 발견 | 위험도 | 조치 필요 |
|---|---|---|---|
| **1. 부호 전파** | rate가 middle보다 먼저 적용, middle은 rate 부호 무시 | 높음 | Servo Trim에서 rate 음수 시 middle 증감 방향 반전 필요 |
| **2. HEADFREE 잔존** | OSD/LED/텔레메트리 6곳 참조, dead code화 | 낮음 | runtime_config.h 정의 유지, 나머지는 자연 dead code |
| **3. getBoxIdState** | BOXHEADFREE는 FLIGHT_MODE 경로, MSP/PINIO만 영향 | 낮음 | 특별 케이스 불필요 |
| **4. forwardFromChannel** | forwarding 활성 시 middle 완전 무시 확인 | 안전 | trim 대상에서 제외해도 안전 |
| **5. Board Align 패턴** | beeper 1회, 래치 축별 독립, rcCommand 고정 | 참고용 | 래치 패턴 재사용, rcCommand 고정 대신 middle 직접 변경 |
| **6. 정적 변수 접근** | currentServoMixer/servoRuleCount 모두 static | 필수 | 접근자 함수 구현 필수 |

---

*본 보고서는 코드 분석 전용으로 작성되었으며, 실제 코드 수정을 포함하지 않습니다.*
*분석 일자: 2026-07-01*
