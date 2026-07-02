# Servo Trim Mode 구현 계획 v5 (최종본 — 코딩 실행용 상세 버전)

> 이 문서는 실제 코드를 작성할 AI(낮은 추론 수준 가정)에게 그대로 전달하기 위한 문서입니다.
> 모든 코드는 완성본 하나만 제시하며, 모든 삽입/삭제 위치는 실제 저장소(`github.com/saydals/my-betaflight`)의 현재 코드를 직접 대조하여 검증된 것입니다.
> **각 섹션의 "찾을 텍스트"를 파일에서 정확히 검색해서 위치를 잡고, "결과" 코드로 치환/삽입하십시오. 임의로 위치를 재해석하지 마십시오.**

---

## 목적

조종기 트림 버튼은 입력값(rcData)의 중립만 바꿀 뿐, 실제 서보 물리 중립(PWM)은 바꾸지 않는다. 자이로를 쓰지 않는 비행기에서 서보 물리 중립을 조정하던 방식을, 자이로를 사용하는 기체에서도 구현한다. Board Align 모드(`BOXUSER2`)의 스틱 히스테리시스 입력 방식을 재사용하여, 에일러론/엘리베이터/러더 서보의 `servoParams()->middle` 값을 CLI 설정 가능한 스텝만큼 증감시킨다.

---

## 구현해야 할 기능 (요약)

1. **모드 박스**: `BOXHEADFREE`(기존 헤드프리 박스, permanentId=6, 고정익에서 원래 안 쓰던 기능)를 "SERVO TRIM"으로 재사용한다. 새 박스(BOXUSER3 등)는 쓰지 않는다 — 실측 결과 Configurator가 BOXUSER3/4를 인식하지 못했다.
2. **CLI 스텝 파라미터**: `servo_trim_step` (uint8, 1~20, 기본값 5).
3. **CLI 방향 파라미터 3종**: `trim_aileron`, `trim_elevator`, `trim_rudder` — 값 0(사용안함)/1(기본, 자동계산)/2(반전).
4. **모드 활성 시 입력 처리**: 쓰로틀 통과, 에일러론 활성 시점 값 유지, 엘리베이터/러더 0 고정.
5. **믹서 인식형 타겟 서보 결정**: `currentServoMixer[]`를 순회하여 `INPUT_STABILIZED_ROLL/PITCH/YAW` 또는 `INPUT_RC_ROLL/PITCH/YAW` 규칙을 찾아 해당 `targetChannel`의 `middle`을 조정.
6. **동일 축 내에서만 중복 조정 방지**: 한 축(예: ROLL)의 한 번의 스텝 판정 안에서, 같은 물리 서보를 가리키는 규칙이 여러 개 있어도 그 서보는 한 번만 조정된다. 이 중복 방지는 **축 단위로 초기화**되므로 축을 넘어선 중복(예: 플라잉윙에서 ROLL 규칙과 PITCH 규칙이 같은 플래퍼론 서보를 같이 가리키는 경우)은 방지 대상이 아니다 — 이 경우 두 축의 trim이 그 서보의 middle에 각각 누적 가산되는 것이 의도된 동작이다(엘리본 방식, "예상 문제점" 2번 참고). **코드 구현 시 `updated[]` 배열을 축 루프(`for (int i = 0; i < 3; i++)`) 밖으로 빼서 3축 전체에 공유시키면 안 된다** — 그러면 플라잉윙에서 엘리베이터 트림이 롤 트림에 막혀 동작하지 않게 된다.
7. **forwardFromChannel 서보 자동 제외**.
8. **트림 한계값**: 모드 진입 시점의 `middle`을 기준으로 ±100pwm.
9. **ANGLE/HORIZON 모드 가드, Board Align 모드와 상호배타**.

---

## 사전 확인 사항 (코드 작성 전에 반드시 확인)

아래 항목은 실제로 비행기 설정을 봐야 확정되는 내용이라, 코드를 작성하기 전에 CLI에서 `smix dump`, `servo` 명령 결과를 먼저 확인해야 한다.

> **Bird Flap(BOXUSER1)과의 충돌 가능성**: Bird Flap은 믹서 규칙에서 `inputSource == INPUT_BIRD_FLAP`을 쓰고, 해당 서보 출력은 `servos.c` 871~879번 줄에서 믹서 룰 누적을 완전히 우회하여 통째로 덮어써진다(`servo[birdFlapServoIndex] = birdFlapOutput + ...`). 이번 트림 코드는 `INPUT_STABILIZED_ROLL`/`INPUT_RC_ROLL` 규칙만 찾아서 대상을 정하므로, **만약 롤(에일러론) 조종을 Bird Flap 서보 자체(날갯짓 좌우 차등)가 전담하는 구성이라면, 에일러론 트림이 그 서보를 아예 찾지 못해 작동하지 않는다.** `smix dump` 결과에서 `INPUT_STABILIZED_ROLL` 또는 `INPUT_RC_ROLL`을 쓰는 규칙이 Bird Flap 서보가 아닌 별도의 에일러론/플래퍼론 서보를 가리키는지 확인할 것. 별도 서보라면 문제없다.

---

## 기존 코드 수정·첨가점

### 1) `src/main/flight/servos.h`

**찾을 텍스트** (구조체 정의):
```c
typedef struct servoConfig_s {
    servoDevConfig_t dev;
    uint16_t servo_lowpass_freq;            // lowpass servo filter frequency selection; 1/1000ths of loop freq
    uint8_t tri_unarmed_servo;              // send tail servo correction pulses even when unarmed
    uint8_t channelForwardingStartChannel;
    uint8_t ready_to_arm_wiggle_hz;          // 0=OFF, 1~6=Hz
} servoConfig_t;
```

**결과** (필드 4개 추가):
```c
typedef struct servoConfig_s {
    servoDevConfig_t dev;
    uint16_t servo_lowpass_freq;            // lowpass servo filter frequency selection; 1/1000ths of loop freq
    uint8_t tri_unarmed_servo;              // send tail servo correction pulses even when unarmed
    uint8_t channelForwardingStartChannel;
    uint8_t ready_to_arm_wiggle_hz;          // 0=OFF, 1~6=Hz
    uint8_t servo_trim_step;                // Servo Trim mode step size in pwm, range 1-20
    uint8_t trim_aileron_dir;               // 0=off, 1=normal(자동계산), 2=reversed
    uint8_t trim_elevator_dir;              // 0=off, 1=normal(자동계산), 2=reversed
    uint8_t trim_rudder_dir;                // 0=off, 1=normal(자동계산), 2=reversed
} servoConfig_t;
```

**추가로 파일 어딘가(다른 `#define` 근처)에 상수 추가**:
```c
#define SERVO_TRIM_LIMIT_PWM 100        // 모드 진입 시점 middle 기준 ± 한계 (pwm)
```

**함수 선언 추가** (`void servoMixer(void);` 선언 근처, 없으면 파일 끝 `#endif` 이전 아무 곳):
```c
uint8_t getActiveServoRuleCount(void);
const servoMixer_t *getCurrentServoMixer(void);
```

---

### 2) `src/main/flight/servos.c`

**2-1) 기본값 설정**

**찾을 텍스트**:
```c
void pgResetFn_servoConfig(servoConfig_t *servoConfig)
```
이 함수 본문 안에 (다른 필드 기본값 대입 라인들 사이 아무 곳에) 아래 4줄을 추가:
```c
    servoConfig->servo_trim_step = 5;
    servoConfig->trim_aileron_dir = 1;
    servoConfig->trim_elevator_dir = 1;
    servoConfig->trim_rudder_dir = 1;
```

**2-2) 접근자 함수 추가 (반드시 선언뿐 아니라 아래 본문까지 그대로 작성할 것)**

파일 아무 곳(예: `loadCustomServoMixer()` 함수 뒤)에 다음 두 함수를 **본문 포함하여** 추가한다. 선언만 있고 본문이 없으면 링커 에러가 난다:

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

---

### 3) `src/main/cli/settings.h`

**찾을 텍스트** (enum 마지막 부분):
```c
#ifdef USE_RX_EXPRESSLRS
    TABLE_FREQ_DOMAIN,
    TABLE_SWITCH_MODE,
#endif
    LOOKUP_TABLE_COUNT
} lookupTableIndex_e;
```

**결과** (`TABLE_TRIM_DIRECTION`을 `LOOKUP_TABLE_COUNT` 바로 앞에 추가 — 반드시 맨 끝에 추가할 것. 중간에 끼워넣으면 그 뒤의 모든 TABLE_* 값이 밀려서 기존 파라미터가 전부 깨진다):
```c
#ifdef USE_RX_EXPRESSLRS
    TABLE_FREQ_DOMAIN,
    TABLE_SWITCH_MODE,
#endif
    TABLE_TRIM_DIRECTION,
    LOOKUP_TABLE_COUNT
} lookupTableIndex_e;
```

---

### 4) `src/main/cli/settings.c`

이 파일은 **3곳**을 수정해야 한다. 순서를 지킬 것.

**4-1) 문자열 배열 선언 추가**

**찾을 텍스트**:
```c
const char * const lookupTableOffOn[] = {
```
이 선언부 근처(파일 상단, 다른 `lookupTableXxx[]` 선언들이 모여있는 구역) 아무 곳에 새 배열을 추가:
```c
static const char * const lookupTableTrimDirection[] = {
    "OFF", "NORMAL", "REVERSED"
};
```

**4-2) `lookupTables[]` 배열에 항목 추가**

**찾을 텍스트** (배열의 마지막 부분):
```c
#ifdef USE_RX_EXPRESSLRS
    LOOKUP_TABLE_ENTRY(lookupTableFreqDomain),
    LOOKUP_TABLE_ENTRY(lookupTableSwitchMode),
#endif
};
```

**결과** (역시 맨 끝에 추가 — `settings.h`의 enum 순서와 정확히 같은 순서여야 함):
```c
#ifdef USE_RX_EXPRESSLRS
    LOOKUP_TABLE_ENTRY(lookupTableFreqDomain),
    LOOKUP_TABLE_ENTRY(lookupTableSwitchMode),
#endif
    LOOKUP_TABLE_ENTRY(lookupTableTrimDirection),
};
```

**4-3) CLI 파라미터 4개 추가**

**찾을 텍스트**:
```c
    { "ready_to_arm_wiggle_hz",    VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 6 }, PG_SERVO_CONFIG, offsetof(servoConfig_t, ready_to_arm_wiggle_hz) },
#endif
```

**결과**:
```c
    { "ready_to_arm_wiggle_hz",    VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 6 }, PG_SERVO_CONFIG, offsetof(servoConfig_t, ready_to_arm_wiggle_hz) },
    { "servo_trim_step",           VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 1, 20 }, PG_SERVO_CONFIG, offsetof(servoConfig_t, servo_trim_step) },
    { "trim_aileron",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_TRIM_DIRECTION }, PG_SERVO_CONFIG, offsetof(servoConfig_t, trim_aileron_dir) },
    { "trim_elevator",             VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_TRIM_DIRECTION }, PG_SERVO_CONFIG, offsetof(servoConfig_t, trim_elevator_dir) },
    { "trim_rudder",               VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_TRIM_DIRECTION }, PG_SERVO_CONFIG, offsetof(servoConfig_t, trim_rudder_dir) },
#endif
```

(이 항목들이 `#ifdef USE_SERVOS` 블록 안에 있는지 확인할 것 — 위 찾을 텍스트의 `ready_to_arm_wiggle_hz` 라인이 이미 그 블록 안에 있으므로 그 바로 뒤에 추가하면 자동으로 같은 블록 안에 들어간다.)

---

### 5) `src/main/msp/msp_box.c`

**5-1) 박스 이름 변경**

**찾을 텍스트**:
```c
    { .boxId = BOXHEADFREE, .boxName = "HEADFREE", .permanentId = 6 },
```

**결과**:
```c
    { .boxId = BOXHEADFREE, .boxName = "SERVO TRIM", .permanentId = 6 },
```

> **주의**: `serializeBoxNameFn()` 함수(140번 줄 부근)의 커스텀 이름 오버라이드 로직은 `BOXUSER1`~`BOXUSER4`만 검사하고 `BOXHEADFREE`는 검사하지 않는다. 따라서 이 함수는 **수정할 필요가 없다** — 위에서 바꾼 `boxName`이 그대로 나간다.

**5-2) `getBoxIdState()` 함수 수정 (필수 — 이거 없으면 Configurator Modes 탭에서 스위치를 켜도 활성 표시가 안 뜬다)**

**찾을 텍스트**:
```c
bool getBoxIdState(boxId_e boxid)
{
    const uint8_t boxIdToFlightModeMap[] = BOXID_TO_FLIGHT_MODE_MAP_INITIALIZER;

    // we assume that all boxId below BOXID_FLIGHTMODE_LAST except BOXARM are mapped to flightmode
    STATIC_ASSERT(ARRAYLEN(boxIdToFlightModeMap) == BOXID_FLIGHTMODE_LAST + 1, FLIGHT_MODE_BOXID_MAP_INITIALIZER_does_not_match_boxId_e);

    if (boxid == BOXARM) {
        return ARMING_FLAG(ARMED);
    } else if (boxid <= BOXID_FLIGHTMODE_LAST) {
        return FLIGHT_MODE(1 << boxIdToFlightModeMap[boxid]);
    } else {
        return IS_RC_MODE_ACTIVE(boxid);
    }
}
```

**결과** (`BOXARM` 분기와 `boxid <= BOXID_FLIGHTMODE_LAST` 분기 **사이**에 새 분기를 끼워넣는다 — 순서가 중요하다. `BOXHEADFREE`는 `BOXID_FLIGHTMODE_LAST`보다 작은 값이라, 아래처럼 먼저 걸러내지 않으면 원래 분기(`FLIGHT_MODE` 조회)로 빠져서 항상 false가 나온다):
```c
bool getBoxIdState(boxId_e boxid)
{
    const uint8_t boxIdToFlightModeMap[] = BOXID_TO_FLIGHT_MODE_MAP_INITIALIZER;

    // we assume that all boxId below BOXID_FLIGHTMODE_LAST except BOXARM are mapped to flightmode
    STATIC_ASSERT(ARRAYLEN(boxIdToFlightModeMap) == BOXID_FLIGHTMODE_LAST + 1, FLIGHT_MODE_BOXID_MAP_INITIALIZER_does_not_match_boxId_e);

    if (boxid == BOXARM) {
        return ARMING_FLAG(ARMED);
    } else if (boxid == BOXHEADFREE) {
        // Servo Trim이 이 박스를 재사용한다. HEADFREE_MODE flight mode flag는
        // 더 이상 어디서도 설정되지 않으므로(아래 core.c 항목 참고), 이 박스는
        // flightModeFlags가 아니라 RC 모드(rcModeActivationMask) 상태를 직접 조회한다.
        return IS_RC_MODE_ACTIVE(BOXHEADFREE);
    } else if (boxid <= BOXID_FLIGHTMODE_LAST) {
        return FLIGHT_MODE(1 << boxIdToFlightModeMap[boxid]);
    } else {
        return IS_RC_MODE_ACTIVE(boxid);
    }
}
```

**5-3) `BME(BOXHEADFREE)` 관련 — 수정 불필요**

`sensors(SENSOR_ACC)` 조건 안에 있는 `BME(BOXHEADFREE);` 줄은 그대로 둔다. 가속도계가 있으면 무조건 노출되므로 손댈 필요 없음.

---

### 6) `src/main/fc/core.c`

아래 3개 블록을 **전부 삭제**한다. 셋 다 삭제 대상이며, 대체 코드로 바꾸는 게 아니라 그냥 지우는 것이다.

**6-1) 삭제 대상 A** (HEADFREE_MODE 플래그 설정/해제):

**찾아서 삭제할 텍스트**:
```c
        if (IS_RC_MODE_ACTIVE(BOXHEADFREE) && !FLIGHT_MODE(GPS_RESCUE_MODE)) {
            if (!FLIGHT_MODE(HEADFREE_MODE)) {
                ENABLE_FLIGHT_MODE(HEADFREE_MODE);
            }
        } else {
            DISABLE_FLIGHT_MODE(HEADFREE_MODE);
        }
```

**6-2) 삭제 대상 B** (BOXHEADADJ — 헤드프리 레퍼런스 각 설정, 본 기능과 무관):

**찾아서 삭제할 텍스트**:
```c
        if (IS_RC_MODE_ACTIVE(BOXHEADADJ) && !FLIGHT_MODE(GPS_RESCUE_MODE)) {
            if (imuQuaternionHeadfreeOffsetSet()) {
               beeper(BEEPER_RX_SET);
            }
        }
```

**6-3) 삭제 대상 C** (고정익에서 HEADFREE_MODE 강제 비활성화 — 더 이상 필요 없음, 어차피 위 A를 지우면 이 플래그 자체가 설정될 일이 없다):

**찾아서 삭제할 텍스트**:
```c
    if (mixerConfig()->mixerMode == MIXER_FLYING_WING || mixerConfig()->mixerMode == MIXER_AIRPLANE) {
        DISABLE_FLIGHT_MODE(HEADFREE_MODE);
    }
```

> **삭제 후 부작용 확인**: A, B를 지우면 `imuQuaternionHeadfreeOffsetSet()` (`flight/imu.c`에 정의됨)를 호출하는 코드가 소스 전체에서 사라진다. 이 함수는 `extern`(비-static) 함수라 "호출부 없음" 자체로는 컴파일 에러나 경고가 나지 않는다. 그대로 둬도 무방하다.

---

### 7) `src/main/osd/osd_elements.c`

**찾을 텍스트**:
```c
    } else if (FLIGHT_MODE(HEADFREE_MODE)) {
        strcpy(element->buff, "HEAD");
```

**결과**:
```c
    } else if (IS_RC_MODE_ACTIVE(BOXHEADFREE)) {
        strcpy(element->buff, "TRIM");
```

(`core.c`의 A블록을 지우면 `FLIGHT_MODE(HEADFREE_MODE)`는 영원히 false이므로, OSD에 파일럿이 트림모드 상태를 보려면 `IS_RC_MODE_ACTIVE(BOXHEADFREE)`로 직접 조회하도록 바꿔야 한다.)

---

### 8) `src/main/osd/osd_warnings.c` (선택 사항, 필수 아님)

`FLIGHT_MODE(HEADFREE_MODE)` 관련 블록("Show warning if in HEADFREE flight mode")은 core.c A블록 삭제 후 자연히 죽은 코드가 되어 절대 실행되지 않는다. 그대로 둬도 동작에 지장 없음. 트림 모드 진입 시 화면에 경고성 블링크 문구를 띄우고 싶다면 osd_elements.c와 같은 패턴으로 바꿀 수 있으나, 지상에서 트림 잡을 때마다 경고가 깜빡이는 게 오히려 거슬릴 수 있어 이번 버전에서는 손대지 않는 것을 권장한다.

---

### 9) `src/main/fc/rc.c`

**9-1) include 추가**

파일 최상단 include 목록에 다음 두 줄을 추가 (현재 `<stdlib.h>`, `<math.h>`는 있지만 `<string.h>`가 없고, `flight/servos.h`도 include돼 있지 않음):

**찾을 텍스트**:
```c
#include <math.h>
```

**결과**:
```c
#include <math.h>
#include <string.h>
```

그리고 다른 `#include "flight/..."` 줄들 근처(예: `#include "flight/pid.h"` 아래)에:
```c
#include "flight/servos.h"
```

**9-2) 675번 줄 근처 — 죽은 조건 정리**

**찾을 텍스트**:
```c
        if (rxConfig()->fpvCamAngleDegrees && IS_RC_MODE_ACTIVE(BOXFPVANGLEMIX) && !FLIGHT_MODE(HEADFREE_MODE)) {
```

**결과**:
```c
        if (rxConfig()->fpvCamAngleDegrees && IS_RC_MODE_ACTIVE(BOXFPVANGLEMIX)) {
```

**9-3) 헤드프리 쿼터니언 변환 블록 삭제**

**찾아서 삭제할 텍스트** (통째로 삭제, 대체 코드 없음):
```c
    if (FLIGHT_MODE(HEADFREE_MODE)) {
        static t_fp_vector_def  rcCommandBuff;

        rcCommandBuff.X = rcCommand[ROLL];
        rcCommandBuff.Y = rcCommand[PITCH];
        if ((!FLIGHT_MODE(ANGLE_MODE) && (!FLIGHT_MODE(HORIZON_MODE)) && (!FLIGHT_MODE(GPS_RESCUE_MODE)))) {
            rcCommandBuff.Z = rcCommand[YAW];
        } else {
            rcCommandBuff.Z = 0;
        }
        imuQuaternionHeadfreeTransformVectorEarthToBody(&rcCommandBuff);
        rcCommand[ROLL] = rcCommandBuff.X;
        rcCommand[PITCH] = rcCommandBuff.Y;
        if ((!FLIGHT_MODE(ANGLE_MODE)&&(!FLIGHT_MODE(HORIZON_MODE)) && (!FLIGHT_MODE(GPS_RESCUE_MODE)))) {
            rcCommand[YAW] = rcCommandBuff.Z;
        }
    }
```

이 블록을 지우고 나면, 바로 다음에 Board Align 블록의 주석(`// --- Board Alignment Tuning Mode ---`)이 이어져야 정상이다.

**9-4) 새 Servo Trim 블록 삽입 (핵심 — 이 코드 하나만 사용할 것, 다른 초안과 섞지 말 것)**

Board Align 블록은 아래처럼 끝난다:
```c
        } else {
            // Mode deactivated: reset state (immediate release, no transition)
            if (boardAlignModeActive) {
                boardAlignModeActive = false;
            }
        }
    }
}
```

**정확히 이 부분에서**, Board Align 블록을 닫는 중괄호(위 코드의 두 번째 줄 `    }`, 즉 `updateRcCommands()` 함수 자체를 닫는 마지막 `}` **바로 앞**)에 아래 블록 전체를 삽입한다. 즉 최종 구조는:

```c
        } else {
            if (boardAlignModeActive) {
                boardAlignModeActive = false;
            }
        }
    }

    // ↓↓↓ 여기부터 새로 삽입 ↓↓↓
    // --- Servo Trim Mode ---
    {
        static bool servoTrimModeActive = false;
        static float servoTrimBaseline[3]; // [ROLL, PITCH, YAW] — ROLL은 활성 시점 값 유지
        static int16_t originalMiddle[MAX_SUPPORTED_SERVOS]; // 모드 진입 시 각 서보의 middle 스냅샷
        static bool latchHi[3];
        static bool latchLo[3];

        if (IS_RC_MODE_ACTIVE(BOXHEADFREE)
            && !IS_RC_MODE_ACTIVE(BOXUSER2)            // Board Align과 상호배타
            && !FLIGHT_MODE(ANGLE_MODE)                // ANGLE 모드에서 차단
            && !FLIGHT_MODE(HORIZON_MODE))             // HORIZON 모드에서 차단
        {
            // 모드 진입 시: 베이스라인 캡처, originalMiddle 캡처, 래치 초기화
            if (!servoTrimModeActive) {
                servoTrimModeActive = true;
                servoTrimBaseline[ROLL]  = rcCommand[ROLL];  // 에일러론은 활성 시점 값 유지
                servoTrimBaseline[PITCH] = 0;                 // 엘리베이터는 0 고정
                servoTrimBaseline[YAW]   = 0;                 // 러더는 0 고정
                for (int s = 0; s < MAX_SUPPORTED_SERVOS; s++) {
                    originalMiddle[s] = servoParams(s)->middle;
                }
                for (int i = 0; i < 3; i++) {
                    latchHi[i] = false;
                    latchLo[i] = false;
                }
            }

            // 스틱 인터셉트: 비행 커맨드 프리즈 (쓰로틀은 건드리지 않음)
            rcCommand[ROLL]  = servoTrimBaseline[ROLL];
            rcCommand[PITCH] = 0;
            rcCommand[YAW]   = 0;

            const int axisDir[3] = {
                servoConfig()->trim_aileron_dir,   // ROLL
                servoConfig()->trim_elevator_dir,  // PITCH
                servoConfig()->trim_rudder_dir,    // YAW
            };
            const int inputSourceForAxis[3] = {
                INPUT_STABILIZED_ROLL,
                INPUT_STABILIZED_PITCH,
                INPUT_STABILIZED_YAW,
            };
            const int inputSourceRCForAxis[3] = {
                INPUT_RC_ROLL,     // 커스텀 믹서에서 사용 가능
                INPUT_RC_PITCH,
                INPUT_RC_YAW,
            };
            const int axes[3] = { ROLL, PITCH, YAW };

            bool changed = false;

            for (int i = 0; i < 3; i++) {
                if (axisDir[i] == 0) continue;  // 0 = 사용안함, 히스테리시스 판정 스킵

                // 히스테리시스 래치 판정 (Board Align과 동일 패턴)
                float raw = rcData[axes[i]];
                bool stepUp   = false;
                bool stepDown = false;

                if (!latchHi[i] && raw > 1750.0f) {
                    stepUp = true;
                    latchHi[i] = true;
                } else if (latchHi[i] && raw < 1600.0f) {
                    latchHi[i] = false;
                }

                if (!latchLo[i] && raw < 1250.0f) {
                    stepDown = true;
                    latchLo[i] = true;
                } else if (latchLo[i] && raw > 1400.0f) {
                    latchLo[i] = false;
                }

                if (!stepUp && !stepDown) continue;

                int8_t step = servoConfig()->servo_trim_step;
                int8_t axisMultiplier = (axisDir[i] == 2) ? -1 : 1;
                int8_t stepSign = stepUp ? 1 : -1;

                // 이번 판정에서 이미 조정한 서보를 추적 (동일 서보에 같은 축 규칙이 여러 개 있어도 한 번만 조정)
                bool updated[MAX_SUPPORTED_SERVOS];
                memset(updated, 0, sizeof(updated));

                for (int r = 0; r < getActiveServoRuleCount(); r++) {
                    const servoMixer_t *rule = &getCurrentServoMixer()[r];

                    if (rule->inputSource != inputSourceForAxis[i]
                        && rule->inputSource != inputSourceRCForAxis[i]) continue;

                    uint8_t target = rule->targetChannel;

                    if (updated[target]) continue;
                    updated[target] = true;

                    // forwardFromChannel 서보는 트림 대상에서 제외
                    if (servoParams(target)->forwardFromChannel
                        != CHANNEL_FORWARDING_DISABLED) continue;

                    // 유효 방향 = servoDirection(reversedSources) x rule.rate 부호 x servoParams.rate 부호
                    int8_t effectiveDir = servoDirection(target, rule->inputSource)
                                        * ((rule->rate < 0) ? -1 : 1)
                                        * ((servoParams(target)->rate < 0) ? -1 : 1);
                    int16_t signedStep = step * effectiveDir * axisMultiplier * stepSign;
                    int16_t currentMiddle = servoParams(target)->middle;
                    int16_t newMiddle = constrain(
                        currentMiddle + signedStep,
                        originalMiddle[target] - SERVO_TRIM_LIMIT_PWM,
                        originalMiddle[target] + SERVO_TRIM_LIMIT_PWM
                    );
                    if (newMiddle != currentMiddle) {
                        servoParamsMutable(target)->middle = newMiddle;
                        changed = true;
                    }
                }
            }
            if (changed) {
                beeper(BEEPER_RX_SET);
            }
        } else {
            if (servoTrimModeActive) {
                servoTrimModeActive = false;
            }
        }
    }
    // ↑↑↑ 여기까지 삽입 ↑↑↑
}
```

**이 블록은 완성본이다. 이것 하나만 넣고, 다른 축약된/미완성된 버전을 만들거나 섞지 말 것.**

---

## 예상 문제점

1. **rate/reversedSources 자동 계산 후에도 방향이 어긋나는 예외**: `servoDirection() × rule.rate 부호 × servoParams.rate 부호` 3요소로 대부분의 케이스를 커버하지만, 물리적 서보 장착이 예상과 다르거나 한 축에 비대칭 구성이 있으면 틀릴 수 있다. 이때는 `trim_aileron/elevator/rudder`를 `2`(반전)로 CLI에서 바꾸면 된다.
2. **FlyingWing류 믹서에서 엘리베이터 트림**: 플래퍼론 두 개가 PITCH 규칙을 공유하는 경우, 엘리베이터 트림이 두 서보의 middle을 동시에 조정한다(엘리본 방식). 에일러론 트림과 동시 적용 시 값이 누적(가산)되는 것이 의도된 동작이다.
3. **트림 한계값의 근거**: middle이 min/max 근처에 접근해서 연산이 깨지는 걸 걱정할 필요는 없다. 그 전에 이미 기체가 심각한 이상 비행 특성을 보이게 된다. `SERVO_TRIM_LIMIT_PWM`은 "연산 안정성"이 아니라 "비행 안전" 목적의 하드 리밋이다. 실기 비행 테스트로 실제 감당 가능한 범위를 확인 후 값을 조정할 것.
4. **영구 저장**: `servoParamsMutable()`로 변경한 값은 즉시 비행에 반영되지만 EEPROM에는 저장되지 않는다. CLI `save` 또는 Configurator 저장을 해야 재부팅 후에도 유지된다. 의도된 안전장치이므로 그대로 둔다.
5. **Bird Flap 상호작용**: 위 "사전 확인 사항" 참고.

---

## 작동계획

1. 모드탭에서 "SERVO TRIM"(구 HEADFREE 박스)에 AUX 스위치 할당.
2. 스위치 ON 시 (ANGLE/HORIZON 모드가 아니고, Board Align이 비활성일 때): 쓰로틀 통과, 에일러론 값 고정, 엘리베이터/러더 0 고정. 모든 서보의 현재 `middle`을 `originalMiddle[]`에 캡처.
3. 스틱을 중립 근처 → 끝점(1750 초과 또는 1250 미만)으로 이동 시 히스테리시스 판정 통과 → `servo_trim_step`만큼 대상 서보들의 `middle` 조정. 방향은 3요소 자동 계산, 실패 시 `trim_*` CLI로 반전. `beeper(BEEPER_RX_SET)` 피드백.
4. 같은 끝점에 머물러도 재판정 안 됨(latch). 중립 근처(1600 미만 또는 1400 초과)로 복귀해야 다음 스텝 가능.
5. 트림 한계는 모드 진입 시점 `originalMiddle` 기준 ±100pwm. 모드 재진입 시마다 새 기준이 잡힌다.
6. 스위치 OFF → 즉시 정상 조종으로 복귀, 변경된 middle 값은 유지된 채 비행.
7. 지상에서 CLI `servo` 명령으로 최종 middle 값 확인 → 만족스러우면 `save`로 영구 저장.
8. 방향이 반대로 동작하면 `set trim_aileron = 2` (또는 elevator/rudder)로 즉시 반전, 코드 재작성 불필요.

---

## 구현 완료 후 테스트 체크리스트

- [ ] 컴파일 성공 (링커 에러 없음 — 접근자 함수 본문 누락 여부 확인)
- [ ] CLI에서 `servo_trim_step`, `trim_aileron`, `trim_elevator`, `trim_rudder` 파라미터가 `get`/`set`으로 조회/설정되는지 확인
- [ ] Configurator Modes 탭에서 "SERVO TRIM"이 표시되고, 스위치 조작 시 활성 표시(초록불)가 뜨는지 확인 (`getBoxIdState` 특별 케이스 검증)
- [ ] 지상에서 스위치 ON 후 각 축 스틱을 끝점으로 움직였을 때 해당 서보가 실제로 움직이는지, 그리고 스틱 방향과 타면 방향이 일치하는지 확인
- [ ] 같은 끝점에 계속 머물러도 한 번만 움직이는지(히스테리시스) 확인
- [ ] forwardFromChannel 설정된 서보가 트림 대상에서 제외되는지 확인
- [ ] `set trim_aileron = 2`로 방향이 즉시 반전되는지 확인 (재컴파일 없이)
- [ ] Bird Flap이 켜진 상태에서 에일러론 트림이 어느 서보에 영향을 주는지(또는 안 주는지) 확인 — "사전 확인 사항" 항목과 일치하는지 재검증
- [ ] `save` 후 재부팅해도 트림 값이 유지되는지 확인
- [ ] OSD에 트림 모드 활성 시 "TRIM" 문구가 뜨는지 확인
- [ ] Board Align(BOXUSER2)과 Servo Trim을 동시에 켰을 때 Board Align이 우선 동작하고 Servo Trim은 대기 상태로 빠지는지 확인

---

## 부록: 트림모드 없이 물리 중립 잡는 법 (forwardFromChannel 트릭)

이번 트림모드와는 별개로 존재하는 기존 기법이다.

- 원하는 조종 채널(예: 에일러론)을 남는 AUX 채널(예: AUX3)에 복사하도록 조종기를 설정한다.
- 서보의 `forwardFromChannel`을 그 AUX 채널로 지정한다.
- 원래 채널(에일러론)에 할당되어 있던 조종기 트림 버튼 기능은 삭제하고, 복사된 AUX 채널 쪽에 트림 버튼을 재할당한다.
- 이렇게 하면 조종기 자체 트림 버튼으로 서보의 물리 중립(PWM)을 직접 바꿀 수 있다.

**제약**: AUX 채널이 여유 있어야 하고, 조종기가 트림 버튼의 삭제/재할당을 지원해야 한다. 일부 조종기 기종은 지원하지 않아 모두가 쓸 수 있는 방법은 아니다. 이번에 만드는 Servo Trim Mode는 이 제약 없이 모든 조종기에서 쓸 수 있게 하는 것이 목적이다.

`forwardFromChannel`이 설정된 서보는 Servo Trim Mode 대상에서 자동 제외되므로, 이 두 방식은 서로 간섭하지 않는다.