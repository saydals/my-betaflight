# Servo Extension Plan

## 문제 정의

Configurator GUI에서 rate는 100%까지만 설정 가능. 사용자가 서보 min/max를 500-2500으로 확장해도 실제 출력은 1000-2000 범위를 벗어나지 못함.

**원인**: `rcCommand` 범위(-500~+500) × rate(100%) = ±500 → 중립(1500) 기준 최대 2000

## 해결 방안

**핵심 아이디어**: 서보 min/max 범위가 확장되면 deviation 비율을 유지한 채 새 범위에 맞게 비례 스케일링

## 고려해야 할 사항

1. **비대칭(Asymmetric) min/max**: min=1000, max=2500 (하위 500, 상위 1000)
2. **중립(middle) 변경**: min=1000, middle=1200, max=2000 (하위 200, 상위 800 → 4배 차이)
3. **비례 스케일링**: 원래 deviation 비율 유지
4. **rate 정상 동작**: rate 변경 시 모든 출력값에 rate가 곱해진 후 스케일링
5. **AUX 채널 포워딩(Channel Forwarding) 충돌**: 포워딩은 rcData를 직접 min-max 범위로 매핑하므로 **이중 스케일링 방지** 필요
6. **부호 확장(Sign Extension) 버그**: `int8_t`와 `uint8_t` 직접 비교 시 항상 true 반환 → `uint8_t` 캐스팅 필요

## AUX Channel Forwarding 분석

### 현재 코드

`determineServoMiddleOrForwardFromChannel()` (line 466-475):
```c
int16_t determineServoMiddleOrForwardFromChannel(servoIndex_e servoIndex)
{
    const uint8_t channelToForwardFrom = servoParams(servoIndex)->forwardFromChannel;
    if (channelToForwardFrom != CHANNEL_FORWARDING_DISABLED && channelToForwardFrom < rxRuntimeState.channelCount) {
        return scaleRangef(constrainf(rcData[channelToForwardFrom], PWM_RANGE_MIN, PWM_RANGE_MAX), PWM_RANGE_MIN, PWM_RANGE_MAX, servoParams(servoIndex)->min, servoParams(servoIndex)->max);
    }
    return servoParams(servoIndex)->middle;
}
```

**AUX 포워딩은 rcData(1000~2000)를 직접 서보 min~max(예: 500~2500)로 매핑**함.

### 해결: AUX 포워딩 활성화 시 스케일링 제외

`scaleServoOutput()` 내부에서 `uint8_t` 캐스팅 후 `forwardFromChannel` 체크:
```c
const uint8_t channelToForwardFrom = servoParams(i)->forwardFromChannel;
if (channelToForwardFrom != CHANNEL_FORWARDING_DISABLED) {
    servo[i] = constrain(servo[i], min, max);
    return;
}
```

## 수정할 파일

### 1. `src/main/flight/servos.c` — `servoTable()` 함수 (line 884-887) ✅ 필수

`servoTable()` 내에서만 `scaleServoOutput()` 호출. `filterServos()`는 단순 constrain 유지.

### 2. `src/main/flight/servos.h` — 수정 불필요 ❌

`servoParam_t.rate`가 이미 `int8_t`로 선언되어 있어 -125~+125 범위 지원 중.

## 실행 순서

```
servoMixer() 실행         ← 믹서 계산, rate 적용, 중립(AUX) 추가
    ↓
servoTable() 호출
    ├── GIMBAL/CAMSTAB 처리
    └── ★ scaleServoOutput() (유일한 스케일링 위치)
    ↓
filterServos() 호출       ← LPF 사용 시에만 실행, 단순 constrain만
    ↓
writeServos() 호출        ← PWM 출력
```

## 공통 스케일링 함수

**배치 위치**: `determineServoMiddleOrForwardFromChannel()` 함수 바로 아래 (line 475 이후)

```c
/**
 * 서보 출력을 사용자 설정 min/max/middle에 맞게 비례 스케일링
 * - 기본 설정(1000-1500-2000)과 다를 때만 적용
 * - AUX Channel Forwarding 활성화 시 이중 스케일링 방지 (constrain만 수행)
 * - uint8_t 캐스팅으로 부호 확장(Sign Extension) 버그 방지
 */
static void scaleServoOutput(int i)
{
    int16_t min = servoParams(i)->min;
    int16_t max = servoParams(i)->max;
    int16_t middle = servoParams(i)->middle;
    
    const int16_t DEFAULT_HALF_RANGE = 500;
    
    int16_t rangeDown = middle - min;
    int16_t rangeUp = max - middle;
    
    // 기본 설정과 다를 때만 스케일링
    if (rangeDown == DEFAULT_HALF_RANGE && rangeUp == DEFAULT_HALF_RANGE) {
        servo[i] = constrain(servo[i], min, max);
        return;
    }
    
    // uint8_t 캐스팅으로 부호 확장 버그 방지
    const uint8_t channelToForwardFrom = servoParams(i)->forwardFromChannel;
    if (channelToForwardFrom != CHANNEL_FORWARDING_DISABLED) {
        servo[i] = constrain(servo[i], min, max);
        return;
    }
    
    int16_t deviation = servo[i] - middle;
    int32_t scaled = middle;
    
    if (deviation < 0) {
        scaled += (int32_t)deviation * rangeDown / DEFAULT_HALF_RANGE;
    } else if (deviation > 0) {
        scaled += (int32_t)deviation * rangeUp / DEFAULT_HALF_RANGE;
    } else {
        servo[i] = constrain(servo[i], min, max);
        return;
    }
    
    servo[i] = constrain((int16_t)scaled, min, max);
}
```

### 호출 위치 — `servoTable()` (line 884-887)

기존:
```c
// constrain servos
for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
    servo[i] = constrain(servo[i], servoParams(i)->min, servoParams(i)->max);
}
```

변경:
```c
// Scale and constrain servos — asymmetric support with AUX forwarding protection
for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
    scaleServoOutput(i);
}
```

### `filterServos()` — 변경 없음 (단순 constrain 유지)

```c
static void filterServos(void)
{
    if (servoConfig()->servo_lowpass_freq) {
        for (int servoIdx = 0; servoIdx < MAX_SUPPORTED_SERVOS; servoIdx++) {
            servo[servoIdx] = lrintf(biquadFilterApply(&servoFilter[servoIdx], (float)servo[servoIdx]));
            servo[servoIdx] = constrain(servo[servoIdx], servoParams(servoIdx)->min, servoParams(servoIdx)->max);
        }
    }
}
```

## 동작 예시

### 예1: min=1000, middle=1500, max=2500 (대칭, 상위만 확장)

| 원래 편차 | 원래 출력 | 새 출력 | 계산 |
|-----------|-----------|---------|------|
| -500 | 1000 | 1000 | 1500 - 500×500/500 = 1000 |
| -250 | 1250 | 1250 | 1500 - 250×500/500 = 1250 |
| +250 | 1750 | **2000** | 1500 + 250×1000/500 = 2000 |
| +500 | 2000 | **2500** | 1500 + 500×1000/500 = 2500 |

### 예2: min=1000, middle=1200, max=2000 (중립 변경, 비대칭)

| 원래 편차 | 원래 출력 | 새 출력 | 계산 |
|-----------|-----------|---------|------|
| -500 | 1000 | **1000** | 1200 - 500×200/500 = 1000 |
| -250 | 1250 | **1100** | 1200 - 250×200/500 = 1100 |
| +250 | 1750 | **1600** | 1200 + 250×800/500 = 1600 |
| +500 | 2000 | **2000** | 1200 + 500×800/500 = 2000 |

### 예3: min=500, middle=1500, max=2500 (대칭, 양방향 확장)

| 원래 편차 | 원래 출력 | 새 출력 | 계산 |
|-----------|-----------|---------|------|
| -500 | 1000 | **500** | 1500 - 500×1000/500 = 500 |
| -250 | 1250 | **1000** | 1500 - 250×1000/500 = 1000 |
| +250 | 1750 | **2000** | 1500 + 250×1000/500 = 2000 |
| +500 | 2000 | **2500** | 1500 + 500×1000/500 = 2500 |

### 예4: min=1000, middle=1500, max=2000 (기본) — 변화 없음

| 원래 편차 | 원래 출력 | 새 출력 | 변화 |
|-----------|-----------|---------|------|
| -500 | 1000 | 1000 | 없음 |
| +500 | 2000 | 2000 | 없음 |

### 예5: AUX 포워딩 사용 (min=500, max=2500, AUX=1750)

| AUX 입력 | 포워딩 출력 | scaleServoOutput | 결과 |
|----------|------------|-----------------|------|
| 1750 | **2000** | constrain만 적용 (스케일링 제외) | **2000** ✅ |
| 1000 | 500 | constrain만 적용 | **500** ✅ |
| 2000 | 2500 | constrain만 적용 | **2500** ✅ |

## Rate와의 관계

line 831-832의 실행 순서 (servoMixer() 함수 내):
1. `servo[i] = ((int32_t)servoParams(i)->rate * servo[i]) / 100L;` → rate 적용
2. `servo[i] += determineServoMiddleOrForwardFromChannel(i);` → 중립(AUX) 더함
3. `servoTable()` 내 `scaleServoOutput(i)` → 비례 스케일링 + constrain (AUX 제외)

rate 125% 예시:
- rate 적용 후 deviation = 500 × 1.25 = **625**
- rangeDown=1000, rangeUp=1000 (500-2500 설정) → scaled = 1500 + 625×1000/500 = **2750**
- constrain(2750, 500, 2500) = **2500** ← 최대 출력

## PID 영향 분석

### 영향 범위 요약

scaleServoOutput()은 서보 출력값에 적용되므로, servo[]를 통해 출력되는 모든 신호에 영향을 줍니다.

| 영향 받는 항목 | 이유 |
|---------------|------|
| **PID P gain** | PID 출력이 servo[]로 전달되어 증폭 |
| **PID I gain** | 적분항도 동일 비율로 증폭 |
| **PID D gain** | 미분항도 동일 비율로 증폭 |
| **Feedforward (FF)** | pidData.Sum에 포함되어 모두 증폭 |
| **Mixer rate** | mixer rate 적용 후 스케일링 |
| **Servo rate** | servo rate 적용 후 스케일링 |
| **GPS Rescue 출력** | gpsRescueAngle도 servo[]로 출력됨 |
| **Board Alignment 출력** | servoTable() 내에서 처리됨 |
| **Bird Flap / Gimbal** | servo[]를 통해 동일하게 스케일링 |

| 영향 받지 않는 항목 | 이유 |
|--------------------|------|
| **RC Rate (회전속도 deg/s)** | Rate는 스틱→목표각 매핑, PID 이전 단계 |
| **Throttle / 모터 출력** | 모터는 별도 출력 경로 |
| **AUX Channel Forwarding** | 코드에서 명시적 제외 (uint8_t 캐스팅) |
| **PID 내부 구조** | P:I:D:FF 비율은 그대로 유지됨 |
| **RC Rate 곡선 / Expo** | 스틱→deg/s 매핑은 변경 없음 |

**RC Rate(회전속도)** 는 scaleServoOutput의 영향을 받지 않습니다. RC Rate는 "스틱 끝에서 목표 회전속도"만 결정하고, PID는 "목표를 추적하는 강도"를 담당합니다. 서보 확장 시에는 PID 값만 조정하면 되며 RC Rate는 변경할 필요가 없습니다.

### 신호 경로

```
PID 계산 (pidData[FD_ROLL].Sum)
    ↓ × PID_SERVO_MIXER_SCALING
input[INPUT_STABILIZED_ROLL] = [-500 ~ +500]
    ↓ × mixer rate / 100
    ↓ × servo rate / 100
    ↓ + middle (1500)
servo[i] = [1000 ~ 2000]  ← 기본 설정
    ↓ scaleServoOutput()  ← servoTable() 내 1회만
servo[i] = [500 ~ 2500]   ← 확장 설정
    ↓
PWM 출력 → 서보 물리적 움직임
```

### 영향 분석

같은 PID 값이라도 서보 범위가 확장되면 물리적 편향이 비례하여 증가합니다.

| PID 출력 | 기본(1000-2000) | 확장(500-2500) | 물리적 변화 |
|----------|----------------|----------------|-----------|
| +250 (50%) | 1750 (250us 편향) | **2000** (500us 편향) | **2배 증가** |
| +500 (100%) | 2000 (500us 편향) | **2500** (1000us 편향) | **2배 증가** |

**결론: 서보 범위를 확장하면 PID 값을 줄여야 합니다.**

### 유효 PID 게인 변화율

```
유효 PID 게인 변화율 = rangeUp_new / rangeUp_old
```

| 서보 설정 | rangeDown | rangeUp | PID 조정 필요 |
|-----------|-----------|---------|--------------|
| 1000-1500-2000 (기본) | 500 | 500 | 기준값 (변화 없음) |
| 500-1500-2500 (양방향 확장) | 1000 | 1000 | **PID 값 절반으로 감소** |
| 1000-1500-2500 (상향만 확장) | 500 | 1000 | **PID P 값 50%로 감소** (상향만 2배) |
| 800-1500-2200 (약간 확장) | 700 | 700 | **PID 값 70%로 감소** |
| 1000-1200-2000 (중립 변경) | 200 | 800 | **PID P 값 62.5%로 감소** (하향 게인 0.4배, 상향 1.6배) |

### 권장 사항

1. 서보 범위 변경 후 **반드시 PID 재튜닝 필요**
2. **확장 시**: PID 값 감소 필요 (유효 게인 증가)
3. **축소 시**: PID 값 증가 필요 (유효 게인 감소)
4. 양방향 대칭 확장(500-1500-2500)이 가장 예측 가능하고 튜닝이 쉬움
5. 비대칭 확장 시 PID 튜닝이 복잡해지므로 가능한 피할 것
6. 변경 비율이 클수록 PID 조정 폭이 커짐

## 계획서 작성 근거 파일

### 1. `src/main/flight/servos.c` (핵심 수정 대상) ✅

분석 내용:
- `servoMixer()` 함수 (line 734-834): 믹서 계산, rate 적용, 중립(AUX) 추가 로직
- `servoTable()` 함수 (line 837-888): GIMBAL/CAMSTAB 처리 후 constrain (유일한 수정 위치)
- `filterServos()` 함수 (line 906-921): LPF 사용 시 단순 constrain 유지 (변경 없음)
- `determineServoMiddleOrForwardFromChannel()` (line 466-475): AUX 포워딩이 scaleRangef로 직접 min-max 매핑 확인, uint8_t 캐스팅 패턴 확인
- `scaleServoOutput()`: **신규 추가할 공통 스케일링 함수** (determineServoMiddleOrForwardFromChannel() 아래 배치)

### 2. `src/main/flight/servos.h` (상수 및 구조체 정의) ✅

분석 내용:
- `DEFAULT_SERVO_MIN = 1000`, `DEFAULT_SERVO_MIDDLE = 1500`, `DEFAULT_SERVO_MAX = 2000`
- `PWM_SERVO_MIN = 500`, `PWM_SERVO_MAX = 2500` (CLI 허용 범위)
- `servoParam_t.rate`: `int8_t`로 이미 -125~+125 지원 중 → **수정 불필요 확인**
- `servoParam_t.forwardFromChannel`: `int8_t` (signed) → **uint8_t 캐스팅 필요 확인**
- `servoParam_t.min/max/middle`: 사용자 설정값 저장 구조체

### 3. `src/main/fc/rc_controls.c` (rcCommand 범위 확인) ✅

분석 내용:
- line 73: `float rcCommand[4]; // [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW`
- Passthrough 모드에서 `rcCommand[ROLL]`의 범위가 -500~+500임을 확인
- 이 값이 servoMixer()에서 input[]으로 사용됨

### 보조 참고 파일

- `src/main/drivers/pwm_output.h`: PWM_RANGE_MIN/PWM_RANGE_MAX 상수
- `src/main/common/maths.h`: constrain, constrainf, scaleRangef 함수
- `src/main/pg/pg_ids.h`: PG ID 등록

## 기능별 최종 동작 요약

| 기능 | AUX 미사용 | AUX 포워딩 사용 |
|------|-----------|----------------|
| 기본 min/max (1000-1500-2000) | 변화 없음 ✅ | 변화 없음 ✅ |
| 확장 min/max (500-1500-2500) | 스케일링 적용 ✅ | AUX 직접 매핑 유지 ✅ (이중 스케일링 방지) |
| 비대칭 (1000-1500-2500) | 상하 비대칭 스케일링 ✅ | AUX 직접 매핑 유지 ✅ |
| 중립 변경 (1000-1200-2000) | 비율 유지 스케일링 ✅ | AUX 직접 매핑 유지 ✅ |
| rate 125% | rate → 스케일링 ✅ | rate만 적용 ✅ |
| PID 영향 | **PID 값 감소 필요** ⚠️ | PID 영향 없음 (직접 매핑) ✅ |

## 영향도

1. **기본 설정(1000-1500-2000)**: rangeDown=500, rangeUp=500 → 조건 불만족 → **변화 없음**
2. **확장 설정(500-1500-2500)**: 스케일링 factor 2.0 → 출력 2배 확장, **PID 값 절반 필요**
3. **비대칭 설정**: 각 방향별 별도 스케일링 factor 적용, **PID 튜닝 복잡**
4. **중립 변경**: 각 방향 범위에 비례하여 자동 조정, **PID 튜닝 필요**
5. **Configurator 호환성**: GUI 변경 불필요
6. **CLI rate 125%와의 관계**: rate 적용 후 스케일링 → 모든 rate에서 정상 동작
7. **AUX Channel Forwarding**: 이중 스케일링 방지 → **확장 계획 실행 후에도 정상 유지** ✅

## 위험 요소

- **다중 입력 믹서(Flying Wing, Heli)**: 한 서보에 ROLL+PITCH 등 복수 입력 합산 시 deviation이 2배로 증가할 수 있음. Airplane/CUSTOMAIRPLANE 구조에서는 영향 없음.
- **정수 연산 오버플로우**: `(int32_t)deviation * rangeDown` 최대값: deviation ±1000 (rate 125% 포함), rangeDown 최대 2000 → 2,000,000, int32_t(≈2.1×10^9) 안전
- **PID 재튜닝 필요**: 서보 범위 확장 시 PID 값을 줄이지 않으면 기체가 과민 반응할 수 있음
- **Bird Flap / Gimbal**: scaleServoOutput이 Bird Flap 및 Gimbal 출력에도 적용됨. 사용자가 일부러 서보 범위를 확장했다면 의도가 있는 것이므로 별도 예외 처리 없이 스케일링 적용. (에러 아님)

## 테스트 시나리오

1. 기본 설정(1000-1500-2000)에서 정상 동작 확인
2. min=500, max=2500 설정 후 passthrough 모드에서 스틱 2000 → 서보 2500 출력 확인
3. min=500, max=2500 설정 후 passthrough 모드에서 스틱 1000 → 서보 500 출력 확인
4. min=1000, max=2500 (비대칭) 설정 후 상하 출력 비대칭 확인
5. min=1000, middle=1200, max=2000 (중립 변경) 설정 후 중립 이하/이상 비율 확인
6. 중립(1500)에서 출력 1500 유지 확인
7. Configurator에서 rate 100%로 설정 후 동일 동작 확인
8. LPF 활성화(servo_lowpass_freq=50) 후에도 스케일링 유지 확인 (이중 스케일링 없음)
9. **AUX Channel Forwarding** 설정 후 AUX 채널 값에 따른 출력이 500-2500 범위에서 정상 출력되는지 확인 (이중 스케일링 없음)
10. **PID 튜닝 검증**: 확장 설정 후 PID 값을 절반으로 줄이고 동일한 비행 특성이 유지되는지 확인