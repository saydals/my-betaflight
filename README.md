# Betaflight Custom Firmware for Fixed-Wing

본 프로젝트는 [Betaflight](https://github.com/betaflight/betaflight) 4.5.3을 기반으로 **RC 고정익(비행기)** 에 최적화된 개조 펌웨어입니다.

---

## 1. 주요 패치 (Key Features)

| # | 패치 내용 | 설명 |
|---|-----------|------|
| 1 | **I-term 회전 최적화** | 소각도 근사 대신 로드리게스 회전 변환(Rodrigues' Rotation) 적용, 급뱅크턴 중 자세 계산 정확도 향상 |
| 2 | **Dshot 모터 버저** | 기체가 Prepped for Arming 상태가 되면 모터 비콘음 알림 (RX_SET 기반) |
| 3 | **비행기 GUI 상시 표시** | 믹서 종류 무관, Configurator 메인화면에 3D 비행기 모델 항상 표시 |
| 4 | **OSD 레스큐 좌표/고도** | 레스큐 작동 중 목표 좌표(Home/A/B) 및 실시간 목표 고도 OSD 표시 |
| 5 | **예약 짐벌 서보 제거** | 강제 짐벌 서보 할당 코드 삭제 → 서보 0번부터 자유 할당 가능 |
| 6 | **OSD Ready 필드** | 레스큐 세부 단계(Phase) 실시간 표시 |
| 7 | **Aux Value → 서보 모니터** | 서보 0~3 출력값(us)을 4자리 숫자로 한 줄 표시 |
| 8 | **단독 셔틀 모드** | 독립 비행 모드로 셔틀 하강/대기만 사용 가능 |
| 9 | **GPS LED** | 사용자 설정 최소 위성 수 완전 만족 시에만 녹색 점등 |
| 10 | **Servo/Smix Rate 125% 확장** | CLI에서 servo/smix rate를 -125~+125%까지 설정 가능 (Configurator GUI는 100% 한계) |
| 11 | **Board Alignment Tuning Mode** | 비행 중 스틱으로 보드 정렬값(align_board_roll/pitch/yaw) 실시간 조절 |

---

## 2. Servo / Smix Rate 125% 확장

### 개요
`cli.c` 검증 로직 변경을 통해 **servo rate**와 **smix rate**의 허용 범위를 기존 `-100~+100%`에서 **`-125~+125%`**로 확장했습니다.

### 중요: Configurator GUI(서보탭) vs CLI 차이

| 구분 | servo rate | smix rate |
|------|-----------|-----------|
| **Configurator 서보탭 GUI** | ❌ **100%까지**만 표시/적용 | ❌ 적용 불가 (smix 전용 GUI 없음) |
| **CLI (`servo` cmd)** | ✅ **125%** 입력 및 작동 | - |
| **CLI (`smix` cmd)** | - | ✅ **125%** 입력 및 작동 |

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
| 출력핀 | 기능 | 설명 |
|--------|------|------|
| S1, S2 | **모터** | Dshot 지원 |
| S3, S4 | 서보 0, 1 | 에일러론 자동 할당 |
| S5 | 서보 2 | 엘리베이터 할당 |
| S6 | 서보 3 | 러더 할당 |

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
| ID | 입력 소스 | ID | 입력 소스 |
|----|----------|----|----------|
| 0 | Stabilized ROLL | 7 | RC THROTTLE |
| 1 | Stabilized PITCH | 8~11 | RC AUX 1~4 |
| 2 | Stabilized YAW | 12 | GIMBAL PITCH |
| 3 | Stabilized THROTTLE | 13 | GIMBAL ROLL |
| 4~6 | RC ROLL/PITCH/YAW | **14** | **Bird Flap** |

**Elevon(전비익) 예시:**
```
smix 0 0 0 50 0 0 100 0     # 우측: 롤 50%
smix 1 0 1 50 0 0 100 0     # 우측: 피치 50%
smix 2 1 0 -50 0 0 100 0    # 좌측: 롤 -50%
smix 3 1 1  50 0 0 100 0    # 좌측: 피치 50%
```


---

## 4. GPS 레스큐 & 셔틀 랜딩

### 비행 단계 흐름
**Ascend Alt** → **Flying Home** → **Point A** → **Shuttle/Descent** → **Final Descent** → **Landing**

1. **Ascend Alt**: 설정 피치각 고정, 3초 직선 상승
2. **Flying Home**: A포인트 타겟 복귀 순항
3. **Point A**: `descentAlt` 짝수=이륙방향, 홀수=복귀방향에 A포인트 매핑
4. **Descent**:
   - 급하강 (shuttleCount=0): A포인트 도달 시 급강하
   - 셔틀 (shuttleCount>0): A포인트~B포인트 왕복 셔틀 후 계단식 하강
5. **Final Descent**: 홈 반경 30m 진입까지 속도/고도 추종
6. **Landing**: 30m 이내에서 landingAlt+landingSpeed 조건 충족 시 활공 스로틀 3초 → 스로틀 1000 차단

### 핵심 파라미터 (Profile 3 전용)
| 파라미터 | 역할 |
|----------|------|
| BankGain / BankPitchGain | 일반 선회 강도 |
| SBankGain / SBankPitchGain | 셔틀 선회 강도 |
| BankYawGain | 러더 기체용 ballooning 감쇠 |
| ShuttleCount / ShuttleDistance | 셔틀 횟수/거리 |
| AscendPitch / MidPitch / LandingPitch | 단계별 피치각 |
| LandingSpeed / landingAlt | 착륙 속도/고도 |
| AltholdGain | 고도 유지 P게인 |
| descentAlt / HeadingYawGain | 하강고도 / 미세요게인 |

### ⚠️ 중요 - PID Profile 3 경고
레스큐 제어 파라미터가 **Profile 3 변수를 전용으로 대체(Repurpose)**하여 사용합니다.  
**수동 비행 시 Profile 3 절대 선택 금지** - Profile 1 또는 2를 사용하세요.

### 무한 셔틀 스탠바이
레스큐 ON 시 AUX < 1500 → 착륙 없이 무한 셔틀 홀딩  
AUX >= 1500 → 정상 착륙 모드 전환  
(어느 단계든 Failsafe 수신 시 강제 안전 절차 우선)

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
| 파라미터 | 기본값 | 범위 | 설명 |
|----------|--------|------|------|
| `bird_flap_max_freq_10x` | 20 | 5~40 | 최대 플랩 주파수 (×0.1 Hz, 기본 2.0Hz) |
| `bird_flap_min_freq_10x` | 3 | 1~10 | 최소 플랩 주파수 (×0.1 Hz, 기본 0.3Hz) |
| `bird_flap_max_amplitude` | 1000 | 0~1000 | 최대 스트로크 진폭 (PWM 단위) |
| `bird_flap_servo_speed` | 500 | 10~500 | 서보 각속도 제한 (deg/s) |
| `bird_flap_up_ratio_100x` | 40 | 30~50 | Upstroke 시간비율 (기본 40=Up 40%, Down 60%) |
| `bird_flap_soft_start_ms` | 800 | 100~3000 | 활성화 시 부드러운 시작 시간 (ms) |
| `bird_flap_soft_stop_ms` | 600 | 100~3000 | 비활성화 시 부드러운 정지 시간 (ms) |

### 동작 흐름
```
OFF → [AUX ON] → STARTING → [진폭 100%] → ACTIVE → [AUX OFF] → STOPPING → [진폭 0%] → OFF
```

AUX 채널 PWM 값(강도)에 따라 주파수가 minFreq ~ maxFreq로 선형 변화합니다.  
AUX에 스로틀을 50% 정도 믹스해두면 스로틀에 따라 날개짓 속도가 변하여 자연스럽습니다.

---

## 6. Board Alignment Tuning Mode (보드 정렬 튜닝 모드)

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

| 증상 | 의심 항목 |
|------|----------|
| 롤 기동 시 피치가 **한쪽 방향으로 밀림** | `align_board_yaw` |
| 롤 기동 시 피치가 **앞뒤로 진동** | `align_board_pitch` |

실제로는 두 가지가 섞여 있는 경우도 많습니다. **Yaw → Pitch** 순서로 조절하는 것을 권장합니다.

#### 2단계 — 모드 진입

롤 스틱을 원하는 방향으로 밀어 롤 회전을 시작한 상태에서 스위치를 켭니다.

모드 진입 시 스틱 입력 처리:

| 스틱 | 모드 진입 후 동작 |
|------|-----------------|
| **Roll** | 진입 순간의 롤 레이트 고정 유지 (손을 떼도 계속 회전) |
| **Pitch** | 강제 중립 (스틱 입력 무시) |
| **Yaw** | 강제 중립 (스틱 입력 무시) |
| **Throttle** | 평소와 동일 (변화 없음) |

롤 회전이 자동으로 유지되므로 파일럿은 손을 떼고 기체 반응만 관찰하면 됩니다. 피치나 Yaw 흔들림이 있다면 순수하게 보드 정렬 오차 때문입니다 (스틱 조작 오염 없음).

#### 3단계 — 스틱으로 조절

롤이 계속 도는 상태에서 스틱으로 정렬값을 조절합니다.

| 조절 항목 | 스틱 방향 | 효과 |
|----------|----------|------|
| **Roll 정렬** | Roll 스틱 오른쪽 | `align_board_roll` +1° |
| **Roll 정렬** | Roll 스틱 왼쪽 | `align_board_roll` -1° |
| **Pitch 정렬** | Pitch 스틱 앞 | `align_board_pitch` +1° |
| **Pitch 정렬** | Pitch 스틱 뒤 | `align_board_pitch` -1° |
| **Yaw 정렬** | Yaw 스틱 오른쪽 | `align_board_yaw` +1° |
| **Yaw 정렬** | Yaw 스틱 왼쪽 | `align_board_yaw` -1° |

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

| 축 | 최대 범위 |
|----|----------|
| **Roll** | ±10° |
| **Pitch** | ±10° |
| **Yaw** | ±15° |

한계에 도달하면 더 이상 값이 바뀌지 않으며 비프음도 울리지 않습니다.

### 현재 값 확인

CLI에서 언제든지 확인할 수 있습니다.

```
get align_board_roll
get align_board_pitch
get align_board_yaw
```

### 구현 상세

- **모드 박스 ID**: `BOXBOARDALIGN` (permanentId = 55)
- **동작 위치**: `src/main/fc/rc.c` — `updateRcCommands()` 함수 내
- **보드 정렬 행렬 갱신**: `src/main/sensors/boardalignment.c` — `updateBoardAlignmentMatrix()`
- **스틱 임계값**: HIGH > 1750, LOW < 1250 (래치 해제: HIGH < 1600, LOW > 1400)
- **수정 파일**: `src/main/fc/rc.c`, `src/main/fc/rc_modes.h`, `src/main/msp/msp_box.c`, `src/main/sensors/boardalignment.c`, `src/main/sensors/boardalignment.h`

---

## License
GNU General Public License v3.0