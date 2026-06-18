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

---

## 2. 믹서 설정 (Mixer)

### 2.1 Airplane Mixer
| 출력핀 | 기능 | 설명 |
|--------|------|------|
| S1, S2 | **모터** | Dshot 지원 |
| S3, S4 | 서보 0, 1 | 에일러론 자동 할당 |
| S5 | 서보 2 | 엘리베이터 할당 |
| S6 | 서보 3 | 러더 할당 |

### 2.2 Custom Airplane Mixer
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

## 3. GPS 레스큐 & 셔틀 랜딩

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

## 4. Bird Flap (새 날개짓)

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

## License
GNU General Public License v3.0