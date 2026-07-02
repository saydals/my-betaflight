# AUX 스위치를 이용한 비행 중 PID 프로파일 전환 기능 — 실행 계획서 v2

- **대상 저장소**: `github.com/saydals/my-betaflight` (`main` 브랜치)
- **관련 경로**: `src/main/fc/rc_adjustments.h`, `src/main/fc/rc_adjustments.c`, `src/main/config/config.c`, `src/main/osd/osd_elements.c`
- **검토 기준**: 2026-07-02, main 브랜치 실제 소스 확인 완료
- **현재 PG 버전**: `PG_ADJUSTMENT_RANGE_CONFIG, 2`
- **문서 버전**: 2.0

---

## 1. 프로젝트 개요

### 1.1 목적
라디오의 AUX 스위치만으로 비행 중에 PID 프로파일(1/2/3)을 즉시 전환할 수 있게 한다.

### 1.2 설계 철학
기존 Adjustments 시스템에 `ADJUSTMENT_PID_PROFILE`을 추가. `ADJUSTMENT_RATE_PROFILE`, `ADJUSTMENT_OSD_PROFILE`, `ADJUSTMENT_LED_PROFILE`과 동일한 `ADJUSTMENT_MODE_SELECT` 패턴을 따름. 기존 설정 인프라(CLI adjrange, Configurator Adjustments 탭)를 그대로 재사용.

### 1.3 기대 효과
- 비행 효율 극대화 (착륙-재설정-이륙 사이클 불필요)
- 안전성 향상 (불안정한 설정에서 즉시 복귀)
- 기존 프레임워크 활용 (코드 중복 최소화)
- 후방 호환성 유지 (기존 CLI/MSP 그대로 유지)


---

## 2. 현행 코드 분석 및 기존 계획서(v1) 대비 차이점

### 2.1 `adjustmentFunction_e` enum (rc_adjustments.h)

**현재 상태** (rc_adjustments.h lines 29-66):
```c
typedef enum {
    ADJUSTMENT_NONE = 0,
    // ... (중략) ...
    ADJUSTMENT_OSD_PROFILE,      // value = 33
    ADJUSTMENT_LED_PROFILE,       // value = 34
    ADJUSTMENT_FUNCTION_COUNT     // value = 35
} adjustmentFunction_e;
```

| 항목 | v1 계획서 가정 | 현재 코드 | 차이 |
|------|---------------|----------|------|
| LED_PROFILE 위치 | FUNCTION_COUNT 직전 | 동일 ✓ | 일치 |
| enum 순서 | OSD(33), LED(34), COUNT(35) | 동일 ✓ | 일치 |
| 추가할 위치 | LED 다음, COUNT 앞 | 빈자리 있음 ✓ | 가능 |

### 2.2 `defaultAdjustmentConfigs[]` (rc_adjustments.c lines 108-230)

- 배열: `static const adjustmentConfig_t defaultAdjustmentConfigs[ADJUSTMENT_FUNCTION_COUNT - 1]`
- 항목 수: 29개, 마지막: LED_PROFILE (SELECT, 3)
- **v1과 100% 일치. LED_PROFILE 뒤에 PID_PROFILE 항목 추가.**

### 2.3 `adjustmentLabels[]` (rc_adjustments.c lines 233-267)

- 총 33개 항목, 마지막: `"OSD PROFILE"`
- **"LED PROFILE" 누락됨** (인덱스 33, v1에서 발견한 기존 버그)
- **v1 기여**: LED PROFILE 누락 발견 및 보정 + PID PROFILE 추가

| 인덱스 | enum 값 | 현재 레이블 | 있어야 할 레이블 |
|--------|---------|------------|----------------|
| 32 | 33 (OSD_PROFILE) | `"OSD PROFILE"` | `"OSD PROFILE"` |
| 33 | 34 (LED_PROFILE) | **없음** | **`"LED PROFILE"`** |
| 34 | 35 (PID_PROFILE, 신규) | **없음** | **`"PID PROFILE"`** |

### 2.4 PG 버전 (line 68)

**현재**: `PG_ADJUSTMENT_RANGE_CONFIG, 2` → **v2 적용: version 3**

### 2.5 `applySelectAdjustment()` (rc_adjustments.c lines 609-656)

| 현재 case | 처리 | beeps? |
|----------|------|--------|
| RATE_PROFILE | `changeControlRateProfile(position)` | ✅ |
| PID_AUDIO | `pidAudioSetMode(newMode)` | ❌ |
| OSD_PROFILE | `changeOsdProfileIndex(position+1)` | ❌ |
| LED_PROFILE | `setLedProfile(position)` | ❌ |
| **PID_PROFILE(추가)** | `changePidProfile(position)` | **❌ (이중비프 방지)** |

**⚠ v1 대비 차이**: v1 작성 시점보다 `PID_AUDIO` case 1개 증가. beeps 패턴은 동일.

### 2.6 `updateOsdAdjustmentData()` 예외 목록 (rc_adjustments.c lines 696-701)

현재 예외: RATE_PROFILE, OSD_PROFILE
LED_PROFILE: 예외 아님 (기존)
**PID_PROFILE: 예외 추가 필요** (OSD_PIDRATE_PROFILE과 중복 방지)

---

## 3. 구현 상세: 변경할 코드 (Diff 형식)

6개 변경 지점. "찾을 텍스트 → 결과" 형식으로 AI 복붙 가능.

---

### 3.1 `rc_adjustments.h`: enum 항목 추가

**찾을 텍스트** (lines 62-66):
```c
    ADJUSTMENT_OSD_PROFILE,
    ADJUSTMENT_LED_PROFILE,
    ADJUSTMENT_FUNCTION_COUNT
} adjustmentFunction_e;
```

**결과**:
```c
    ADJUSTMENT_OSD_PROFILE,
    ADJUSTMENT_LED_PROFILE,
    ADJUSTMENT_PID_PROFILE,
    ADJUSTMENT_FUNCTION_COUNT
} adjustmentFunction_e;
```

**설명**: OSD=33, LED=34, **PID_PROFILE=35**, FUNCTION_COUNT=36.

---

### 3.2 `rc_adjustments.c`: `defaultAdjustmentConfigs[]` 항목 추가

**찾을 텍스트** (lines 226-230):
```c
        .adjustmentFunction = ADJUSTMENT_LED_PROFILE,
        .mode = ADJUSTMENT_MODE_SELECT,
        .data = { .switchPositions = 3 }
    }
};
```

**결과**:
```c
        .adjustmentFunction = ADJUSTMENT_LED_PROFILE,
        .mode = ADJUSTMENT_MODE_SELECT,
        .data = { .switchPositions = 3 }
    }, {
        .adjustmentFunction = ADJUSTMENT_PID_PROFILE,
        .mode = ADJUSTMENT_MODE_SELECT,
        .data = { .switchPositions = PID_PROFILE_COUNT }
    }
};
```

**설명**: `PID_PROFILE_COUNT`(보통 3) 매크로 사용.

---

### 3.3 `rc_adjustments.c`: `adjustmentLabels[]` 보정 및 추가

**찾을 텍스트** (lines 266-267):
```c
    "OSD PROFILE",
};
```

**결과**:
```c
    "OSD PROFILE",
    "LED PROFILE",
    "PID PROFILE",
};
```

**설명**: LED PROFILE(기존 누락 보정) + PID PROFILE(신규) 추가.

---

### 3.4 `rc_adjustments.c`: PG 버전 증가

**찾을 텍스트** (line 68):
```c
PG_REGISTER_ARRAY(adjustmentRange_t, MAX_ADJUSTMENT_RANGE_COUNT, adjustmentRanges, PG_ADJUSTMENT_RANGE_CONFIG, 2);
```

**결과**:
```c
PG_REGISTER_ARRAY(adjustmentRange_t, MAX_ADJUSTMENT_RANGE_COUNT, adjustmentRanges, PG_ADJUSTMENT_RANGE_CONFIG, 3);
```

---

### 3.5 `rc_adjustments.c`: `applySelectAdjustment()` case 추가

**찾을 텍스트** (lines 639-649):
```c
    case ADJUSTMENT_LED_PROFILE:
#ifdef USE_LED_STRIP
        if (getLedProfile() != position) {
            setLedProfile(position);
        }
#endif
        break;

    default:
        break;
```

**결과**:
```c
    case ADJUSTMENT_LED_PROFILE:
#ifdef USE_LED_STRIP
        if (getLedProfile() != position) {
            setLedProfile(position);
        }
#endif
        break;
    case ADJUSTMENT_PID_PROFILE:
        if (getCurrentPidProfileIndex() != position) {
            changePidProfile(position);
        }
        break;

    default:
        break;
```

**설명**: `changePidProfile()`이 내부 비프를 처리하므로 `beeps` 미설정(이중 비프 방지). `if (getCurrentPidProfileIndex() != position)` 조건으로 동일 위치 재진입 방지.

---

### 3.6 `rc_adjustments.c`: `updateOsdAdjustmentData()` 예외 목록 추가

**찾을 텍스트** (lines 696-701):
```c
    if (newValue != -1
        && adjustmentFunction != ADJUSTMENT_RATE_PROFILE  // Rate profile already has an OSD element
#ifdef USE_OSD_PROFILES
        && adjustmentFunction != ADJUSTMENT_OSD_PROFILE
#endif
        ) {
```

**결과**:
```c
    if (newValue != -1
        && adjustmentFunction != ADJUSTMENT_RATE_PROFILE  // Rate profile already has an OSD element
#ifdef USE_OSD_PROFILES
        && adjustmentFunction != ADJUSTMENT_OSD_PROFILE
#endif
        && adjustmentFunction != ADJUSTMENT_PID_PROFILE   // PID profile already has an OSD element (OSD_PIDRATE_PROFILE)
        ) {
```

**설명**: `ADJUSTMENT_PID_PROFILE`을 예외 목록에 추가하여 OSD Adjustments 오버레이가 `OSD_PIDRATE_PROFILE` 엘리먼트와 중복 표시되지 않도록 한다. `ADJUSTMENT_LED_PROFILE`은 전용 OSD 엘리먼트가 없으므로 예외 처리하지 않으며, 범용 OSD Adjustments 오버레이를 통해 사용자 피드백을 제공한다.


---

## 4. Phase별 마일스톤 (M1~M5)

| 단계 | 작업 내용 | 예상 공수 | 완료 조건 |
|------|-----------|-----------|-----------|
| **M1** | 핵심 기능 구현 (Phase 1~3) | 1~2시간 | ① enum 추가 ② 설정/레이블/PG/case 핸들러 추가 ③ 컴파일 성공 |
| **M2** | SITL 빌드 및 기본 동작 확인 | 1시간 | ① SITL 빌드 성공 ② adjrange CLI 설정 가능 ③ 3포지션 전환 확인 |
| **M3** | 실제 하드웨어 테스트 | 2~3시간 | ① FC 플래싱 ② 스위치 전환 확인 ③ 비프음 확인 ④ 이중비프 없음 |
| **M4** | OSD 표시 검증 | 1시간 | ① OSD 표시 활성화 ② 실시간 갱신 확인 ③ 오버레이 중복 없음 |
| **M5** | 회귀 테스트 및 문서화 | 1~2시간 | ① 기존 3개 프로파일 전환 정상 ② CLI profile 정상 ③ diff 정상 ④ 문서화 완료 |

**총 예상 공수**: 6~9시간 (숙련자 기준, 디버깅 포함)

---

## 5. 테스트 계획

### 5.1 단위 테스트 (UT-01~UT-05)

| ID | 항목 | 검증 내용 | 자동화 |
|----|------|---------|--------|
| UT-01 | enum 순서 | `ADJUSTMENT_PID_PROFILE`이 `LED_PROFILE` 다음, `FUNCTION_COUNT` 이전 | ✅ (static_assert) |
| UT-02 | 배열 크기 | `defaultAdjustmentConfigs` 크기 = `ADJUSTMENT_FUNCTION_COUNT - 1` | ✅ (컴파일 타임) |
| UT-03 | labels 배열 | `adjustmentLabels` 크기 = `ADJUSTMENT_FUNCTION_COUNT - 1` (USE_OSD 시) | ✅ (런타임 assert) |
| UT-04 | position 계산 | 3포지션 LOW=0, MID=1, HIGH=2 정확히 산출 | ✅ |
| UT-05 | 경계값 처리 | position 범위 초과 시 `changePidProfile()` 미호출 | ✅ |

### 5.2 통합 테스트 (IT-01~IT-10)

| ID | 항목 | 절차 | 기대 결과 |
|----|------|------|-----------|
| IT-01 | 3포지션 기본 전환 | LOW→MID→HIGH 순차 이동 | 프로파일 0→1→2, 비프 1→2→3회 |
| IT-02 | 동일 위치 재진입 | MID 유지 | `changePidProfile()` 재호출 없음 |
| IT-03 | HIGH→LOW 직접 전환 | HIGH→LOW 이동 | 프로파일 2→0, 비프 1회 |
| IT-04 | 비행 중 전환 | Armed 상태 스위치 전환 | 기체 안정적 유지 |
| IT-05 | CLI adjrange 설정 | `adjrange 0 0 4 900 2100 35 5 900 2100` + `save` | 저장 및 `diff` 확인 |
| IT-06 | EEPROM 영속성 | 프로파일 전환 후 재부팅 | 마지막 프로파일 유지 |
| IT-07 | PG 버전 마이그레이션 | v2 EEPROM → v3 펌웨어 부팅 | 기본값으로 리셋 |
| IT-08 | Rate Profile 독립성 | PID+Rate 동시 전환 | 충돌 없음 |
| IT-09 | 블랙박스 로깅 | 전환 후 로그 분석 | `ADJUSTMENT_PID_PROFILE`(35) 이벤트 기록 |
| IT-10 | OSD 표시 정합성 | OSD PIDRATE_PROFILE 활성화 후 전환 | 표시 프로파일 번호 일치 |

### 5.3 회귀 테스트

| 항목 | 검증 내용 | 우선순위 |
|------|---------|---------|
| CLI `profile` 명령어 | 후방 호환성 | **상** |
| Configurator `save`/`diff` | 설정 정상 감지 | **상** |
| 기존 adjrange (PID P/I/D step) | 충돌 없음 | **상** |
| `changePidProfileFromCellCount()` | AUX 수동 전환 우선 | **중** |
| OSD Adjustments 메뉴 | "UNKNOWN" 표시되어도 기능 정상 | **하** |

---

## 6. 위험 요소 및 대응 전략

### 6.1 Configurator 표시 호환성 (위험도: 낮음)

**문제**: Configurator가 `ADJUSTMENT_PID_PROFILE`(35)을 인식 못함 → "UNKNOWN" 표시.
**대응**: 표시 문제일 뿐 동작 정상. CLI `adjrange 0 0 4 900 2100 35 5 900 2100`로 우회 설정 가능. Configurator 업데이트는 별도 PR.

### 6.2 비행 중 PID 급변 불안정성 (위험도: 중간)

**문제**: 프로파일 간 P/I/D 차이로 전환 순간 기체 불안정 가능.
**대응**: `pidInit()`이 필터/I-term 등을 재초기화하여 이전 상태 영향 차단. 사용자에게 스로틀 낮추고 안전 고도에서 테스트 권장.

### 6.3 EEPROM 저장 정책 (위험도: 낮음)

**분석**: `setConfigDirtyIfNotPermanent()` → `configIsDirty`. RATE/OSD/LED 프로파일과 완전 동일. 별도 정책 불필요. 재부팅 시 마지막 프로파일 유지.

### 6.4 mixerInitProfile() 고정익 영향 (위험도: 중간, 고정익 한정)

**문제**: `changePidProfile()`이 `mixerInitProfile()` 호출, 고정익 서보 출력 점프 가능.
**대응**: 지상 테스트 필수. 고정익 사용자 프로파일 간 게인 차이 작게 유지 권장.

### 6.5 schedulerIgnoreTaskExecTime() (위험도: 낮음)

`changePidProfile()` 내장. 별도 조치 불필요.

---

## 7. 사용자 설정 가이드

### 7.1 CLI adjrange 설정

```
# 형식: adjrange <index> <auxCh> <startStep> <endStep> <func> <switchCh> <center> <scale>
# adjustmentFunction = 35 (ADJUSTMENT_PID_PROFILE)

adjrange 0 0 4 900 2100 35 5 900 2100
save
diff
```

| 파라미터 | 값 | 의미 |
|----------|-----|------|
| index | 0~29 | adjrange 슬롯 번호 |
| auxCh | 0 | 모니터링 AUX1 |
| range | 900~2100 | 풀레인지 |
| func | **35** | `ADJUSTMENT_PID_PROFILE` |
| switchCh | 5 | 3포지션 AUX5 |

### 7.2 OSD 표시 활성화

```
set osd_pidrate_profile_pos = 2450  # 원하는 OSD 위치
save
```

### 7.3 추천 스위치 설정

| 스위치 | 전환 | 권장 |
|--------|------|------|
| 3포지션 (AUX5/6) | LOW=PID1, MID=PID2, HIGH=PID3 | ✅ 강력 권장 |
| 2포지션 | LOW=PID1, HIGH=PID2 | ⚠ PID3 전환 불가 |
| 6포지션 | 중간 구간 중복 매핑 | ❌ 비권장 |

---

## 8. 참고 코드 위치 (빠른 참조)

| 항목 | 파일 | 행 번호 |
|------|------|---------|
| `adjustmentFunction_e` enum | `fc/rc_adjustments.h` | 29-66 |
| `defaultAdjustmentConfigs[]` | `fc/rc_adjustments.c` | 108-230 |
| `adjustmentLabels[]` | `fc/rc_adjustments.c` | 233-267 |
| `PG_REGISTER_ARRAY` | `fc/rc_adjustments.c` | 68 |
| `applySelectAdjustment()` | `fc/rc_adjustments.c` | 609-656 |
| `updateOsdAdjustmentData()` | `fc/rc_adjustments.c` | 691-711 |
| `changePidProfile()` | `config/config.c` | 868-883 |
| `PID_PROFILE_COUNT` | `flight/pid.h` | (`#define PID_PROFILE_COUNT 3`) |
| `PG_ADJUSTMENT_RANGE_CONFIG` ID | `pg/pg_ids.h` | 60 (ID 37) |

---

## 부록: 기존 계획서(v1) 대비 v2 변경 요약

| 항목 | v1 | v2 |
|------|----|----|
| enum 추가 위치 | LED_PROFILE 다음 | 동일 (현재 코드 확인) |
| defaultAdjustmentConfigs[] | SELECT, PID_PROFILE_COUNT | 동일 |
| adjustmentLabels[] | LED PROFILE 보정 + PID PROFILE | 동일 |
| PG 버전 | +1 (version 3) | 동일 (현재 2 확인) |
| applySelectAdjustment() case | changePidProfile(), beeps 미설정 | 동일 + getCurrentPidProfileIndex 조건 추가 |
| updateOsdAdjustmentData() 예외 | PID_PROFILE 추가 | **PID_PROFILE만 추가** (LED_PROFILE은 전용 OSD 요소가 없어 예외 불필요, 기존 피드백 유지) |
| Phase별 마일스톤 | 없음 | **신규**: M1~M5 (예상 공수 포함) |
| 단위 테스트 | 없음 | **신규**: UT-01~UT-05 |
| 통합 테스트 | 없음 | **신규**: IT-01~IT-10 |
| 회귀 테스트 | 없음 | **신규**: 6개 항목 + 우선순위 |
| 위험 요소 | 5가지 | **확장**: 5가지 + EEPROM 정책 + mixerInit + scheduler |
| CLI 설정 예시 | enum 이름 사용 | **구체화**: 실제 숫자 35 사용 |
| Configurator 우회 | "별도 작업"만 기재 | **구체화**: CLI 숫자 지정 방법 |
| "찾을 텍스트" diff | 없음 | **신규**: 6개 지점 모두 제공 |
| diff 라인 번호 | 추정치 | **정확함**: 실제 소스 기준 |

---

> **문서 버전**: v2.0 | **최종 검토일**: 2026-07-02 | **작성 기준**: main 브랜치 실제 소스



