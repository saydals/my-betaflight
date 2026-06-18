/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 *  [고정익 특화 GPS Rescue 및 셔틀 랜딩 시스템]
 *  1. 기체 구성: 에일러론(Roll), 엘리베이터(Pitch), 차동추력(Yaw/Throttle) 기반 고정익 지원.
 *  2. 셔틀 랜딩 (Shuttle Landing): 홈 포인트 상공에서 지정된 두 점(A-B)을 왕복하며 정밀 대기 및 단계적 하강 수행.
 *  3. 스마트 헤딩 래치 (Smart Heading Latch): 180도 배면 상황에서 센서 노이즈로 인한 좌우 요동(Hunting) 방지 히스테리시스 적용.
 *  4. PID 매핑: 사용 편의성을 위해 PID Profile 3의 값을 레스큐 제어 파라미터로 전용 매핑하여 실시간 튜닝 지원.
 *  5. CPA 터치 판정: 거리 미분을 통한 최근접점 통과 감지로 바람의 영향에도 정확한 타겟 전환 보장.
 *
 */

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "platform.h"

#ifdef USE_GPS_RESCUE

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/time.h"

#include "io/gps.h"

#include "config/config.h"
#include "fc/core.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/position.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"

#include "gps_rescue.h"

/* ================================================================
 * 상수 정의 (Magic Number 정리)
 * ================================================================ */

#define GPS_RESCUE_MAX_YAW_RATE              180  // 최대 yaw 속도 (도/초)

#define ATTAIN_ALT_TIMEOUT_US                3000000 // 상승 단계 타임아웃 (3초)

// GPS 속도 PI 제어 게인
#define VELOCITY_KP                          4.00f
#define VELOCITY_KI                          1.00f

// 고도 유지 피치 제어 상수
#define ALT_DEADBAND_M                       0.3f
#define ALT_I_LIMIT                                 20.0f  // 최대 20도까지 누적
#define ALT_ERR_LIMIT_SHUTTLE_M              10.0f // 셔틀 단계 오차 제한
#define ALT_ERR_LIMIT_FLYHOME_M              5.0f  // 귀환 단계 오차 제한

#define BANK_FF_MAX_DEG                      -36.0f // 뱅크각에 따른 피치 보상 최대치

// 헤딩 제어 히스테리시스 (뱅크턴 vs Yaw PI 전환점)
#define HEADING_HYST_LOW_DEG                 30.0f
#define HEADING_HYST_HIGH_DEG                45.0f
#define YAW_I_LIMIT                          (GPS_RESCUE_MAX_YAW_RATE * 0.3f)

#define BANK_BOOST_FLYHOME                   4.0f  // 귀환 단계 뱅크 쓰로틀 보상 계수

#define HEADING_LATCH_ON_DEG                 160.0f // 래치 활성화 임계값 (150~170도 권장)
#define HEADING_LATCH_OFF_DEG                90.0f  // 래치 해제 임계값 (정렬 완료 간주)

#define MAX_PITCH_SHUTTLE_DEG                15.0f // 셔틀 단계 최대 피치각
#define MAX_PITCH_FLYHOME_DEG                20.0f // 귀환 단계 최대 피치각 
#define MAX_ROLL_DEG                         75.0f

#define GPS_RESCUE_TOUCH_ACTIVATION_CM       2000.0f // 터치 판정(CPA) 감시 시작 거리 (20m)
#define GPS_RESCUE_TOUCH_PROXIMITY_CM        500.0f  // 즉시 터치 판정 근접 거리 (5m)



/* ================================================================
 * 열거형 / 구조체 정의
 * ================================================================ */

typedef enum {
    RESCUE_HEALTHY,
    RESCUE_FLYAWAY,
    RESCUE_GPSLOST,
    RESCUE_LOWSATS,
    RESCUE_CRASH_FLIP_DETECTED,
    RESCUE_STALLED,
    RESCUE_TOO_CLOSE,
    RESCUE_NO_HOME_POINT
} rescueFailureState_e;

typedef struct {
    float maxAltitudeCm;
    float returnAltitudeCm;
    float targetAltitudeCm;
    float targetLandingAltitudeCm;
    float targetVelocityCmS;
    float descentDistanceM;
    int8_t secondsFailing;
    float yawAttenuator;
    float disarmThreshold;
    uint32_t distanceToTargetCm;   // 타겟까지의 거리 (cm)
    int32_t  directionToTargetCd;  // 타겟 방향 (0.01도 단위)
} rescueIntent_s;

typedef struct {
    float currentAltitudeCm;
    float distanceToHomeCm;
    float distanceToHomeM;
    uint16_t groundSpeedCmS;
    int16_t directionToHome;
    float accMagnitude;
    bool healthy;
    float errorAngle;
    float gpsDataIntervalSeconds;
    float altitudeDataIntervalSeconds;
    float gpsRescueTaskIntervalSeconds;
    float velocityToHomeCmS;
    float absErrorAngle;
    float imuYawCogGain;
} rescueSensorData_s;

typedef struct {
    rescuePhase_e phase;
    rescueFailureState_e failure;
    rescueSensorData_s sensor;
    rescueIntent_s intent;
    bool isAvailable;
} rescueState_s;

/* ================================================================
 * 전역 변수 (Profile 3 튜닝 파라미터 매핑 포함)
 * ================================================================ */

static float       ascendPitch     = -30.0f; // 상승 단계 피치 (Profile3 Pitch P)
static float       midPitch        =   0.0f; // 기체 수평 기준 피치 (Profile3 Pitch I)
static float       landingPitch    = -10.0f; // 착륙 단계 피치 (Profile3 Pitch D)
static float       altHoldGain     =   2.5f; // 고도유지 피치 게인 (Profile3 Pitch F)
static float       bankGain        =   1.0f; // 헤딩 에러 뱅크각 게인 (Profile3 Roll P)
static float       bankPitchGain   =   0.3f; // 뱅크 시 피치 보정 게인 (Profile3 Roll I)

// 셔틀 전용 제어 변수 (Profile3 Yaw P/I 에서 할당)
static float       sbankGain       =   0.5f; // 셔틀용 뱅크각 게인
static float       sbankPitchGain  =   0.2f; // 셔틀용 피치 보정 게인
static float       bankYawGain     =   1.0f; // 조화선회 Yaw 게인 (Profile3 Roll D)
static float       shuttleDistance =  30.0f; // 가상 지점까지의 거리 (Profile3 Roll F)
static float       shuttleCount    =   0.0f; // 셔틀 왕복 횟수 (Profile3 Roll d_min)
static float       descentAlt      =   20.0f; // 셔틀하강 종료 및 홈하강 시작 고도 (Profile3 Yaw D)
static float       landingSpeed    = 100.0f; // 최종 착륙 시도 기체 속도 (Profile3 Pitch d_min) 1입력당 1m/s (100cm/s)
static float       landingAlt      =   1.0f; // 레스큐 최종 착륙 시도 고도 (Profile3 Yaw d_min), 하강단계에서 천천히 감속해서 도달 할 기체속도
static float       headingYawGain  =   1.0f; // 헤딩 추적 게인 (Profile3 Yaw F)

// static int8_t      shuttleDirection =    1; // 1: 우측, -1: 좌측
static gpsLocation_t rescuePointA;   // 하강고도 짝수 A포인트, 이륙방향 하강거리 위치에 생성되어 디스암때까지 레스큐 발동시 홈보다 rescuePointA를 먼저 터치해야함. 목적은 동일 활주로를 이착륙시 공용.
static gpsLocation_t shuttlePointA;  // 하강고도 홀수 A포인트 ( descentAlt 홀수일때 레스큐시 홈포인트로 귀환 중 하강거리 위치에 생성됨)
static gpsLocation_t shuttlePointB;  // 셔틀 반환점
static gpsLocation_t rescuePointC;   // 무한 셔틀 발동 위치 ( 무한셔틀시 C-B 를 왕복 )
static bool        takeoffVectorCaptured = false;
static bool        aPointValid = false;      // rescuePointA가 정상적으로 설정되었는지 여부
static bool        shuttleTargetB   = false; // 현재 목적지가 B(True)인지 A(False)인지 여부
static float       currentShuttleTrips = 0.0f; // 현재까지 완료한 왕복 횟수
static bool        shuttleInfinite  = false; // 무한 셔틀 모드 (AUX 스위치 연동)
static int32_t     currentVCLat     =    0; // OSD 표시용 현재 타겟 위도
static int32_t     currentVCLon     =    0; // OSD 표시용 현재 타겟 경도
static bool        shuttleHeadingToA = false; // B 통과 후 A로 돌아오는 중인지 여부 (하강 단계 탈출 조건)
static bool        descentAltReached = false; //  하강 고도 도달 여부 래치 (전 구간 감지용)
static int8_t      turnDirectionSign = 0;     // 0: 자유, 1: 우회전 고정, -1: 좌회전 고정
static bool        isDescentFalling = false;
static bool        descentFallAligned = false;


static float       rescueThrottle;
static float       rescueYaw;
static timeUs_t    attainAltStartTime = 0;
static bool        smoothedPitchNeedsReset = false; // 페이즈 전환 시 피치 LPF 강제 초기화 플래그

float              gpsRescueAngle[ANGLE_INDEX_COUNT] = { 0, 0 };
bool               magForceDisable = false;
static bool        newGPSData = false;

static pt2Filter_t throttleDLpf;
static pt1Filter_t velocityDLpf;
static pt3Filter_t velocityUpsampleLpf;

rescueState_s rescueState;

// 리팩토링으로 공통 함수화된 PID 적분항들
static float velocityIterm          = 0.0f;
static float altitudePitchIterm     = 0.0f;
static float yawHeadingIterm        = 0.0f; // Renamed from yawDescentIterm to match functionality
static float lastRescueYaw          = 0.0f;
static float prevDistanceToHomeCm   = 0.0f; // Moved to file scope for phase-transition reset
static float prevAltM               = 0.0f;
static bool  prevAltMInitialized    = false;

// CPA(Closest Point of Approach) 기반 터치 판정 변수
// - 타겟까지의 거리가 감소하다가 증가로 전환되는 순간을 최근접점 통과로 판정
// - 속도·뱅크각·바람 조건에 무관하게 기하학적으로 정확한 터치 검출
static float  cpaDistToTargetCm     = -1.0f; // 이전 프레임 타겟 거리 (-1: 미초기화)
static bool   cpaWasClosing         = false;  // 이전 프레임에서 거리가 감소 중이었는지
static float  abVecLat              = 0.0f;   // A→B 방향 단위 벡터 lat 성분 (initShuttlePoints에서 계산)
static float  abVecLon              = 0.0f;   // A→B 방향 단위 벡터 lon 성분

/* ================================================================
 * 내부 함수 전방 선언
 * ================================================================ */

static float convertPidToPitchDeg(float pidValue);
static void  initShuttlePoints(void);
static bool  isShuttlePhase(rescuePhase_e phase);
static void  rescueAttainPosition(void);
static void  performSanityChecks(void);
static void  sensorUpdate(void);
static bool  checkGPSRescueIsAvailable(void);
static void  setReturnAltitude(void);
static void  rescueStart(void);
static void  rescueStop(void);
void         disarmOnImpact(void);
void         initialiseRescueValues(void);
static uint16_t getRescueAuxValue(void);

// 리팩토링 추가 핸들러 및 핼퍼
static float calculateVelocityThrottle(void);
static float calculateAltitudePitch(float altErrM, bool isShuttlePhase, bool descentAllowed);

static void  handleShuttlePhase(void);
static void  handleShuttleDescentPhase(void);
static void  handleDescentPhase(void);
static void  handleLandingPhase(void);
static void  handleDoNothingPhase(void);
static float getSmartHeadingError(float currentError);
static void  updateRescueParams(void);

/* ================================================================
 * 초기화 및 기초 함수
 * ================================================================ */

void gpsRescueInit(void)
{
    // 시스템 초기화 단계에서 Profile 3 파라미터를 1회 로드
    updateRescueParams();

    rescueState.sensor.gpsRescueTaskIntervalSeconds = HZ_TO_INTERVAL(TASK_GPS_RESCUE_RATE_HZ);
    float cutoffHz, gain;

    // 쓰로틀 및 속도 필터 초기화
    cutoffHz = positionConfig()->altitude_d_lpf / 100.0f;
    gain = pt2FilterGain(cutoffHz, rescueState.sensor.gpsRescueTaskIntervalSeconds);
    pt2FilterInit(&throttleDLpf, gain);

    cutoffHz = gpsRescueConfig()->pitchCutoffHz / 100.0f;
    gain = pt1FilterGain(cutoffHz, rescueState.sensor.gpsRescueTaskIntervalSeconds);
    pt1FilterInit(&velocityDLpf, gain);

    cutoffHz *= 4.0f;
    gain = pt3FilterGain(cutoffHz, rescueState.sensor.gpsRescueTaskIntervalSeconds);
    pt3FilterInit(&velocityUpsampleLpf, gain);
}

void gpsRescueNewGpsData(void) { newGPSData = true; }
static void rescueStart(void)  { rescueState.phase = RESCUE_INITIALIZE; }
static void rescueStop(void)   { rescueState.phase = RESCUE_IDLE; }

// 셔틀 단계인지 판별
static bool isShuttlePhase(rescuePhase_e phase)
{
    return (phase == RESCUE_SHUTTLE          ||
            phase == RESCUE_SHUTTLE_INFINITE ||
            phase == RESCUE_SHUTTLE_DESCENT);
}

/**
 * PID 입력값(0~250)을 실제 피치 각도로 변환
 * 기준값 100 = 피치 0도, 100 미만 → 양수(하강), 100 초과 → 음수(상승)
 * 공식: pitchDeg = 100 - pidValue
 * 예) 80→+20, 90→+10, 100→0, 110→-10
 * 출력 제한: +60(하강) ~ -75(상승)
 */
static float convertPidToPitchDeg(float pidValue)
{
    pidValue = constrainf(pidValue, 0.0f, 250.0f);
    return constrainf(100.0f - pidValue, -75.0f, 60.0f);
}


/* ================================================================
 * 공통 제어 로직 (Helper Functions)
 * ================================================================ */

/**
 * 지상 속도 추종을 위한 쓰로틀 계산 (PI 제어)
 */
static float calculateVelocityThrottle(void)
{
    float targetVelocityBase = rescueState.intent.targetVelocityCmS;

    // 쓰로틀은 목표 속도 유지만 담당. 하강률은 calculateAltitudePitch가 담당.
    const float targetVelocity  = targetVelocityBase;
    const float currentVelocity = rescueState.sensor.groundSpeedCmS;
    const float velocityError   = targetVelocity - currentVelocity;
    const float dt              = rescueState.sensor.gpsRescueTaskIntervalSeconds;

    // 속도 오차 필터링 및 비례항 계산
    float filteredVelocityError = pt1FilterApply(&velocityDLpf, velocityError);
    float proportional = VELOCITY_KP * filteredVelocityError;

    // 적분항 업데이트 및 Anti-windup
    velocityIterm += VELOCITY_KI * velocityError * dt;
    const float throttleHover = gpsRescueConfig()->throttleHover;
    const float throttleMax   = gpsRescueConfig()->throttleMax;
    // [Fix] throttleHover > throttleMax - 150 인 경우 iLimit이 음수가 되어
    // constrainf의 min/max가 역전되는 버그 수정. fabsf로 절댓값 보장 후 최솟값 50 확보.
    const float iLimit        = fmaxf(fabsf((throttleMax - 150.0f) - throttleHover), 50.0f);
    velocityIterm = constrainf(velocityIterm, -iLimit, iLimit);

    // 뱅크 시 기체가 눕는 만큼 쓰로틀 보상 (수직 양력 유지)
    // 셔틀 단계에서는 → bankBoost 적용 안 함
    float bankBoost = 0.0f;
    if (rescueState.phase == RESCUE_FLY_HOME || rescueState.phase == RESCUE_ATTAIN_ALT) {
        bankBoost = fabsf(gpsRescueAngle[AI_ROLL] / 100.0f) * BANK_BOOST_FLYHOME;
    }

    float throttleCmd = throttleHover + proportional + velocityIterm + bankBoost;
    // 전체 단계 공통: throttleHover 기준에서 일정 치를 가감하여 쓰로틀 변화
    // float hoverMin = fmaxf(throttleHover - 150.0f, (float)(gpsRescueConfig()->throttleMin + 10));
    float hoverMin;
     if (rescueState.phase == RESCUE_DESCENT) {
         // 하강 단계에서는 throttleMin까지 감속 허용
         hoverMin = (float)(gpsRescueConfig()->throttleMin);
     } else {
         hoverMin = fmaxf(throttleHover - 150.0f, (float)(gpsRescueConfig()->throttleMin + 10));
     }
    float hoverMax = fminf(throttleHover + 250.0f, throttleMax - 100.0f);
    float throttleCmdConstrained = constrainf(throttleCmd, hoverMin, hoverMax);

    return pt2FilterApply(&throttleDLpf, throttleCmdConstrained);
}

/**
 * 고도 유지를 위한 피치 제어 (P + I + FF)
 * 고도 낮을 때 음수 피치(상승), 높을 때 양수 피치(하강) - 뱅크턴 중 음수 피치(기수 올림)만 사용
 */
static float calculateAltitudePitch(float altErrM, bool isShuttlePhase, bool descentAllowed)
{
    float errLimit = isShuttlePhase ? ALT_ERR_LIMIT_SHUTTLE_M : ALT_ERR_LIMIT_FLYHOME_M;
    altErrM = constrainf(altErrM, -errLimit, errLimit);

    // 1. P-term 계산 (반응성 강화를 위해 계수 확대 100 -> 200)
    float pTerm = altHoldGain * altErrM * 200.0f;

    // 2. D-term (Damping) 및 FF 계산
    const float dt = rescueState.sensor.gpsRescueTaskIntervalSeconds;
    float currentAltM = rescueState.sensor.currentAltitudeCm * 0.01f;
    if (!prevAltMInitialized) { prevAltM = currentAltM; prevAltMInitialized = true; }
    float climbRateM = (currentAltM - prevAltM) / fmaxf(dt, 0.01f);
    climbRateM = constrainf(climbRateM, -10.0f, 10.0f); // [Fix] GPS drop/scheduler jitter 시 spike 방지
    prevAltM = currentAltM;

    float damping = climbRateM * 100.0f; 
    float bankDeg = fabsf(attitude.values.roll / 10.0f);
    float currentBankPitchGain = isShuttlePhase ? sbankPitchGain : bankPitchGain;
    float bankFF = fmaxf(-(bankDeg * currentBankPitchGain / 2.0f), BANK_FF_MAX_DEG) * 100.0f;

    // 3. I-term 업데이트 (표준 대칭 적분)
    // accelZM 기반 감쇠 제거 — 고장난 기압계 환경의 임시 방편이었으므로 단순 적분으로 복원.
    // 뱅크턴(descentAllowed=false) 중 쌓인 I-term은 뱅크 종료 후 고도 회복에 활용됨.
    if (fabsf(altErrM) > ALT_DEADBAND_M) {
        altitudePitchIterm += altErrM * dt;
    }
    altitudePitchIterm = constrainf(altitudePitchIterm, -ALT_I_LIMIT, ALT_I_LIMIT);

    // 4. 출력 한계 결정 (뱅크턴 중 음수 피치(기수 올림)만 사용 시 Max를 0으로 설정)
    float pitchLimit = isShuttlePhase ? MAX_PITCH_SHUTTLE_DEG : MAX_PITCH_FLYHOME_DEG;
    float outputMin = -pitchLimit * 100.0f; // 상승 한계 (-25도)
    float outputMax = descentAllowed ? (pitchLimit * 100.0f) : 0.0f; // 하강 한계 (+25도 또는 0도)

    // 5. Back-Calculation Anti-Windup (P+I 기준, damping/bankFF 제외)
    // P+I 합이 출력 한계를 초과할 때만 I-term 역산 조정.
    // damping(climbRate)·bankFF는 feedforward/derivative 성격 → anti-windup 대상에서 제외.
    // → 이 값들의 순간 변화가 I-term을 킥하는 현상 방지.
    float piOutput = pTerm + (altitudePitchIterm * 100.0f);
    if (piOutput < outputMin) {
        if (pTerm > outputMin) {
            altitudePitchIterm = (outputMin - pTerm) / 100.0f;
        }
        piOutput = outputMin;
    } else if (piOutput > outputMax) {
        if (pTerm < outputMax) {
            altitudePitchIterm = (outputMax - pTerm) / 100.0f;
        }
        piOutput = outputMax;
    }
    altitudePitchIterm = constrainf(altitudePitchIterm, -ALT_I_LIMIT, ALT_I_LIMIT);

    // bankFF·damping은 P+I 클램핑 후 합산 → 최종 출력에서 constrainf로 재한계
    float rawPitch = piOutput + bankFF + damping;

// 6. 최종 출력 및 저주파 필터링
float rawPitchLimited = constrainf(rawPitch, outputMin, outputMax);
float targetFinalPitch = (midPitch * 100.0f) + rawPitchLimited;

static float smoothedPitch = 0.0f;
static bool smoothedPitchInitialized = false;

if (!smoothedPitchInitialized || smoothedPitchNeedsReset) {
    // Phase 전환 시 실제 피치 + midPitch trim으로 부드럽게 초기화
    smoothedPitch = (midPitch * 100.0f) + (attitude.values.pitch * 10.0f);
    smoothedPitch = constrainf(smoothedPitch, outputMin, outputMax);  // 안전 범위
    smoothedPitchInitialized = true;
    smoothedPitchNeedsReset = false;
}

// 2.0Hz LPF (기수 출렁임 방지)
const float cutoff = 2.0f;
const float alpha = (2.0f * M_PIf * dt * cutoff) / (2.0f * M_PIf * dt * cutoff + 1.0f);
smoothedPitch += alpha * (targetFinalPitch - smoothedPitch);   // += 로 간결하게

    return smoothedPitch;
}

/* ================================================================
 * 상세 비행 로직 핸들러 (Phase Specific Handlers)
 * ================================================================ */

/**
 * 셔틀 포인트 A, B 초기화
 * 홈 방향을 기준으로 좌/우 선회 방향을 결정하고 포인트 설정
 */
static void initShuttlePoints(void)
{
    // 포인트 A는 모드에 따라 결정
    if (shuttleInfinite) {
        rescuePointC.lat = gpsSol.llh.lat;
        rescuePointC.lon = gpsSol.llh.lon;
        shuttlePointA = rescuePointC;
    } else {
        shuttlePointA = rescuePointA;
    }

    // A에서 홈으로의 방향 계산 (B포인트를 A-홈 일직선상에 배치하기 위함)
    int32_t dLat = GPS_home[0] - shuttlePointA.lat;
    int32_t dLon = GPS_home[1] - shuttlePointA.lon;

    float latDegF = (float)shuttlePointA.lat * 1e-7f;
    float cosLat = fmaxf(fabsf(cosf(DEGREES_TO_RADIANS(latDegF))), 0.01f);

    float dLonMeter = (float)dLon * 111111.0f * cosLat / 1e7f;
    float dLatMeter = (float)dLat * 111111.0f / 1e7f;

    // A -> Home 방향 벡터의 각도 (라디안)
    float angleToHomeRad = atan2f(dLonMeter, dLatMeter);

    // 포인트 B는 포인트 A에서 홈 방향으로 shuttleDistance 만큼 떨어진 지점
    int32_t latOffset = (int32_t)(cosf(angleToHomeRad) * shuttleDistance / 111111.0f * 1e7f);
    int32_t lonOffset = (int32_t)(sinf(angleToHomeRad) * shuttleDistance / (111111.0f * cosLat) * 1e7f);

    shuttlePointB.lat = shuttlePointA.lat + latOffset;
    shuttlePointB.lon = shuttlePointA.lon + lonOffset;

    shuttleTargetB = true; // 먼저 B로 향함
    currentShuttleTrips = 0.0f;
    rescueState.intent.targetAltitudeCm = rescueState.sensor.currentAltitudeCm;

    // A→B 방향 단위 벡터 계산 (CPA 터치 판정 보조 — Along-Track 방향 기준)
    float dlat = (float)(shuttlePointB.lat - shuttlePointA.lat);
    float dlon = (float)(shuttlePointB.lon - shuttlePointA.lon);
    float mag  = sqrtf(dlat * dlat + dlon * dlon);
    if (mag > 0.1f) { abVecLat = dlat / mag; abVecLon = dlon / mag; }

    // CPA 터치 판정 변수 초기화 (새 셔틀 시작 시 이전 상태 제거)
    cpaDistToTargetCm = -1.0f;
    cpaWasClosing     = false;
    turnDirectionSign = 0;
}

/**
 * 셔틀 비행 로직 (A와 B를 왕복)
 * flyHome 로직을 기반으로 타겟 포인트만 스위칭함
 */
static void handleShuttleProgress(void)
{
    int32_t targetLat = shuttleTargetB ? shuttlePointB.lat : shuttlePointA.lat;
    int32_t targetLon = shuttleTargetB ? shuttlePointB.lon : shuttlePointA.lon;
    currentVCLat = targetLat;
    currentVCLon = targetLon;

    uint32_t distToTargetCm;
    int32_t  bearingToTargetCd;
    GPS_distance_cm_bearing(&gpsSol.llh.lat, &gpsSol.llh.lon, &targetLat, &targetLon, &distToTargetCm, &bearingToTargetCd);
    
    // OSD 연동용 데이터 업데이트
    rescueState.intent.distanceToTargetCm = distToTargetCm;
    rescueState.intent.directionToTargetCd = bearingToTargetCd;

    // GPS ground track 기반 헤딩 에러 계산
    // attitude.values.yaw(기수 방향) 대신 gpsSol.groundCourse(실제 이동 방향)를 사용하여
    // 바람에 의한 편류(크랩각)를 자동 보정 — 셔틀 속도 구간에서 groundCourse 신뢰도 높음
    float bearingToTarget  = (float)bearingToTargetCd / 100.0f;
    float currentTrackDeg  = gpsSol.groundCourse / 10.0f;   // decidegrees → degrees
    float error = currentTrackDeg - bearingToTarget;
    if (error > 180.0f) error -= 360.0f;
    else if (error <= -180.0f) error += 360.0f;
    // [Fix] bank 계산에도 getSmartHeadingError 적용: raw error 사용 시 180° 근처에서
    // +179 → -179 부호 반전이 발생하면 bank 방향이 순간적으로 뒤집히는 hunting 방지.
    // turnDirectionSign latch가 yaw에만 적용되고 bank 계산에는 누락되어 있던 버그.
    error = getSmartHeadingError(error);

    float absError = fabsf(error);
    rescueState.sensor.absErrorAngle = absError; // 다른 단계에서 고도 제어 구간 판정을 위해 공유

    // ----------------------------------------------------------------
    // 포인트 터치 판정 — CPA(Closest Point of Approach) 기반 거리 미분
    //
    // 설계 근거:
    //   타겟까지의 거리가 감소하다가 증가로 전환되는 순간 = 최근접점 통과
    //   정확 통과·오버슈트·바람 빗겨 통과 모든 경우를 커버한다.
    //
    //   touchedByCPA      : 거리 미분 부호 전환 (감소→증가), 15cm 히스테리시스
    //                       GPS 1Hz 노이즈 환경에서 5cm보다 안정적
    //                       + 셔틀거리 1.5배 이내에서만 인정 (원거리 오검출 방지)
    //   touchedByProximity: 3m 이내 도달 (직진 정확 통과 또는 CPA 미검출 폴백)
    //
    //   전역 변수 cpaDistToTargetCm/cpaWasClosing 사용:
    //   initShuttlePoints() 및 RESCUE_INITIALIZE에서 외부 리셋 가능
    // ----------------------------------------------------------------
    float dCm = (float)distToTargetCm;
    bool touchedByCPA = false;

    // CPA 판정 로직: 목표물 근처에서만 CPA 활성화
    float activationThresholdCm = GPS_RESCUE_TOUCH_ACTIVATION_CM; 

    if (dCm < activationThresholdCm) {
        if (cpaDistToTargetCm < 0.0f) {
            cpaDistToTargetCm = dCm;
            cpaWasClosing = true;
        } else {
            // 20cm 히스테리시스로 노이즈 내성 강화
            bool isClosing = (dCm < cpaDistToTargetCm - 20.0f);
            if (!isClosing && cpaWasClosing) {
                touchedByCPA = true; // 거리가 줄다가 늘기 시작 = 최근접점 통과 판정
            }
            cpaWasClosing = isClosing;
        }
        cpaDistToTargetCm = dCm;
    }
    
    // 근접 폴백 포함 터치 판정
    // 근접 터치 판정
    if (touchedByCPA || dCm < GPS_RESCUE_TOUCH_PROXIMITY_CM) {
        // [중요] 다음 타겟 비행을 위해 CPA 상태 완전 리셋 (오작동 방지용 큰 값 설정)
        cpaDistToTargetCm = 200000.0f; 
        cpaWasClosing     = true;
        yawHeadingIterm   = 0.0f;  // 타겟 전환 시 급격한 방향 전환으로 인한 I-term 킥(Kick) 방지
        turnDirectionSign = 0;

        if (shuttleTargetB) {
            shuttleTargetB = false; // B 도착 -> A로
            shuttleHeadingToA = true; // A로 향함 표시
        } else {
            shuttleTargetB = true;  // A 도착 (왕복 완료) -> 다시 B로
            currentShuttleTrips += 1.0f;
            // A 도착 시 하강 고도 조건 만족하면 즉시 DESCENT로 전환
            if (rescueState.phase == RESCUE_SHUTTLE_DESCENT && descentAltReached) {
                rescueState.phase = RESCUE_DESCENT;
                return; // 셔틀 진행 불필요
            }
        }
    }

    // 셔틀 진행 방향 추적 보완: B로 향하는 중이고 충분히 멀어지면 HeadingToA 해제
    if (shuttleTargetB && distToTargetCm < (shuttleDistance * 100.0f * 0.8f)) {
        shuttleHeadingToA = false;
    }

    // 셔틀 전용 sbankGain 사용
    float targetBankDeg = -(error * sbankGain);



    targetBankDeg = constrainf(targetBankDeg, -MAX_ROLL_DEG, MAX_ROLL_DEG);
    gpsRescueAngle[AI_ROLL] = targetBankDeg * 100.0f;

    // Yaw 제어
    if (absError < HEADING_HYST_LOW_DEG && turnDirectionSign == 0) {
        float errorBoost = constrainf(1.0f + (absError / HEADING_HYST_LOW_DEG), 1.0f, 2.0f);
        float yawP = error * gpsRescueConfig()->yawP * errorBoost * rescueState.intent.yawAttenuator / 10.0f;
        yawHeadingIterm += gpsRescueConfig()->yawP * 0.05f * error * rescueState.sensor.gpsRescueTaskIntervalSeconds;
        yawHeadingIterm = constrainf(yawHeadingIterm, -YAW_I_LIMIT, YAW_I_LIMIT);
        rescueYaw = (yawP + yawHeadingIterm) * headingYawGain;
    } else if (absError >= HEADING_HYST_HIGH_DEG || turnDirectionSign != 0) {
        yawHeadingIterm = 0.0f;
        rescueYaw = -(attitude.values.roll / 10.0f * bankYawGain * 3.0f);
    } else {
        yawHeadingIterm = 0.0f; rescueYaw = 0.0f;
    }
    rescueYaw = constrainf(rescueYaw, -GPS_RESCUE_MAX_YAW_RATE, GPS_RESCUE_MAX_YAW_RATE) * GET_DIRECTION(rcControlsConfig()->yaw_control_reversed);
}

static void handleShuttlePhase(void)
{
    handleShuttleProgress();
    float altErrM = (rescueState.sensor.currentAltitudeCm - rescueState.intent.targetAltitudeCm) * 0.01f;
    float absError = rescueState.sensor.absErrorAngle;
    float currentRollDeg = fabsf(attitude.values.roll / 10.0f);
    // 45도 이내 직선 구간이거나 기체가 수평(15도 이내)일 때만 양수 피치 허용
    bool descentAllowed = (absError < 45.0f) || (currentRollDeg < 15.0f);
    gpsRescueAngle[AI_PITCH] = calculateAltitudePitch(altErrM, true, descentAllowed);
    rescueThrottle = calculateVelocityThrottle();
}

static void handleShuttleDescentPhase(void)
{
    handleShuttleProgress();
    // 셔틀하강 고도 감소 (CLI 입력 하강률 적용)
    // 목표 고도를 descentAlt까지 낮춤 (하강 단계 진입 전까지 유지할 고도)
    float targetDescentAltCm = descentAlt * 100.0f;
    rescueState.intent.targetAltitudeCm = targetDescentAltCm;
    
    float altErrM = (rescueState.sensor.currentAltitudeCm - rescueState.intent.targetAltitudeCm) * 0.01f;
    float absError = rescueState.sensor.absErrorAngle;
    float currentRollDeg = fabsf(attitude.values.roll / 10.0f);
    bool descentAllowed = (absError < 45.0f) || (currentRollDeg < 10.0f);
    gpsRescueAngle[AI_PITCH] = calculateAltitudePitch(altErrM, true, descentAllowed);
    rescueThrottle = calculateVelocityThrottle();
}

static void handleDescentPhase(void)
{
    float currentAltM = rescueState.sensor.currentAltitudeCm * 0.01f;
    float distToHomeM = rescueState.sensor.distanceToHomeM;
    float landingAltM = landingAlt;

    // 공통 타겟 정보 업데이트
    currentVCLat = GPS_home[0];
    currentVCLon = GPS_home[1];
    uint32_t distToTargetCm;
    int32_t  bearingToTargetCd;
    GPS_distance_cm_bearing(&gpsSol.llh.lat, &gpsSol.llh.lon, &currentVCLat, &currentVCLon, &distToTargetCm, &bearingToTargetCd);
    rescueState.intent.distanceToTargetCm = distToTargetCm;
    rescueState.intent.directionToTargetCd = bearingToTargetCd;

    if (isDescentFalling) {
        if (currentAltM <= descentAlt) {
            isDescentFalling = false;
            descentFallAligned = false;
            smoothedPitchNeedsReset = true; // 급하강 종료 후 PID 제어 시 부드러운 전환을 위해 LPF 초기화

        } else {
            // 헤딩 에러 계산 (Fly home 방식)
            float currentYawDeg = (float)attitude.values.yaw / 10.0f;
            float bearingToTargetDeg = (float)bearingToTargetCd / 100.0f;
            float rawHeadingError = currentYawDeg - bearingToTargetDeg;
            if (rawHeadingError <= -180.0f) rawHeadingError += 360.0f;
            else if (rawHeadingError > 180.0f) rawHeadingError -= 360.0f;
            float headingError = getSmartHeadingError(rawHeadingError);
            float absError = fabsf(headingError);

            if (!descentFallAligned) {
                // [정렬 단계] 45도 이내로 들어올 때까지 Fly home 제어 사용
                float targetBankDeg = -(headingError * bankGain);
                targetBankDeg = constrainf(targetBankDeg, -MAX_ROLL_DEG, MAX_ROLL_DEG);
                gpsRescueAngle[AI_ROLL] = targetBankDeg * 100.0f;

                if (absError < HEADING_HYST_LOW_DEG) {
                    rescueYaw = (headingError * gpsRescueConfig()->yawP / 10.0f) * headingYawGain;
                } else {
                    rescueYaw = -(attitude.values.roll / 10.0f * bankYawGain * 3.0f);
                }
                rescueYaw = constrainf(rescueYaw, -GPS_RESCUE_MAX_YAW_RATE, GPS_RESCUE_MAX_YAW_RATE) * GET_DIRECTION(rcControlsConfig()->yaw_control_reversed);
                
                // 정렬 중에는 현재 타겟 고도 유지 또는 완만한 하강 (기존 calculateAltitudePitch 활용)
                float altErrM = (rescueState.sensor.currentAltitudeCm - rescueState.intent.targetAltitudeCm) * 0.01f;
                gpsRescueAngle[AI_PITCH] = calculateAltitudePitch(altErrM, false, true);
                rescueThrottle = calculateVelocityThrottle();

                if (absError <= 5.0f) {
                    descentFallAligned = true;
                }
                return;
            } else {


                // [급하강 단계] 정렬 완료 후 급하강 수행 ( 급하강 중 잠시 롤 요 = 0 )
                // descentAlt + 5m 이하로 내려오면 홈 추적 제어로 전환 (롤/요/쓰로틀 정상 동작)

                float maxPitch = (float)gpsRescueConfig()->maxRescueAngle;
                float altAboveDescent = currentAltM - descentAlt;
                float pitchDeg = 0.0f;

                // 계단식(stepwise) 피치각 결정: descentAlt; 기준

                if (altAboveDescent >= 15.0f) {
                    pitchDeg = maxPitch;
                } else if (altAboveDescent >= 10.0f) {
                    pitchDeg = 30.0f;
                } else if (altAboveDescent >= 8.0f) {
                    pitchDeg = 20.0f;
                } else if (altAboveDescent >= 6.0f) {
                    pitchDeg = 10.0f;
                } else if (altAboveDescent >= 4.0f) {
                    pitchDeg = 5.0f;
                } else {
                    pitchDeg = 0.0f;
                }

                // maxRescueAngle보다 더 가파르지 않도록 제한
                pitchDeg = constrainf(pitchDeg, 0.0f, maxPitch);
                gpsRescueAngle[AI_PITCH] = pitchDeg * 100.0f;

                // 조건: 현재 고도가 (descentAlt + 5m) 이하이면 홈 추적 제어 사용
                currentAltM = rescueState.sensor.currentAltitudeCm * 0.01f;
                if (currentAltM <= (descentAlt + 5.0f)) {

                // 홈 방향 추적 (롤/요/쓰로틀)
                float headingError = rescueState.sensor.errorAngle;
                float targetBankDeg = -(headingError * bankGain);
                targetBankDeg = constrainf(targetBankDeg, -MAX_ROLL_DEG, MAX_ROLL_DEG);
                gpsRescueAngle[AI_ROLL] = targetBankDeg * 100.0f;

                rescueYaw = (headingError * gpsRescueConfig()->yawP / 10.0f) * headingYawGain;
                rescueYaw = constrainf(rescueYaw, -GPS_RESCUE_MAX_YAW_RATE, GPS_RESCUE_MAX_YAW_RATE)
                            * GET_DIRECTION(rcControlsConfig()->yaw_control_reversed);

                rescueThrottle = calculateVelocityThrottle();

                } else {

                    // 고도가 높을 때는 기존 급하강 모드 (롤/요 0, 쓰로틀 최소)
                    gpsRescueAngle[AI_ROLL] = 0.0f;
                    rescueYaw = 0.0f;
                    rescueThrottle = (float)gpsRescueConfig()->throttleMin;
                }

                return;
            }
        }
    }

    // 정상 하강 단계 (헤딩 정렬이 끝났거나 급하강이 필요 없는 경우)
    float headingError = rescueState.sensor.errorAngle;
    float targetBankDeg = -(headingError * bankGain);
    targetBankDeg = constrainf(targetBankDeg, -MAX_ROLL_DEG, MAX_ROLL_DEG);
    gpsRescueAngle[AI_ROLL] = targetBankDeg * 100.0f;
    
    rescueYaw = (headingError * gpsRescueConfig()->yawP / 10.0f) * headingYawGain;
    rescueYaw = constrainf(rescueYaw, -GPS_RESCUE_MAX_YAW_RATE, GPS_RESCUE_MAX_YAW_RATE) * GET_DIRECTION(rcControlsConfig()->yaw_control_reversed);

    // 하강률 자동 조절: 홈 30m 지점에서 landingAlt에 도달하도록 하강률 계산
    float distTo30m = fmaxf(1.0f, distToHomeM - 30.0f);
    float altDiffM = currentAltM - landingAltM;
    float groundSpeedMps = fmaxf(1.0f, (float)rescueState.sensor.groundSpeedCmS * 0.01f);
    
    float requiredDescendRateMps = (altDiffM * groundSpeedMps) / distTo30m;
    
    rescueState.intent.targetAltitudeCm -= (requiredDescendRateMps * 100.0f) * rescueState.sensor.altitudeDataIntervalSeconds;
    rescueState.intent.targetAltitudeCm = fmaxf(rescueState.intent.targetAltitudeCm, landingAlt * 100.0f);

    float altErrM = (rescueState.sensor.currentAltitudeCm - rescueState.intent.targetAltitudeCm) * 0.01f;
    gpsRescueAngle[AI_PITCH] = calculateAltitudePitch(altErrM, false, true);

    float startDist = rescueState.intent.descentDistanceM;
    float distRange = fmaxf(1.0f, startDist - 30.0f);
    float progress = constrainf((startDist - distToHomeM) / distRange, 0.0f, 1.0f);
    
    float startVel = (float)gpsRescueConfig()->groundSpeedCmS;
    float endVel = landingSpeed;
    rescueState.intent.targetVelocityCmS = startVel - progress * (startVel - endVel);

    rescueThrottle = calculateVelocityThrottle();
}

/**
 * 단계별 상세 동작 처리
 */
static void handleAttainAltPhase(void)
{
    if (aPointValid) {
        currentVCLat = rescuePointA.lat;
        currentVCLon = rescuePointA.lon;
    } else {
        currentVCLat = GPS_home[0];
        currentVCLon = GPS_home[1];
    }
    GPS_distance_cm_bearing(&gpsSol.llh.lat, &gpsSol.llh.lon, &currentVCLat, &currentVCLon, 
                            &rescueState.intent.distanceToTargetCm, &rescueState.intent.directionToTargetCd);

    gpsRescueAngle[AI_PITCH] = ascendPitch * 100.0f;
    rescueYaw = 0.0f; gpsRescueAngle[AI_ROLL] = 0.0f;
    rescueThrottle = calculateVelocityThrottle();
}

static void handleFlyHomePhase(void)
{
    // A포인트가 아직 생성되지 않았으면 홈포인트를 타겟으로 비행
    if (aPointValid) {
        currentVCLat = rescuePointA.lat;
        currentVCLon = rescuePointA.lon;
    } else {
        currentVCLat = GPS_home[0];
        currentVCLon = GPS_home[1];
    }

    uint32_t distToTargetCm;
    int32_t  bearingToTargetCd;
    GPS_distance_cm_bearing(&gpsSol.llh.lat, &gpsSol.llh.lon, &currentVCLat, &currentVCLon, &distToTargetCm, &bearingToTargetCd);

    // OSD 연동용 데이터 업데이트
    rescueState.intent.distanceToTargetCm = distToTargetCm;
    rescueState.intent.directionToTargetCd = bearingToTargetCd;

    float currentYawDeg = (float)attitude.values.yaw / 10.0f;
    float bearingToTargetDeg = (float)bearingToTargetCd / 100.0f;
    float rawHeadingError = currentYawDeg - bearingToTargetDeg;
    if (rawHeadingError <= -180.0f) rawHeadingError += 360.0f;
    else if (rawHeadingError > 180.0f) rawHeadingError -= 360.0f;

    float headingError = getSmartHeadingError(rawHeadingError);
    float absError = fabsf(headingError);

    // 뱅크턴: 헤딩 오차에 비례하여 기체를 눕힘 (bankGain 0.01 적용됨)
    float targetBankDeg = -(headingError * bankGain);
    targetBankDeg = constrainf(targetBankDeg, -75.0f, 75.0f);
    gpsRescueAngle[AI_ROLL] = targetBankDeg * 100.0f;

    // Yaw 제어: 미세 헤딩 보정(PI) 또는 큰 헤딩 시 조화 선회 보조
    if (absError < HEADING_HYST_LOW_DEG && turnDirectionSign == 0) {
        float errorBoost = constrainf(1.0f + (absError / HEADING_HYST_LOW_DEG), 1.0f, 2.0f);
        float yawP = headingError * gpsRescueConfig()->yawP * errorBoost * rescueState.intent.yawAttenuator / 10.0f;
        yawHeadingIterm += gpsRescueConfig()->yawP * 0.05f * headingError * rescueState.sensor.gpsRescueTaskIntervalSeconds;
        yawHeadingIterm = constrainf(yawHeadingIterm, -YAW_I_LIMIT, YAW_I_LIMIT);
        rescueYaw = (yawP + yawHeadingIterm) * headingYawGain;
    } else if (absError >= HEADING_HYST_HIGH_DEG || turnDirectionSign != 0) {
        yawHeadingIterm = 0.0f;
        // 조화선회 보조
        rescueYaw = -(attitude.values.roll / 10.0f * bankYawGain * 3.0f);
    } else {
        yawHeadingIterm = 0.0f; rescueYaw = 0.0f;
    }
    rescueYaw = constrainf(rescueYaw, -GPS_RESCUE_MAX_YAW_RATE, GPS_RESCUE_MAX_YAW_RATE) * GET_DIRECTION(rcControlsConfig()->yaw_control_reversed);

    float altErrM = (rescueState.sensor.currentAltitudeCm - rescueState.intent.targetAltitudeCm) * 0.01f;
    float currentRollDeg = fabsf(attitude.values.roll / 10.0f);
    // 직선 구간(45도 이내)이거나 기체가 수평(15도 이내)일 때 양수 피치 허용하여 안전 확보
    bool descentAllowed = (absError < 45.0f) || (currentRollDeg < 15.0f);
    gpsRescueAngle[AI_PITCH] = calculateAltitudePitch(altErrM, false, descentAllowed);
    rescueThrottle = calculateVelocityThrottle();
}



static void handleLandingPhase(void)
{
    currentVCLat = GPS_home[0];
    currentVCLon = GPS_home[1];
    GPS_distance_cm_bearing(&gpsSol.llh.lat, &gpsSol.llh.lon, &currentVCLat, &currentVCLon, 
                            &rescueState.intent.distanceToTargetCm, &rescueState.intent.directionToTargetCd);
    gpsRescueAngle[AI_PITCH] = landingPitch * 100.0f;
    gpsRescueAngle[AI_ROLL]  = 0.0f;
    rescueYaw = 0.0f;

    // 랜딩 진입 후 3초간 throttleMin 유지 → 이후 PWM 1000으로 모터 정지
    // attainAltStartTime 재활용: ATTAIN_ALT→FLY_HOME 전환 시 이미 0으로 초기화됨
    // PWM_RANGE_MIN(1000)은 디스암이 아니므로 필요 시 사용자가 쓰로틀 올릴 수 있음
    if (attainAltStartTime == 0) attainAltStartTime = micros();
    if (cmpTimeUs(micros(), attainAltStartTime) >= ATTAIN_ALT_TIMEOUT_US) {
        rescueThrottle = PWM_RANGE_MIN;  // 1000 PWM — 모터 정지 (디스암 아님)
    } else {
        rescueThrottle = (float)gpsRescueConfig()->throttleMin;
    }
}

static void handleDoNothingPhase(void)
{
    currentVCLat = GPS_home[0];
    currentVCLon = GPS_home[1];
    GPS_distance_cm_bearing(&gpsSol.llh.lat, &gpsSol.llh.lon, &currentVCLat, &currentVCLon, 
                            &rescueState.intent.distanceToTargetCm, &rescueState.intent.directionToTargetCd);
    // 고정익 나선 강하 방지를 위해 Roll 0 유지 및 착륙 피치 적용
    gpsRescueAngle[AI_PITCH] = landingPitch * 100.0f;
    gpsRescueAngle[AI_ROLL]  = 0.0f;
    rescueThrottle = gpsRescueConfig()->throttleMin;
}

/* ================================================================
 * 메인 제어 Dispatcher
 * ================================================================ */

static void rescueAttainPosition(void)
{
    switch (rescueState.phase) {
        case RESCUE_IDLE:
            gpsRescueAngle[AI_PITCH] = 0.0f; gpsRescueAngle[AI_ROLL] = 0.0f;
            rescueThrottle = rcCommand[THROTTLE];
            lastRescueYaw = 0.0f;
            return;
        case RESCUE_INITIALIZE:
            velocityIterm = 0.0f; altitudePitchIterm = 0.0f; yawHeadingIterm = 0.0f;
            shuttleInfinite = false;
            currentShuttleTrips = 0.0f; shuttleTargetB = false; attainAltStartTime = 0;
            cpaDistToTargetCm = -1.0f;  // CPA 터치 판정 상태 초기화
            cpaWasClosing     = false;
            shuttleHeadingToA = false;
            descentAltReached = false;  // [Fix] 레스큐 초기화 시 고도 래치 리셋
            turnDirectionSign = 0;
            // 새 A포인트를 생성하지 않고 낡은 좌표로 비행하는 버그가 발생함.
            if ((int)descentAlt % 2 != 0) {
                aPointValid = false; 
            }
            rescueState.intent.disarmThreshold = gpsRescueConfig()->disarmThreshold * 0.1f;
            rescueState.sensor.imuYawCogGain = 1.0f;
            gpsRescueAngle[AI_PITCH] = attitude.values.pitch / 10.0f * 100.0f;
            lastRescueYaw = 0.0f;
            return;
        case RESCUE_DO_NOTHING: handleDoNothingPhase(); break;
        case RESCUE_ATTAIN_ALT: handleAttainAltPhase(); break;
        case RESCUE_FLY_HOME:   handleFlyHomePhase();   break;
        case RESCUE_SHUTTLE:
        case RESCUE_SHUTTLE_INFINITE:
                                handleShuttlePhase();   break;
        case RESCUE_SHUTTLE_DESCENT: handleShuttleDescentPhase(); break;
        case RESCUE_DESCENT:    handleDescentPhase();   break;
        case RESCUE_LANDING:    handleLandingPhase();   break;
        default: break;
    }

    // Yaw 명령에 저역통과필터 추가 - 모든 활성 Phase에 공통 적용
    // (IDLE/INITIALIZE는 위에서 return하므로 여기에 도달하는 모든 Phase에 LPF 적용)
    rescueYaw = lastRescueYaw * 0.7f + rescueYaw * 0.3f;
    lastRescueYaw = rescueYaw;
}

/* ================================================================
 * 센서 및 안전 진단 (Original Logic)
 * ================================================================ */

static void performSanityChecks(void)
{
    static timeUs_t previousTimeUs = 0;
    static int8_t secondsLowSats = 0;
    const timeUs_t currentTimeUs = micros();

    if (rescueState.phase == RESCUE_IDLE) {
        rescueState.failure = RESCUE_HEALTHY; return;
    } else if (rescueState.phase == RESCUE_INITIALIZE) {
        previousTimeUs = currentTimeUs; prevDistanceToHomeCm = rescueState.sensor.distanceToHomeCm; secondsLowSats = 0;
    }

    if (rescueState.failure != RESCUE_HEALTHY || crashRecoveryModeActive()) {
        rescueState.phase = RESCUE_DO_NOTHING;
    }

    if (!rescueState.sensor.healthy) rescueState.failure = RESCUE_GPSLOST;

    const timeDelta_t dTime = cmpTimeUs(currentTimeUs, previousTimeUs);
    if (dTime < 1000000) return;
    previousTimeUs = currentTimeUs;

    // 귀환 중 홈과의 거리가 좁혀지지 않으면 실패로 간주
    if (rescueState.phase == RESCUE_FLY_HOME) {
        const float velocityToHomeCmS = rescueState.sensor.velocityToHomeCmS;
        rescueState.intent.secondsFailing += (velocityToHomeCmS < 0.1f * rescueState.intent.targetVelocityCmS) ? 1 : -1;
        rescueState.intent.secondsFailing = constrain(rescueState.intent.secondsFailing, 0, 30);
        if (rescueState.intent.secondsFailing >= 30) {
#ifdef USE_MAG
            if (sensors(SENSOR_MAG) && gpsRescueConfig()->useMag && !magForceDisable) {
                magForceDisable = true; rescueState.intent.secondsFailing = 0;
            } else
#endif
            { rescueState.failure = RESCUE_FLYAWAY; }
        }
    }

    secondsLowSats += (!STATE(GPS_FIX) || (gpsSol.numSat < GPS_MIN_SAT_COUNT)) ? 1 : -1;
    secondsLowSats = constrain(secondsLowSats, 0, 10);
    if (secondsLowSats == 10) rescueState.failure = RESCUE_LOWSATS;
}

/**
 * 스마트 헤딩 에러 계산 함수
 * 기체가 180도 부근에서 헤딩 에러 부호 급변으로 진동하는 것을 방지 (Hysteresis Latch)
 */
static float getSmartHeadingError(float currentError)
{
    float absError = fabsf(currentError);

    // 1. 래치 진입: 에러가 매우 클 때 현재 방향으로 선회 고정
    if (turnDirectionSign == 0 && absError > HEADING_LATCH_ON_DEG) {
        turnDirectionSign = (currentError > 0) ? 1 : -1;
    }

    // 2. 래치 해제: 목표에 충분히 근접하면 정밀 제어 모드로 복구
    if (turnDirectionSign != 0 && absError < HEADING_LATCH_OFF_DEG) {
        turnDirectionSign = 0;
    }

    // 3. 래치 활성화 중 에러 보정 (핵심 로직)
    if (turnDirectionSign != 0) {
        // 부호가 반대로 튀었을 경우 360도 보정하여 방향 유지
        if (turnDirectionSign == 1 && currentError < 0) {
            return currentError + 360.0f; // 실제 -175도 -> 보정 +185도
        } else if (turnDirectionSign == -1 && currentError > 0) {
            return currentError - 360.0f; // 실제 +175도 -> 보정 -185도
        }
    }

    return currentError;
}

static void sensorUpdate(void)
{
    const timeUs_t  currentTimeUs = micros();
    static timeUs_t previousAltitudeDataTimeUs = 0;

    const timeDelta_t altitudeDataIntervalUs = cmpTimeUs(currentTimeUs, previousAltitudeDataTimeUs);
    rescueState.sensor.altitudeDataIntervalSeconds = altitudeDataIntervalUs * 0.000001f;
    previousAltitudeDataTimeUs = currentTimeUs;

    rescueState.sensor.currentAltitudeCm = getAltitude();
    rescueState.sensor.healthy = gpsIsHealthy();

    if (rescueState.phase == RESCUE_LANDING) {
        rescueState.sensor.accMagnitude = (float) sqrtf(sq(acc.accADC[Z] - acc.dev.acc_1G) + sq(acc.accADC[X]) + sq(acc.accADC[Y])) * acc.dev.acc_1G_rec;
    }

    rescueState.sensor.directionToHome = GPS_directionToHome;
    float rawHeadingError = (attitude.values.yaw - rescueState.sensor.directionToHome) / 10.0f;
    if (rawHeadingError <= -180) rawHeadingError += 360;
    else if (rawHeadingError > 180) rawHeadingError -= 360;

    rescueState.sensor.errorAngle = getSmartHeadingError(rawHeadingError);
    rescueState.sensor.absErrorAngle = fabsf(rescueState.sensor.errorAngle);

    // GPS 갱신 주기보다 빠른 제어 루프를 위해 속도 업샘플링 필터 적용
    rescueState.sensor.groundSpeedCmS = (uint16_t)pt3FilterApply(&velocityUpsampleLpf, (float)gpsSol.groundSpeed);

    if (!newGPSData) return;

    rescueState.sensor.distanceToHomeCm  = GPS_distanceToHomeCm;
    rescueState.sensor.distanceToHomeM   = rescueState.sensor.distanceToHomeCm / 100.0f;
    rescueState.sensor.gpsDataIntervalSeconds = getGpsDataIntervalSeconds();
    rescueState.sensor.velocityToHomeCmS = ((prevDistanceToHomeCm - rescueState.sensor.distanceToHomeCm) / rescueState.sensor.gpsDataIntervalSeconds);
    prevDistanceToHomeCm = rescueState.sensor.distanceToHomeCm;

    if (gpsRescueConfig()->groundSpeedCmS) {
        const float rescueGroundspeed = (float)gpsRescueConfig()->groundSpeedCmS;
        // 지상 속도와 홈 방향 속도 성분의 차이를 통해 바람의 영향을 계산
        const float groundspeedErrorRatio = fabsf(rescueState.sensor.groundSpeedCmS - rescueState.sensor.velocityToHomeCmS) / rescueGroundspeed;
        // 기수가 들린 정도(피치)에 따른 보정치 계산
        const float pitchForwardAngle = (gpsRescueAngle[AI_PITCH] > 0.0f) ? fminf(gpsRescueAngle[AI_PITCH] / 3000.0f, 2.0f) : 0.0f;
        // IMU Yaw를 COG(Course Over Ground)처럼 사용하기 위한 가변 게인 (바람 편류 보정용)
        rescueState.sensor.imuYawCogGain = (rescueState.phase != RESCUE_FLY_HOME && rescueState.phase != RESCUE_DESCENT) ? pitchForwardAngle : pitchForwardAngle + fminf(groundspeedErrorRatio, 3.5f);
    }


// 이륙시 이륙방향 벡터를 기반으로 하강거리 위치에 A포인트를 생성해 귀환 경로를 고정하는 코드.


 if (ARMING_FLAG(ARMED) && !takeoffVectorCaptured && STATE(GPS_FIX_HOME) && rescueState.sensor.distanceToHomeM >= 20.0f) {
    if (((int)descentAlt % 2 == 0)) {
        float distToHomeM = rescueState.sensor.distanceToHomeM;

        // 🚀 GPS 노이즈 판별 (미터 기준)
        bool isNoise = (distToHomeM < 20.0f || distToHomeM > 100.0f); 

        if (!isNoise) {
            // --- 유효한 A포인트 계산 로직 ---
            int32_t dLat = gpsSol.llh.lat - GPS_home[0];
            int32_t dLon = gpsSol.llh.lon - GPS_home[1];
            float distanceM = fmaxf(gpsRescueConfig()->descentDistanceM, 10.0f);
            float latDegF = (float)GPS_home[0] * 1e-7f;
            float cosLat = fmaxf(fabsf(cosf(DEGREES_TO_RADIANS(latDegF))), 0.01f);

            float dLonMeter = (float)dLon * 111111.0f * cosLat / 1e7f;
            float dLatMeter = (float)dLat * 111111.0f / 1e7f;
            float angleRad = atan2f(dLonMeter, dLatMeter); // 홈 → 현재 위치 방향

            int32_t latOffset = (int32_t)(cosf(angleRad) * distanceM / 111111.0f * 1e7f);
            int32_t lonOffset = (int32_t)(sinf(angleRad) * distanceM / (111111.0f * cosLat) * 1e7f);

            rescuePointA.lat = GPS_home[0] + latOffset;
            rescuePointA.lon = GPS_home[1] + lonOffset;
            aPointValid = true;
            takeoffVectorCaptured = true; // 성공 시에만 플래그 설정 ( 맞는지 확인필요 )
         }
       }  // else: 노이즈 발생 시 아무것도 하지 않고, 다음 프레임에서 재시도
    }
 }



static bool checkGPSRescueIsAvailable(void)
{
    static timeUs_t previousTimeUs = 0;
    static int8_t secondsLowSats = 0;
    static bool lowsats = false, noGPSfix = false;
    const timeUs_t currentTimeUs = micros();

    if (!gpsIsHealthy() || !STATE(GPS_FIX_HOME)) return false;
    if (cmpTimeUs(currentTimeUs, previousTimeUs) < 1000000) return !(noGPSfix || lowsats);
    previousTimeUs = currentTimeUs;

    noGPSfix = !STATE(GPS_FIX);
    secondsLowSats = constrain(secondsLowSats + ((gpsSol.numSat < GPS_MIN_SAT_COUNT) ? 1 : -1), 0, 2);
    lowsats = (secondsLowSats == 2);
    return !(noGPSfix || lowsats);
}

static void setReturnAltitude(void)
{
    if (!ARMING_FLAG(ARMED)) {
        takeoffVectorCaptured = false;
        aPointValid = false;
        if (!gpsConfig()->gps_set_home_point_once) {
            rescueState.intent.maxAltitudeCm = 0.0f;
        }
        return;
    }
    rescueState.intent.maxAltitudeCm = fmaxf(rescueState.sensor.currentAltitudeCm, rescueState.intent.maxAltitudeCm);

    if (newGPSData) {
        rescueState.intent.targetAltitudeCm = rescueState.sensor.currentAltitudeCm;
        rescueState.intent.descentDistanceM = gpsRescueConfig()->descentDistanceM;
        const float initialClimbCm = gpsRescueConfig()->initialClimbM * 100.0f;
        switch (gpsRescueConfig()->altitudeMode) {
            case GPS_RESCUE_ALT_MODE_FIXED: rescueState.intent.returnAltitudeCm = gpsRescueConfig()->returnAltitudeM * 100.0f; break;
            case GPS_RESCUE_ALT_MODE_CURRENT: rescueState.intent.returnAltitudeCm = fmaxf(initialClimbCm, rescueState.sensor.currentAltitudeCm + initialClimbCm); break;
            default: rescueState.intent.returnAltitudeCm = rescueState.intent.maxAltitudeCm + initialClimbCm; break;
        }
    }
}

void disarmOnImpact(void)
{
    if (rescueState.sensor.accMagnitude > rescueState.intent.disarmThreshold) {
        setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
        disarm(DISARM_REASON_GPS_RESCUE);
        rescueStop();
    }
}

void initialiseRescueValues(void)
{
    rescueState.intent.secondsFailing = 0; rescueState.intent.yawAttenuator = 0.0f;
    rescueState.intent.targetVelocityCmS = (float)gpsRescueConfig()->groundSpeedCmS;
}

/**
 * PID Profile 3에서 레스큐 제어 파라미터를 통합 로드
 * 아밍 시점에 1회 호출되어 모든 전역 변수를 업데이트함
 */
static void updateRescueParams(void)
{
    pidProfile_t *profile3 = pidProfilesMutable(2);

    bankGain        = (float)profile3->pid[PID_ROLL].P * 0.01f;
    bankPitchGain   = (float)profile3->pid[PID_ROLL].I / 100.0f;
    sbankGain       = (float)profile3->pid[PID_YAW].P * 0.01f;
    sbankPitchGain  = (float)profile3->pid[PID_YAW].I / 100.0f;
    bankYawGain     = constrainf((float)profile3->pid[PID_ROLL].D / 50.0f, 0.0f, 5.0f);
    shuttleCount    = (float)profile3->d_min[PID_ROLL];
    shuttleDistance = (float)profile3->pid[PID_ROLL].F;

    ascendPitch     = convertPidToPitchDeg(profile3->pid[PID_PITCH].P);
    midPitch        = convertPidToPitchDeg(profile3->pid[PID_PITCH].I);
    landingPitch    = convertPidToPitchDeg(profile3->pid[PID_PITCH].D);
    altHoldGain     = constrainf((float)profile3->pid[PID_PITCH].F / 10.0f, 0.0f, 25.0f);

    descentAlt      = (float)profile3->pid[PID_YAW].D;
    landingAlt      = (float)profile3->d_min[PID_YAW];
    headingYawGain  = constrainf((float)profile3->pid[PID_YAW].F / 100.0f, 0.0f, 2.5f);
    landingSpeed    = (float)profile3->d_min[PID_PITCH] * 100.0f; // 1 input = 1m/s = 100cm/s

    rescueState.intent.targetLandingAltitudeCm = 100.0f * landingAlt;
}

static uint16_t getRescueAuxValue(void)
{
    for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        const modeActivationCondition_t *mac = modeActivationConditions(i);
        if (mac->modeId == BOXGPSRESCUE) return rcData[mac->auxChannelIndex + NON_AUX_CHANNEL_COUNT];
    }
    return 0;
}

/* ================================================================
 * 메인 업데이트 루프 (State Machine Control)
 * ================================================================ */

void gpsRescueUpdate(void)
{
    if (!FLIGHT_MODE(GPS_RESCUE_MODE)) {
        rescueStop();
    } else if (FLIGHT_MODE(GPS_RESCUE_MODE) && rescueState.phase == RESCUE_IDLE) {
        rescueStart(); rescueAttainPosition(); performSanityChecks();
    }

    sensorUpdate();
    bool initialVelocityLow = (rescueState.sensor.groundSpeedCmS < (float)gpsRescueConfig()->groundSpeedCmS);
    rescueState.isAvailable = checkGPSRescueIsAvailable();

    static rescuePhase_e lastPhase = RESCUE_IDLE;
    if (rescueState.phase != lastPhase) {
        velocityIterm = 0.0f;     // Phase 전환 시 속도 I-term 초기화
        altitudePitchIterm = 0.0f;
        yawHeadingIterm = 0.0f;     // Phase 전환 시 I-term 초기화
        prevAltMInitialized = false; // Phase 전환 시 prevAltM 초기화 (첫 루프 climbRate 오류 방지)
        smoothedPitchNeedsReset = true; // [Fix] 페이즈 전환 시 피치 LPF 즉시 재초기화
        turnDirectionSign = 0;      // Phase 전환 시 래치 상태 초기화 (Bug 1 대응)
        isDescentFalling = false;   // Phase 전환 시 급하강 상태 초기화
        descentFallAligned = false; // Phase 전환 시 정렬 상태 초기화

        // 하강 단계(DESCENT) 진입 시 landingAlt보다 15미터 이상 높으면 급하강(isDescentFalling) 발동
        if (rescueState.phase == RESCUE_DESCENT && lastPhase != RESCUE_DESCENT) {
            if (shuttleCount == 0.0f && rescueState.sensor.currentAltitudeCm > (descentAlt + 15.0f) * 100.0f) {
                isDescentFalling = true;
            }
        }

        // FLY_HOME 진입 시 Flyaway 오검출 방지를 위해 카운터와 거리 지표 초기화 
        if (rescueState.phase == RESCUE_FLY_HOME) {
            rescueState.intent.secondsFailing = 0;
            prevDistanceToHomeCm = rescueState.sensor.distanceToHomeCm;
        }

        lastPhase = rescueState.phase;
    }

    switch (rescueState.phase) {
        case RESCUE_IDLE: setReturnAltitude(); break;
        case RESCUE_INITIALIZE:
            //  A 포인트가 있으면(aPointValid == true) A 포인트 좌표를, 없으면 홈 포인트 좌표를 OSD에 표시.
            currentVCLat = aPointValid ? rescuePointA.lat : GPS_home[0];
            currentVCLon = aPointValid ? rescuePointA.lon : GPS_home[1];
            GPS_distance_cm_bearing(&gpsSol.llh.lat, &gpsSol.llh.lon, &currentVCLat, &currentVCLon, 
                                    &rescueState.intent.distanceToTargetCm, &rescueState.intent.directionToTargetCd);

            if (!STATE(GPS_FIX_HOME)) {
                rescueState.failure = RESCUE_NO_HOME_POINT;
            } else if (rescueState.sensor.distanceToHomeM < 30.0F && !aPointValid) {
                // 30m 이내 홈거리에서 A포인트가 없을때  레스큐 실행 시 아무것도 하지 않음 (안전 예방)
                rescueState.phase = RESCUE_DO_NOTHING;
            } else if (rescueState.sensor.distanceToHomeM < 5.0f &&
                       rescueState.sensor.currentAltitudeCm < rescueState.intent.targetLandingAltitudeCm) {
                rescueState.phase = RESCUE_ABORT;
            } else {
                initialiseRescueValues();
                
                if (failsafeIsReceivingRxData() && getRescueAuxValue() < 1500) {
                    shuttleInfinite = true; rescueState.intent.yawAttenuator = 1.0f;
                    initShuttlePoints(); rescueState.phase = RESCUE_SHUTTLE_INFINITE;
                } else {
                    shuttleInfinite = false;
                    // 현재 고도가 목표 고도보다  높으면 바로 FLY_HOME으로 진입
                    if (rescueState.sensor.currentAltitudeCm >= rescueState.intent.returnAltitudeCm) {
                        rescueState.intent.targetAltitudeCm = rescueState.intent.returnAltitudeCm;
                        rescueState.intent.yawAttenuator = 1.0f;
                        rescueState.phase = RESCUE_FLY_HOME;
                    } else {
                        rescueState.phase = RESCUE_ATTAIN_ALT;
                    }
                }
            }
            break;

        case RESCUE_ATTAIN_ALT:
            if (failsafeIsReceivingRxData() && getRescueAuxValue() < 1500) {
                shuttleInfinite = true; initShuttlePoints(); rescueState.phase = RESCUE_SHUTTLE_INFINITE; break;
            }
            if (attainAltStartTime == 0) attainAltStartTime = micros();
            if (cmpTimeUs(micros(), attainAltStartTime) >= ATTAIN_ALT_TIMEOUT_US ||
                rescueState.sensor.currentAltitudeCm >= rescueState.intent.returnAltitudeCm) {
                rescueState.intent.targetAltitudeCm = rescueState.intent.returnAltitudeCm;
                rescueState.intent.yawAttenuator = 1.0f; rescueState.phase = RESCUE_FLY_HOME; attainAltStartTime = 0;
            }
            break;


case RESCUE_FLY_HOME:
    if (failsafeIsReceivingRxData() && getRescueAuxValue() < 1500) {
        shuttleInfinite = true; initShuttlePoints(); rescueState.phase = RESCUE_SHUTTLE_INFINITE; break;
    }
    float targetVelErr = gpsRescueConfig()->groundSpeedCmS - rescueState.intent.targetVelocityCmS;
    bool targetVelocityIsLow = rescueState.intent.targetVelocityCmS < gpsRescueConfig()->groundSpeedCmS;
    if (initialVelocityLow == targetVelocityIsLow) {
        rescueState.intent.targetVelocityCmS += rescueState.sensor.gpsRescueTaskIntervalSeconds * targetVelErr;
    }
    if (newGPSData) {
        // A포인트가 없으면 현재 위치로 fallback 생성 (홀수 descentAlt 또는 이륙 캡쳐 실패)
        if (!aPointValid && rescueState.sensor.distanceToHomeM <= rescueState.intent.descentDistanceM) {
            rescuePointA.lat = gpsSol.llh.lat;
            rescuePointA.lon = gpsSol.llh.lon;
            aPointValid = true;
            // takeoffVectorCaptured는 false 유지 → 아래 CPA 분기 진입 안 함
        }

        int32_t targetLat = aPointValid ? rescuePointA.lat : GPS_home[0];
        int32_t targetLon = aPointValid ? rescuePointA.lon : GPS_home[1];

        uint32_t distToTargetCm;
        int32_t  bearingToTargetCd;
        GPS_distance_cm_bearing(&gpsSol.llh.lat, &gpsSol.llh.lon,
                                &targetLat, &targetLon,
                                &distToTargetCm, &bearingToTargetCd);
        float distToTargetM = distToTargetCm / 100.0f;

        bool shouldTransition = false;

        if (takeoffVectorCaptured) {
            // ── 짝수 descentAlt (shuttleCount 무관) ─────────────────────────
            // 이륙 시 캡쳐된 A포인트를 실제로 통과한 후 전환(DESCENT 또는 SHUTTLE) 진입.
            // 셔틀과 동일한 CPA 터치 판정 로직 재사용.
            // 통과 후 handleDescentPhase()의 기존 홈방향 정렬 코드(isDescentFalling)가
            // 자동으로 실행되므로 별도 정렬 처리 불필요.
            // 목표물 근처에서만 CPA 감시 활성화
            float dCm = (float)distToTargetCm;
            float activationThresholdCm = GPS_RESCUE_TOUCH_ACTIVATION_CM;

            if (dCm < activationThresholdCm) {
                if (cpaDistToTargetCm < 0.0f) {
                    // CPA 첫 진입: 초기화
                    cpaDistToTargetCm = dCm;
                    cpaWasClosing = true;
                } else {
                    bool isClosingNow = (dCm < cpaDistToTargetCm - 20.0f);
                    if (!isClosingNow && cpaWasClosing) {
                        shouldTransition = true; // 거리 감소→증가 전환 = 최근접점 통과
                    }
                    cpaWasClosing = isClosingNow;
                }
                cpaDistToTargetCm = dCm;
            }
            // 근접 폴백 (CPA 미검출 대비)
            if (dCm < GPS_RESCUE_TOUCH_PROXIMITY_CM) {
                shouldTransition = true;
            }
            // ─────────────────────────────────────────────────────────────────
        } else {
            // 홀수 descentAlt:
            // 기존 거리 기반 전환 유지 (전환 시점에 현재 위치를 A포인트로 캡쳐)
            if (distToTargetM <= rescueState.intent.descentDistanceM) {
                shouldTransition = true;
            }
        }

        if (shouldTransition &&
            rescueState.phase != RESCUE_SHUTTLE &&
            rescueState.phase != RESCUE_SHUTTLE_DESCENT &&
            rescueState.phase != RESCUE_DESCENT) {
            cpaDistToTargetCm = 200000.0f; // CPA 상태 리셋
            cpaWasClosing = true;
            if (shuttleCount == 0.0f) {
                rescueState.phase = RESCUE_DESCENT;
            } else {
                initShuttlePoints();
                rescueState.phase = RESCUE_SHUTTLE;
            }
        }
    }
    break;

        case RESCUE_SHUTTLE:
            // 셔틀 중에서 하강고도 도달시 다시 상승하더라도 셔틀카운트 완료후 (= A포인트 도착) 하강 시작)
            if (rescueState.sensor.currentAltitudeCm <= (descentAlt * 100.0f)) {
                descentAltReached = true;
            }
            if (currentShuttleTrips >= shuttleCount) {
                rescueState.phase = RESCUE_SHUTTLE_DESCENT;
            }
            break;

        case RESCUE_SHUTTLE_INFINITE:
            if (failsafeIsReceivingRxData() && getRescueAuxValue() >= 1500) rescueState.phase = RESCUE_INITIALIZE;
            break;

        case RESCUE_SHUTTLE_DESCENT:
            if (failsafeIsReceivingRxData() && getRescueAuxValue() < 1500) {
                shuttleInfinite = true; rescueState.phase = RESCUE_SHUTTLE_INFINITE; break;
            }

            // 셔틀 하강 전 구간 하강고도 도달 감지
            if (rescueState.sensor.currentAltitudeCm <= (descentAlt * 100.0f)) {
                descentAltReached = true;
            }

            // A포인트 도착 시(handleShuttleProgress 내) 즉시 RESCUE_DESCENT로 전환되므로 
            // 여기서는 추가적인 전환 로직을 수행하지 않음.
            break;

        case RESCUE_DESCENT:
            if (failsafeIsReceivingRxData() && getRescueAuxValue() < 1500) {
                shuttleInfinite = true; rescueState.phase = RESCUE_SHUTTLE_INFINITE; break;
            }
            // 랜딩 전환 조건: 홈30m이내 + 착륙고도(landingAlt) 모두 만족시 랜딩 시작
            if (rescueState.sensor.distanceToHomeM <= 30.0f && rescueState.sensor.currentAltitudeCm <= (landingAlt * 100.0f)) {
                rescueState.phase = RESCUE_LANDING;
            }
            break;

        case RESCUE_LANDING:
            disarmOnImpact();
            break;

        case RESCUE_COMPLETE: rescueStop(); break;
        case RESCUE_ABORT: rescueState.phase = RESCUE_DO_NOTHING; break;
        case RESCUE_DO_NOTHING: disarmOnImpact(); break;
        default: break;
    }

    performSanityChecks();
    rescueAttainPosition();
    newGPSData = false;
}

/* ================================================================
 * 외부 인터페이스 (OSD / LED / Blackbox Data Access)
 * ================================================================ */

float gpsRescueGetYawRate(void) { return rescueYaw; }
float gpsRescueGetImuYawCogGain(void) { return rescueState.sensor.imuYawCogGain; }
float gpsRescueGetThrottle(void) {
    float cmd = scaleRangef(rescueThrottle, MAX(rxConfig()->mincheck, PWM_RANGE_MIN), PWM_RANGE_MAX, 0.0f, 1.0f);
    return constrainf(cmd, 0.0f, 1.0f);
}
rescuePhase_e gpsRescueGetPhase(void) { return rescueState.phase; }
float gpsRescueGetTargetAltitude(void) { return rescueState.intent.targetAltitudeCm; }
float gpsRescueGetTargetVelocity(void) { return rescueState.intent.targetVelocityCmS; }
int32_t gpsRescueGetTargetLat(void) { return currentVCLat; }
int32_t gpsRescueGetTargetLon(void) { return currentVCLon; }
uint32_t gpsRescueGetTargetDistance(void) { return rescueState.intent.distanceToTargetCm; }
int32_t gpsRescueGetTargetDirection(void) { return rescueState.intent.directionToTargetCd; }

char gpsRescueGetTargetLabel(void)
{
    if (isShuttlePhase(rescueState.phase)) {
        return shuttleTargetB ? 'B' : 'A';
    }
    // A포인트가 유효하고, 아직 통과 전인 단계(INITIALIZE, ATTAIN_ALT, FLY_HOME)라면 'A' 표시
    if (aPointValid && (rescueState.phase == RESCUE_INITIALIZE || 
                        rescueState.phase == RESCUE_ATTAIN_ALT || 
                        rescueState.phase == RESCUE_FLY_HOME)) {
        return 'A';
    }
    return 'H'; // Default to Home
}
bool gpsRescueIsConfigured(void) { return failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_GPS_RESCUE || isModeActivationConditionPresent(BOXGPSRESCUE); }
bool gpsRescueIsAvailable(void) { return rescueState.isAvailable; }
bool gpsRescueIsDisabled(void) { return (!STATE(GPS_FIX_HOME)); }

#ifdef USE_MAG
bool gpsRescueDisableMag(void) { return !(gpsRescueConfig()->useMag && rescueState.phase != RESCUE_FLY_HOME && !magForceDisable); }
#endif

uint16_t gpsRescueGetCurrentShuttleTrips(void) { return (uint16_t)currentShuttleTrips; }


#endif /* USE_GPS_RESCUE */