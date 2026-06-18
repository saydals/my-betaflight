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
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>

#include "common/axis.h"

#include "pg/gps_rescue.h"

#define TASK_GPS_RESCUE_RATE_HZ 100  // in sync with altitude task rate

#ifdef USE_MAG
#define GPS_RESCUE_USE_MAG  true
#else
#define GPS_RESCUE_USE_MAG  false
#endif

/* ================================================================
 * 구조 단계 (rescuePhase_e) — 헤더 공개
 *
 * 단계 순서:
 *   IDLE        : 비활성
 *   INITIALIZE  : 구조 시작 초기화 및 Profile 3 파라미터 로드
 *   ATTAIN_ALT  : 귀환 고도 상승 (ascendPitch 사용)
 *   FLY_HOME    : 홈 방향 직선 비행 (bankTurn + yaw 제어)
 *   SHUTTLE     : 셔틀 왕복 (A <-> B 포인트 사이 지정된 횟수만큼 왕복)
 *   SHUTTLE_INFINITE : 무한 셔틀 모드 (AUX 토글 시 진입)
 *   SHUTTLE_DESCENT  : 셔틀 비행을 유지하며 descentEnd 고도까지 하강
 *   DESCENT     : 홈포인트를 타겟으로 직선 비행하며 최종 하강
 *   LANDING     : 최종 착륙 (홈 20m 이내, landingPitch + 최소 쓰로틀)
 *   DO_NOTHING  : 비상 선회 활공 (실패 시)
 *   ABORT       : 즉시 중단
 *   COMPLETE    : 구조 완료 (착륙 성공)
 * ================================================================ */
typedef enum {
    RESCUE_IDLE,
    RESCUE_INITIALIZE,
    RESCUE_ATTAIN_ALT,          // 상승
    RESCUE_FLY_HOME,            // Fly-home
    RESCUE_SHUTTLE,             // 셔틀
    RESCUE_SHUTTLE_INFINITE,    // 단독 셔틀 모드
    RESCUE_SHUTTLE_DESCENT,     // 셔틀하강
    RESCUE_DESCENT,             // 하강 (홈포인트 타겟)
    RESCUE_LANDING,             // 최종 랜딩 (최종 20m)
    RESCUE_DO_NOTHING,
    RESCUE_ABORT,
    RESCUE_COMPLETE
} rescuePhase_e;

typedef enum {
    RESCUE_SANITY_OFF = 0,
    RESCUE_SANITY_ON,
    RESCUE_SANITY_FS_ONLY,
    RESCUE_SANITY_COUNT
} gpsRescueSanity_e;

typedef enum {
    GPS_RESCUE_ALT_MODE_MAX = 0,
    GPS_RESCUE_ALT_MODE_FIXED,
    GPS_RESCUE_ALT_MODE_CURRENT,
    GPS_RESCUE_ALT_MODE_COUNT
} gpsRescueAltitudeMode_e;

extern float gpsRescueAngle[ANGLE_INDEX_COUNT]; // NOTE: ANGLES ARE IN CENTIDEGREES

void gpsRescueInit(void);
void gpsRescueUpdate(void);
void gpsRescueNewGpsData(void);

float         gpsRescueGetYawRate(void);
float         gpsRescueGetThrottle(void);
rescuePhase_e gpsRescueGetPhase(void);      // 현재 구조 단계 반환 (OSD/LED용)
float         gpsRescueGetTargetAltitude(void);
float         gpsRescueGetTargetVelocity(void);  // 목표 속도 (cm/s) — OSD 표시용
int32_t       gpsRescueGetTargetLat(void);
int32_t       gpsRescueGetTargetLon(void);
uint32_t      gpsRescueGetTargetDistance(void); // 타겟까지의 거리 (cm)
int32_t       gpsRescueGetTargetDirection(void); // 타겟 방향 (0.01도 단위)
char          gpsRescueGetTargetLabel(void);
bool          gpsRescueIsConfigured(void);
bool          gpsRescueIsAvailable(void);
bool          gpsRescueIsDisabled(void);
bool          gpsRescueDisableMag(void);
float         gpsRescueGetImuYawCogGain(void);
uint16_t      gpsRescueGetCurrentShuttleTrips(void);
