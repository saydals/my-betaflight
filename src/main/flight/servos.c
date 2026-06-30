/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_SERVOS

#include "build/build_config.h"

#include "common/filter.h"
#include "common/maths.h"
#include "common/time.h"

#include "config/config.h"
#include "config/config_reset.h"
#include "config/feature.h"

#include "drivers/pwm_output.h"
#include "drivers/time.h"

#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/servos.h"

#include "io/gimbal.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "rx/rx.h"


PG_REGISTER_WITH_RESET_FN(servoConfig_t, servoConfig, PG_SERVO_CONFIG, 0);

void pgResetFn_servoConfig(servoConfig_t *servoConfig)
{
    servoConfig->dev.servoCenterPulse = 1500;
    servoConfig->dev.servoPwmRate = 50;
    servoConfig->tri_unarmed_servo = 1;
    servoConfig->servo_lowpass_freq = 0;
    servoConfig->channelForwardingStartChannel = AUX1;
    servoConfig->ready_to_arm_wiggle_hz = 4;

#ifdef SERVO1_PIN
    servoConfig->dev.ioTags[0] = IO_TAG(SERVO1_PIN);
#endif
#ifdef SERVO2_PIN
    servoConfig->dev.ioTags[1] = IO_TAG(SERVO2_PIN);
#endif
#ifdef SERVO3_PIN
    servoConfig->dev.ioTags[2] = IO_TAG(SERVO3_PIN);
#endif
#ifdef SERVO4_PIN
    servoConfig->dev.ioTags[3] = IO_TAG(SERVO4_PIN);
#endif
}

PG_REGISTER_ARRAY(servoMixer_t, MAX_SERVO_RULES, customServoMixers, PG_SERVO_MIXER, 0);

PG_REGISTER_ARRAY_WITH_RESET_FN(servoParam_t, MAX_SUPPORTED_SERVOS, servoParams, PG_SERVO_PARAMS, 0);

void pgResetFn_servoParams(servoParam_t *instance)
{
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        RESET_CONFIG(servoParam_t, &instance[i],
            .min = DEFAULT_SERVO_MIN,
            .max = DEFAULT_SERVO_MAX,
            .middle = DEFAULT_SERVO_MIDDLE,
            .rate = 100,
            .forwardFromChannel = CHANNEL_FORWARDING_DISABLED
        );
    }
}

// no template required since default is zero
PG_REGISTER(gimbalConfig_t, gimbalConfig, PG_GIMBAL_CONFIG, 0);

int16_t servo[MAX_SUPPORTED_SERVOS];

static uint8_t servoRuleCount = 0;
static servoMixer_t currentServoMixer[MAX_SERVO_RULES];
static int useServo;

// ============================================================
// Bird Flap PG registration and defaults
// ============================================================
PG_REGISTER_WITH_RESET_FN(birdFlapConfig_t, birdFlapConfig, PG_BIRD_FLAP_CONFIG, 0);

void pgResetFn_birdFlapConfig(birdFlapConfig_t *config)
{
    config->maxFreqX10 = 40;       // 4.0 Hz
    config->minFreqX10 = 5;        // 0.5 Hz
    config->servo_speed = 500;     // 500 deg/s
    config->up_amplitude = 500;    // 500 servo units (upward)
    config->down_amplitude = 500;  // 500 servo units (downward)
    config->upRatio100x = 35;      // 0.35 (35% rise, 65% fall)
    config->softStartMs = 1000;    // 1000 ms
    config->softStopMs = 1500;     // 1500 ms
    config->freqTauMs = 200;       // 200 ms LPF time constant
    config->slewRate = 4000;       // 4000 units/s
}

// ============================================================
// Bird Flap static state variables
// ============================================================
static uint8_t birdFlapState = BIRD_FLAP_STATE_OFF;
static uint8_t birdFlapServoIndex = 0;
static bool birdFlapConfigured = false;

static float birdFlapPhase = 0.0f;
static float birdFlapFreqSmoothed = 0.0f;
static float birdFlapOutput = 0.0f;
static float birdFlapUpAmplitude = 0.0f;
static float birdFlapDownAmplitude = 0.0f;

static float birdFlapFreqLpfCoef = 0.0f;
static timeUs_t birdFlapLastUs = 0;

// ============================================================
// Ready-to-Arm Wiggle constants
// ============================================================
#define READY_TO_ARM_WIGGLE_DURATION_US   1000000   // 1초
#define READY_TO_ARM_WIGGLE_INTERVAL_US   10000000  // 10초
#define READY_TO_ARM_WIGGLE_BOOT_DELAY_MS 5000      // 5초
#define READY_TO_ARM_WIGGLE_AMPLITUDE     250        // ±250

// ============================================================
// Ready-to-Arm Wiggle state variables
// ============================================================
static bool     wiggleActive     = false;
static timeUs_t wiggleStartUs    = 0;
static timeUs_t lastWiggleTimeUs = 0;

// ============================================================
// Bird Flap - Check if mixer uses INPUT_BIRD_FLAP
// ============================================================
static void birdFlapScanMixer(void)
{
    birdFlapConfigured = false;
    for (int i = 0; i < servoRuleCount; i++) {
        if (currentServoMixer[i].inputSource == INPUT_BIRD_FLAP) {
            birdFlapServoIndex = currentServoMixer[i].targetChannel;
            birdFlapConfigured = true;
            break;
        }
    }
}

// ============================================================
// Bird Flap - Update LPF coefficient based on tau
// Called when tau changes (init / config reload)
// ============================================================
static void birdFlapUpdateCoeffs(void)
{
    // freqTauMs is in ms; convert to LPF coefficient for ~1kHz update
    // coef = dt / (dt + tau) where dt is approximate loop time
    const float dt = 0.001f; // assume 1kHz update
    const float tau = (float)birdFlapConfig()->freqTauMs * 0.001f;
    birdFlapFreqLpfCoef = dt / (dt + tau);

}

// ============================================================
// Bird Flap - Core update function
// Called from servoMixer() when bird flap is configured
// ============================================================
static void birdFlapUpdate(const int16_t inputValue)
{
    timeUs_t nowUs = micros();
    if (birdFlapLastUs == 0) {
        birdFlapLastUs = nowUs;
        return;
    }
    float dt = (float)(nowUs - birdFlapLastUs) * 1e-6f;
    if (dt > 0.1f) dt = 0.1f; // limit dt
    birdFlapLastUs = nowUs;

    const bool boxActive = IS_RC_MODE_ACTIVE(BOXUSER1);
    const birdFlapConfig_t *cfg = birdFlapConfig();

    // === State Machine (up/down separate amplitudes) ===
    switch (birdFlapState) {
    case BIRD_FLAP_STATE_OFF:
        if (boxActive) {
            birdFlapState = BIRD_FLAP_STATE_STARTING;
            birdFlapUpAmplitude = 0.0f;
            birdFlapDownAmplitude = 0.0f;
            birdFlapPhase = 0.0f;
            birdFlapOutput = 0.0f;
            birdFlapFreqSmoothed = (float)cfg->minFreqX10 * 0.1f;
            birdFlapUpdateCoeffs();
        }
        break;

    case BIRD_FLAP_STATE_STARTING:
        if (!boxActive) {
            birdFlapState = BIRD_FLAP_STATE_STOPPING;
            break;
        }
        // Guard against zero-division: if softStartMs <= 0, jump directly to full amplitude
        if (cfg->softStartMs <= 0) {
            birdFlapUpAmplitude = (float)cfg->up_amplitude;
            birdFlapDownAmplitude = (float)cfg->down_amplitude;
            birdFlapState = BIRD_FLAP_STATE_ACTIVE;
            break;
        }
        // Ramp up/down amplitudes separately toward configured values
        birdFlapUpAmplitude += (float)cfg->up_amplitude * dt / ((float)cfg->softStartMs * 0.001f);
        if (birdFlapUpAmplitude >= (float)cfg->up_amplitude) {
            birdFlapUpAmplitude = (float)cfg->up_amplitude;
        }
        birdFlapDownAmplitude += (float)cfg->down_amplitude * dt / ((float)cfg->softStartMs * 0.001f);
        if (birdFlapDownAmplitude >= (float)cfg->down_amplitude) {
            birdFlapDownAmplitude = (float)cfg->down_amplitude;
        }
        if (birdFlapUpAmplitude >= (float)cfg->up_amplitude && birdFlapDownAmplitude >= (float)cfg->down_amplitude) {
            birdFlapState = BIRD_FLAP_STATE_ACTIVE;
        }
        break;

    case BIRD_FLAP_STATE_ACTIVE:
        if (!boxActive) {
            birdFlapState = BIRD_FLAP_STATE_STOPPING;
            break;
        }
        birdFlapUpAmplitude = (float)cfg->up_amplitude;
        birdFlapDownAmplitude = (float)cfg->down_amplitude;
        break;

    case BIRD_FLAP_STATE_STOPPING:
        // Guard against zero-division: if softStopMs <= 0, jump to off immediately
        if (cfg->softStopMs <= 0) {
            birdFlapUpAmplitude = 0.0f;
            birdFlapDownAmplitude = 0.0f;
            birdFlapState = BIRD_FLAP_STATE_OFF;
            birdFlapOutput = 0.0f;
            return;
        }
        // Ramp both amplitudes toward 0 over softStopMs at their own rates
        birdFlapUpAmplitude -= (float)cfg->up_amplitude * dt / ((float)cfg->softStopMs * 0.001f);
        birdFlapDownAmplitude -= (float)cfg->down_amplitude * dt / ((float)cfg->softStopMs * 0.001f);
        if (birdFlapUpAmplitude <= 0.0f) birdFlapUpAmplitude = 0.0f;
        if (birdFlapDownAmplitude <= 0.0f) birdFlapDownAmplitude = 0.0f;
        if (birdFlapUpAmplitude <= 0.0f && birdFlapDownAmplitude <= 0.0f) {
            birdFlapUpAmplitude = 0.0f;
            birdFlapDownAmplitude = 0.0f;
            birdFlapState = BIRD_FLAP_STATE_OFF;
            birdFlapOutput = 0.0f;
            return;
        }
        break;
    }

    // === Frequency from input ===
    // Map inputValue [-500:+500] to [minFreq, maxFreq]
    // Try to find BOXUSER1 range from modeActivationConditions.
    // If found, scale the corresponding channel's range to [minFreq, maxFreq].
    // If not found, fallback to the original logic using inputValue.
    float rawFreq;
    bool foundRange = false;
    for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        const modeActivationCondition_t *mac = modeActivationConditions(i);
        if (mac->modeId == BOXUSER1 && IS_RANGE_USABLE(&mac->range)) {
            const uint16_t channelValue = rcData[mac->auxChannelIndex + NON_AUX_CHANNEL_COUNT];
            const float pwmStart = 900 + mac->range.startStep * 25;
            const float pwmEnd = 900 + mac->range.endStep * 25;
            if (pwmEnd > pwmStart) {
                const float minFreq = (float)cfg->minFreqX10 * 0.1f;
                const float maxFreq = (float)cfg->maxFreqX10 * 0.1f;
                rawFreq = minFreq + (maxFreq - minFreq) * ((float)(channelValue - pwmStart) / (pwmEnd - pwmStart));
                rawFreq = constrainf(rawFreq, minFreq, maxFreq);
                foundRange = true;
                break;
            }
        }
    }

    if (!foundRange) {
        if (inputValue > -10 && inputValue < 10) {
            rawFreq = (float)cfg->minFreqX10 * 0.1f;
        } else {
            const float freqRange = ((float)cfg->maxFreqX10 - (float)cfg->minFreqX10) * 0.1f;
            rawFreq = (float)cfg->minFreqX10 * 0.1f + freqRange * ((float)(inputValue + 500) / 1000.0f);
            rawFreq = constrainf(rawFreq, (float)cfg->minFreqX10 * 0.1f, (float)cfg->maxFreqX10 * 0.1f);
        }
    }

    // === Frequency LPF ===
    birdFlapFreqSmoothed += (rawFreq - birdFlapFreqSmoothed) * birdFlapFreqLpfCoef;

    // === Phase Accumulator ===
    birdFlapPhase += birdFlapFreqSmoothed * dt;
    if (birdFlapPhase >= 1.0f) {
        birdFlapPhase -= 1.0f;
    }

    // === Asymmetric Sine Wave (phase-warped, up-ratio controlled) ===
    const float upRatio = (float)cfg->upRatio100x * 0.01f;
    float warpedPhase;
    if (birdFlapPhase < upRatio) {
        // Map [0, upRatio] → [0, 0.5]
        warpedPhase = 0.5f * birdFlapPhase / upRatio;
    } else {
        // Map [upRatio, 1.0] → [0.5, 1.0]
        warpedPhase = 0.5f + 0.5f * (birdFlapPhase - upRatio) / (1.0f - upRatio);
    }
    // Sine wave: sin(0)=0 → sin(π)=0 → sin(2π)=0
    // With warp: 0 → +1 → 0 → -1 → 0 (smooth, no discontinuities)
    float sineOut = sin_approx(warpedPhase * 2.0f * M_PIf);

    // === Scale to up/down amplitudes (asymmetric stroke) ===
    float target;
    if (sineOut >= 0.0f) {
        // Upward stroke (sineOut from 0 to +1)
        target = sineOut * birdFlapUpAmplitude;
    } else {
        // Downward stroke (sineOut from 0 to -1)
        target = sineOut * birdFlapDownAmplitude;
    }

    // === Slew Rate Limiting ===
    if (cfg->slewRate > 0 && dt > 0.0f) {
        // Convert slewRate (units/sec) to max change per dt
        float slewPerDt = (float)cfg->slewRate * dt;
        if (slewPerDt > 0.0f) {
            float diff = target - birdFlapOutput;
            if (diff > slewPerDt) diff = slewPerDt;
            if (diff < -slewPerDt) diff = -slewPerDt;
            birdFlapOutput += diff;
        }
    } else {
        birdFlapOutput = target;
    }
}

// ============================================================

#define COUNT_SERVO_RULES(rules) (sizeof(rules) / sizeof(servoMixer_t))
// mixer rule format servo, input, rate, speed, min, max, box
static const servoMixer_t servoMixerAirplane[] = {
    { SERVO_FLAPPERON_1, INPUT_STABILIZED_ROLL,  100, 0, 0, 100, 0 },
    { SERVO_FLAPPERON_2, INPUT_STABILIZED_ROLL,  100, 0, 0, 100, 0 },
    { SERVO_ELEVATOR,    INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    { SERVO_RUDDER,      INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
    { SERVO_THROTTLE,    INPUT_STABILIZED_THROTTLE, 100, 0, 0, 100, 0 },
};

static const servoMixer_t servoMixerFlyingWing[] = {
    { SERVO_FLAPPERON_1, INPUT_STABILIZED_ROLL,  100, 0, 0, 100, 0 },
    { SERVO_FLAPPERON_1, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    { SERVO_FLAPPERON_2, INPUT_STABILIZED_ROLL, -100, 0, 0, 100, 0 },
    { SERVO_FLAPPERON_2, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    { SERVO_THROTTLE,    INPUT_STABILIZED_THROTTLE, 100, 0, 0, 100, 0 },
};

static const servoMixer_t servoMixerTri[] = {
    { SERVO_RUDDER, INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
};

#if defined(USE_UNCOMMON_MIXERS)
static const servoMixer_t servoMixerBI[] = {
    { SERVO_BICOPTER_LEFT, INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
    { SERVO_BICOPTER_LEFT, INPUT_STABILIZED_PITCH, -100, 0, 0, 100, 0 },
    { SERVO_BICOPTER_RIGHT, INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
    { SERVO_BICOPTER_RIGHT, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
};

static const servoMixer_t servoMixerDual[] = {
    { SERVO_DUALCOPTER_LEFT, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    { SERVO_DUALCOPTER_RIGHT, INPUT_STABILIZED_ROLL,  100, 0, 0, 100, 0 },
};

static const servoMixer_t servoMixerSingle[] = {
    { SERVO_SINGLECOPTER_1, INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_1, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_2, INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_2, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_3, INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_3, INPUT_STABILIZED_ROLL,  100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_4, INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_4, INPUT_STABILIZED_ROLL,  100, 0, 0, 100, 0 },
};

static const servoMixer_t servoMixerHeli[] = {
    { SERVO_HELI_LEFT, INPUT_STABILIZED_PITCH,   -50, 0, 0, 100, 0 },
    { SERVO_HELI_LEFT, INPUT_STABILIZED_ROLL,    -87, 0, 0, 100, 0 },
    { SERVO_HELI_LEFT, INPUT_RC_AUX1,    100, 0, 0, 100, 0 },
    { SERVO_HELI_RIGHT, INPUT_STABILIZED_PITCH,  -50, 0, 0, 100, 0 },
    { SERVO_HELI_RIGHT, INPUT_STABILIZED_ROLL,  87, 0, 0, 100, 0 },
    { SERVO_HELI_RIGHT, INPUT_RC_AUX1,    100, 0, 0, 100, 0 },
    { SERVO_HELI_TOP, INPUT_STABILIZED_PITCH,   100, 0, 0, 100, 0 },
    { SERVO_HELI_TOP, INPUT_RC_AUX1,    100, 0, 0, 100, 0 },
    { SERVO_HELI_RUD, INPUT_STABILIZED_YAW, 100, 0, 0, 100, 0 },
};
#else
#define servoMixerBI NULL
#define servoMixerDual NULL
#define servoMixerSingle NULL
#define servoMixerHeli NULL
#endif // USE_UNCOMMON_MIXERS

static const servoMixer_t servoMixerGimbal[] = {
    { SERVO_GIMBAL_PITCH, INPUT_GIMBAL_PITCH, 125, 0, 0, 100, 0 },
    { SERVO_GIMBAL_ROLL, INPUT_GIMBAL_ROLL,  125, 0, 0, 100, 0 },
};

const mixerRules_t servoMixers[] = {
    { 0, NULL },                // entry 0
    { COUNT_SERVO_RULES(servoMixerTri), servoMixerTri },       // MULTITYPE_TRI
    { 0, NULL },                // MULTITYPE_QUADP
    { 0, NULL },                // MULTITYPE_QUADX
    { COUNT_SERVO_RULES(servoMixerBI), servoMixerBI },        // MULTITYPE_BI
    { COUNT_SERVO_RULES(servoMixerGimbal), servoMixerGimbal },    // * MULTITYPE_GIMBAL
    { 0, NULL },                // MULTITYPE_Y6
    { 0, NULL },                // MULTITYPE_HEX6
    { COUNT_SERVO_RULES(servoMixerFlyingWing), servoMixerFlyingWing },// * MULTITYPE_FLYING_WING
    { 0, NULL },                // MULTITYPE_Y4
    { 0, NULL },                // MULTITYPE_HEX6X
    { 0, NULL },                // MULTITYPE_OCTOX8
    { 0, NULL },                // MULTITYPE_OCTOFLATP
    { 0, NULL },                // MULTITYPE_OCTOFLATX
    { COUNT_SERVO_RULES(servoMixerAirplane), servoMixerAirplane },  // * MULTITYPE_AIRPLANE
    { COUNT_SERVO_RULES(servoMixerHeli), servoMixerHeli },                // * MULTITYPE_HELI_120_CCPM
    { 0, NULL },                // * MULTITYPE_HELI_90_DEG
    { 0, NULL },                // MULTITYPE_VTAIL4
    { 0, NULL },                // MULTITYPE_HEX6H
    { 0, NULL },                // * MULTITYPE_PPM_TO_SERVO
    { COUNT_SERVO_RULES(servoMixerDual), servoMixerDual },      // MULTITYPE_DUALCOPTER
    { COUNT_SERVO_RULES(servoMixerSingle), servoMixerSingle },    // MULTITYPE_SINGLECOPTER
    { 0, NULL },                // MULTITYPE_ATAIL4
    { 0, NULL },                // MULTITYPE_CUSTOM
    { 0, NULL },                // MULTITYPE_CUSTOM_PLANE
    { 0, NULL },                // MULTITYPE_CUSTOM_TRI
    { 0, NULL },
};

int16_t determineServoMiddleOrForwardFromChannel(servoIndex_e servoIndex)
{
    const uint8_t channelToForwardFrom = servoParams(servoIndex)->forwardFromChannel;

    if (channelToForwardFrom != CHANNEL_FORWARDING_DISABLED && channelToForwardFrom < rxRuntimeState.channelCount) {
        return scaleRangef(constrainf(rcData[channelToForwardFrom], PWM_RANGE_MIN, PWM_RANGE_MAX), PWM_RANGE_MIN, PWM_RANGE_MAX, servoParams(servoIndex)->min, servoParams(servoIndex)->max);
    }

    return servoParams(servoIndex)->middle;
}

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

int servoDirection(int servoIndex, int inputSource)
{
    // determine the direction (reversed or not) from the direction bitfield of the servo
    if (servoParams(servoIndex)->reversedSources & (1 << inputSource)) {
        return -1;
    } else {
        return 1;
    }
}

void loadCustomServoMixer(void)
{
    // reset settings
    servoRuleCount = 0;
    memset(currentServoMixer, 0, sizeof(currentServoMixer));

    // load custom mixer into currentServoMixer
    for (int i = 0; i < MAX_SERVO_RULES; i++) {
        // check if done
        if (customServoMixers(i)->rate == 0) {
            break;
        }
        currentServoMixer[i] = *customServoMixers(i);
        servoRuleCount++;
    }
}

static void servoConfigureOutput(void)
{
    if (useServo) {
        servoRuleCount = servoMixers[getMixerMode()].servoRuleCount;
        if (servoMixers[getMixerMode()].rule) {
            for (int i = 0; i < servoRuleCount; i++)
                currentServoMixer[i] = servoMixers[getMixerMode()].rule[i];
        }
    }

    switch (getMixerMode()) {
    case MIXER_CUSTOM_AIRPLANE:
    case MIXER_CUSTOM_TRI:
        loadCustomServoMixer();
        break;
    default:
        break;
    }
}


void servosInit(void)
{
    // enable servos for mixes that require them. note, this shifts motor counts.
    useServo = mixers[getMixerMode()].useServo;
    // if we want camstab/trig, that also enables servos, even if mixer doesn't
    if (featureIsEnabled(FEATURE_SERVO_TILT) || featureIsEnabled(FEATURE_CHANNEL_FORWARDING)) {
        useServo = 1;
    }

    // give all servos a default command
    for (uint8_t i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo[i] = DEFAULT_SERVO_MIDDLE;
    }

    if (mixerIsTricopter()) {
        servosTricopterInit();
    }

    servoConfigureOutput();

    // Initialize bird flap state
    birdFlapScanMixer();
    birdFlapState = BIRD_FLAP_STATE_OFF;
    birdFlapLastUs = 0;
    birdFlapUpAmplitude = 0.0f;
    birdFlapDownAmplitude = 0.0f;
    birdFlapOutput = 0.0f;
    birdFlapFreqSmoothed = 0.0f;
    birdFlapPhase = 0.0f;

    // Initialize Ready-to-Arm Wiggle state
    wiggleActive = false;
    wiggleStartUs = 0;
    lastWiggleTimeUs = 0;
}

void servoMixerLoadMix(int index)
{
    // we're 1-based
    index++;
    // clear existing
    for (int i = 0; i < MAX_SERVO_RULES; i++) {
        customServoMixersMutable(i)->targetChannel = customServoMixersMutable(i)->inputSource = customServoMixersMutable(i)->rate = customServoMixersMutable(i)->box = 0;
    }
    for (int i = 0; i < servoMixers[index].servoRuleCount; i++) {
        *customServoMixersMutable(i) = servoMixers[index].rule[i];
    }
}

STATIC_UNIT_TESTED void forwardAuxChannelsToServos(uint8_t firstServoIndex)
{
    // start forwarding from this channel
    int channelOffset = servoConfig()->channelForwardingStartChannel;
    const int maxAuxChannelCount = MIN(MAX_AUX_CHANNEL_COUNT, rxConfig()->max_aux_channel);
    for (int servoOffset = 0; servoOffset < maxAuxChannelCount && channelOffset < MAX_SUPPORTED_RC_CHANNEL_COUNT; servoOffset++) {
        pwmWriteServo(firstServoIndex + servoOffset, rcData[channelOffset++]);
    }
}

// Write and keep track of written servos

static uint32_t servoWritten;

STATIC_ASSERT(sizeof(servoWritten) * 8 >= MAX_SUPPORTED_SERVOS, servoWritten_is_too_small);

static void writeServoWithTracking(uint8_t index, servoIndex_e servoname)
{
    pwmWriteServo(index, servo[servoname]);
    servoWritten |= (1 << servoname);
}

static void updateGimbalServos(uint8_t firstServoIndex)
{
    writeServoWithTracking(firstServoIndex + 0, SERVO_GIMBAL_PITCH);
    writeServoWithTracking(firstServoIndex + 1, SERVO_GIMBAL_ROLL);
}

static void servoTable(void);
static void filterServos(void);

void writeServos(void)
{
    servoWritten = 0;
    servoTable();
    filterServos();

    uint8_t servoIndex = 0;
    switch (getMixerMode()) {
    case MIXER_TRI:
    case MIXER_CUSTOM_TRI:
        // We move servo if unarmed flag set or armed
        if (!(servosTricopterIsEnabledServoUnarmed() || ARMING_FLAG(ARMED))) {
            servo[SERVO_RUDDER] = 0; // kill servo signal completely.
        }
        writeServoWithTracking(servoIndex++, SERVO_RUDDER);
        break;

    case MIXER_FLYING_WING:
        writeServoWithTracking(servoIndex++, SERVO_FLAPPERON_1);
        writeServoWithTracking(servoIndex++, SERVO_FLAPPERON_2);
        break;

    case MIXER_CUSTOM_AIRPLANE:
    case MIXER_AIRPLANE:
        for (int i = SERVO_PLANE_INDEX_MIN; i <= SERVO_PLANE_INDEX_MAX; i++) {
            writeServoWithTracking(servoIndex++, i);
        }
        break;

#ifdef USE_UNCOMMON_MIXERS
    case MIXER_BICOPTER:
        writeServoWithTracking(servoIndex++, SERVO_BICOPTER_LEFT);
        writeServoWithTracking(servoIndex++, SERVO_BICOPTER_RIGHT);
        break;

    case MIXER_HELI_120_CCPM:
        writeServoWithTracking(servoIndex++, SERVO_HELI_LEFT);
        writeServoWithTracking(servoIndex++, SERVO_HELI_RIGHT);
        writeServoWithTracking(servoIndex++, SERVO_HELI_TOP);
        writeServoWithTracking(servoIndex++, SERVO_HELI_RUD);
        break;

    case MIXER_DUALCOPTER:
        writeServoWithTracking(servoIndex++, SERVO_DUALCOPTER_LEFT);
        writeServoWithTracking(servoIndex++, SERVO_DUALCOPTER_RIGHT);
        break;

    case MIXER_SINGLECOPTER:
        for (int i = SERVO_SINGLECOPTER_INDEX_MIN; i <= SERVO_SINGLECOPTER_INDEX_MAX; i++) {
            writeServoWithTracking(servoIndex++, i);
        }
        break;
#endif // USE_UNCOMMON_MIXERS

    default:
        break;
    }

    // Two servos for SERVO_TILT, if enabled
    if (featureIsEnabled(FEATURE_SERVO_TILT) || getMixerMode() == MIXER_GIMBAL) {
        updateGimbalServos(servoIndex);
        servoIndex += 2;
    }

    // Scan servos and write those marked forwarded and not written yet
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        const uint8_t channelToForwardFrom = servoParams(i)->forwardFromChannel;
        if ((channelToForwardFrom != CHANNEL_FORWARDING_DISABLED) && !(servoWritten & (1 << i))) {
            pwmWriteServo(servoIndex++, servo[i]);
        }
    }

    // forward AUX to remaining servo outputs (not constrained)
    if (featureIsEnabled(FEATURE_CHANNEL_FORWARDING)) {
        forwardAuxChannelsToServos(servoIndex);
        servoIndex += MAX_AUX_CHANNEL_COUNT;
    }
}

// ============================================================
// Ready-to-Arm Wiggle — Arming 준비 완료 시 에일러론 타면 주기적 알림
// ============================================================
static void updateReadyToArmWiggle(void)
{
    // OFF (0Hz)
    if (servoConfig()->ready_to_arm_wiggle_hz == 0) return;

    // 부팅 유예
    if (millis() < READY_TO_ARM_WIGGLE_BOOT_DELAY_MS) return;

    // Armed → 즉시 중단
    if (ARMING_FLAG(ARMED)) {
        wiggleActive = false;
        return;
    }

    // Arming 불가 → 중단 + 타이머 리셋
    if (isArmingDisabled()) {
        wiggleActive = false;
        lastWiggleTimeUs = 0;
        return;
    }

    // Arming 가능 → 윙글 활성화 (최초 or 10초 주기)
    timeUs_t now = micros();
    if (!wiggleActive) {
        if (lastWiggleTimeUs == 0 ||
            cmpTimeUs(now, lastWiggleTimeUs) >= READY_TO_ARM_WIGGLE_INTERVAL_US) {
            wiggleActive = true;
            wiggleStartUs = now;
            lastWiggleTimeUs = now;
        }
    }

    // 1초 지속시간 종료
    if (wiggleActive && cmpTimeUs(now, wiggleStartUs) > READY_TO_ARM_WIGGLE_DURATION_US) {
        wiggleActive = false;
    }
}

static int16_t getReadyToArmWiggleOffset(void)
{
    if (!wiggleActive) return 0;

    float t = (float)cmpTimeUs(micros(), wiggleStartUs) * 1e-6f;
    return (int16_t)(sin_approx(2.0f * M_PIf * (float)servoConfig()->ready_to_arm_wiggle_hz * t)
                     * READY_TO_ARM_WIGGLE_AMPLITUDE);
}

void servoMixer(void)
{
    int16_t input[INPUT_SOURCE_COUNT]; // Range [-500:+500]
    static int16_t currentOutput[MAX_SERVO_RULES];

    if (FLIGHT_MODE(PASSTHRU_MODE)) {
        // Direct passthru from RX
        input[INPUT_STABILIZED_ROLL] = rcCommand[ROLL];
        input[INPUT_STABILIZED_PITCH] = rcCommand[PITCH];
        input[INPUT_STABILIZED_YAW] = rcCommand[YAW];
    } else {
        // Assisted modes (gyro only or gyro+acc according to AUX configuration in Gui
        input[INPUT_STABILIZED_ROLL] = pidData[FD_ROLL].Sum * PID_SERVO_MIXER_SCALING;
        input[INPUT_STABILIZED_PITCH] = pidData[FD_PITCH].Sum * PID_SERVO_MIXER_SCALING;
        input[INPUT_STABILIZED_YAW] = pidData[FD_YAW].Sum * PID_SERVO_MIXER_SCALING;

        // Reverse yaw servo when inverted in 3D mode
        if (featureIsEnabled(FEATURE_3D) && (rcData[THROTTLE] < rxConfig()->midrc)) {
            input[INPUT_STABILIZED_YAW] *= -1;
        }
    }

    input[INPUT_GIMBAL_PITCH] = scaleRange(attitude.values.pitch, -1800, 1800, -500, +500);
    input[INPUT_GIMBAL_ROLL] = scaleRange(attitude.values.roll, -1800, 1800, -500, +500);

    input[INPUT_STABILIZED_THROTTLE] = motor[0] - 1000 - 500;  // Since it derives from rcCommand or mincommand and must be [-500:+500]

    // center the RC input value around the RC middle value
    // by subtracting the RC middle value from the RC input value, we get:
    // data - middle = input
    // 2000 - 1500 = +500
    // 1500 - 1500 = 0
    // 1000 - 1500 = -500
    input[INPUT_RC_ROLL]     = rcData[ROLL]     - rxConfig()->midrc;
    input[INPUT_RC_PITCH]    = rcData[PITCH]    - rxConfig()->midrc;
    input[INPUT_RC_YAW]      = rcData[YAW]      - rxConfig()->midrc;
    input[INPUT_RC_THROTTLE] = rcData[THROTTLE] - rxConfig()->midrc;
    input[INPUT_RC_AUX1]     = rcData[AUX1]     - rxConfig()->midrc;
    input[INPUT_RC_AUX2]     = rcData[AUX2]     - rxConfig()->midrc;
    input[INPUT_RC_AUX3]     = rcData[AUX3]     - rxConfig()->midrc;
    input[INPUT_RC_AUX4]     = rcData[AUX4]     - rxConfig()->midrc;

    // Bird Flap input: use the RC channel mapped to INPUT_BIRD_FLAP
    // If a mixer rule uses INPUT_BIRD_FLAP, we need the RC data from the assigned forward channel
    // The mixer rule for bird flap should have a rate value that determines the channel mapping
    // For now, use AUX5 (index 16) as the default bird flap RC channel if available
    if (rxRuntimeState.channelCount > AUX4 + 1) {
        input[INPUT_BIRD_FLAP] = rcData[AUX4 + 1] - rxConfig()->midrc;
    } else {
        input[INPUT_BIRD_FLAP] = 0;
    }

    // ★ Ready-to-Arm Wiggle
    updateReadyToArmWiggle();
    if (birdFlapConfigured) {
        // Bird Flap 사용 시: Roll이 아닌 Bird Flap 서보에 오프셋을 나중에 추가
    } else {
        // Bird Flap 미사용: 기존대로 에일러론(Flapperon) Roll 입력에 오프셋 주입
        input[INPUT_STABILIZED_ROLL] += getReadyToArmWiggleOffset();
    }

    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo[i] = 0;
    }

    // mix servos according to rules
    for (int i = 0; i < servoRuleCount; i++) {
        // consider rule if no box assigned or box is active
        if (currentServoMixer[i].box == 0 || IS_RC_MODE_ACTIVE(BOXSERVO1 + currentServoMixer[i].box - 1)) {
            uint8_t target = currentServoMixer[i].targetChannel;
            uint8_t from = currentServoMixer[i].inputSource;
            uint16_t servo_width = servoParams(target)->max - servoParams(target)->min;
            int16_t min = currentServoMixer[i].min * servo_width / 100 - servo_width / 2;
            int16_t max = currentServoMixer[i].max * servo_width / 100 - servo_width / 2;

            if (currentServoMixer[i].speed == 0)
                currentOutput[i] = input[from];
            else {
                if (currentOutput[i] < input[from])
                    currentOutput[i] = constrain(currentOutput[i] + currentServoMixer[i].speed, currentOutput[i], input[from]);
                else if (currentOutput[i] > input[from])
                    currentOutput[i] = constrain(currentOutput[i] - currentServoMixer[i].speed, input[from], currentOutput[i]);
            }

            servo[target] += servoDirection(target, from) * constrain(((int32_t)currentOutput[i] * currentServoMixer[i].rate) / 100, min, max);
        } else {
            currentOutput[i] = 0;
        }
    }

    // === Bird Flap override ===
    // If bird flap is configured (mixer uses INPUT_BIRD_FLAP), run the kinematic model
    if (birdFlapConfigured) {
        birdFlapUpdate(input[INPUT_BIRD_FLAP]);
        // Override the servo output with the bird flap kinematic position
        // birdFlapOutput is centered around 0. The middle offset is added
        // later by determineServoMiddleOrForwardFromChannel() at line 704.
        // Only set the offset here to avoid double-adding the middle value.
        // Wiggle offset is applied to bird flap servo (not aileron).
        servo[birdFlapServoIndex] = (int16_t)(birdFlapOutput + getReadyToArmWiggleOffset());
    }

    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo[i] = ((int32_t)servoParams(i)->rate * servo[i]) / 100L;
        servo[i] += determineServoMiddleOrForwardFromChannel(i);
    }
}


static void servoTable(void)
{
    // airplane / servo mixes
    switch (getMixerMode()) {
    case MIXER_CUSTOM_TRI:
    case MIXER_TRI:
        servosTricopterMixer();
        break;
    case MIXER_CUSTOM_AIRPLANE:
    case MIXER_FLYING_WING:
    case MIXER_AIRPLANE:
    case MIXER_BICOPTER:
    case MIXER_DUALCOPTER:
    case MIXER_SINGLECOPTER:
    case MIXER_HELI_120_CCPM:
    case MIXER_GIMBAL:
        servoMixer();
        break;

    /*
    case MIXER_GIMBAL:
        servo[SERVO_GIMBAL_PITCH] = (((int32_t)servoParams(SERVO_GIMBAL_PITCH)->rate * attitude.values.pitch) / 50) + determineServoMiddleOrForwardFromChannel(SERVO_GIMBAL_PITCH);
        servo[SERVO_GIMBAL_ROLL] = (((int32_t)servoParams(SERVO_GIMBAL_ROLL)->rate * attitude.values.roll) / 50) + determineServoMiddleOrForwardFromChannel(SERVO_GIMBAL_ROLL);
        break;
    */

    default:
        break;
    }

    // camera stabilization
    if (featureIsEnabled(FEATURE_SERVO_TILT)) {
        // center at fixed position, or vary either pitch or roll by RC channel
        servo[SERVO_GIMBAL_PITCH] = determineServoMiddleOrForwardFromChannel(SERVO_GIMBAL_PITCH);
        servo[SERVO_GIMBAL_ROLL] = determineServoMiddleOrForwardFromChannel(SERVO_GIMBAL_ROLL);

        if (IS_RC_MODE_ACTIVE(BOXCAMSTAB)) {
            if (gimbalConfig()->mode == GIMBAL_MODE_MIXTILT) {
                servo[SERVO_GIMBAL_PITCH] -= (-(int32_t)servoParams(SERVO_GIMBAL_PITCH)->rate) * attitude.values.pitch / 50 - (int32_t)servoParams(SERVO_GIMBAL_ROLL)->rate * attitude.values.roll / 50;
                servo[SERVO_GIMBAL_ROLL] += (-(int32_t)servoParams(SERVO_GIMBAL_PITCH)->rate) * attitude.values.pitch / 50 + (int32_t)servoParams(SERVO_GIMBAL_ROLL)->rate * attitude.values.roll / 50;
            } else {
                servo[SERVO_GIMBAL_PITCH] += (int32_t)servoParams(SERVO_GIMBAL_PITCH)->rate * attitude.values.pitch / 50;
                servo[SERVO_GIMBAL_ROLL] += (int32_t)servoParams(SERVO_GIMBAL_ROLL)->rate * attitude.values.roll  / 50;
            }
        }
    }

    // Scale and constrain servos — asymmetric support with AUX forwarding protection
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        scaleServoOutput(i);
    }
}

bool isMixerUsingServos(void)
{
    return useServo;
}

static biquadFilter_t servoFilter[MAX_SUPPORTED_SERVOS];

void servosFilterInit(void)
{
    if (servoConfig()->servo_lowpass_freq) {
        for (int servoIdx = 0; servoIdx < MAX_SUPPORTED_SERVOS; servoIdx++) {
            biquadFilterInitLPF(&servoFilter[servoIdx], servoConfig()->servo_lowpass_freq, targetPidLooptime);
        }
    }

}
static void filterServos(void)
{
#if defined(MIXER_DEBUG)
    uint32_t startTime = micros();
#endif
    if (servoConfig()->servo_lowpass_freq) {
        for (int servoIdx = 0; servoIdx < MAX_SUPPORTED_SERVOS; servoIdx++) {
            servo[servoIdx] = lrintf(biquadFilterApply(&servoFilter[servoIdx], (float)servo[servoIdx]));
            // Sanity check
            servo[servoIdx] = constrain(servo[servoIdx], servoParams(servoIdx)->min, servoParams(servoIdx)->max);
        }
    }
#if defined(MIXER_DEBUG)
    debug[0] = (int16_t)(micros() - startTime);
#endif
}
#endif // USE_SERVOS
