#include "state_machine.h"
#include <math.h>

StateMachine::StateMachine() {
    currentState = FlightState::STANDBY;
    stateEnteredAt_ms = 0;

    boostDetected = false;
    boostStartTime_ms = 0;
    lowVspeedConsecutiveCount = 0;
    burnoutConfirmed = false;
    burnoutConfirmedTime_ms = 0;

    rollingMaxAltitude_m = -999999.0f;
    gpsApogeeConsecutiveCount = 0;
    apogeeFired = false;
    mainDeployed = false;

    lastAltitude_m = 0;
    landedConsecutiveCount = 0;

    lastVspeedAltitude_m = 0;
    lastVspeedTime_ms = 0;
    haveLastSample = false;

    fusedAltitude_m = 0;
    lastBaroAltitude_m = 0;
    haveLastBaroSample = false;
    haveFusedAltitude = false;

    for (int i = 0; i < NUM_PYRO_CHANNELS; i++)
        pyroFireLatched[i] = false;
}

const char* StateMachine::getStateName() const {
    switch (currentState) {
        case FlightState::STANDBY:  return "STANDBY";
        case FlightState::READY:    return "READY";
        case FlightState::BOOST:    return "BOOST";
        case FlightState::APOGEE:   return "APOGEE";
        case FlightState::FREEFALL: return "FREEFALL";
        case FlightState::LANDED:   return "LANDED";
    }
    return "UNKNOWN";
}

void StateMachine::enterState(FlightState newState, uint32_t now_ms) {
    currentState = newState;
    stateEnteredAt_ms = now_ms;
}

void StateMachine::resetBoostTracking() {
    boostDetected = false;
    lowVspeedConsecutiveCount = 0;
    burnoutConfirmed = false;
    burnoutConfirmedTime_ms = 0;
}

void StateMachine::resetApogeeTracking() {
    rollingMaxAltitude_m = -999999.0f;
    gpsApogeeConsecutiveCount = 0;
    apogeeFired = false;
}

float StateMachine::computeVerticalSpeed(const SensorData& s) {
    if (!s.gps_alt_valid) return NAN;

    if (!haveLastSample) {
        lastVspeedAltitude_m = s.gps_altitude_agl_m;
        lastVspeedTime_ms = s.timestamp_ms;
        haveLastSample = true;
        return NAN;
    }

    uint32_t dt_ms = s.timestamp_ms - lastVspeedTime_ms;
    if (dt_ms < 200) return NAN;

    float dt_s = dt_ms / 1000.0f;
    float vspeed = (s.gps_altitude_agl_m - lastVspeedAltitude_m) / dt_s;

    lastVspeedAltitude_m = s.gps_altitude_agl_m;
    lastVspeedTime_ms = s.timestamp_ms;

    return vspeed;
}

float StateMachine::computeFusedAltitude(const SensorData& s, bool& altValidOut) {
    bool baroOk = s.baro_alt_valid;
    bool gpsOk = s.gps_alt_valid;

    altValidOut = baroOk || gpsOk;
    if (!altValidOut) return fusedAltitude_m;

    if (!haveFusedAltitude) {
        fusedAltitude_m = baroOk ? s.baro_altitude_agl_m : s.gps_altitude_agl_m;

        if (baroOk) {
            lastBaroAltitude_m = s.baro_altitude_agl_m;
            haveLastBaroSample = true;
        }

        haveFusedAltitude = true;
        return fusedAltitude_m;
    }

    if (baroOk && gpsOk) {
        float baroDelta = haveLastBaroSample ?
            (s.baro_altitude_agl_m - lastBaroAltitude_m) : 0.0f;

        fusedAltitude_m =
            BARO_GPS_COMPLEMENTARY_ALPHA * (fusedAltitude_m + baroDelta) +
            (1.0f - BARO_GPS_COMPLEMENTARY_ALPHA) * s.gps_altitude_agl_m;

        lastBaroAltitude_m = s.baro_altitude_agl_m;
        haveLastBaroSample = true;

    } else if (baroOk) {
        float baroDelta = haveLastBaroSample ?
            (s.baro_altitude_agl_m - lastBaroAltitude_m) : 0.0f;

        fusedAltitude_m += baroDelta;
        lastBaroAltitude_m = s.baro_altitude_agl_m;
        haveLastBaroSample = true;

    } else {
        fusedAltitude_m = s.gps_altitude_agl_m;
    }

    return fusedAltitude_m;
}

void StateMachine::setStateOutputs(FlightCommands& cmd) {
    //RGB state indicator
    //STANDBY = BLUE
    //READY   = RED
    //BOOST   = MAGENTA
    //APOGEE  = YELLOW
    //FREEFALL = CYAN
    //LANDED  = GREEN
    cmd.rgb_r = false;
    cmd.rgb_g = false;
    cmd.rgb_b = false;

    switch (currentState) {
        case FlightState::STANDBY:
            cmd.rgb_b = true;
            break;

        case FlightState::READY:
            cmd.rgb_r = true;
            break;

        case FlightState::BOOST:
            cmd.rgb_r = true;
            cmd.rgb_b = true;
            break;

        case FlightState::APOGEE:
            cmd.rgb_r = true;
            cmd.rgb_g = true;
            break;

        case FlightState::FREEFALL:
            cmd.rgb_g = true;
            cmd.rgb_b = true;
            break;

        case FlightState::LANDED:
            cmd.rgb_g = true;
            break;
    }
}

float StateMachine::getTelemetryRateHz() const {
    switch (currentState) {
        case FlightState::STANDBY:  return RATE_HZ_STANDBY;
        case FlightState::READY:    return RATE_HZ_READY;
        case FlightState::BOOST:    return RATE_HZ_BOOST;
        case FlightState::APOGEE:   return RATE_HZ_APOGEE;
        case FlightState::FREEFALL: return RATE_HZ_FREEFALL;
        case FlightState::LANDED:   return RATE_HZ_LANDED;
    }
    return 1.0f;
}

FlightCommands StateMachine::update(const SensorData& snap) {
    FlightCommands cmd = {};

    for (int i = 0; i < NUM_PYRO_CHANNELS; i++)
        cmd.fire_pyro[i] = false;

    cmd.telemetry_rate_hz = RATE_HZ_STANDBY;
    cmd.log_enable = false;

    float vspeed = computeVerticalSpeed(snap);

    bool altValid = false;
    float fusedAlt = computeFusedAltitude(snap, altValid);

    FlightState nextState = currentState;

    switch (currentState) {
        case FlightState::STANDBY:
            nextState = handleStandby(snap);
            break;

        case FlightState::READY:
            nextState = handleReady(snap, vspeed);
            break;

        case FlightState::BOOST:
            nextState = handleBoost(snap, vspeed);
            break;

        case FlightState::APOGEE:
            nextState = handleApogee(snap, cmd, fusedAlt, altValid);
            break;

        case FlightState::FREEFALL:
            nextState = handleFreefall(snap, cmd, fusedAlt, altValid, vspeed);
            break;

        case FlightState::LANDED:
            nextState = handleLanded(snap, fusedAlt, altValid);
            break;
    }

    if (nextState != currentState)
        enterState(nextState, snap.timestamp_ms);

    switch (currentState) {
        case FlightState::READY:
            cmd.telemetry_rate_hz = RATE_HZ_READY;
            cmd.log_enable = true;
            break;

        case FlightState::BOOST:
            cmd.telemetry_rate_hz = RATE_HZ_BOOST;
            cmd.log_enable = true;
            break;

        case FlightState::APOGEE:
            cmd.telemetry_rate_hz = RATE_HZ_APOGEE;
            cmd.log_enable = true;
            break;

        case FlightState::FREEFALL:
            cmd.telemetry_rate_hz = RATE_HZ_FREEFALL;
            cmd.log_enable = true;
            break;

        case FlightState::LANDED:
            cmd.telemetry_rate_hz = RATE_HZ_LANDED;
            cmd.log_enable = true;
            break;

        default:
            cmd.telemetry_rate_hz = RATE_HZ_STANDBY;
            cmd.log_enable = false;
            break;
    }

    setStateOutputs(cmd);
    return cmd;
}

FlightState StateMachine::handleStandby(const SensorData& s) {
    bool gpsOk = s.gps_lock && (s.gps_satellites >= READY_MIN_SATELLITES);
    bool rfOk = s.rf_link_ok;

    if (gpsOk && rfOk)
        return FlightState::READY;

    return FlightState::STANDBY;
}

FlightState StateMachine::handleReady(const SensorData& s, float vspeed) {
    if (!s.gps_alt_valid || isnan(vspeed))
        return FlightState::READY;

    if (vspeed >= LIFTOFF_VSPEED_MPS &&
        s.gps_altitude_agl_m > LIFTOFF_MIN_ALTITUDE_M) {

        resetBoostTracking();

        boostDetected = true;
        boostStartTime_ms = s.timestamp_ms;
        burnoutConfirmedTime_ms = s.timestamp_ms;

        resetApogeeTracking();

        return FlightState::BOOST;
    }

    return FlightState::READY;
}

FlightState StateMachine::handleBoost(const SensorData& s, float vspeed) {
    uint32_t elapsed = s.timestamp_ms - boostStartTime_ms;

    if (elapsed >= BOOST_MAX_DURATION_MS) {
        resetApogeeTracking();
        burnoutConfirmedTime_ms = boostStartTime_ms;
        return FlightState::APOGEE;
    }

    if (!s.gps_alt_valid || isnan(vspeed))
        return FlightState::BOOST;

    if (vspeed <= BURNOUT_LOW_VSPEED_MPS) {
        if (lowVspeedConsecutiveCount < BURNOUT_DEBOUNCE_SAMPLES)
            lowVspeedConsecutiveCount++;
    } else {
        lowVspeedConsecutiveCount = 0;
    }

    if (lowVspeedConsecutiveCount >= BURNOUT_DEBOUNCE_SAMPLES &&
        !burnoutConfirmed) {

        burnoutConfirmed = true;
        burnoutConfirmedTime_ms = s.timestamp_ms;
        resetApogeeTracking();

        return FlightState::APOGEE;
    }

    return FlightState::BOOST;
}

FlightState StateMachine::handleApogee(
    const SensorData& s,
    FlightCommands& cmd,
    float fusedAlt,
    bool altValid
) {
    uint32_t sinceBurnout = s.timestamp_ms - burnoutConfirmedTime_ms;
    uint32_t sinceLiftoff = s.timestamp_ms - boostStartTime_ms;

    bool lockoutExpired = sinceBurnout >= COAST_LOCKOUT_MIN_MS;
    bool hardTimeoutHit = sinceLiftoff >= APOGEE_HARD_TIMEOUT_MS;
    bool shouldFire = false;

    if (!apogeeFired && lockoutExpired && altValid) {
        if (fusedAlt > rollingMaxAltitude_m) {
            rollingMaxAltitude_m = fusedAlt;
            gpsApogeeConsecutiveCount = 0;
        } else {
            if (gpsApogeeConsecutiveCount < GPS_APOGEE_CONFIRM_SAMPLES)
                gpsApogeeConsecutiveCount++;
        }

        if (gpsApogeeConsecutiveCount >= GPS_APOGEE_CONFIRM_SAMPLES)
            shouldFire = true;
    }

    if (!apogeeFired && hardTimeoutHit)
        shouldFire = true;

    if (shouldFire && !apogeeFired) {
        apogeeFired = true;

        if (!pyroFireLatched[PYRO_IDX_APOGEE1]) {
            cmd.fire_pyro[PYRO_IDX_APOGEE1] = true;
            pyroFireLatched[PYRO_IDX_APOGEE1] = true;
        }

        if (!pyroFireLatched[PYRO_IDX_APOGEE2]) {
            cmd.fire_pyro[PYRO_IDX_APOGEE2] = true;
            pyroFireLatched[PYRO_IDX_APOGEE2] = true;
        }

        lastAltitude_m = fusedAlt;
        landedConsecutiveCount = 0;

        return FlightState::FREEFALL;
    }

    return FlightState::APOGEE;
}

FlightState StateMachine::handleFreefall(
    const SensorData& s,
    FlightCommands& cmd,
    float fusedAlt,
    bool altValid,
    float vspeed
) {
    if (!mainDeployed &&
        altValid &&
        fusedAlt <= MAIN_DEPLOY_ALTITUDE_M) {

        mainDeployed = true;

        if (!pyroFireLatched[PYRO_IDX_MAIN1]) {
            cmd.fire_pyro[PYRO_IDX_MAIN1] = true;
            pyroFireLatched[PYRO_IDX_MAIN1] = true;
        }

        if (!pyroFireLatched[PYRO_IDX_MAIN2]) {
            cmd.fire_pyro[PYRO_IDX_MAIN2] = true;
            pyroFireLatched[PYRO_IDX_MAIN2] = true;
        }
    }

    if (!altValid) {
        landedConsecutiveCount = 0;
        return FlightState::FREEFALL;
    }

    float altDelta = fabsf(fusedAlt - lastAltitude_m);
    lastAltitude_m = fusedAlt;

    bool lowVerticalSpeed =
        isnan(vspeed) ||
        fabsf(vspeed) <= LANDED_MAX_VSPEED_MPS;

    if (fusedAlt < LANDED_CHECK_ALTITUDE_M &&
        altDelta <= LANDED_ALT_DELTA_M &&
        lowVerticalSpeed) {

        if (landedConsecutiveCount < LANDED_CONFIRM_SAMPLES)
            landedConsecutiveCount++;

    } else {
        landedConsecutiveCount = 0;
    }

    if (landedConsecutiveCount >= LANDED_CONFIRM_SAMPLES)
        return FlightState::LANDED;

    return FlightState::FREEFALL;
}

FlightState StateMachine::handleLanded(
    const SensorData& s,
    float fusedAlt,
    bool altValid
) {
    return FlightState::LANDED;
}
