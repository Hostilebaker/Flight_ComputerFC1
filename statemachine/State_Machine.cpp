#include "state_machine.h"

StateMachine::StateMachine() {
    currentState = FlightState::STANDBY;
    stateEnteredAt_ms = 0;
    boostDetected = false;
    boostStartTime_ms = 0;
    zeroGConsecutiveCount = 0;
    burnoutConfirmedTime_ms = 0;
    burnoutConfirmed = false;
    rollingMaxAltitude_m = -999999.0f;
    baroApogeeConsecutiveCount = 0;
    apogeeFired = false;
    lastAltitude_m = 0;
    landedConsecutiveCount = 0;
    apogeeChannel1 = 0;
    apogeeChannel2 = 1;
    for (int i = 0; i < NUM_PYRO_CHANNELS; i++) pyroFireLatched[i] = false;
}

void StateMachine::setApogeePyroChannels(uint8_t ch1, uint8_t ch2) {
    apogeeChannel1 = ch1;
    apogeeChannel2 = ch2;
}

void StateMachine::setBoostPyroChannel(uint8_t channel) {
    (void)channel;
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
    zeroGConsecutiveCount = 0;
    burnoutConfirmed = false;
    burnoutConfirmedTime_ms = 0;
}

void StateMachine::resetApogeeTracking() {
    rollingMaxAltitude_m = -999999.0f;
    baroApogeeConsecutiveCount = 0;
    apogeeFired = false;
}

//main updaet
FlightCommands StateMachine::update(const SensorData& snap) {
    FlightCommands cmd = {};
    cmd.telemetry_rate_hz = RATE_HZ_STANDBY;
    for (int i = 0; i < NUM_PYRO_CHANNELS; i++) cmd.fire_pyro[i] = false;
    cmd.buzzer_on = false;
    cmd.log_to_sd = false;

    FlightState nextState = currentState;

    switch (currentState) {
        case FlightState::STANDBY:
            cmd.telemetry_rate_hz = RATE_HZ_STANDBY;
            cmd.rgb_r = 255; cmd.rgb_g = 0; cmd.rgb_b = 0;//red -> unsafe/not armed
            nextState = handleStandby(snap);
            break;

        case FlightState::READY:
            cmd.telemetry_rate_hz = RATE_HZ_READY;
            cmd.log_to_sd = true;
            //blue -> pyros armed, green -> safe but not armed
            if (snap.armed_switch_on) {
                cmd.rgb_r = 0; cmd.rgb_g = 0; cmd.rgb_b = 255;
            } else {
                cmd.rgb_r = 0; cmd.rgb_g = 255; cmd.rgb_b = 0;
            }
            nextState = handleReady(snap);
            break;

        case FlightState::BOOST:
            cmd.telemetry_rate_hz = RATE_HZ_BOOST;
            cmd.log_to_sd = true;
            cmd.rgb_r = 0; cmd.rgb_g = 0; cmd.rgb_b = 255;
            nextState = handleBoost(snap, cmd);
            break;

        case FlightState::APOGEE:
            cmd.telemetry_rate_hz = RATE_HZ_APOGEE;
            cmd.log_to_sd = true;
            cmd.rgb_r = 255; cmd.rgb_g = 0; cmd.rgb_b = 255;//magenta -> event in progress
            nextState = handleApogee(snap, cmd);
            break;

        case FlightState::FREEFALL:
            cmd.telemetry_rate_hz = RATE_HZ_FREEFALL;
            cmd.log_to_sd = true;
            cmd.rgb_r = 0; cmd.rgb_g = 255; cmd.rgb_b = 255;//cyan
            nextState = handleFreefall(snap);
            break;

        case FlightState::LANDED:
            cmd.telemetry_rate_hz = RATE_HZ_LANDED;
            cmd.log_to_sd = true;
            cmd.buzzer_on = true;
            cmd.rgb_r = 255; cmd.rgb_g = 255; cmd.rgb_b = 0;//yellow
            nextState = handleLanded(snap);
            break;
    }

    if (nextState != currentState) {
        enterState(nextState, snap.timestamp_ms);
    }

    return cmd;
}

//standby -> ready
FlightState StateMachine::handleStandby(const SensorData& s) {
    //gate on GPS lock + sat count, valid baro, valid IMU, RF link up; this function assumes the caller (main loop) is already tracking
    //n valid samples upstream via the sensor drivers - here we just check the instantaneous valid flags plus sat count, since sample
    //counting belongs with the driver, not the state machine
    bool gpsOk  = s.gps_lock && (s.gps_satellites >= READY_MIN_SATELLITES);
    bool baroOk = s.baro_valid;
    bool imuOk  = s.imu_valid;
    bool rfOk   = s.rf_link_ok;

    if (gpsOk && baroOk && imuOk && rfOk) {
        return FlightState::READY;
    }
    return FlightState::STANDBY;
}

//ready -> boost
//boost begins the moment sustained high-g is sensed, the vehicle will boost whether or not pyros are armed, and we still want telemetry
//to reflect reality, but note: if armed_switch_on is false, the pyro fire commands later in APOGEE will still be controlled by
//hardware (manual switch cuts power to MOSFET channels) - firmware mirrors that by never asserting fire_pyro if switch is off, as a
//second layer of protection
FlightState StateMachine::handleReady(const SensorData& s) {
    if (!s.imu_valid) return FlightState::READY;

    if (s.accel_y_g >= BOOST_CONFIRM_ACCEL_G) {
        resetBoostTracking();
        boostDetected = true;
        boostStartTime_ms = s.timestamp_ms;
        resetApogeeTracking();
        return FlightState::BOOST;
    }
    return FlightState::READY;
}

//boost -> apogee
//burnout detection with debounce -> require BURNOUT_DEBOUNCE_SAMPLES
//consecutive samples inside the zero-g band before declaring burnout, this is what filters out the +g/0g/-g/0g/+g inertial oscillation, a single in-band sample proves
//nothing, but N in a row (spaced at the IMU sample interval) means the airframe has actually settled into coast.
//once burnout is confirmed, we move into APOGEE state, but APOGEE state itself enforces the coast lockout before it will
//look at apogee detection at all, BOOST's job is just to detect burnout and hand off
FlightState StateMachine::handleBoost(const SensorData& s, FlightCommands& cmd) {
    uint32_t elapsed = s.timestamp_ms - boostStartTime_ms;

    //hard ceiling failsafe -> if boost has run longer than real
    //motor burntime (as per a 20-30s burntime), something is wrong
    //with detection - force the transition anyway rather than
    //getting stuck in boost forever
    if (elapsed >= BOOST_MAX_DURATION_MS) {
        resetApogeeTracking();
        return FlightState::APOGEE;
    }

    if (!s.imu_valid) {
        //losing IMU mid-boost, fall back to the hard timer alone for this flight (handled by the ceiling check above every cycle)
        return FlightState::BOOST;
    }

    bool inZeroGBand = fabsf(s.accel_y_g) <= ZERO_G_BAND;

    if (inZeroGBand) {
        zeroGConsecutiveCount++;
    } else {
        zeroGConsecutiveCount = 0;//any out of band sample resets the debounce
    }

    if (zeroGConsecutiveCount >= BURNOUT_DEBOUNCE_SAMPLES && !burnoutConfirmed) {
        burnoutConfirmed = true;
        burnoutConfirmedTime_ms = s.timestamp_ms;
        resetApogeeTracking();
        return FlightState::APOGEE;
    }

    return FlightState::BOOST;
}

//apogee -> freefall
//three layers, in priority order:
//1.coast lockout - refuse to even evaluate apogee logic until COAST_LOCKOUT_MIN_MS has passed since burnout, this alone
//should get rid of the inertial-oscillation misfire risk, since that oscillation only lasts tens of ms, nowhere near the lockout
//
//2.barometric primary detection - altitude rolling max stops increasing for BARO_APOGEE_CONFIRM_SAMPLES in a row, immune
//to IMU vibration/noise, reliable through coast which is dominated by air drag IMU zero-g/sign vote is logged but not required, baro alone
//is sufficient to fire, since baro is the better sensor for this specific event + we will have an additional one in the nosecone so that also helps 
//
//3.hard timeout backup - APOGEE_HARD_TIMEOUT_MS since liftoff (approximated here as since boost start) with NO confirmation
//yet -> fire anyway; last-resort failsafe if there is a total sensor loss....
FlightState StateMachine::handleApogee(const SensorData& s, FlightCommands& cmd) {
    uint32_t sinceBurnout = s.timestamp_ms - burnoutConfirmedTime_ms;
    uint32_t sinceLiftoff = s.timestamp_ms - boostStartTime_ms;

    bool lockoutExpired = sinceBurnout >= COAST_LOCKOUT_MIN_MS;
    bool hardTimeoutHit = sinceLiftoff >= APOGEE_HARD_TIMEOUT_MS;

    bool shouldFire = false;

    if (!apogeeFired && lockoutExpired && s.baro_valid) {
        if (s.baro_altitude_m > rollingMaxAltitude_m) {
            rollingMaxAltitude_m = s.baro_altitude_m;
            baroApogeeConsecutiveCount = 0;//still climbing, reset confirm count
        } else {
            baroApogeeConsecutiveCount++;
        }

        if (baroApogeeConsecutiveCount >= BARO_APOGEE_CONFIRM_SAMPLES) {
            shouldFire = true;
        }
    }

    //backup pyro firing regardless of sensor state if we've gone past the absolute max time to apogee with no confirmation
    if (!apogeeFired && hardTimeoutHit) {
        shouldFire = true;
    }

    if (shouldFire && !apogeeFired) {
        apogeeFired = true;
        //only actually pull the fire command high if the hardware arming switch is on
        if (s.armed_switch_on) {
            if (!pyroFireLatched[apogeeChannel1]) {
                cmd.fire_pyro[apogeeChannel1] = true;
                pyroFireLatched[apogeeChannel1] = true;
            }
            if (!pyroFireLatched[apogeeChannel2]) {
                cmd.fire_pyro[apogeeChannel2] = true;
                pyroFireLatched[apogeeChannel2] = true;
            }
        }
        lastAltitude_m = s.baro_altitude_m;
        landedConsecutiveCount = 0;
        return FlightState::FREEFALL;
    }

    return FlightState::APOGEE;
}

//freefall -> landed state
FlightState StateMachine::handleFreefall(const SensorData& s) {
    return handleLanded(s) == FlightState::LANDED ? FlightState::LANDED : FlightState::FREEFALL;
}

FlightState StateMachine::handleLanded(const SensorData& s) {
    if (!s.baro_valid || !s.imu_valid) {
        landedConsecutiveCount = 0;
        return (currentState == FlightState::LANDED) ? FlightState::LANDED : FlightState::FREEFALL;
    }

    float altDelta = fabsf(s.baro_altitude_m - lastAltitude_m);
    float totalAccel = sqrtf(s.accel_x_g * s.accel_x_g +
                              s.accel_y_g * s.accel_y_g +
                              s.accel_z_g * s.accel_z_g);
    bool accelStable = fabsf(totalAccel - 1.0f) <= LANDED_ACCEL_STABLE_G;
    bool altStable = altDelta <= LANDED_ALT_DELTA_M;

    lastAltitude_m = s.baro_altitude_m;

    if (accelStable && altStable) {
        landedConsecutiveCount++;
    } else {
        landedConsecutiveCount = 0;
    }

    if (landedConsecutiveCount >= LANDED_CONFIRM_SAMPLES) {
        return FlightState::LANDED;
    }

    return (currentState == FlightState::LANDED) ? FlightState::LANDED : FlightState::FREEFALL;
}
