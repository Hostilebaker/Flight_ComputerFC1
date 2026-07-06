#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <Arduino.h>
#include "mission_config.h"

//state machine for the flight computer

enum class FlightState : uint8_t {
    STANDBY = 0,
    READY,
    BOOST,
    APOGEE,
    FREEFALL,
    LANDED
};

//plain data snapshot passed into the state machine each loop.
//driver modules (IMU, baro, GPS, etc.) fill this; the state
//machine does not know how to talk to any sensor directly.
//this keeps state logic testable without real hardware.
struct SensorData {
    //IMU
    float accel_x_g;
    float accel_y_g;   //assuming Y is the thrust axis
    float accel_z_g;
    bool  imu_valid;

    //barometers, onboard is primary, external is secondary
    float baro_altitude_m;
    float baro_pressure_pa;
    bool  baro_valid;

    //GPS
    uint8_t gps_satellites;
    bool    gps_lock;
    float   gps_velocity_mps;

    bool rf_link_ok;//RF link

    bool pyro_continuity[NUM_PYRO_CHANNELS];//pyro continuity (per channel, true = continuity present)

    bool armed_switch_on;//arming switch (hardware manual switch state)
    uint32_t timestamp_ms;
};

//output commands the state machine wants executed this loop.
//kept separate from SensorSnapshot so the state machine is a
//pure function of (state, snapshot) -> (new state, commands).
struct FlightCommands {
    bool fire_pyro[NUM_PYRO_CHANNELS];//edge-triggered fire request
    uint16_t telemetry_rate_hz;
    uint8_t  rgb_r, rgb_g, rgb_b;
    bool     buzzer_on;
    bool     log_to_sd;
};

class StateMachine {
public:
    StateMachine();

    //call every loop with the latest sensor snapshot.
    //returns commands to execute this cycle.
    FlightCommands update(const SensorData& snap);

    FlightState getState() const { return currentState; }
    const char* getStateName() const;

    //which pyro channels are assigned to which event.
    //configure once at startup
    void setBoostPyroChannel(uint8_t channel);//which channel = ignition confirm (usually N/A, boost is motor-lit)
    void setApogeePyroChannels(uint8_t ch1, uint8_t ch2);//drogue/main or both mains

private:
    FlightState currentState;
    uint32_t stateEnteredAt_ms;

    //boost/burnout tracking
    bool boostDetected;
    uint32_t boostStartTime_ms;
    uint8_t zeroGConsecutiveCount;
    uint32_t burnoutConfirmedTime_ms;
    bool burnoutConfirmed;

    //apogee tracking
    float rollingMaxAltitude_m;
    uint8_t baroApogeeConsecutiveCount;
    bool apogeeFired;

    //landed tracking
    float lastAltitude_m;
    uint8_t landedConsecutiveCount;

    //pyro config
    uint8_t apogeeChannel1, apogeeChannel2;

    //edge-triggered fire latches so we command each pyro exactly once
    bool pyroFireLatched[NUM_PYRO_CHANNELS];

    //per-state handlers
    FlightState handleStandby(const SensorData& s);
    FlightState handleReady(const SensorData& s);
    FlightState handleBoost(const SensorData& s, FlightCommands& cmd);
    FlightState handleApogee(const SensorData& s, FlightCommands& cmd);
    FlightState handleFreefall(const SensorData& s);
    FlightState handleLanded(const SensorData& s);

    void resetBoostTracking();
    void resetApogeeTracking();
    void enterState(FlightState newState, uint32_t now_ms);
};

#endif //STATE_MACHINE_H
