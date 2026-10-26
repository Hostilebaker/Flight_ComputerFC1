#pragma once
#include <stdint.h>

//flight states
enum class FlightState : uint8_t {
    STANDBY,
    READY,
    BOOST,
    APOGEE,
    FREEFALL,
    LANDED
};

//pyro defs
#define NUM_PYRO_CHANNELS 4
#define PYRO_IDX_APOGEE1 0   //drogue chute 1
#define PYRO_IDX_APOGEE2 1   //redundant drogue chute 2
#define PYRO_IDX_MAIN1   2   //main chute 1
#define PYRO_IDX_MAIN2   3   //redundant main chute 2

//***********NO IMU**********
static const uint8_t READY_MIN_SATELLITES = 3;
static const float LIFTOFF_VSPEED_MPS = 10.0f;
static const float LIFTOFF_MIN_ALTITUDE_M = 10.0f;
static const float BURNOUT_LOW_VSPEED_MPS = 5.0f;
static const uint8_t BURNOUT_DEBOUNCE_SAMPLES = 3;
static const uint32_t BOOST_MAX_DURATION_MS = 15000;
static const uint32_t COAST_LOCKOUT_MIN_MS = 2000;
static const uint8_t GPS_APOGEE_CONFIRM_SAMPLES = 3;
static const uint32_t APOGEE_HARD_TIMEOUT_MS = 30000;
static const float MAIN_DEPLOY_ALTITUDE_M = 500.0f;
static const float LANDED_ALT_DELTA_M = 2.0f;
static const float LANDED_MAX_VSPEED_MPS = 1.5f;
static const float LANDED_CHECK_ALTITUDE_M = 30.0f;
static const uint8_t LANDED_CONFIRM_SAMPLES = 5;

//baro/gps complementary filter
static const float BARO_GPS_COMPLEMENTARY_ALPHA = 0.98f;

static const float RATE_HZ_STANDBY = 1.0f;
static const float RATE_HZ_READY = 1.0f;
static const float RATE_HZ_BOOST = 5.0f;
static const float RATE_HZ_APOGEE = 5.0f;
static const float RATE_HZ_FREEFALL = 2.0f;
static const float RATE_HZ_LANDED = 1.0f;

struct SensorData {
    uint32_t timestamp_ms;

    bool gps_lock;
    uint8_t gps_satellites;
    bool gps_alt_valid;
    float gps_altitude_agl_m;
    float gps_latitude_deg;
    float gps_longitude_deg;

    bool baro_alt_valid;
    float baro_altitude_agl_m;

    bool rf_link_ok;
};

struct FlightCommands {
    bool fire_pyro[NUM_PYRO_CHANNELS];
    float telemetry_rate_hz;
    bool log_enable;

    //RGB state indicator
    bool rgb_r;
    bool rgb_g;
    bool rgb_b;
};

class StateMachine {
public:
    StateMachine();

    FlightCommands update(const SensorData& snap);
    const char* getStateName() const;
    FlightState getState() const { return currentState; }
    float getFusedAltitude_m() const { return fusedAltitude_m; }
    float getTelemetryRateHz() const;

private:
    FlightState currentState;
    uint32_t stateEnteredAt_ms;

    //liftoff/burnout tracking
    bool boostDetected;
    uint32_t boostStartTime_ms;
    uint8_t lowVspeedConsecutiveCount;
    bool burnoutConfirmed;
    uint32_t burnoutConfirmedTime_ms;

    //apogee tracking
    float rollingMaxAltitude_m;
    uint8_t gpsApogeeConsecutiveCount;
    bool apogeeFired;
    bool mainDeployed;

    //landed tracking
    float lastAltitude_m;
    uint8_t landedConsecutiveCount;

    //vertical speed estimate
    float lastVspeedAltitude_m;
    uint32_t lastVspeedTime_ms;
    bool haveLastSample;

    //fused altitude tracking
    float fusedAltitude_m;
    float lastBaroAltitude_m;
    bool haveLastBaroSample;
    bool haveFusedAltitude;

    bool pyroFireLatched[NUM_PYRO_CHANNELS];

    void enterState(FlightState newState, uint32_t now_ms);
    void resetBoostTracking();
    void resetApogeeTracking();
    float computeVerticalSpeed(const SensorData& s);
    float computeFusedAltitude(const SensorData& s, bool& altValidOut);
    void setStateOutputs(FlightCommands& cmd);

    FlightState handleStandby(const SensorData& s);
    FlightState handleReady(const SensorData& s, float vspeed);
    FlightState handleBoost(const SensorData& s, float vspeed);
    FlightState handleApogee(const SensorData& s, FlightCommands& cmd, float fusedAlt, bool altValid);
    FlightState handleFreefall(const SensorData& s, FlightCommands& cmd, float fusedAlt, bool altValid, float vspeed);
    FlightState handleLanded(const SensorData& s, float fusedAlt, bool altValid);
};
