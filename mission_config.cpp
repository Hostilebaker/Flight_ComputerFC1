#ifndef MISSION_CONFIG_H
#define MISSION_CONFIG_H

//MISSION PARAMETERS
//tune values per-flight based on sim data / motor specs.

//telemetry logging rates
#define RATE_HZ_STANDBY 2 //slow logging while on pad
#define RATE_HZ_READY 10 //few minutes before liftoff
#define RATE_HZ_BOOST 200 //high rate through powered flight
#define RATE_HZ_APOGEE 200 //keep high rate through the apogee event itself
#define RATE_HZ_FREEFALL 10 //reduce logging rate during coast
#define RATE_HZ_LANDED 1 //slow logging after landing

//boost / burnout detection
//high-g threshold that confirms motor is actually burning
//(must clear this before we even start looking for burnout)
#define BOOST_CONFIRM_ACCEL_G 3.0f
#define BOOST_MAX_DURATION_MS 30000UL //hard ceiling as per a 20-30s burn spec

//burnout / zero-g detection with debounce.
//we do not trust a single sample near 0g - inertia/flex causes the
//signal to oscillate (+g -> 0g -> -g -> 0g -> +g arbitrary) for tens
//of ms after actual burnout. we require n consecutive samples inside
//the zero-g band before declaring burnout detected.
#define ZERO_G_BAND 0.5
#define BURNOUT_DEBOUNCE_SAMPLES 3 //consecutive in-band samples required
#define IMU_SAMPLE_INTERVAL_MS 5 //200hz imu sampling during boost

//coast lockout (anti-misfire for apogee pyros)
//after burnout is confirmed, apogee-detection logic is LOCKED OUT for
//this long. this is the primary defense against the +g/0g/-g inertial
//oscillation being mistaken for apogee.
#define COAST_LOCKOUT_MIN_MS 3000UL //do not evaluate apogee before this

//apogee detection
//primary logic -> barometric - n consecutive samples where altitude stops
//increasing (rolling max has not increased) = apogee.
#define BARO_APOGEE_CONFIRM_SAMPLES 5
#define BARO_APOGEE_SAMPLE_INTERVAL_MS 20 //50Hz baro polling during coast

//secondary logic -> IMU vote - net accel sign flip to negative (deceleration
//past drag-only) adds confidence but isnt sufficient alone.
#define IMU_APOGEE_VOTE_BAND 0.3f

//backup logic -> absolute max time from liftoff to apogee (from sim,
//+20% margin). even if we hit this with NO baro/IMU confirmation at all,
//fire apogee pyros anyway.
#define APOGEE_HARD_TIMEOUT_MS 45000UL //ADJUST from simulated apogee time

//GPS
//GPS is expected to lose lock above this velocity (module spec limit).
//above mach 2 for this airframe this WILL happen and treating as expected,
//not a fault, and do not fire any aborts off of it alone.
#define GPS_MAX_VELOCITY_MPS 500.0f

//freefall / landing detection
#define LANDED_ACCEL_STABLE_G 0.1f // accel settles near 1g (gravity only)
#define LANDED_ALT_DELTA_M 1.0f // altitude change per sample below this
#define LANDED_CONFIRM_SAMPLES 20 // ~2s at 10Hz before declaring landed
#define LANDED_SAMPLE_INTERVAL_MS 100

//pyro
#define PYRO_FIRE_DURATION_MS 1000UL //defines how long the mosfet gate stays active
#define NUM_PYRO_CHANNELS 4

//ready-state parameters
#define READY_MIN_SATELLITES 5
#define READY_BARO_VALID_SAMPLES 10
#define READY_IMU_VALID_SAMPLES 10

#endif //MISSION_CONFIG_H
