#ifndef MISSION_CONFIG_H
#define MISSION_CONFIG_H

//MISSION PARAMETERS
//tune values per-flight based on sim data / motor specs

//telemetry logging rates
#define standbyLogging 2 //slow logging while on pad
#define readyLogging 10 //few minutes before liftoff
#define boostLogging 200 //high rate through powered flight
#define apogeeLogging 200 //keep high rate through the apogee event itself
#define freefallLogging 10 //reduce logging rate during coast
#define landedLogging 1 //slow logging after landing

//boost / burnout detection
//high-g threshold that confirms motor is actually burning
#define boostConfirmGs 3.0f
#define boostMaxDuration 30000UL //hard ceiling as per a 20-30s burn spec

//burnout / zero-g detection with debounce
//we do not trust a single sample near 0g - inertia/flex causes the signal to oscillate (+g -> 0g -> -g -> 0g -> +g arbitrary) for tens
//of ms after actual burnout. we require n consecutive samples inside the zero-g band before declaring burnout detected
#define zeroGBand 0.5
#define burnoutDebounceSamples 3 //consecutive in-band samples required
#define IMUsampleInterval 5 //200hz imu sampling during boost

//coast lockout (anti-misfire for apogee pyros)
//after burnout is confirmed, apogee-detection logic is LOCKED OUT for this long, which is the primary defense against the +g/0g/-g inertial
//oscillation being mistaken for multiple apogees
#define coastLockoutMinMS 3000UL //do not evaluate apogee before this

//apogee detection
//primary logic -> barometric - n consecutive samples where altitude stops increasing (max has not increased) = apogee.
#define barometricSampling 5 //to check if max altitude is reached
#define barometricSamplingInterval 20 //50Hz baro polling during coast

//secondary logic -> IMU vote - net accel sign flip to negative (deceleration past drag only) adds confidence but isnt sufficient alone because of debounce
#define IMUApogeeVote 0.3f

//backup logic -> absolute max time from liftoff to apogee (from simulations, +20% margin), even if we hit this with NO baro/IMU confirmation at all, parachute pyros will fire
#define apogeeTimeoutMS 45000UL //ADJUST from simulated apogee time

//GPS is expected to lose lock above this velocity (missile control rules)
//above mach 2 for this rocket this WILL happen and treating as expected
#define GPSMaxVel 500.0f

//freefall / landing detection
#define landedAccel 0.1f //accel settles near 1g 
#define altitudeChange 1.0f //altitude change per sample below this
#define landingConfirmSamples 20 // ~2s at 10Hz before declaring landed
#define landedSamplingInterval 100

//pyro
#define pyroActiveDuration 1000UL //defines how long the mosfet gate stays pulled high
#define pyroNumber 4

//ready state params
#define minimumSatellitesInView 5
#define validBaroSamples 10
#define validIMUSamples 10

#endif //MISSION_CONFIG_H
