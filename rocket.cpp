#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <Adafruit_MAX31855.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_Sensor.h>
#include <LittleFS.h>
#include <SD.h>
#include <TinyGPS++.h>

//pin defs
//spi bus
#define spi_sck 36
#define spi_miso 37
#define spi_mosi 35
//i2c bus
#define i2c_sda 8
#define i2c_scl 17
//cs lines
#define rf_cs 4
#define sd_cs 16
#define cs_temp 33
#define cs_baro 34
#define cs_baro2 47
//mosfet outputs
#define Pyro1 14 //drogue chute at apogee
#define Pyro2 2 //main chute at 1kfeet (304m)
#define Pyro3 9
#define Pyro4 1
//rf
#define rf_reset 5
#define dio0 6
#define dio1 7
//status led
#define rgb_red 40
#define rgb_green 41
#define rgb_blue 42
#define buzzer 45
#define gps_reset 3
#define gps_txd 13
#define gps_rxd 48
#define gps_led 18
#define vbat_sense 15 //analog
#define user_sw 0

#define sealevel_HPA (1013.25f)

//parameters
#define boostConfirm 3.0f //3g accel threshold for boost state
#define timetoApogee 30000 //going by 30s max time to apogee
#define apogeeDuration 3000 //apogee state lasts for 3 seconds
#define Pyro2Deploy 304.8f //main parachute deploument at 1k feet or 304.8m
#define GPS_LOCK_TIMEOUT_MS 10000 //10s gps timeout constant

//flight states
enum FlightState {
  Safe = 0,
  Armed,
  Boost,
  Apogee,
  Descent,
  Landed
};
struct __attribute__((packed)) TelemetryPacket { //forces the compiler to remove hidden padding bytes and guarantees 25 bytes packet everywhere
  //normal "struct TelemetryPacket" would leave byte alignment upto the cpu, no guarantee of perfect alignment of bits with reciever because
  //it would add padding bits to the transmitted packet
  uint32_t timestamp_ms; //milliseconds since boot -> better accuracy for 200hz
  int32_t gps_lat;
  int32_t gps_lon; //gps lat, lon * 10000 
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z; //accel x, y, z * 10000
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z; //gyro x, y, z * 10000
  uint8_t battery; //volts*10
  int8_t rssi; //dBm
  uint8_t flags; //deploy and status bitmask
};

SPIClass sharedSPI(FSPI);
SX1276 radio = new Module(rf_cs, dio0, rf_reset, dio1, sharedSPI);
Adafruit_MAX31855 thermocouple(cs_temp, &sharedSPI);
Adafruit_BMP3XX bmpOnboard;
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

//global state variables
FlightState currentState = Safe; //fc boots up in safe state
uint32_t stateBegin = 0; 
uint32_t boostBegin = 0;
float maxAlt = -999.0f;
uint8_t baroFallingCount = 0;
bool drogueFired = false;
bool mainFired = false;
uint32_t drogueFireStart = 0;
uint32_t mainFireStart = 0;
bool droguePulseActive = false;
bool mainPulseActive = false;
uint32_t lastLogTime = 0;
uint32_t lastTelemetryTime = 0;
uint32_t flashLog = 0;
bool sdDump = false;
int32_t currentLatScaled = 0;
int32_t currentLonScaled = 0;
bool gpsHasFix = false;
uint32_t lastGPSResetAttempt = 0;
//20 packet ram buffer before flushing to flash
#define BUFFER_SIZE 20
TelemetryPacket logBuffer[BUFFER_SIZE];
uint8_t bufferIndex = 0;
void setRGB(uint8_t r, uint8_t g, uint8_t b) {
  analogWrite(rgb_red, r);
  analogWrite(rgb_green, g);
  analogWrite(rgb_blue, b);
}
void triggerGPSReset(){
  Serial.println(F("[GPS] triggering gps reset pulse..."));
  pinMode(gps_reset, OUTPUT);
  digitalWrite(gps_reset, LOW);
  delay(50); //>10ms pulse reqd. as per the datasheet of lc76gabmd
  digitalWrite(gps_reset, HIGH);
  pinMode(gps_reset, INPUT);
  lastGPSResetAttempt = millis();
}
void flushBufferToFlash(){
  if (bufferIndex == 0) return;
  File flashFile = LittleFS.open("/flightLog.bin", "a"); //append binary
  if (flashFile){
    flashFile.write((uint8_t*)logBuffer, bufferIndex * sizeof(TelemetryPacket));
    flashFile.close();
    bufferIndex = 0; //reset buffer
  }
}
void logFlash (TelemetryPacket &pkt){
  logBuffer[bufferIndex++] = pkt;
  //dump to flash when buffer fills(~20seconds)
  if (bufferIndex >= BUFFER_SIZE){
    flushBufferToFlash();
  }
}
void dumpIntoSD(){ //dumps flash memory into sd card once landed state is achieved
  flushBufferToFlash();
  Serial.println(F("[STORAGE] sd card init..."));
  if (!SD.begin(sd_cs, sharedSPI)){
    Serial.println(F("[STORAGE] sd card mount failed"));
    return;
  }
  File flashFile = LittleFS.open("/flightLog.bin", "r");
  if (!flashFile){
    Serial.println(F("[STORAGE] no littleFS log found to dump"));
    return;
  }
  File sdFile = SD.open("/flightLog.bin", FILE_APPEND);
  if (!sdFile){
    Serial.println(F("[STORAGE] failed to open sd /flightLog.bin for writing"));
    flashFile.close();
    return;
  }
  Serial.printf("[STORAGE] dumping %u packets from flash into sd...\n", flashFile.size());
  uint8_t chunkBuffer[256];
  while(flashFile.available()){
    size_t bytesRead = flashFile.read(chunkBuffer, sizeof(chunkBuffer));
    sdFile.write(chunkBuffer, bytesRead);
  }
  flashFile.close();
  sdFile.close();
  Serial.println(F("[STORAGE] flash to sd dump complete..."));
  sdDump = true;
}

void setup(){
  Serial.begin(115200);
  //chip select pins ar active-LOW, hence pulling everything high before SPI is init, no sensor is selected
  pinMode(rf_cs, OUTPUT);
  digitalWrite(rf_cs, HIGH);
  pinMode(sd_cs, OUTPUT);
  digitalWrite(sd_cs, HIGH);
  pinMode(cs_temp, OUTPUT);
  digitalWrite(cs_temp, HIGH);
  pinMode(cs_baro, OUTPUT);
  digitalWrite(cs_baro, HIGH);
  pinMode(cs_baro2, OUTPUT);
  digitalWrite(cs_baro2, HIGH);
  //safety off for pyrochannels
  digitalWrite(Pyro1, LOW);
  pinMode(Pyro1, OUTPUT);
  digitalWrite(Pyro2, LOW);
  pinMode(Pyro2, OUTPUT);
  digitalWrite(Pyro3, LOW);
  pinMode(Pyro3, OUTPUT);
  digitalWrite(Pyro4, LOW);
  pinMode(Pyro4, OUTPUT);
  pinMode(user_sw, INPUT_PULLUP);
  pinMode(rgb_red, OUTPUT);
  pinMode(rgb_green, OUTPUT);
  pinMode(rgb_blue, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(gps_reset, OUTPUT);
  digitalWrite(gps_reset, HIGH);
  pinMode(gps_led, OUTPUT);
  digitalWrite(gps_led, LOW); //off until a gps fix is achieved
  setRGB(0,0,255); //fc boots in blue color
  gpsSerial.begin(115200, SERIAL_8N1, gps_rxd, gps_txd); //gps module init at 115200 baudrate
  
  //LittleFS internal flash init
  if (!LittleFS.begin(true)){
    Serial.println(F("[STORAGE] LittleFS mount failed!"));
  } else{
    Serial.println(F("[STORAGE] LittleFS flash storage ready"));
  }

  //spi init
  sharedSPI.begin(spi_sck, spi_miso, spi_mosi);
  //i2c init
  Wire.begin(i2c_sda, i2c_scl, 400000);
  //onboard baro init
  if (!bmpOnboard.begin_SPI(cs_baro, &sharedSPI)){
    Serial.println(F("BMP384 init failed"));
  } else{
    bmpOnboard.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
    bmpOnboard.setPressureOversampling(BMP3_OVERSAMPLING_8X);
  }
  //imu init with i2c
  if (!mpu.begin(0x68, &Wire)){
    Serial.println(F("MPU6500 init failed"));
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setFilterBandwidth(MPU6050_BAND_94_HZ); //higher bw for 200hz logging
  }
  //thermocouple init
  thermocouple.begin();
  //radio init
  radio.begin(865.0, 125.0, 9, 7, RADIOLIB_SX127X_SYNC_WORD, 17);
  Serial.println(F("All Sensors Online, FC ready for flight."));
}

void loop(){
  uint32_t now = millis();
  bool isArmed = (digitalRead(user_sw) == LOW);
  float vbatVolts = (analogRead(vbat_sense) / 4095.0f) *3.3f*2.0f;
  if (bmpOnboard.performReading()){
    float baroAlt = bmpOnboard.readAltitude(sealevel_HPA);
    if (baroAlt > maxAlt){
      maxAlt = baroAlt;
      baroFallingCount = 0;
    } else{
      baroFallingCount++;
    }
  }
  sensors_event_t accelEvent, gyroEvent, tempEvent;
  mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);
  float totalAccel_G = sqrtf((accelEvent.acceleration.x*accelEvent.acceleration.x)+(accelEvent.acceleration.y*accelEvent.acceleration.y)+(accelEvent.acceleration.z*accelEvent.acceleration.z)) / 9.80665f;
  if(currentState == Apogee||currentState == Descent||currentState == Landed){
    while(gpsSerial.available()>0){
      gps.encode(gpsSerial.read());
    }
    if (gps.location.isValid()){
      gpsHasFix = true;
      digitalWrite(gps_led, HIGH); //turn on the gps lock led once location fix is achieved
      currentLatScaled = (int32_t)(gps.location.lat()*10000.0f);
      currentLonScaled = (int32_t)(gps.location.lng()*10000.0f);
    } else{
      gpsHasFix = false;
      digitalWrite(gps_led, LOW); //keep led off when lock is lost or searching
      if(now - lastGPSResetAttempt >= GPS_LOCK_TIMEOUT_MS){
        triggerGPSReset(); //reset and retry gps module re-acquisition
      }
    }
  }

//statemachine
switch (currentState){
  case Safe:
    setRGB(0,255,0); //green for safe state
    digitalWrite(buzzer, LOW);
    if (isArmed){
      currentState = Armed;
      stateBegin = now;
    } break;
  case Armed:
    setRGB(255, 0,0); //red for armed state
    if (!isArmed){
      currentState = Safe;
    } else if (totalAccel_G >= boostConfirm){
      currentState = Boost;
      boostBegin = now;
      stateBegin = now;
    } break;
  case Boost:
    setRGB(255,0,0); //continue in red for boost state
    if(baroFallingCount >= 5|| (now - boostBegin >= timetoApogee)){
      currentState = Apogee;
      stateBegin = now;
      triggerGPSReset(); //immediately at state shift into apogee state, signal gps module to reset to start recovery
    }
    break;
  case Apogee:
    setRGB(255,128,0); //orange for apogee state
    if (!drogueFired){
      digitalWrite(Pyro1, HIGH);
      droguePulseActive = true;
      drogueFireStart = now;
      drogueFired = true;
    }
    if(now - stateBegin >= apogeeDuration){
      currentState = Descent;
      stateBegin = now;
    } break;
  case Descent:
    setRGB(0, 255, 255); //cyan color for descent state
    if (!mainFired && bmpOnboard.altitude <= Pyro2Deploy) {
      digitalWrite(Pyro2, HIGH);
      mainPulseActive = true;
      mainFireStart = now;
      mainFired = true;
    }
    if (now - stateBegin > 60000 && bmpOnboard.altitude < 10.0f){
      currentState = Landed;
      stateBegin = now;
    } break;
  case Landed:
    setRGB(255,255,0); //yellow color for landed state
    digitalWrite(buzzer, HIGH);
    if(!sdDump){
      dumpIntoSD();
    }break;
}
//time based safety cutoff for mosfet gate triggers
if (droguePulseActive && (now - drogueFireStart >= 1000)){
  digitalWrite(Pyro1, LOW);
  droguePulseActive = false;
}
if(mainPulseActive && (now - mainFireStart >= 1000)){
  digitalWrite(Pyro2, LOW);
  mainPulseActive = false;
}
//variable rate flash logging to save space while descending
uint32_t logInterval = 0;
if(currentState == Boost|| currentState == Apogee) { 
  logInterval = 5; //logs every 5ms = 200hz
} else if (currentState == Descent) {
  logInterval = 100; //logs every 100ms = 10hz
}
if (logInterval > 0 && (now - lastLogTime >= logInterval)) {
  TelemetryPacket pkt;
  pkt.timestamp_ms = now;
  pkt.gps_lat = currentLatScaled;
  pkt.gps_lon = currentLonScaled;
  pkt.accel_x = (int16_t)(accelEvent.acceleration.x * 10000.0f);
  pkt.accel_y = (int16_t)(accelEvent.acceleration.y * 10000.0f);
  pkt.accel_z = (int16_t)(accelEvent.acceleration.z * 10000.0f);
  pkt.gyro_x= (int16_t)(gyroEvent.gyro.x * 10000.0f);
  pkt.gyro_y= (int16_t)(gyroEvent.gyro.y * 10000.0f);
  pkt.gyro_z = (int16_t)(gyroEvent.gyro.z * 10000.0f);
  pkt.battery = (uint8_t)(vbatVolts * 10.0f);
  pkt.rssi = (int8_t)radio.getRSSI();
  pkt.flags = 0;
  if (isArmed) pkt.flags |= (1 << 0);
  pkt.flags |= (currentState & 0x07) << 1;
  if (droguePulseActive) pkt.flags |= (1 << 4);
  if (mainPulseActive) pkt.flags |= (1 << 5);
  logFlash(pkt);
  lastLogTime = now;
}
//live telemetry transmission at 1 hz
if (now - lastTelemetryTime >= 1000) {
  TelemetryPacket radioPkt;
  radioPkt.timestamp_ms = now;
  radioPkt.gps_lat = currentLatScaled;
  radioPkt.gps_lon = currentLonScaled;
  radioPkt.accel_x = (int16_t)(accelEvent.acceleration.x * 10000.0f);
  radioPkt.accel_y = (int16_t)(accelEvent.acceleration.y * 10000.0f);
  radioPkt.accel_z= (int16_t)(accelEvent.acceleration.z * 10000.0f);
  radioPkt.gyro_x = (int16_t)(gyroEvent.gyro.x * 10000.0f);
  radioPkt.gyro_y = (int16_t)(gyroEvent.gyro.y * 10000.0f);
  radioPkt.gyro_z= (int16_t)(gyroEvent.gyro.z * 10000.0f);
  radioPkt.battery = (uint8_t)(vbatVolts * 10.0f);
  radioPkt.rssi= (int8_t)radio.getRSSI();
  radioPkt.flags = 0;
  if (isArmed) radioPkt.flags |= (1 << 0);
  radioPkt.flags |= (currentState & 0x07) << 1;
  if (droguePulseActive) radioPkt.flags |= (1 << 4);
  if (mainPulseActive) radioPkt.flags |= (1 << 5);
  radio.transmit((uint8_t*)&radioPkt, sizeof(TelemetryPacket));
  lastTelemetryTime = now;
}
  delayMicroseconds(500);
}
