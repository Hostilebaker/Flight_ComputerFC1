#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_BMP280.h>
#include <LittleFS.h>
#include <SD.h>
#include "state_machine.h"

//pindefs
#define PYRO1_PIN   14
#define PYRO2_PIN   2
#define PYRO3_PIN   9
#define PYRO4_PIN   1

#define GPS_RX      13
#define GPS_TX      48
#define GPS_BAUD    9600

#define BMP_CS      47

#define LORA_SCK    36
#define LORA_MISO   37
#define LORA_MOSI   35
#define LORA_CS     4
#define LORA_RST    5
#define LORA_DIO0   6
#define LORA_FREQ   868E6

#define BUZZER_PIN    45
#define READY_LED_PIN 18

//SPI SD Card CS & DET pins
#define SD_CS_PIN    16
#define SD_DET_PIN   39

//RGB LED pins
#define RGB_R_PIN       40
#define RGB_G_PIN       41
#define RGB_B_PIN       42
#define RGB_ACTIVE_LOW  0

const unsigned long PYRO_FIRE_MS = 1000;
const float SEA_LEVEL_HPA = 1013.25;

//globals
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
Adafruit_BMP280 bmp(BMP_CS, &SPI);
bool bmpOK = false;

StateMachine flightFSM;

bool gpsBaseSet = false;
float gpsBaseAltitude = 0;
float lastKnownAglAlt = 0;
double lastKnownLat = 0.0;
double lastKnownLon = 0.0;
uint32_t lastSatCount = 0;

bool baroBaseSet = false;
float baroBaseAltitude = 0;
float lastBaroAltAGL = 0;
float rawBaroPressPa = 0;
float rawBaroTempC = 0;
bool lastBaroValid = false;
unsigned long lastBaroSampleTime = 0;
bool boostSamplingActive = false;

//flash buffered flight log
const char* FLASH_LOG_PATH = "/flightlog.csv";
File flashLogFile;
bool touchdownHandled = false;
bool buzzerLatched = false;

//audio beacon for landed state
unsigned long lastBeaconToggle = 0;
bool beaconHighPitch = false;

//GPS lock audio indication
bool gpsLockAlertDone = false;
bool gpsToneActive = false;
unsigned long gpsToneStartTime = 0;

bool pyroActive[NUM_PYRO_CHANNELS] = {false, false, false, false};
unsigned long pyroStart[NUM_PYRO_CHANNELS] = {0, 0, 0, 0};
const int pyroPin[NUM_PYRO_CHANNELS] = {
  PYRO1_PIN, PYRO2_PIN, PYRO3_PIN, PYRO4_PIN
};

unsigned long lastTxTime = 0;
unsigned long lastPrint = 0;
unsigned long lastLedBlinkTime = 0;
bool ledState = false;

//timeout watchdog tracking
unsigned long lastFsmUpdateTime = 0;

//buzzer
void playChirp(int freq, int durationMs) {
  tone(BUZZER_PIN, freq);
  delay(durationMs);
  noTone(BUZZER_PIN);
}

void playSuccessTone() {
  playChirp(2000, 60);
  delay(40);
  playChirp(3000, 80);
}

//5-tone startup sequence
void playBootSequence() {
  const int notes[] = {2000, 2500, 3000, 3500, 4000};

  for (int i = 0; i < 5; i++) {
    tone(BUZZER_PIN, notes[i]);
    delay(100);
    noTone(BUZZER_PIN);
    delay(50);
  }
}

//RGB control
void setRgbColor(bool r, bool g, bool b) {
  if (RGB_ACTIVE_LOW) {
    digitalWrite(RGB_R_PIN, r ? LOW : HIGH);
    digitalWrite(RGB_G_PIN, g ? LOW : HIGH);
    digitalWrite(RGB_B_PIN, b ? LOW : HIGH);
  } else {
    digitalWrite(RGB_R_PIN, r ? HIGH : LOW);
    digitalWrite(RGB_G_PIN, g ? HIGH : LOW);
    digitalWrite(RGB_B_PIN, b ? HIGH : LOW);
  }
}

//RGB startup sequence
void startupRgbSequence() {
  //red
  setRgbColor(true, false, false);
  delay(150);

  //green
  setRgbColor(false, true, false);
  delay(150);

  //blue
  setRgbColor(false, false, true);
  delay(150);

  //yellow
  setRgbColor(true, true, false);
  delay(150);

  //magenta
  setRgbColor(true, false, true);
  delay(150);

  //cyan
  setRgbColor(false, true, true);
  delay(150);

  //white
  setRgbColor(true, true, true);
  delay(200);

  //off for 1 second
  setRgbColor(false, false, false);
  delay(1000);

  //standby = blue
  setRgbColor(false, false, true);
}

//RGB state indication
void updateRgbFromState() {
  switch (flightFSM.getState()) {

    case FlightState::STANDBY:
      setRgbColor(false, false, true);   //blue
      break;

    case FlightState::READY:
      setRgbColor(true, false, false);   //red
      break;

    case FlightState::BOOST:
      setRgbColor(true, false, true);    //magenta
      break;

    case FlightState::APOGEE:
      setRgbColor(true, true, false);    //yellow
      break;

    case FlightState::FREEFALL:
      setRgbColor(false, true, true);    //cyan
      break;

    case FlightState::LANDED:
      setRgbColor(false, true, false);   //green
      break;

    default:
      setRgbColor(false, false, false);
      break;
  }
}

//dual-pitch beacon for LANDED state
void updateLandedAudioBeacon() {
  if (!buzzerLatched) return;

  unsigned long now = millis();

  if (now - lastBeaconToggle >= 250) {
    lastBeaconToggle = now;
    beaconHighPitch = !beaconHighPitch;

    if (beaconHighPitch) {
      tone(BUZZER_PIN, 4000);
    } else {
      tone(BUZZER_PIN, 2500);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("================================");
  Serial.println("  ROCKET AVIONICS BOOT (v5, FSM)");
  Serial.println("================================");

  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  //RGB LED
  pinMode(RGB_R_PIN, OUTPUT);
  pinMode(RGB_G_PIN, OUTPUT);
  pinMode(RGB_B_PIN, OUTPUT);

  //start with RGB off
  setRgbColor(false, false, false);

  //RGB startup sequence
  startupRgbSequence();

  //5-tone boot sequence
  playBootSequence();

  pinMode(READY_LED_PIN, OUTPUT);
  digitalWrite(READY_LED_PIN, LOW);

  for (int i = 0; i < NUM_PYRO_CHANNELS; i++) {
    pinMode(pyroPin[i], OUTPUT);
    digitalWrite(pyroPin[i], LOW);
  }

  //force ALL Chip Select pins HIGH
  pinMode(BMP_CS, OUTPUT);
  digitalWrite(BMP_CS, HIGH);

  pinMode(LORA_CS, OUTPUT);
  digitalWrite(LORA_CS, HIGH);

  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);

  gpsSerial.setRxBufferSize(2048);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS serial initialized.");

  //init SPI bus
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI);
  delay(100);

  //BMP280 init
  for (int retry = 0; retry < 3; retry++) {
    digitalWrite(BMP_CS, HIGH);
    digitalWrite(LORA_CS, HIGH);
    digitalWrite(SD_CS_PIN, HIGH);
    delay(20);

    bmpOK = bmp.begin();

    if (bmpOK) break;

    Serial.print("BMP280 init retry ");
    Serial.println(retry + 1);
    delay(50);
  }

  if (bmpOK) {
    bmp.setSampling(
      Adafruit_BMP280::MODE_NORMAL,
      Adafruit_BMP280::SAMPLING_X2,
      Adafruit_BMP280::SAMPLING_X16,
      Adafruit_BMP280::FILTER_X16,
      Adafruit_BMP280::STANDBY_MS_63
    );

    Serial.println("BMP280 OK.");
  } else {
    Serial.println("BMP280 NOT FOUND.");
  }

  //LoRa init
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed!");
  } else {
    Serial.println("LoRa initialized at 868MHz.");
  }

  pinMode(SD_DET_PIN, INPUT_PULLUP);

  //LittleFS init
  if (!LittleFS.begin(true)) {
    Serial.println(
      "LittleFS mount failed - flight log will NOT be recorded!"
    );
  } else {
    flashLogFile = LittleFS.open(
      FLASH_LOG_PATH,
      FILE_WRITE
    );

    if (flashLogFile) {

      if (flashLogFile.size() == 0) {
        flashLogFile.println(
          "timestamp_ms,state,gps_lock,gps_sats,gps_lat,gps_lon,"
          "gps_raw_alt_m,gps_alt_agl_m,gps_speed_mps,gps_course_deg,"
          "baro_press_pa,baro_temp_c,baro_raw_alt_m,baro_alt_agl_m,"
          "fused_alt_agl_m,pyro1_apogee1,pyro2_apogee2,"
          "pyro3_main1,pyro4_main2"
        );

        flashLogFile.flush();
      }

      Serial.println(
        "LittleFS CSV flight log initialized."
      );

    } else {
      Serial.println(
        "Could not create flight log on LittleFS!"
      );
    }
  }

  //force final startup state
  //STANDBY = BLUE
  updateRgbFromState();

  Serial.println("Setup complete.");
  Serial.println("================================");
}

//pyro handling
void startPyro(int idx) {
  if (idx < 0 || idx >= NUM_PYRO_CHANNELS) return;
  if (pyroActive[idx]) return;

  digitalWrite(pyroPin[idx], HIGH);
  pyroActive[idx] = true;
  pyroStart[idx] = millis();

  Serial.print(">>> PYRO CH");
  Serial.print(idx);
  Serial.println(" FIRED <<<");
}

void updatePyroTimers() {
  unsigned long now = millis();

  for (int i = 0; i < NUM_PYRO_CHANNELS; i++) {
    if (pyroActive[i] &&
        (now - pyroStart[i] >= PYRO_FIRE_MS)) {

      digitalWrite(pyroPin[i], LOW);
      pyroActive[i] = false;
    }
  }
}

//baro rate by state
float baroRateHzForState(FlightState st) {
  switch (st) {

    case FlightState::READY:
      return 10.0f;

    case FlightState::BOOST:
      return 100.0f;

    case FlightState::APOGEE:
    case FlightState::FREEFALL:
      return 10.0f;

    case FlightState::LANDED:
      return 0.0f;

    default:
      return 10.0f;
  }
}

void setBaroSamplingProfile(bool highRate) {
  if (highRate) {

    bmp.setSampling(
      Adafruit_BMP280::MODE_NORMAL,
      Adafruit_BMP280::SAMPLING_X1,
      Adafruit_BMP280::SAMPLING_X1,
      Adafruit_BMP280::FILTER_OFF,
      Adafruit_BMP280::STANDBY_MS_1
    );

  } else {

    bmp.setSampling(
      Adafruit_BMP280::MODE_NORMAL,
      Adafruit_BMP280::SAMPLING_X2,
      Adafruit_BMP280::SAMPLING_X16,
      Adafruit_BMP280::FILTER_X16,
      Adafruit_BMP280::STANDBY_MS_63
    );
  }
}

//Flash Log Writer
void appendFlashLogRow(
  const SensorData& snap,
  const FlightCommands& cmd,
  float fusedAlt
) {
  if (!flashLogFile ||
      touchdownHandled ||
      !cmd.log_enable) return;

  char logBuf[320];

  snprintf(
    logBuf,
    sizeof(logBuf),
    "%lu,%s,%d,%u,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,"
    "%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d\n",
    snap.timestamp_ms,
    flightFSM.getStateName(),
    snap.gps_lock ? 1 : 0,
    snap.gps_satellites,
    snap.gps_latitude_deg,
    snap.gps_longitude_deg,
    gps.altitude.isValid() ?
      (float)gps.altitude.meters() : 0.0f,
    snap.gps_alt_valid ?
      snap.gps_altitude_agl_m : 0.0f,
    gps.speed.isValid() ?
      (float)gps.speed.mps() : 0.0f,
    gps.course.isValid() ?
      (float)gps.course.deg() : 0.0f,
    rawBaroPressPa,
    rawBaroTempC,
    bmpOK ?
      (baroBaseAltitude +
       snap.baro_altitude_agl_m) : 0.0f,
    snap.baro_alt_valid ?
      snap.baro_altitude_agl_m : 0.0f,
    fusedAlt,
    cmd.fire_pyro[0] ? 1 : 0,
    cmd.fire_pyro[1] ? 1 : 0,
    cmd.fire_pyro[2] ? 1 : 0,
    cmd.fire_pyro[3] ? 1 : 0
  );

  flashLogFile.print(logBuf);
  flashLogFile.flush();
}

//Shared-SPI Safe SD Dump
void dumpFlashLogToSD() {
  Serial.println(
    "Touchdown confirmed - dumping CSV log from Flash to SD card..."
  );

  if (flashLogFile) {
    flashLogFile.flush();
    flashLogFile.close();
  }

  if (digitalRead(SD_DET_PIN) != LOW) {
    Serial.println(
      "No SD card detected (SD_DET) - CSV saved on ESP32 Flash only."
    );
    return;
  }

  //force ALL SPI CS pins HIGH
  pinMode(BMP_CS, OUTPUT);
  digitalWrite(BMP_CS, HIGH);

  pinMode(LORA_CS, OUTPUT);
  digitalWrite(LORA_CS, HIGH);

  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);

  delay(50);

  //dummy clock cycles
  SPI.beginTransaction(
    SPISettings(400000, MSBFIRST, SPI_MODE0)
  );

  for (int i = 0; i < 10; i++) {
    SPI.transfer(0xFF);
  }

  SPI.endTransaction();
  delay(20);

  //SD initialization
  bool mounted = false;

  if (SD.begin(SD_CS_PIN, SPI, 400000)) {
    mounted = true;
  } else {

    delay(100);

    if (SD.begin(SD_CS_PIN, SPI, 1000000)) {
      mounted = true;
    }
  }

  if (!mounted) {
    Serial.println(
      "SD SPI Mount failed - CSV saved on ESP32 Flash only."
    );

    digitalWrite(SD_CS_PIN, HIGH);
    return;
  }

  File src = LittleFS.open(
    FLASH_LOG_PATH,
    FILE_READ
  );

  if (!src) {
    Serial.println(
      "Could not reopen Flash flight log for reading."
    );

    SD.end();
    digitalWrite(SD_CS_PIN, HIGH);
    return;
  }

  char filenameBuf[32];

  snprintf(
    filenameBuf,
    sizeof(filenameBuf),
    "/flight_%lu.csv",
    millis()
  );

  File dst = SD.open(
    filenameBuf,
    FILE_WRITE
  );

  if (!dst) {
    Serial.println(
      "Could not create flight CSV on SD card."
    );

    src.close();
    SD.end();
    digitalWrite(SD_CS_PIN, HIGH);
    return;
  }

  uint8_t buf[512];
  size_t totalBytes = 0;

  while (src.available()) {
    size_t n = src.read(
      buf,
      sizeof(buf)
    );

    dst.write(buf, n);
    totalBytes += n;
  }

  dst.flush();
  dst.close();
  src.close();

  Serial.print(
    "Flight CSV dumped to SD card: "
  );

  Serial.print(filenameBuf);

  Serial.print(" (");
  Serial.print(totalBytes);
  Serial.println(" bytes)");

  SD.end();
  digitalWrite(SD_CS_PIN, HIGH);
}

//LoRa telemetry
void sendTelemetry(
  double lat,
  double lon,
  float gpsAltAGL,
  float baroAltAGL,
  float fusedAltAGL,
  float rateHz
) {
  if (rateHz <= 0.0f) return;

  unsigned long intervalMs =
    (unsigned long)(1000.0f / rateHz);

  if (millis() - lastTxTime < intervalMs)
    return;

  lastTxTime = millis();

  digitalWrite(SD_CS_PIN, HIGH);
  digitalWrite(BMP_CS, HIGH);

  char txBuf[160];

  snprintf(
    txBuf,
    sizeof(txBuf),
    "LAT:%.6f,LON:%.6f,ALT:%.1f,STATE:%s,"
    "BAROALT:%.1f,FUSEDALT:%.1f",
    lat,
    lon,
    gpsAltAGL,
    flightFSM.getStateName(),
    bmpOK ? baroAltAGL : 0.0f,
    fusedAltAGL
  );

  Serial.print("TX LoRa Packet: ");
  Serial.println(txBuf);

  LoRa.beginPacket();
  LoRa.print(txBuf);
  LoRa.endPacket();
}

void debugPrint(
  float gpsAltAGL,
  float baroAltAGL,
  float fusedAltAGL
) {
  if (millis() - lastPrint < 1000)
    return;

  lastPrint = millis();

  bool hasLock =
    gps.location.isValid() ||
    (gps.satellites.value() >= 3 &&
     gps.location.lat() != 0.0);

  Serial.println(
    "---------- STATUS ----------"
  );

  Serial.print("STATE: ");
  Serial.println(
    flightFSM.getStateName()
  );

  Serial.print("SATS: ");
  Serial.println(
    gps.satellites.isValid() ?
    gps.satellites.value() : 0
  );

  Serial.print("LAT: ");
  Serial.println(
    hasLock ? lastKnownLat : 0.0,
    6
  );

  Serial.print("LON: ");
  Serial.println(
    hasLock ? lastKnownLon : 0.0,
    6
  );

  Serial.print("GPS Alt AGL: ");
  Serial.print(
    gpsBaseSet ? gpsAltAGL : -1
  );
  Serial.println(" m");

  Serial.print("Baro Alt AGL: ");
  Serial.print(
    (bmpOK && baroBaseSet) ?
    baroAltAGL : -1
  );
  Serial.println(" m");

  Serial.print("Fused Alt AGL: ");
  Serial.print(fusedAltAGL);
  Serial.println(" m");

  Serial.println(
    "-----------------------------"
  );
}

void loop() {

  //feed GPS parser continuously
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  //GPS fix detection
  bool hasGpsFix =
    gps.location.isValid() ||
    (gps.satellites.value() >= 3 &&
     gps.location.lat() != 0.0);

  if (hasGpsFix) {

    if (gps.location.lat() != 0.0) {
      lastKnownLat =
        gps.location.lat();

      lastKnownLon =
        gps.location.lng();
    }
  }

  //landed state
  if (touchdownHandled) {

    updateLandedAudioBeacon();

    //keep LED green
    setRgbColor(
      false,
      true,
      false
    );

    sendTelemetry(
      lastKnownLat,
      lastKnownLon,
      lastKnownAglAlt,
      lastBaroAltAGL,
      flightFSM.getFusedAltitude_m(),
      1.0f
    );

    return;
  }

  //GPS satellite audio
  if (gps.satellites.isUpdated()) {

    uint32_t currentSats =
      gps.satellites.value();

    if (currentSats > lastSatCount &&
        !gpsToneActive) {

      playChirp(
        3500,
        30
      );

    } else if (
      currentSats < lastSatCount &&
      !gpsToneActive
    ) {

      playChirp(
        1500,
        40
      );
    }

    lastSatCount =
      currentSats;
  }

  //GPS lock alert
  if (hasGpsFix &&
      !gpsLockAlertDone) {

    gpsLockAlertDone = true;
    gpsToneActive = true;
    gpsToneStartTime = millis();

    tone(
      BUZZER_PIN,
      4000
    );
  }

  if (gpsToneActive &&
      (millis() -
       gpsToneStartTime >= 5000)) {

    gpsToneActive = false;

    if (!buzzerLatched) {
      noTone(BUZZER_PIN);
    }
  }

  //only count a new GPS altitude as fresh
  bool freshFix =
    gps.altitude.isUpdated() &&
    gps.altitude.isValid();

  float gpsAltAGL =
    lastKnownAglAlt;

  if (freshFix) {

    float rawAlt =
      gps.altitude.meters();

    if (!gpsBaseSet) {

      gpsBaseAltitude =
        rawAlt;

      gpsBaseSet = true;

      gpsAltAGL = 0.0f;
      lastKnownAglAlt = 0.0f;

    } else {

      gpsAltAGL =
        rawAlt -
        gpsBaseAltitude;

      lastKnownAglAlt =
        gpsAltAGL;
    }
  }

  //GPS lock LED
  if (hasGpsFix) {

    digitalWrite(
      READY_LED_PIN,
      HIGH
    );

  } else {

    if (millis() -
        lastLedBlinkTime >= 100) {

      lastLedBlinkTime =
        millis();

      ledState =
        !ledState;

      digitalWrite(
        READY_LED_PIN,
        ledState ?
        HIGH : LOW
      );
    }
  }

  FlightState curState =
    flightFSM.getState();

  bool wantHighRate =
    (curState ==
     FlightState::BOOST);

  if (wantHighRate !=
      boostSamplingActive) {

    setBaroSamplingProfile(
      wantHighRate
    );

    boostSamplingActive =
      wantHighRate;
  }

  bool sampledBaroThisLoop =
    false;

  unsigned long now =
    millis();

  float targetBaroHz =
    baroRateHzForState(
      curState
    );

  //barometer sampling
  if (bmpOK &&
      targetBaroHz > 0.0f) {

    unsigned long
    baroIntervalMs =
      (unsigned long)
      (1000.0f /
       targetBaroHz);

    if (now -
        lastBaroSampleTime >=
        baroIntervalMs) {

      lastBaroSampleTime =
        now;

      sampledBaroThisLoop =
        true;

      digitalWrite(
        SD_CS_PIN,
        HIGH
      );

      rawBaroPressPa =
        bmp.readPressure();

      rawBaroTempC =
        bmp.readTemperature();

      float baroAltRaw =
        bmp.readAltitude(
          SEA_LEVEL_HPA
        );

      if (!baroBaseSet) {

        baroBaseAltitude =
          baroAltRaw;

        baroBaseSet =
          true;

        lastBaroAltAGL =
          0.0f;

        lastBaroValid =
          false;

      } else {

        lastBaroAltAGL =
          baroAltRaw -
          baroBaseAltitude;

        lastBaroValid =
          true;
      }
    }
  }

  //FSM watchdog
  bool baroTimedOut =
    (now -
     lastFsmUpdateTime >=
     200);

  if ((baroBaseSet ||
       gpsBaseSet) &&
      (sampledBaroThisLoop ||
       baroTimedOut)) {

    lastFsmUpdateTime =
      now;

    SensorData snap;

    snap.timestamp_ms =
      now;

    snap.gps_lock =
      hasGpsFix;

    snap.gps_satellites =
      gps.satellites.isValid() ?
      gps.satellites.value() :
      0;

    snap.gps_alt_valid =
      gpsBaseSet &&
      freshFix;

    snap.gps_altitude_agl_m =
      gpsAltAGL;

    snap.gps_latitude_deg =
      (float)lastKnownLat;

    snap.gps_longitude_deg =
      (float)lastKnownLon;

    snap.baro_alt_valid =
      lastBaroValid &&
      !baroTimedOut;

    snap.baro_altitude_agl_m =
      lastBaroAltAGL;

    snap.rf_link_ok =
      true;

    //FSM update
    FlightCommands cmd =
      flightFSM.update(
        snap
      );

    //update RGB immediately
    //from the actual FSM state
    updateRgbFromState();

    //pyro commands
    for (int i = 0;
         i < NUM_PYRO_CHANNELS;
         i++) {

      if (cmd.fire_pyro[i] &&
          !pyroActive[i]) {

        startPyro(i);
      }
    }

    //flight logging
    appendFlashLogRow(
      snap,
      cmd,
      flightFSM.getFusedAltitude_m()
    );

    FlightState newState =
      flightFSM.getState();

    //landing handling
    if (newState ==
        FlightState::LANDED &&
        !touchdownHandled) {

      touchdownHandled =
        true;

      //force landed green
      setRgbColor(
        false,
        true,
        false
      );

      Serial.println(
        "-----------------------------"
      );

      Serial.println(
        "STATE: LANDED"
      );

      Serial.println(
        "-----------------------------"
      );

      buzzerLatched =
        true;

      updatePyroTimers();

      dumpFlashLogToSD();
    }
  }

  updatePyroTimers();

  //Always enforce RGB state indication.
  updateRgbFromState();

  //FSM-defined telemetry rate
  sendTelemetry(
    lastKnownLat,
    lastKnownLon,
    gpsAltAGL,
    lastBaroAltAGL,
    flightFSM.getFusedAltitude_m(),
    flightFSM.getTelemetryRateHz()
  );

  debugPrint(
    gpsAltAGL,
    lastBaroAltAGL,
    flightFSM.getFusedAltitude_m()
  );
}
