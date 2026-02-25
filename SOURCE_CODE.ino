/*
 * PROJECT: GAIT ANALYSIS - V13.5 
*/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>

// --- CONFIGURATION ---
#define FSR_PIN A0
#define LED_GREEN  6  
#define LED_RED    8  
#define LED_YELLOW 7  
#define BUTTON_PIN 5  

#define BT_RX_PIN  2
#define BT_TX_PIN  3
#define SD_CS_PIN  10

// --- THRESHOLDS ---
const int FSR_THRESH_RELEASE = 120;
const int FSR_THRESH_PRESS   = 200;  
const int RAISE_PHASE_FIXED_DURATION = 150;

const float WALKING_HORIZONTAL_THRESHOLD = 5.0; 
const float MIN_SWING_GYRO = 1.5; 

Adafruit_MPU6050 mpu;
SoftwareSerial btSerial(BT_RX_PIN, BT_TX_PIN);
File dataFile; // Global file object

// --- SYSTEM VARIABLES ---
enum SysState { OFF, RUNNING, PAUSED };
SysState sysState = OFF;

enum GaitPhase {
  PHASE_1_FLAT = 0,
  PHASE_2_LIFT = 1,
  PHASE_3_SWING = 2,
  PHASE_4_STRIKE = 3
};
GaitPhase currentGaitPhase = PHASE_1_FLAT;

unsigned long stepCount = 0;
unsigned long totalDuration[4] = {0, 0, 0, 0};
unsigned long lastLoopTime = 0;
unsigned long startTime = 0;
unsigned long stateStartTime = 0;

// Step detection variables
float maxHorizontalAccel = 0;
float maxSwingGyro = 0; 

// --- PAUSE TIME MANAGEMENT ---
unsigned long totalPausedTime = 0; 
unsigned long pauseStartTime = 0;  

String currentFileName = "";
int sampleCounter = 0; 

// Button variables
int buttonState = HIGH; 
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;
unsigned long pressTime = 0;
unsigned long longPressDuration = 1000;
bool isPressed = false;
bool longPressTriggered = false;

void hardwareError() {
  while (1) {
    digitalWrite(LED_RED, HIGH); delay(200);
    digitalWrite(LED_RED, LOW);  delay(200);
  }
}

void setup() {
  // Serial removed
  btSerial.begin(9600);
 
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  if (!mpu.begin()) hardwareError();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG); 
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  if (!SD.begin(SD_CS_PIN)) hardwareError();

  sysState = OFF;
  updateLedState();
}

void loop() {
  handleButton();
  if (sysState == RUNNING) {
    runGaitLogic();
  }
}

// ================================================================
// MAIN GAIT LOGIC
// ================================================================
void runGaitLogic() {
  unsigned long now = millis();
  unsigned long dt = now - lastLoopTime;
  lastLoopTime = now;
 
  totalDuration[currentGaitPhase] += dt;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  int fsr = analogRead(FSR_PIN);

  float currentHorizontal = sqrt(sq(a.acceleration.x) + sq(a.acceleration.y));
  float currentGyroMag = sqrt(sq(g.gyro.x) + sq(g.gyro.y) + sq(g.gyro.z));
  float currentAm = sqrt(sq(a.acceleration.x) + sq(a.acceleration.y) + sq(a.acceleration.z));

  GaitPhase nextState = currentGaitPhase;
  unsigned long durationInState = now - stateStartTime;

  switch (currentGaitPhase) {
    case PHASE_1_FLAT:
      if (fsr < FSR_THRESH_RELEASE) nextState = PHASE_2_LIFT;
      break;

    case PHASE_2_LIFT:
      if (durationInState >= RAISE_PHASE_FIXED_DURATION) {
        nextState = PHASE_3_SWING;
        maxHorizontalAccel = 0; 
        maxSwingGyro = 0; 
      }
      else if (fsr > FSR_THRESH_PRESS) nextState = PHASE_1_FLAT;
      break;

    case PHASE_3_SWING:
      if (currentHorizontal > maxHorizontalAccel) maxHorizontalAccel = currentHorizontal;
      if (currentGyroMag > maxSwingGyro) maxSwingGyro = currentGyroMag;
      
      if (fsr > FSR_THRESH_RELEASE) nextState = PHASE_4_STRIKE;
      break;

    case PHASE_4_STRIKE:
      bool transitionToFlat = false;
      if (fsr > FSR_THRESH_PRESS) transitionToFlat = true;
      else if (durationInState > 100 && fsr > FSR_THRESH_RELEASE) transitionToFlat = true;
      else if (fsr < (FSR_THRESH_RELEASE - 20) && durationInState > 50) nextState = PHASE_3_SWING;
      
      if (transitionToFlat) {
        nextState = PHASE_1_FLAT;
        if (maxHorizontalAccel > WALKING_HORIZONTAL_THRESHOLD && maxSwingGyro > MIN_SWING_GYRO) {
          stepCount++;
        } 
      }
      break;
  }

  if (nextState != currentGaitPhase) {
    currentGaitPhase = nextState;
    stateStartTime = now;
  }

  // --- SD LOGGING (20ms) - KEEP OPEN MODE ---
  static unsigned long lastSDPrint = 0;
  if (now - lastSDPrint >= 20) {
    lastSDPrint = now;
    if (dataFile) {
      unsigned long effectiveTimeMillis = now - startTime - totalPausedTime;
      float runTimeSec = effectiveTimeMillis / 1000.0;

      // Format: Time,FSR,Am,Gyro,Phase,Step
      dataFile.print(runTimeSec, 2); dataFile.print(",");
      dataFile.print(fsr);           dataFile.print(",");
      dataFile.print(currentAm, 2);  dataFile.print(",");
      dataFile.print(currentGyroMag, 2); dataFile.print(",");
      dataFile.print((int)currentGaitPhase); dataFile.print(","); 
      dataFile.println(stepCount);

      // Auto-flush every ~1 second (50 samples) for safety
      sampleCounter++;
      if (sampleCounter >= 50) {
        dataFile.flush();
        sampleCounter = 0;
      }
    }
  }

  // --- BLUETOOTH OUTPUT (100ms) ---
  static unsigned long lastBTPrint = 0;
  if (now - lastBTPrint >= 100) {
    lastBTPrint = now;
    printDataPieceBT(now);
  }
}

void prepareNewFileName() {
  int i = 1;
  while (true) {
    String filename = String(i) + ".csv";
    if (!SD.exists(filename)) {
      currentFileName = filename;
      break;
    }
    i++;
  }
}

void printHeaderToBT() {
  btSerial.println("Time(s) | Step | P1_Flat | P2_Lift | P3_Swing | P4_Strike");
}

void printDataPieceBT(unsigned long now) {
  unsigned long effectiveTimeMillis = now - startTime - totalPausedTime;
  float t = effectiveTimeMillis / 1000.0;
  
  btSerial.print(t, 1); btSerial.print(" | ");
  btSerial.print(stepCount); btSerial.print(" | ");
  btSerial.print(totalDuration[0]); btSerial.print(" | ");
  btSerial.print(totalDuration[1]); btSerial.print(" | ");
  btSerial.print(totalDuration[2]); btSerial.print(" | ");
  btSerial.println(totalDuration[3]);
}

void printFinalReport() {
  btSerial.println(F("\n=== REPORT ==="));
  if (stepCount == 0) {
    btSerial.println(F("Step = 0")); 
  } else {
    unsigned long avgFlat   = totalDuration[PHASE_1_FLAT] / stepCount;
    unsigned long avgLift   = totalDuration[PHASE_2_LIFT] / stepCount;
    unsigned long avgSwing  = totalDuration[PHASE_3_SWING] / stepCount;
    unsigned long avgStrike = totalDuration[PHASE_4_STRIKE] / stepCount;
    unsigned long avgCycle  = avgFlat + avgLift + avgSwing + avgStrike;
    
    btSerial.print(F("Steps: ")); btSerial.println(stepCount);
    btSerial.print(F("1. Flat: ")); btSerial.println(avgFlat);
    btSerial.print(F("2. Lift: ")); btSerial.println(avgLift);
    btSerial.print(F("3. Swing: ")); btSerial.println(avgSwing);
    btSerial.print(F("4. Strike: ")); btSerial.println(avgStrike);
    btSerial.print(F(">> TOTAL: ")); btSerial.println(avgCycle);
  }
  btSerial.println(F("=============\n"));
}

void handleButton() {
  int reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonState) lastDebounceTime = millis();
 
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {
        isPressed = true;
        pressTime = millis();
        longPressTriggered = false;
      } else {
        isPressed = false;
        if (!longPressTriggered) handleShortPress();
      }
    }
  }

  if (isPressed && !longPressTriggered && (millis() - pressTime) > longPressDuration) {
    longPressTriggered = true;
    handleLongPress();
  }

  lastButtonState = reading;
}

void handleShortPress() {
  switch (sysState) {
    case OFF:
      // START
      resetSystemData();
      prepareNewFileName();
      
      // Open file (keep-open mode)
      dataFile = SD.open(currentFileName, FILE_WRITE);
      sampleCounter = 0;

      sysState = RUNNING;
      startTime = millis();
      lastLoopTime = millis();
      totalPausedTime = 0; 
      printHeaderToBT();
      break;

    case RUNNING:
      // PAUSE
      sysState = PAUSED;
      pauseStartTime = millis(); 
      
      // Keep file open; flush for safety
      if (dataFile) dataFile.flush();

      btSerial.println(F("\n[PAUSED]")); 
      printFinalReport();
      break;

    case PAUSED:
      // RESUME
      sysState = RUNNING;
      totalPausedTime += (millis() - pauseStartTime);
      lastLoopTime = millis();
      
      btSerial.println(F("\n[RESUME]")); 
      break;
  }
  updateLedState();
}

void handleLongPress() {
  if (sysState == RUNNING || sysState == PAUSED) {
    // Close file only on STOP
    if (dataFile) dataFile.close();
    printFinalReport();
  }
  btSerial.println(F("\n[RESET/OFF]"));
  sysState = OFF;
  resetSystemData();
  updateLedState();
}

void updateLedState() {
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, LOW);
  switch (sysState) {
    case OFF:     digitalWrite(LED_RED, HIGH); break;
    case RUNNING: digitalWrite(LED_GREEN, HIGH); break;
    case PAUSED:  digitalWrite(LED_YELLOW, HIGH); break;
  }
}

void resetSystemData() {
  stepCount = 0;
  maxHorizontalAccel = 0;
  maxSwingGyro = 0; 
  for(int i=0; i<4; i++) totalDuration[i] = 0;
  currentGaitPhase = PHASE_1_FLAT;
}



