#include <LiquidCrystal.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>

// ====================== Pin Definitions ======================
#define ENCA 2   // Encoder A input
#define ENCB 3   // Encoder B input
#define PWM 5    // PWM pin for motor speed control
#define IN2 6    // Motor direction pin 2
#define IN1 7    // Motor direction pin 1

// ====================== LCD Setup ============================
LiquidCrystal lcd(8, 9, 10, 11, 12, 13);

// ====================== Encoder and Motor Variables =========
volatile long posi = 0; // Current motor encoder position

// Encoder counts per revolution (adjust to your encoder specs)
const float CPR = 1024.0;  
const float countsPerDegree = CPR / 360.0;

// Control parameters
float Kp = 1.0;     // Proportional gain
int tolerance = 5;   // Encoder counts tolerance
int minPWM = 50;     // Minimum PWM to ensure motion at low error

// ====================== Accelerometer Setup ==================
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// ====================== Helper Functions =====================

void displaySensorDetails(void) {
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       "); Serial.println(sensor.name);
  Serial.print("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displayDataRate(void) {
  Serial.print("Data Rate:    ");
  switch (accel.getDataRate()) {
    case ADXL345_DATARATE_3200_HZ: Serial.print("3200 "); break;
    case ADXL345_DATARATE_1600_HZ: Serial.print("1600 "); break;
    case ADXL345_DATARATE_800_HZ:  Serial.print("800 "); break;
    case ADXL345_DATARATE_400_HZ:  Serial.print("400 "); break;
    case ADXL345_DATARATE_200_HZ:  Serial.print("200 "); break;
    case ADXL345_DATARATE_100_HZ:  Serial.print("100 "); break;
    case ADXL345_DATARATE_50_HZ:   Serial.print("50 "); break;
    case ADXL345_DATARATE_25_HZ:   Serial.print("25 "); break;
    case ADXL345_DATARATE_12_5_HZ: Serial.print("12.5 "); break;
    case ADXL345_DATARATE_6_25HZ:  Serial.print("6.25 "); break;
    case ADXL345_DATARATE_3_13_HZ: Serial.print("3.13 "); break;
    case ADXL345_DATARATE_1_56_HZ: Serial.print("1.56 "); break;
    case ADXL345_DATARATE_0_78_HZ: Serial.print("0.78 "); break;
    case ADXL345_DATARATE_0_39_HZ: Serial.print("0.39 "); break;
    case ADXL345_DATARATE_0_20_HZ: Serial.print("0.20 "); break;
    case ADXL345_DATARATE_0_10_HZ: Serial.print("0.10 "); break;
    default: Serial.print("???? "); break;
  }
  Serial.println(" Hz");
}

void displayRange(void) {
  Serial.print("Range:         +/- ");
  switch (accel.getRange()) {
    case ADXL345_RANGE_16_G: Serial.print("16 "); break;
    case ADXL345_RANGE_8_G:  Serial.print("8 "); break;
    case ADXL345_RANGE_4_G:  Serial.print("4 "); break;
    case ADXL345_RANGE_2_G:  Serial.print("2 "); break;
    default: Serial.print("?? "); break;
  }
  Serial.println(" g");
}

// ====================== Setup ===============================
void setup() {
  Serial.begin(9600);
  
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);

  // Attach interrupt for the encoder
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Initialize LCD
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Motor Angle:");

  // Initialize accelerometer
  if (!accel.begin()) {
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while (1);
  }
  accel.setRange(ADXL345_RANGE_16_G);

  displaySensorDetails();
  displayDataRate();
  displayRange();
  Serial.println("");
}

// ====================== Main Loop ===========================
void loop() {
  // Read accelerometer (acting as a stand-in for gyro angle)
  sensors_event_t event;
  accel.getEvent(&event);

  // Compute angle (pitch) from y and z axes
  float angle = atan2(event.acceleration.y, event.acceleration.z) * 180.0 / PI;

  // Compute the target encoder position from angle
  long targetPos = (long)(angle * countsPerDegree);

  // Compute error
  int error = targetPos - posi;

  // Check if within tolerance
  if (abs(error) <= tolerance) {
    // Close enough to the target
    setMotor(0, 0, PWM, IN1, IN2);
  } else {
    // Proportional control
    int controlSignal = (int)(error * Kp);
    // Constrain control signal to PWM range
    controlSignal = constrain(controlSignal, -255, 255);

    // Enforce a minimum effort if error is not zero
    if (controlSignal > 0 && controlSignal < minPWM) controlSignal = minPWM;
    if (controlSignal < 0 && controlSignal > -minPWM) controlSignal = -minPWM;

    // Set motor direction and speed
    if (controlSignal > 0) {
      setMotor(1, controlSignal, PWM, IN1, IN2);
    } else {
      setMotor(-1, -controlSignal, PWM, IN1, IN2);
    }
  }

  // Update LCD display
    lcd.setCursor(0, 0);
  lcd.print("Motor Angle:");
  lcd.print(angle);
  lcd.setCursor(0, 1);
  lcd.print("Pos: ");
  lcd.print(posi);
  lcd.print("    ");

  // Debug info
  Serial.print("Angle: "); Serial.print(angle);
  Serial.print(" deg | posi: "); Serial.print(posi);
  Serial.print(" | targetPos: "); Serial.print(targetPos);
  Serial.print(" | error: "); Serial.print(error);
  Serial.println();

  delay(50);
}

// ====================== Functions ===========================

// Motor control function
void setMotor(int dir, int pwmVal, int pwmPin, int in1, int in2) {
  analogWrite(pwmPin, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    // Motor off
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

// Encoder interrupt handler
void readEncoder() {
  int b = digitalRead(ENCB);
  // Determine direction based on ENCB
  if (b == HIGH) {
    posi++;
  } else {
    posi--;
  }
}
