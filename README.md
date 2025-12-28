# Project-Week-3-Task-3
Use Mpu 3050
#include <Wire.h>
#include <LiquidCrystal.h>

#define MOTOR_A_EN 3   
#define MOTOR_A_IN1 13 
#define MOTOR_A_IN2 2  

#define MOTOR_B_EN 11  
#define MOTOR_B_IN1 12 
#define MOTOR_B_IN2 1  

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

double Kp = 45;   
double Ki = 0;    
double Kd = 1.0;  

double yaw_setpoint = 0;  
double pid_error = 0;
double pid_integral = 0;
double pid_derivative = 0;
double pid_previous_error = 0;
double pid_output = 0;

int baseSpeed = 255;  
int motorSpeedA = 0;
int motorSpeedB = 0;

const int MPU = 0x68; 
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float pitch = 0.0, yaw = 0.0; 
float maxPitch = 0.0; 
float GyroErrorZ = 0;
float elapsedTime, currentTime, previousTime;

// 0: Search, 1: Climb, 2: Push, 3: Wait, 4: Spin, 5: Down, 6: End
int pitchState = 0; 

unsigned long stateTimer = 0;
bool stopOperation = false;
unsigned long flatCheckTime = 0;

void setup() {
  Wire.begin(); 
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  Wire.write(0x00); 
  Wire.endTransmission(true);

  lcd.begin(16, 2); 
  lcd.clear();
  lcd.print("Init angles..."); 
  delay(1000);

  pinMode(MOTOR_A_EN, OUTPUT); pinMode(MOTOR_A_IN1, OUTPUT); pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_EN, OUTPUT); pinMode(MOTOR_B_IN1, OUTPUT); pinMode(MOTOR_B_IN2, OUTPUT);

  setMotorSpeed('A', 0);
  setMotorSpeed('B', 0);

  calculate_IMU_error(); 
  delay(100);
}

void loop() {
  previousTime = currentTime;        
  currentTime = millis();            
  elapsedTime = (currentTime - previousTime) / 1000.0; 

  readMPUData();

  if (pitch > maxPitch) {
    maxPitch = pitch;
  }

  pid_error = yaw_setpoint - yaw; 
  if(abs(pid_error) < 10) {
      pid_integral = constrain(pid_integral + (pid_error * elapsedTime), -50, 50);
  } else {
      pid_integral = 0; 
  }
  
  pid_derivative = (pid_error - pid_previous_error) / elapsedTime;
  pid_output = (Kp * pid_error) + (Ki * pid_integral) + (Kd * pid_derivative);
  pid_previous_error = pid_error;

  switch (pitchState) {
    
 
    case 0: 
      stopOperation = false;
      baseSpeed = 255; 
      yaw_setpoint = 0; 
      if (abs(pitch) > 15) {
         pitchState = 1; 
      }
      break;

   
    case 1:
      baseSpeed = 255; 
      if (pitch <= 2) { 
         if (flatCheckTime == 0) flatCheckTime = millis();
         if (millis() - flatCheckTime > 600) { 
             pitchState = 2; 
             stateTimer = millis(); 
         }
      } else {
         flatCheckTime = 0; 
      }
      break;

   
    case 2:
      stopOperation = false;
      baseSpeed = 255;
      if (millis() - stateTimer >= 1000) { 
         stopOperation = true; 
         stateTimer = millis(); 
         pitchState = 3;
      }
      break;

    case 3:
      stopOperation = true;
      if (millis() - stateTimer >= 4000) { 
         yaw_setpoint = yaw + 360; 
         baseSpeed = 0;      
         stopOperation = false;
         pid_previous_error = 0;
         pid_integral = 0;
         stateTimer = millis(); 
         pitchState = 4;
      }
      break;

    case 4:
      if (yaw >= (yaw_setpoint - 5)) { 
         stopOperation = true; 
         delay(500); 
         yaw_setpoint = yaw; 
         baseSpeed = 100;    
         stateTimer = millis();
         pitchState = 5;
      }
      if (millis() - stateTimer > 6000) {
         yaw_setpoint = yaw; 
         baseSpeed = 100;
         stateTimer = millis();
         pitchState = 5;
      }
      break;

    case 5:
      stopOperation = false;
      if (millis() - stateTimer >= 2000) { 
         pitchState = 6;
      }
      break;

    case 6:
      stopOperation = true;
      setMotorSpeed('A', 0);
      setMotorSpeed('B', 0);
      
      lcd.clear();
      
      lcd.setCursor(0, 0); lcd.print("Run Finished!");
      lcd.setCursor(0, 1); lcd.print("Max Angle: "); lcd.print((int)maxPitch);
      
      while(1) { }
      break;
  }


  if (!stopOperation) {
    if (pitchState == 4) { 
      motorSpeedA = pid_output; 
      motorSpeedB = -pid_output; 
    } else {
      double effective_pid = pid_output;
      if (pitchState == 0 || pitchState == 1 || pitchState == 2) {
          effective_pid = constrain(pid_output, -40, 40); 
      }
      motorSpeedA = baseSpeed + effective_pid; 
      motorSpeedB = baseSpeed - effective_pid; 
    }

    motorSpeedA = constrain(motorSpeedA, -255, 255);
    motorSpeedB = constrain(motorSpeedB, -255, 255);

    setMotorSpeed('A', motorSpeedA);
    setMotorSpeed('B', motorSpeedB);
  } else {
    setMotorSpeed('A', 0);
    setMotorSpeed('B', 0);
  }

  if (millis() % 200 == 0) { 
     if (pitchState != 6) {
        lcd.clear();
        
        lcd.setCursor(0, 0);
        lcd.print("P:"); lcd.print((int)pitch);
        lcd.print(" Max:"); lcd.print((int)maxPitch);

        lcd.setCursor(0, 1);
        lcd.print("Yaw Angle:"); lcd.print((int)yaw);
     }
  }
}

void readMPUData() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true); 

  AccX = (int16_t)(Wire.read() << 8 | Wire.read());
  AccY = (int16_t)(Wire.read() << 8 | Wire.read());
  AccZ = (int16_t)(Wire.read() << 8 | Wire.read());
  Wire.read(); Wire.read(); 
  GyroX = (int16_t)(Wire.read() << 8 | Wire.read());
  GyroY = (int16_t)(Wire.read() << 8 | Wire.read());
  GyroZ = (int16_t)(Wire.read() << 8 | Wire.read());

  // pitchAngle = atan2(accY, -accZ) * 180.0 / PI; 
  float accPitch = atan2(AccY, -AccZ) * 180 / PI;
  
  pitch = 0.98 * (pitch + (GyroY / 131.0) * elapsedTime) + 0.02 * accPitch;

  // yawAngle += ((gyroZ - gyroZOffset) / 131.0) * dt;
  yaw = yaw + ((GyroZ / 131.0) - GyroErrorZ) * elapsedTime;
}

void setMotorSpeed(char motor, int speed) {
  int enPin, in1Pin, in2Pin;
  if (motor == 'A') { enPin = MOTOR_A_EN; in1Pin = MOTOR_A_IN1; in2Pin = MOTOR_A_IN2; }
  else              { enPin = MOTOR_B_EN; in1Pin = MOTOR_B_IN1; in2Pin = MOTOR_B_IN2; }

  if (speed > 0) {
    digitalWrite(in1Pin, HIGH); digitalWrite(in2Pin, LOW);
    analogWrite(enPin, constrain(abs(speed), 0, 255));
  } else if (speed < 0) {
    digitalWrite(in1Pin, LOW); digitalWrite(in2Pin, HIGH);
    analogWrite(enPin, constrain(abs(speed), 0, 255));
  } else {
    
    digitalWrite(in1Pin, LOW); digitalWrite(in2Pin, LOW);
    analogWrite(enPin, 0); 
  }
}

void calculate_IMU_error() {
  float sum = 0;
  lcd.setCursor(0, 1); lcd.print("Calibrating...");
  for (int i = 0; i < 200; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 2, true);
    sum += (int16_t)(Wire.read() << 8 | Wire.read());
    delay(3);
  }
  GyroErrorZ = (sum / 200.0) / 131.0;
  lcd.clear();
}
