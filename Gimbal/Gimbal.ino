// Rewritten demo from Adafruit's MPU6050 library

#include <Wire.h>
#include <Servo.h>

const int MPU = 0x68; // MPU I2C address. Confirm with i2c_scanner example
float gyrox, gyroy, gyroz, accelx, accely, accelz;
float xrotation, yrotation, zrotation;
float deltaTime, prevTime = 0;
Servo servo;
int servo_target;
int offset = 90;
float FS_gyro_factor = 131.0; // change according to MPU +-G setting. Datasheet p32
float FS_accel_factor = 16384.0; // change according to MPU +-G setting. Datasheet p30
int accel_expected_max = 16384.0; // observed 1G value. Bandaid fix

const int DATA_WINDOW_SIZE = 128;
int dataWindow[DATA_WINDOW_SIZE];
int dataWindowIndex = 0;

void setup(void) {
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); // PWR_MGNT register. Reset this on setup
  Wire.write(0x00); // TODO: experiment with 0b1000_0000. See if this also properly resets
  Wire.endTransmission(true);
  servo.attach(9);
  Serial.println("Setup Complete");
  delay(100);
  /*
  Optionally configure MPU
  We'll use default +/-2g, +/-250 degree/s
  */
}

void loop() {

  /* Get new sensor events with the readings */
  //request signal from ACCEL_OUT registers
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // address of ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  //consume signal
  accelx = (Wire.read() << 8 | Wire.read()) / 1.0; // read 2 bytes/registers (OUT_H and OUT_L) and reassemble the 16b data
  accely = (Wire.read() << 8 | Wire.read()) / 1.0;
  accelz = (Wire.read() << 8 | Wire.read()) / 1.0;
  //request the signal from GYRO_OUT registers
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // GYRO_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  // consume the signal
  gyrox = (Wire.read() << 8 | Wire.read()) / FS_gyro_factor; // for setting +-250 degree/s, divide by 131.0
  gyroy = (Wire.read() << 8 | Wire.read()) / FS_gyro_factor; 
  gyroz = (Wire.read() << 8 | Wire.read()) / FS_gyro_factor; 

  // Gyro data is in degrees/s. Convert to degrees and add to current rotational position (like adding velo to pos)
  deltaTime = millis()/1000.0 - prevTime;
  prevTime = millis()/1000.0;
  xrotation = xrotation + gyrox * deltaTime;
  yrotation = yrotation + gyroy * deltaTime;
  zrotation = zrotation + gyroz * deltaTime;
  // Record datum
  dataWindow[dataWindowIndex] = accelx;
  dataWindowIndex++;
  if (dataWindowIndex >= DATA_WINDOW_SIZE) dataWindowIndex = 0; // loop back to beginning of window
  /* Print out the values */
  printAccelVariables();
  // printGyroVariables();
  printStats();
  if (Serial.available()){
    offset = Serial.parseInt();
  }
  // servo.write(0); // Run one time so I know which direction to attach the servo horn
  servo_target = map(accelx, -accel_expected_max, accel_expected_max, -90, 90) + offset;
  constrain(servo_target, 0, 180);
  servo.write(servo_target);
  Serial.print("servo: ");
  Serial.println(servo.read());
  Serial.print("offset: ");
  Serial.println(offset);

  Serial.println("");
  delay(250);
}

void printGyroVariables(){
  Serial.println();
  Serial.print("x: ");
  Serial.print(xrotation);
  Serial.print("\t");
  Serial.println(gyrox);
  Serial.print("y: ");
  Serial.print(yrotation);
  Serial.print("\t");
  Serial.println(gyroy);
  Serial.print("z: ");
  Serial.print(zrotation);
  Serial.print("\t");
  Serial.println(gyroz);
}

void printAccelVariables(){
  Serial.println();
  Serial.print("x: ");
  Serial.println(accelx);
  Serial.print("y: ");
  Serial.println(accely);
  Serial.print("z: ");
  Serial.println(accelz);

}

void printStats(){
  Serial.print("mean = ");
  Serial.print(calculateMean(&dataWindow));
  Serial.print("\t");
  Serial.print("std = ");
  Serial.println(calculateSTD(&dataWindow));
}

float calculateMean(int (*arr)[DATA_WINDOW_SIZE]){
  size_t len = sizeof(*arr) / sizeof((*arr)[0]);
  long sum = 0;
  for (int i=0; i < len; i++) sum += (*arr)[i];
  return sum / (float)len;
}

float calculateSTD(int (*arr)[DATA_WINDOW_SIZE]){
  float m = calculateMean(arr);
  double sum = 0.0;
  for (int i=0; i<DATA_WINDOW_SIZE; i++) sum += sq((*arr)[i] - m);
  return sqrt(sum / DATA_WINDOW_SIZE);
}