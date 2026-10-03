// Rewritten demo from Adafruit's MPU6050 library

#include <Wire.h>
#include <Servo.h>

const int MPU = 0x68; // MPU I2C address. Confirm with i2c_scanner example
float gyrox, gyroy, gyroz, accelx, accely, accelz;
float errgx, errgy, errgz, errax, erray, erraz; // inherent error measurements of the MPU
float xrotation, yrotation, zrotation;
float deltaTime, prevTime = 0;
Servo servo1;
Servo servo2;
int servo_target1;
int servo_target2;
int offset = 90;
float FS_gyro_factor = 131.0; // change according to MPU +-G setting. Datasheet p32
float FS_accel_factor = 16384.0; // change according to MPU +-G setting. Datasheet p30
int accel_expected_max = 16384.0; // observed 1G value. Bandaid fix

const int DATA_WINDOW_SIZE = 128;
int dataWindow[DATA_WINDOW_SIZE];
int dataWindowIndex = 0;
const int DELAY_INITIAL = 10;
const int DELAY_FINAL = 250;
int delay_value = DELAY_INITIAL;

void setup(void) {
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); // PWR_MGNT register. Reset this on setup
  Wire.write(0x00); // TODO: experiment with 0b1000_0000. See if this also properly resets
  Wire.endTransmission(true);
  servo1.attach(9);
  servo2.attach(10);
  measureMPUerror(200); // 200 samples
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
  gyrox = ((Wire.read() << 8 | Wire.read()) - errgx) / FS_gyro_factor; // for setting +-250 degree/s, divide by 131.0, adjust by err
  gyroy = ((Wire.read() << 8 | Wire.read()) - errgy) / FS_gyro_factor; 
  gyroz = ((Wire.read() << 8 | Wire.read()) - errgz) / FS_gyro_factor;
  // Gyro data is in degrees/s. Convert to degrees and add to current rotational position (like adding velo to pos)
  deltaTime = millis()/1000.0 - prevTime;
  prevTime = millis()/1000.0;
  xrotation = xrotation + gyrox * deltaTime;
  yrotation = yrotation + gyroy * deltaTime;
  zrotation = zrotation + gyroz * deltaTime;
  // Record datum
  dataWindow[dataWindowIndex] = gyrox;
  dataWindowIndex++;
  if (dataWindowIndex >= DATA_WINDOW_SIZE) dataWindowIndex = 0; // loop back to beginning of window
  /* Print out the values */
  printAccelVariables();
  printGyroVariables();
  printStats();
  if (Serial.available()){
    offset = Serial.parseInt();
  }
  // servo2.write(0); // Run one time so I know which direction to attach the servo horn
  servo_target1 = map(accelx, -accel_expected_max, accel_expected_max, -90, 90) + offset;
  servo_target2 = map(accely, -accel_expected_max, accel_expected_max, -90, 90) + offset;
  constrain(servo_target1, 0, 180);
  constrain(servo_target2, 0, 180);
  servo1.write(servo_target1);
  servo2.write(servo_target2);
  Serial.print("servo1: ");
  Serial.print(servo1.read());
  Serial.print("\tservo2: ");
  Serial.println(servo2.read());
  Serial.print("offset: ");
  Serial.println(offset);

  Serial.println("");
  delay(delay_value);
  if (dataWindowIndex == DATA_WINDOW_SIZE-1) delay_value = DELAY_FINAL;
}

void printGyroVariables(){
  Serial.print("gyro:\n");
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
  Serial.print("accel:\n");
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

// Collect measurementCount samples and average out to estimate the
//  inherent error in measurement of the MPU. It doesn't quite report
//  0 acceleration and 9.8 m/s^2 at rest, so we need to set some offsets
//  as errgx/y/z and errax/y/z to correct for it.
// Do not move the MPU while this is running (in setup())
void measureMPUerror(int measurementCount){
  // Normally we compute average as
  //  sum = n1 + n2 + n3...;
  //  mean = sum / N;
  // but this may overflow the variable storing sum if n_i is a large number.
  // Instead we can use
  //  sum += n1 / N;
  //  sum += n2 / N; ...
  // probably increasing time (many div operations), but that cost is probably negligible here
  double meax = 0, meay = 0, meaz = 0, megx = 0, megy = 0, megz = 0 ; // "M.ean of E.rror A.ccel X."
  float ax, ay, az, gx, gy, gz;
  for (int i=0; i<measurementCount; i++){
    Wire.beginTransmission(MPU);
    Wire.write(0x3B); // address of ACCEL_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6+2+6,true); // read 6 bytes/registers of ACCEL, 2 bytes TEMP, 6 bytes GYRO
    //consume signal
    ax = (Wire.read() << 8 | Wire.read());
    ay = (Wire.read() << 8 | Wire.read());
    az = (Wire.read() << 8 | Wire.read());
    Wire.read(); // unused TEMP
    Wire.read(); // unused TEMP
    gx = (Wire.read() << 8 | Wire.read());
    gy = (Wire.read() << 8 | Wire.read());
    gz = (Wire.read() << 8 | Wire.read());
    // divide and add to rolling Mean
    meax += ax / measurementCount; // TODO: This formula does not work on a (seconds^0) measurement. Use the atan() formula instead
    meay += ay / measurementCount;
    meaz += az / measurementCount;
    megx += gx / measurementCount;
    megy += gy / measurementCount;
    megz += gz / measurementCount;
  }
  // Uncomment these after plugging in that black magic atan formula
  // errax = meax;
  // erray = meay;
  // erraz = meaz;
  errgx = megx;
  errgy = megy;
  errgz = megz;

  Serial.println("Error measurements:");
  Serial.print("ax (UNFINISHED): ");
  Serial.println(meax);
  Serial.print("ay (UNFINISHED): ");
  Serial.println(meay);
  Serial.print("az (UNFINISHED): ");
  Serial.println(meaz);
  Serial.print("gx: ");
  Serial.println(megx);
  Serial.print("gy: ");
  Serial.println(megy);
  Serial.print("gz: ");
  Serial.println(megz);
  Serial.println();
}