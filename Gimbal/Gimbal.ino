// Rewritten demo from Adafruit's MPU6050 library

#include <Wire.h>
#include <Servo.h>

const int MPU = 0x68; // MPU I2C address. Confirm with i2c_scanner example
float gyrox, gyroy, gyroz;
float xrotation, yrotation, zrotation;
float deltaTime, prevTime = 0;
Servo servo;
int servo_target;
int offset = 90;
float FS_factor = 131.0; // change according to MPU +-G setting. Datasheet p32

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
  //request the signal from GYRO_OUT registers
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // GYRO_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  // consume the signal
  gyrox = (Wire.read() << 8 | Wire.read()) / FS_factor; // for setting +-250 degree/s, divide by 131.0
  gyroy = (Wire.read() << 8 | Wire.read()) / FS_factor; 
  gyroz = (Wire.read() << 8 | Wire.read()) / FS_factor; 

  // Gyro data is in degrees/s. Convert to degrees and add to current rotational position (like adding velo to pos)
  deltaTime = millis()/1000.0 - prevTime;
  prevTime = millis()/1000.0;
  xrotation = xrotation + gyrox * deltaTime;
  yrotation = yrotation + gyroy * deltaTime;
  zrotation = zrotation + gyroz * deltaTime;
  /* Print out the values */
  printGyroVariables();
  if (Serial.available()){
    offset = Serial.parseInt();
  }
  // servo.write(0); // Run one time so I know which direction to attach the servo horn
  servo_target = xrotation + offset;
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