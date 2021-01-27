#include <Arduino.h>
#include <Servo.h>

// PWM specs of the Victor SP motor controller.
//      https://www.vexrobotics.com/217-9090.html
#define victorMax 2350
#define victorMin 650
#define pinGripL 5
#define pinGripR 6

Servo servoGripLeft;
Servo servoGripRight;
int posLeft = 90;
int posRight = 120;

struct JOINTPINS{
  int wrist_roll = 9; // wrist roll pin
  int wrist_pitch = 10; // wrist pitch pin
  int upper_elbow = 11; // upper elbow pin
  int lower_elbow = 12;  // lower elbow pin
  int base_yaw = 13;  // base yaw pin
  int cam_tilt = 7; // camera servo pin
}pins;

struct RECIEVED{
  float wrist_roll,wrist_pitch,upper_elbow,lower_elbow,base_yaw,gripper,winch,cam_tilt;
}data;

Servo joint0, joint1, joint2, joint3, joint4, cam_tilt;

int mapToVictor(float input){
  return map((input*100),-100,100,victorMin,victorMax);
}
void recievedData(){
  if(Serial.available()>=sizeof(float)){
    Serial.readBytes((char*)&data.gripper,sizeof(float));
    Serial.readBytes((char*)&data.wrist_roll,sizeof(float));
    Serial.readBytes((char*)&data.wrist_pitch,sizeof(float));
    Serial.readBytes((char*)&data.upper_elbow,sizeof(float));
    Serial.readBytes((char*)&data.lower_elbow,sizeof(float));
    Serial.readBytes((char*)&data.base_yaw,sizeof(float));
    Serial.readBytes((char*)&data.cam_tilt,sizeof(float));
  }
}

void writeToJoints(){
  joint0.writeMicroseconds(mapToVictor(data.wrist_roll));
  joint1.writeMicroseconds(mapToVictor(data.wrist_pitch));
  joint2.writeMicroseconds(mapToVictor(data.upper_elbow));
  joint3.writeMicroseconds(mapToVictor(data.lower_elbow));
  joint4.writeMicroseconds(mapToVictor(data.base_yaw));
  cam_tilt.write((int)data.cam_tilt);

  // Gripper Open
  if (data.gripper == 1){
      posLeft=posLeft+5;
      posRight=posRight-5;
      if(posLeft<0){
          posLeft=0;}
      if(posLeft>180){
          posLeft=180;}
      if(posRight<0){
          posRight=0;}
      if(posRight>180){
          posRight=180;}
      servoGripLeft.write(posLeft);
      servoGripRight.write(posRight);
      delay(100);
  }
  // Gripper Close
  else if (data.gripper == -1){
      posLeft=posLeft-5;
      posRight=posRight+5;
      if(posLeft<0){
          posLeft=0;}
      if(posLeft>180){
          posLeft=180;}
      if(posRight<0){
          posRight=0;}
      if(posRight>180){
          posRight=180;}
      servoGripLeft.write(posLeft);
      servoGripRight.write(posRight);
      delay(100);
  }
  // Gripper Stop
  else if (data.gripper == 0){
      servoGripLeft.write(posLeft);
      servoGripRight.write(posRight);
  }
}
void setup() {
  Serial.begin(9600);
  joint0.attach(pins.wrist_roll);
  joint1.attach(pins.wrist_pitch);
  joint2.attach(pins.upper_elbow);
  joint3.attach(pins.lower_elbow);
  joint4.attach(pins.base_yaw);
  cam_tilt.attach(pins.cam_tilt);
  servoGripLeft.attach(pinGripL);
  servoGripRight.attach(pinGripR);
  recievedData();
}

void loop() {
  recievedData();
  writeToJoints();
}
