/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <stdint.h>
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

ros::NodeHandle  nh;

Servo tiltLeft, tiltRight;
Servo panLeft, panRight;

int16_t *data;
void servo_cb( const std_msgs::Int16MultiArray& cmd_msg){
  data = cmd_msg.data;
  setPan((int16_t )data[0], panLeft); //set servo angle, should be from 0-180  
  setPan((int16_t )data[1], panRight);
  setTilt((int16_t )data[2], tiltLeft);
  setTilt((int16_t )data[3], tiltRight);
}


ros::Subscriber<std_msgs::Int16MultiArray> sub("servo", servo_cb);

void setup(){

  nh.initNode();
  nh.subscribe(sub);
  
  tiltLeft.attach(3); 
  panLeft.attach(9);
  tiltRight.attach(10); 
  panRight.attach(11);

  
}

//0 <= degress <= 180
void setPan(int16_t degrees, Servo servo){
  servo.writeMicroseconds(1100 + ((degrees / 250.0) * 850));
  }

//0 <= degress <= 90
void setTilt(int16_t degrees, Servo servo){
  servo.writeMicroseconds(1100 + ((degrees / 90.0) * 400));
  }
  
void loop(){
  nh.spinOnce();
  delay(1);
}
