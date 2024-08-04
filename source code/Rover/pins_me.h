#ifndef pins_me_h
#define pins_me_h


#define GYRO_INTERRUPT_PIN 35 
//MOTOR Right
const int IN_3=25;
const int IN_4=26;
const int EN_right=33; //pwm speed control
const int enc_right_y = 18;
const int enc_right_g = 19;

//MOTOR Left
const int IN_1=14;// REMEMBER 34,35 cannot give output
const int IN_2=27;
const int EN_left=32; //pwm speed control
const int enc_left_y = 17;
const int enc_left_g = 13;

//Servo & pump
const int servo_pin = 23;
const int pump_pin = 4;
#endif