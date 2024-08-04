#include <Arduino.h>
#include "I2Cdev.h"
#include "pins_me.h"
#include <esp_now.h>
#include <WiFi.h>
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif



class SimplePID{
  private:
    float kp, kd, ki, umax; // Parameters
    float eprev, eintegral, valprev; // Storage

  public:
  // Constructor
  SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0){}

  // A function to set the parameters
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
    kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
  }

  // A function to compute the control signal
  void evalu(float value,int val_sign, float target,int trg_sign, float deltaT, int &pwr, int &dir){
    // error
    float e = trg_sign*target + val_sign*value;
  
    // derivative
    float dedt = (e-eprev)/(deltaT);
    float dvaldt = val_sign*(value-valprev)/(deltaT);
  
    // integral
    eintegral = eintegral + e*deltaT;
  
    // control signal
    //float u = kp*e + kd*dedt + ki*eintegral;
    float u = kp*e + kd*(dvaldt+dedt) + ki*eintegral;
  
    // motor power
    pwr = (int) fabs(u); //Absolute value of float;
    if( pwr > umax ){
      pwr = umax;
    }
  
    // motor direction
    dir = 1;
    if(u<0){
      dir = -1;
    }
  
    // store previous error
    eprev = e;
    valprev = value;
  }
  
};
// Define a data structure
typedef struct struct_message {
  long cam_info[2] ; // camera_id ,camera_frame
  bool fire_avail;
  float predic_score;
  int x_;
  int y_;
  int width_;
  int height_;

} struct_message;
// Create a structured object
struct_message myData;


// Setting PWM properties
const int freq = 300000;
const int freq_servo = 50;
const int c1 = 0;
const int c2 = 1;
const int c3 = 2;
const int resolution = 8;
int dutyCycle = 200;

// Globals
long t_prev=0;
volatile int pos_l =0;
volatile int pos_r =0;

//float target[16] ={0,22.5,45,67.5,90,112.5,135,157.5,180,202.5,225,247.5,270,292.5,315,337.5};
float target[15] ={-157.5,-135,-112.5,-90,-67.5,-45,-22.5,0,22.5,45,67.5,90,112.5,135,157.5};
//float target[8] ={0,45,90,135,180,225,270,315};
int buff = 7; // starts at 0 degree
int sgn = 1;
float align_angle;
int speed = 0;
volatile long t_prev_speed = 0,t_prev_pump=0,currT,currT_speed;
//long currT


float tmp_angl = 0;
bool found1 = false;// detection for first camera
int frame1 = 0;
int score1 = 0;//how many of 5 frames contain fire

bool found2 = false;// detection for second camera
int frame2 = 0;
int score2 = 0;//how many of 5 frames contain fire
float dist = 0;
long time_req = 0;
int itr = 0;
bool reached = false;
int pump_cycle = 0;

bool dmpReady = false;  // set true if DMP init was successful
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3]={0.0, 0.0, 0.0},ypr_prev[3]={0.0, 0.0, 0.0};           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// PID class instances
SimplePID pid_left;
SimplePID pid_right;
MPU6050 mpu;

void IRAM_ATTR isr_right() {
	int c = digitalRead(enc_right_y);
  if(c>0)
  {pos_r--;}
  else
  {pos_r++;}
}

void IRAM_ATTR isr_left() {
	int b = digitalRead(enc_left_y);
  if(b>0)
  {pos_l--;}
  else
  {pos_l++;}
}
void dmpDataReady() {
  mpuInterrupt = true;
}
// Callback function executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  // Serial.print("Camera iD:------- ");
  // Serial.println(myData.cam_no);
  // Serial.print("Fire Available ?: ");
  // Serial.println(myData.fire_avail);
  // Serial.print("Prediction Value: ");
  // Serial.println(myData.predic_score);
  // Serial.print("x Value: ");
  // Serial.println(myData.x_);
  // Serial.print("y Value: ");
  // Serial.println(myData.y_);
  // Serial.println();
}

void setup() {
   // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  Serial.begin(115200);
  
  
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
    // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register callback function
  esp_now_register_recv_cb(OnDataRecv);


  pinMode(servo_pin,OUTPUT);
  pinMode(pump_pin,OUTPUT);

  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(EN_left, OUTPUT);
  pinMode(enc_left_y, INPUT);
  pinMode(enc_left_g, INPUT);
  attachInterrupt(enc_left_g, isr_left, RISING);
  pid_left.setParams(2.9,0.03,1/120000,90); //55,0.75,0,25 //1.75,0.42,0.0001,75// new sys :5.75,3.42,0.00001,95//20/3,80/3,0.00001,95

  pinMode(IN_3, OUTPUT);
  pinMode(IN_4, OUTPUT);
  pinMode(EN_right, OUTPUT);
  pinMode(enc_right_y, INPUT);
  pinMode(enc_right_g, INPUT);
  attachInterrupt(enc_right_g, isr_right, RISING);
  pid_right.setParams(2.9,0.03,1/120000,95); //best 1,0,0,150


  // configure LED PWM functionalitites
  ledcSetup(c1, freq, resolution);
  ledcSetup(c2, freq, resolution);
  ledcSetup(c3, freq_servo , resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(EN_left, c1);
  ledcAttachPin(EN_right, c2);
  ledcAttachPin(servo_pin, c3);
   

  delay(2000);
    // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(GYRO_INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  // while (Serial.available() && Serial.read()); // empty buffer
  // while (!Serial.available());                 // wait for data
  // while (Serial.available() && Serial.read()); // empty buffer again
  delay(200);
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(421);
  mpu.setYGyroOffset(-50);
  mpu.setZGyroOffset(214);
  mpu.setXAccelOffset(-4809);
  mpu.setYAccelOffset(-763);
  mpu.setZAccelOffset(1855);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(GYRO_INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(GYRO_INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

}

void loop() {
  
  currT = micros();

    if(!found1 && itr>15500) // if detected stop angle increment and align
    {
      itr = 0;
      buff = buff+sgn*1;
      if(buff>=14 || buff<=0)
        sgn = -1*sgn;
      // if(buff>=16)
      //   buff = 0;
      // itr = 0;
      // target[0] = target[0]+2;
      // if(target[0] >= 360)
      //   target[0] = 0;
    }
  if((myData.cam_info[0] == 1)&&!found1)
  {
    if(myData.fire_avail)
      {
        float val  = myData.x_;
         tmp_angl =  tmp_angl + map(val,0,88,-20,20);
         score1++;
      }   
    frame1 = myData.cam_info[1];
  }

  if(frame1 >=7)
  {
    frame1 = 0;
    if(score1>=5)
    {
      align_angle = tmp_angl/score1;
      found1 = true;
      score1 = 0;
      tmp_angl = 0;
      t_prev=currT;
      
    }
  }
//.............For Camera 2.....................................

  if((myData.cam_info[0] == 2)&&!found2&&((currT-t_prev)>5e6)&&(frame2<myData.cam_info[1]))
  {
    if(myData.fire_avail)
      {
        float val  = myData.x_;
         tmp_angl =  tmp_angl + map(val,0,88,-20,20);
         score2++;
      }   
    //frame2++;
    frame2 = myData.cam_info[1];
  }

  if(frame2 >=20 && !found2)
  {
    frame2 = 0;
    if(score2>=18)
    {
      tmp_angl = 90-tmp_angl/score2;
      // Required distance calculation
      dist = 9*tan(tmp_angl*M_PI/180);
      
      //Serial.println(tmp_angl);
      time_req = (dist/24.56)*1e6; // in micro seconds
      Serial.println(dist/24.56);
      Serial.println(dist);
      currT_speed = micros();
      t_prev_speed=currT_speed; // initialize timer
      found2 = true;
      score2 = 0;
      tmp_angl = 0;
      
    }
  }
  if(found2&&!reached)
  {
    
    speed = 17;
    //Serial.println(currT-t_prev_speed);
    if((currT-t_prev_speed)>=0.97*(time_req))
    {
      speed = 0;
      t_prev_pump = currT;
      found2 = false;
      reached = true;
    }

  }
   
  if(reached && (pump_cycle<3))
  { // 22 -------> left side
    // 14 -------> right side
    digitalWrite(pump_pin,HIGH);
    if((currT-t_prev_pump)>=0.8e6)
      {
        ledcWrite(c3,14);
        //t_prev_pump = currT;
        if((currT-t_prev_pump)>=2*0.8e6)
        {
          t_prev_pump = currT;
          pump_cycle++;
        }
        //t_prev_pump = ((currT-t_prev_pump)>=2*0.8e6)?currT:t_prev_pump;
        
      }else
        ledcWrite(c3,22);
      
  }
  else digitalWrite(pump_pin,LOW);


  //target_motion(currT,22+align_angle,speed);
  target_motion(currT,target[buff]+align_angle,speed);
  itr++;
  //Serial.println(ypr[0] * 180 / M_PI);

  
  //Serial.println(target[buff]+align_angle);
    // Serial.print(target[2]);
    // Serial.print(" ");
    // Serial.print(ypr[0] * 180 / M_PI);
    // Serial.println(" ");

    
  //Serial.println(buff);

}

// bool fire_detected(float &avg_align_ang)
// {
//   int i=0;
//   int pred_flg=0;
//   avg_align_ang = 0.0;
//   while(i<=5)
//   {
//     if(myData.cam_info[0] == 1)
//       {
//         pred_flg=pred_flg+myData.fire_avail;
//         avg_align_ang = avg_align_ang + map(myData.x_,96,0,-22.5,22.5);
//         i++;
//       }
//   }

//   if(pred_flg>=3){
//     avg_align_ang = avg_align_ang/pred_flg;
//     return true;
//   }else
//   {
//     avg_align_ang = 0;
//     return false;
//   }
   
// }

void target_motion(float currT,float target,int speed)
{
  static long prevT = 0; 
  static long prev_speedT = 0; 
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
  else
  {
    ypr[0]=ypr_prev[0];ypr[1]=ypr_prev[1];ypr[2]=ypr_prev[2];
  }
  
  // Read the position
  float pos_l_tmp,pos_r_tmp;
  // noInterrupts(); // disable interrupts temporarily while reading
  // pos_l_tmp = pos_l;
  // pos_r_tmp = pos_r;
  // interrupts(); // turn interrupts back on
  pos_l_tmp = ypr[0] * 180 / M_PI;
  //pos_l_tmp = (pos_l_tmp<0)?(pos_l_tmp+360):(pos_l_tmp);
  pos_r_tmp = pos_l_tmp;
  //Serial.println(pos_l_tmp);
  int pwr_l, dir_l;
    // evaluate the control signal
  pid_left.evalu(pos_l_tmp,-1,target,1,deltaT,pwr_l,dir_l);
  int pwr_r, dir_r;
    // evaluate the control signal
  pid_right.evalu(pos_r_tmp,1,target,-1,deltaT,pwr_r,dir_r);
  int flg =1;
    // signal the motor
  if((currT-prev_speedT)>(1*10e-6)){
    pwr_l = constrain(pwr_l+dir_l*speed,0,255);
    pwr_r = constrain(pwr_r+dir_r*speed,0,255);
    setMotor(dir_l,pwr_l,c1,IN_1,IN_2);
    setMotor(dir_r,pwr_r,c2,IN_3,IN_4);
    prev_speedT = currT;
    flg = 0;
  }



  if(((currT-prevT)>(10e-5))&&flg){
    setMotor(dir_l,pwr_l,c1,IN_1,IN_2);
    setMotor(dir_r,pwr_r,c2,IN_3,IN_4);
    prevT = currT;
    flg=1;
  }
  




  // if((currT-prevT)>(10e-6)){
  //   setMotor(dir_l,pwr_l,c1,IN_1,IN_2);
  //   setMotor(dir_r,pwr_r,c2,IN_3,IN_4);
  //   prevT = currT;
  // }
 
    // setMotor(dir_l,pwr_l,c1,IN_1,IN_2);
    // setMotor(dir_r,pwr_r,c2,IN_3,IN_4);
    // prevT = currT;

  //setMotor(dir_r,pwr_r-dir_r*speed,c2,IN_3,IN_4);  



    // Serial.print(deltaT);
    // Serial.print(" ");
    // Serial.print(pos_l_tmp);
    // Serial.print(" ");
    // Serial.print(pos_r_tmp);
    // Serial.println();

    ypr_prev[0] = ypr[0];
    ypr_prev[1] = ypr[1];
    ypr_prev[2] = ypr[2];
    //Serial.println(pos_l_tmp);
    //prevT = currT;

}

void setMotor(int dir,int pwmVal, int enx ,int in1,int in2)
{
  ledcWrite(enx, pwmVal);
  if(dir == -1)
  {
      digitalWrite(in1,HIGH);
      digitalWrite(in2,LOW);

  }
  else if(dir == 1)
  {
      digitalWrite(in1,LOW);
      digitalWrite(in2,HIGH);

  }
  else
  {
      digitalWrite(in1,LOW);
      digitalWrite(in2,LOW);
  }
}
