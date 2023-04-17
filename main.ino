#include <util/atomic.h>
#include <NewPing.h>

#define TRIGGER_PINL  A1  // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PINL     A0  // Arduino pin tied to echo pin on ping sensor.

#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#define TRIGGER_PINF  A3  // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PINF     A2  // Arduino pin tied to echo pin on ping sensor.

#define TRIGGER_PINR  A5  // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PINR     A4  // Arduino pin tied to echo pin on ping sensor.

NewPing sonarLeft(TRIGGER_PINL, ECHO_PINL, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarRight(TRIGGER_PINR, ECHO_PINR, MAX_DISTANCE);
NewPing sonarFront(TRIGGER_PINF, ECHO_PINF, MAX_DISTANCE);
unsigned int pingSpeed = 30; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long pingTimer;     // Holds the next ping time.

class SimplePID{
  private:
    float kp, kd, ki, umax; // Parameters
    float eprev, eintegral; // Storage

  public:
  // Constructor
  SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0){}

  // A function to set the parameters
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
    kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
  }

  // A function to compute the control signal
  void evalu(int value, int target, float deltaT, int &pwr, bool check){ //bool check to check for run with wall or not 
    // error
    int e;  float u;
    if(check==0)
      e = target + value; //run with no wall
    else 
      e = target - value; //run with wall 
    // derivative
    float dedt = (e-eprev)/(deltaT);//
  
    // integral
      eintegral = eintegral + e*deltaT;// 
    
    // control signal
     //u = kp*e;
     u = kp*e + kd*dedt + ki*eintegral;

    // motor power
    pwr = max(min(70,u),-70);

 eprev = e;
 
  // store previous error  
  }

  // A function to compute the control signal
  void evalu_turn(int value, int target, float deltaT, int &pwr, int &dir){
    // error
    int e = target - value;
  
    // derivative
    float dedt = (e-eprev)/(deltaT);
  
    // integral
    eintegral = eintegral + e*deltaT;
  
    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;
  
    // motor power
    pwr = (int) fabs(u);
    if( pwr > umax ){
      pwr = umax;
    }
  
    // motor direction
    dir = -1;
    if(u<0){
      dir = 1;
    }
  
    // store previous error
    eprev = e;
  }


};
// How many motors
#define NMOTORS 2

// Pins
const int enca[] = {2,3};
const int encb[] = {4,5};
const int pwm[] = {10,11};
const int in1[] = {6,8};
const int in2[] = {7,9};

// Globals
long prevT = 0;
volatile int posi[] = {0,0};

// PID class instances
SimplePID pid[NMOTORS];


int wall_threshold = 13 ;
int front_threshold = 7 ;

boolean frontwall = false;
boolean leftwall = false;
boolean rightwall = false;

long currT;
float deltaT;
int current_time;

float oldLeftSensor, oldRightSensor, leftSensor, rightSensor, frontSensor, oldFrontSensor, lSensor, rSensor, fSensor;
// set target position
  int target[NMOTORS];
  int pwr;
void setup()
{
	Serial.begin(9600);

  for(int k = 0; k < NMOTORS; k++){
    pinMode(enca[k],INPUT);
    pinMode(encb[k],INPUT);
    pinMode(pwm[k],OUTPUT);
    pinMode(in1[k],OUTPUT);
    pinMode(in2[k],OUTPUT);

    pid[k].setParams(2,0.1,0,100);
  }

  attachInterrupt(digitalPinToInterrupt(enca[0]),readEncoder<0>,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]),readEncoder<1>,RISING);
  
  Serial.println("target pos"); 

  // Go_straight_NO_wall();
  
  // turn_left();
  
  // turn_right();

  ReadSensors();
  while (leftSensor > 10)
  {
    ReadSensors();
  }

  }





void loop()
{
	 // time difference
  currT = micros();
  deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position in an atomic block to avoid a potential misread
  int pos[NMOTORS];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    for(int k = 0; k < NMOTORS; k++){
      pos[k] = posi[k];
    }
  }

    ReadSensors();
    walls();

    
    
    if(leftwall == true && rightwall == true && frontwall == false) { //go straight with wall
        //PID to go straight bettwen two wall
        for(int k = 0; k < NMOTORS; k++){
        int dir[]={-1,1};
        // evaluate the control signal
        pid[k].setParams(13,2.5,0,100);
        pid[k].evalu(leftSensor,rightSensor,deltaT,pwr,1);
        // signal the motor
        setMotor(dir[k],pwr,pwm[k],in1[k],in2[k],0);  
    }      
    }
    if(leftwall == false && rightwall == true && frontwall == true){
      turn_left();
      Stop();
    }
    if(leftwall == true && rightwall == false && frontwall == true){
      turn_right();
      Stop();
    }
    if(leftwall == false && rightwall == false && frontwall == true){
      turn_right();
      Stop();
    }
     
     Go_straight_NO_wall();


}
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2,bool check_turn){   // check_turn
if(check_turn == 0 ){
  if(pwm==10)
  {
   if(pwmVal >0)
     analogWrite(pwm,100-abs(pwmVal));
   else
     analogWrite(pwm,100+abs(pwmVal)); 
    }
  if(pwm == 11)
  {
   if(pwmVal >0)
     analogWrite(pwm,100+abs(pwmVal));
   else
     analogWrite(pwm,100-abs(pwmVal)); 
    }
}
else
  analogWrite(pwm,pwmVal);

  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
\
  //dir=-1 thì bánh phải tiến còn bánh trái lùi
  //dir = 1 thì ngược lại
}

template <int j>
void readEncoder(){
  int b = digitalRead(encb[j]);
  if(b > 0){
    posi[j]++;
  }
  else{
    posi[j]--;
  }
}


void ReadSensors() {

  lSensor = sonarLeft.ping_cm(); //ping in cm
  rSensor = sonarRight.ping_cm();
  fSensor = sonarFront.ping_cm();


  leftSensor = (lSensor + oldLeftSensor) / 2; //average distance between old & new readings to make the change smoother
  rightSensor = (rSensor + oldRightSensor) / 2;
  frontSensor = (fSensor + oldFrontSensor) / 2;


  oldLeftSensor = leftSensor; // save old readings for movment
  oldRightSensor = rightSensor;
  oldFrontSensor = frontSensor;

}
void walls() {


  if ( leftSensor < wall_threshold ) {
    leftwall = true ;
  }
  else {
    leftwall = false ;
  }


  if ( rightSensor < wall_threshold ) {
    rightwall = true ;
  }
  else {
    rightwall = false ;


  } if ( frontSensor < front_threshold ) {
    frontwall = true ;
  }
  else {
    frontwall = false ;
  }

}

void turn_left()
{
  for(int i=0; i<2; i++)
    posi[i]=0;
  Serial.println("TURN LEFT");
  unsigned long times= millis();
  while(millis() - times <1200) //
  {
    int target[NMOTORS];
  target[0] = 670; // 720
  target[1] = 670; //

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position in an atomic block to avoid a potential misread
  int pos[NMOTORS];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    for(int k = 0; k < NMOTORS; k++){
      pos[k] = posi[k];
    }
  }
  
  // loop through the motors
  for(int k = 0; k < NMOTORS; k++){
    int pwr, dir;
    pid[k].setParams(1,0.1,0,200);
    // evaluate the control signal
    pid[k].evalu_turn(pos[k],target[k],deltaT,pwr,dir);
    // signal the motor
    setMotor(dir,pwr,pwm[k],in1[k],in2[k],1);
  }
  }}

void turn_right()
{
  for(int i=0; i<2; i++)
    posi[i]=0;
  Serial.println("TURN RIGHT");
  unsigned long times= millis();
  while(millis() - times <1200) //
  {
    int target[NMOTORS];
  target[0] = -670;  // degree -700
  target[1] = -670; //

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position in an atomic block to avoid a potential misread
  int pos[NMOTORS];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    for(int k = 0; k < NMOTORS; k++){
      pos[k] = posi[k];
    }
  }
  
  // loop through the motors
  for(int k = 0; k < NMOTORS; k++){
    int pwr, dir;
     pid[k].setParams(1,0.01,0,200);
    // evaluate the control signal
    pid[k].evalu_turn(pos[k],target[k],deltaT,pwr,dir);
    // signal the motor
    setMotor(dir,pwr,pwm[k],in1[k],in2[k],1);
  }
  }}
  void Go_straight_NO_wall()
  {
    posi[0]=0;  posi[1]=0;
    while(frontwall== false)
    {
        ReadSensors();
        walls();
        
      if(leftwall == true && rightwall == true && frontwall == false)
        break;

     currT = micros();
     deltaT = ((float) (currT - prevT))/( 1.0e6 );
     prevT = currT;

    for(int k = 0; k < NMOTORS; k++){
    int dir[]={-1,1}; //go straight
    // evaluate the control signal
    pid[k].setParams(2,0.1,0,100);

    pid[k].evalu(posi[0],posi[1],deltaT,pwr,0);
    // signal the motor
    setMotor(dir[k],pwr,pwm[k],in1[k],in2[k],0);  
     }
    }
    if(frontwall == true)
    Stop();
  }
  void Stop(){
    setMotor(0,0,pwm[0],in1[0],in2[0],0);
    setMotor(0,0,pwm[1],in1[1],in2[1],0);
    posi[0]=0;  posi[1]=0;
    delay(50);
  }