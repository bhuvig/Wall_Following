#include<Ultrasonic.h>

Ultrasonic ultraright(A5,A4);   // (Trig PIN,Echo PIN)
Ultrasonic ultraleft(5,7);  // (Trig PIN,Echo PIN)

#define wall_dist 10.0
#define timestep 100

#define f_range 4 //forward range, the three region partition
#define PWM1 10
#define In1 13
#define In2 12
#define PWM2 11
#define In3 A1
#define In4 A0

float read_ul1()
{
  
  return(ultraleft.Ranging(CM));
}
float read_ul2()
{
  
  return(ultraright.Ranging(CM));
}


void forward()
{
  digitalWrite(In1,HIGH);
  digitalWrite(In3,HIGH);
  digitalWrite(In2,LOW);
  digitalWrite(In4,LOW);
  analogWrite(PWM1,128);
  analogWrite(PWM2,128);
  delay(2*timestep);
}

void left()
{
  analogWrite(PWM1,128);
  analogWrite(PWM2,128);
  digitalWrite(In1,HIGH);
  digitalWrite(In3,LOW);
  digitalWrite(In2,LOW);
  digitalWrite(In4,LOW);
  
  delay(timestep);
}

void right()
{
  digitalWrite(In1,LOW);
  digitalWrite(In3,HIGH);
  digitalWrite(In2,LOW);
  digitalWrite(In4,LOW);
  analogWrite(PWM1,128);
  analogWrite(PWM2,128);
  delay(timestep);
}

void sharpturn(int sensor1, int sensor2){
  if(sensor1==1)
  {
    left();left();left();left();
  }
  else
  {
    right();right();right();right();
  }
  
}

void run_v(float dist1,float dist2)
{
  if(abs(dist1-dist2)<f_range)
    forward();
  else if(dist1-dist2>0)
    right();
  else
    left();    
}

//SETUP:
void setup() 
{
  Serial.begin(9600);
  
  pinMode(In1,OUTPUT);  //hercules motor driver
  pinMode(In2,OUTPUT);
  pinMode(PWM1,OUTPUT);
  pinMode(In3,OUTPUT);  //hercules motor driver
  pinMode(In4,OUTPUT);
  pinMode(PWM2,OUTPUT);
  
}
float Udist1=0, Udist2=0, pdist1=wall_dist, pdist2=wall_dist;
int overshoot=-1; //-1 means false
//LOOP:
void loop() 
{
  
  Udist1=read_ul1();
  Udist2=read_ul2();
  Serial.print(Udist1);
  Serial.print('\t');
  Serial.println(Udist2);

  pdist1=Udist1;
  pdist2=Udist2;
  delay(100);
//  if(abs(Udist1-pdist1)>5){
//    if(overshoot==-1){    //this is the first overshoot
//      overshoot=overshoot*-1;
//      return;
//    }
//    else
//      overshoot=-1;
//      sharpturn(1,0);  
//      return;   
//  }
//  else if(abs(Udist2-pdist2)>5){
//    if(overshoot==-1){    //this is the first overshoot
//      overshoot=overshoot*-1;
//      return;
//    }
//    else
//      overshoot=-1;
//      sharpturn(0,1);
//      return;
//  }
//  
//
//  else{
  run_v(Udist1-wall_dist, Udist2-wall_dist);
//  }
  
  
}
