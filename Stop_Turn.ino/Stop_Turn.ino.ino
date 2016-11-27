#include<Ultrasonic.h>

Ultrasonic ultraright(A5,A4);   // (Trig PIN,Echo PIN)
Ultrasonic ultraleft(5,7);  // (Trig PIN,Echo PIN)

#define wall_dist 15.0
#define timestep 10

//#define kpd 5
//#define kid 0
//#define kdd 0

#define kpa 0.9     // nice at 1.0
#define kia 0.8 //was 0.5
#define kda 0.1

#define dErrormax 

#define f_range 2//forward range, the three region partition
#define PWM1 10
#define In3 13
#define In4 12
#define PWM2 11
#define In1 A1
#define In2 A0

float error[4]={0,0,0,0};
float t_error[2]={0,0};
float Udist1=0,Udist2=0;
float pid_output = 0;

float read_ul1()
{
  return(ultraleft.Ranging(CM));
}
float read_ul2()
{
  return(ultraright.Ranging(CM));
}


//int pid1(float v1,float v2)
//{
//  error[0]= error[1];         // error[0] is now the last error[1]
//  error[1] = v1 - v2;         // new error[1]
//  
//  t_error[0] += error[1];     // total error summation for integration part
//  int correction = kpd*error[1] + kid*t_error[0] + kdd*(error[1] - error[0]);     //correction function
//  return correction;
//}

int pid2(float v1,float v2, float integral)
{
  error[2]= error[3];         // error[0] is now the last error[1]
  error[3] = v1 - v2;         // new error[1]
  int m_time = millis();
  if(m_time%500==0)
  {
    t_error[1] += integral;     // total error summation for integration part
  }
  if(t_error[1]>100)
  {
    t_error[1] = 100;
  }
  int correction_angle = kpa*error[3] + kia*t_error[1] + kda*(error[3] - error[2]);     //correction function
  
  return (correction_angle);
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
  digitalWrite(In1,HIGH); 
  digitalWrite(In3,HIGH);
  digitalWrite(In2,LOW);
  digitalWrite(In4,LOW);

  //for starting
  Udist1=read_ul1();
  Udist2=read_ul2();
  while(Udist1==51 || Udist2==51)
  {
    
    if(Udist1==51)
    {
      analogWrite(PWM1,0); 
      analogWrite(PWM2,225);
    }
    else
     {
      analogWrite(PWM1,220); 
      analogWrite(PWM2,0);
     }
    Udist1=read_ul1();
    Udist2=read_ul2();  
  }
  
}
float pdist1=wall_dist, pdist2=wall_dist;
int overshoot=-1; //-1 means false
//LOOP:
void loop() 
{

  float w1=4,w2=13;            //2,13
  //error 1oo units
  Udist1=read_ul1();
  Udist2=read_ul2();
  pdist1=Udist1;
  pdist2=Udist2;
  Serial.print(Udist1);
  Serial.print("\t");
  Serial.print(Udist2);
  Serial.print("\t\t\t");
//  run_v(Udist1-wall_dist, Udist2-wall_dist);
  if(Udist1==51 || Udist2==51)
  {
    pid_output = (((w2*(Udist1-Udist2))+(w1*(Udist1+Udist2)/2))-w1*wall_dist)*0.4;
  }
  else 
    pid_output = pid2((w2*(Udist1-Udist2))+(w1*(Udist1+Udist2)/2),w1*wall_dist, (Udist1+Udist2)/2);
  int motor1=195 - pid_output;
  int motor2=200 + pid_output;
  if(motor1>255)
    motor1=255;
  if(motor2>255)
    motor2=255;
  if(motor1<0)
    motor1=0;
  if(motor2<0)
    motor2=0;
  analogWrite(PWM1,motor1);
  analogWrite(PWM2,motor2);
  Serial.print(motor1);
  Serial.print("\t");
  Serial.println(motor2);

}
  
  
//2, 13, 1
