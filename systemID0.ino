//#include <Metro.h>
//#include "SoftwareSerial.h";

#include <ros.h>
#include <std_msgs/Float32.h>


#define encoder0PinA  2
#define encoder0PinB  3

#define motorIn1 6
#define InA 4
#define InB 5

#define LOOPTIME 5

int pinAState = 0;
int pinAStateOld = 0;
int pinBState = 0;
int pinBStateOld = 0;

double omega_actual = 0.0;

volatile long Encoderpos = 0;

long dT = 0;
volatile int lastEncoded = 0;
unsigned long lastMilli = 0;                    // loop timing 
unsigned long startMilli = 0;


int PWM_val = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)

int CPR = 16 * 4;   // CPR=12*4  for JGA25-371 rpm=19
int gear_ratio = 30;  //gear_ratio=226

ros::NodeHandle nh;

std_msgs::Float32 omega;
ros::Publisher publisher("/omega", &omega);

void messageCb( const std_msgs::Float32& voltage){

  PWM_val = (float) 255 * voltage.data/12;

  //PWM_val = 212;
}

ros::Subscriber<std_msgs::Float32> subscriber("/voltage", &messageCb );


void setup() { 
 pinMode(encoder0PinA, INPUT); 
 digitalWrite(encoder0PinA, HIGH);       // turn on pullup resistor
 pinMode(encoder0PinB, INPUT); 
 digitalWrite(encoder0PinB, HIGH);       // turn on pullup resistor

 attachInterrupt(0, doEncoder, CHANGE);  // encoder pin on interrupt 0 - pin 2
 attachInterrupt(1, doEncoder, CHANGE);
// pinMode(Rx, INPUT); pinMode(Tx, OUTPUT);
 pinMode(InA, OUTPUT); 
 pinMode(InB, OUTPUT); 
 //mySerial.begin (19200);
 //Serial.begin (57600);
 //Serial.begin (57600);

 startMilli = millis();

 nh.getHardware()->setBaud(1000000);
 nh.initNode();
 nh.subscribe(subscriber);
 nh.advertise(publisher);
} 

void loop() 
{  
  //readCmd_wheel_angularVel();
  //omega_target = 6.28;  //for debugging
/*
  if (millis() - startMilli >= 10000) {
    PWM_val = 212;  // try a step with 10 volts

  }
*/
      
  if ((millis()-lastMilli) >= LOOPTIME)   
  {                                    // enter timed loop
        dT = millis()-lastMilli;
        lastMilli = millis();
        getMotorData(); 

        sendFeedback_wheel_angularVel(); // send actually speed to mega
        
                
        if (PWM_val <= 0)   { analogWrite(motorIn1,abs(PWM_val));  digitalWrite(InA, LOW);  digitalWrite(InB, HIGH); }
        if (PWM_val > 0)    { analogWrite(motorIn1,abs(PWM_val));  digitalWrite(InA, HIGH);   digitalWrite(InB, LOW);}
        
        // print Motor info for debugging
        //printMotorInfo();
  }
     
  nh.spinOnce();
}



void sendFeedback_wheel_angularVel()
{

  omega.data = omega_actual;
  publisher.publish( &omega );

  
  /*
  actual_send = int(omega_actual/0.00031434064); //convert rad/s to 16 bit integer to send
  char sT='{'; //send start byte
  byte sH = highByte(actual_send); //send high byte
  byte sL = lowByte(actual_send);  //send low byte
  char sP='}'; //send stop byte
 
  Serial.write(sT); Serial.write(sH); Serial.write(sL); Serial.write(sP);
  */
}

void getMotorData()  
{                               
  static long EncoderposPre = 0;       
  //converting ticks/s to rad/s
  //omega_actual = 4.5;
  omega_actual = ((Encoderpos - EncoderposPre)*(1000/dT))*2*PI/(CPR*gear_ratio);  //ticks/s to rad/s
  EncoderposPre = Encoderpos;                 
}

/*
double updatePid(double targetValue,double currentValue)   
{            
  
  static double last_error=0;                            
  error = targetValue - currentValue; 

  // Added by KKuei to remove the noises
  //if (error <= 0.1 && error >= -0.1) error = 0.0;
  
  sum_error = sum_error + error * dT;

  // Added by KKuei to bound sum_error range
  //sum_error = constrain(sum_error, -3000, 3000);
  
  d_error = (error - last_error) / dT;
  pidTerm = Kp * error + Ki * sum_error + Kd * d_error;   
  //pidTerm = Kp * error + Kd * d_error;                         
  last_error = error; 

  //calculated_pidTerm = pidTerm/0.04039215686;

  // added by KKuei
  calculated_pidTerm = pidTerm/0.047058824;
  constrained_pidterm = constrain(calculated_pidTerm, -255, 255);
  
  return constrained_pidterm;
}

*/

void doEncoder() {
//   Encoderpos++;
  pinAState = digitalRead(2);
  pinBState = digitalRead(3);

  if (pinAState == 0 && pinBState == 0) {
    if (pinAStateOld == 1 && pinBStateOld == 0) // forward
      {
        Encoderpos ++;
        goto rp;
      }
    if (pinAStateOld == 0 && pinBStateOld == 1) // reverse
      {
        Encoderpos --;
        goto rp;
      }
  }
  if (pinAState == 0 && pinBState == 1) {
    if (pinAStateOld == 0 && pinBStateOld == 0) // forward
      {
        Encoderpos ++;
        goto rp;
      }
    if (pinAStateOld == 1 && pinBStateOld == 1) // reverse
      {
        Encoderpos --;
        goto rp;
      }
  }
  if (pinAState == 1 && pinBState == 1) {
    if (pinAStateOld == 0 && pinBStateOld == 1) // forward
      {
        Encoderpos ++;
        goto rp;
      }
    if (pinAStateOld == 1 && pinBStateOld == 0) // reverse
      {
        Encoderpos --;
        goto rp;
      }
  }

  if (pinAState == 1 && pinBState == 0) {
    if (pinAStateOld == 1 && pinBStateOld == 1) // forward
      {
        Encoderpos ++;
        goto rp;
      }
    if (pinAStateOld == 0 && pinBStateOld == 0) // reverse
      {
        Encoderpos --;
        goto rp;
      }
  }

  rp:
  pinAStateOld = pinAState;
  pinBStateOld = pinBState;
}

/*
void printMotorInfo()  
{                                                                      
   static int samples = 0;
   
   //Serial.print("  actual:");                  Serial.print(omega_actual);
   //Serial.println();
  
   //Serial.print("  target:");                  Serial.print(omega_target);
   
   //Serial.println();

   if (samples >= 1) {
      Serial.print("  target:");                  Serial.print(omega_target);
      Serial.print("  actual:");                  Serial.print(omega_actual);
      
      Serial.print("  error:");                  Serial.print(error);
      Serial.print("  sum_err:");                  Serial.print(sum_error);
      
      Serial.print("  dT:");                  Serial.print(dT);
      Serial.print("  pidTerm:");             Serial.print(pidTerm);
      Serial.print("  calculated_pidTerm:");  Serial.print(calculated_pidTerm);
      Serial.print("  constrained_pidterm:"); Serial.print(constrained_pidterm); 
      //Serial.println();

      Serial.println();

      samples = 0;
   } else 
   {
     samples ++;
   }
   
   
}
*/

