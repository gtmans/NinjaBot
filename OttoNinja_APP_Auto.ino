/*
 * This code was adapted by Github/gtmans dec 2024
 * It combines robotlk code Remote_XY (control by phone/app) and selfmove avoid code
 * You can use left and right arm button to turn selfmove avoid on or off
 * since I made a breakout board for the D1 using Dual Base v2 for LOLIN(WEMOS)
 * there are some port changes and an added Led (optional) to show if (battery)power is on or off  
 * The new led blinks depending on distance measurements
 * 
 * PwrLed            on Gpio 12 (D6)
 * ServoRightFootPin on Gpio 4  (D2) // you can use original port Gpio 2 (D4) but that interferes with buildin led and uploading code
 * ServoRightLegPin  on Gpio 5  (D1) // you can use original port
 * 
 * Because of these changes some of the 3D-parts were modified. You can download them at Github/gtmans.
 * You can use the original parts by making some ajustments (with saw and drill and take some parts away)
 * Also changed parts to make bigger servo-arms fit and added wheelcovers
*/

/*
   source: https://www.youtube.com/watch?v=HNLLzTBRTtg
   source: https://robotlk.com/how-to-make-a-ninja-robot/

   -- Remote Control Otto Ninja --
   
   This source code of graphical user interface 
   has been generated automatically by RemoteXY editor.
   To compile this code using RemoteXY library 2.4.3 or later version 
   download by link http://remotexy.com/en/library/
   To connect using RemoteXY mobile app by link http://remotexy.com/en/download/                   
     - for ANDROID 4.5.1 or later version;
     - for iOS 1.4.1 or later version;
    
   This source code is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.    
*/

//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__ESP8266WIFI_LIB_POINT
#include <ESP8266WiFi.h>
#include <RemoteXY.h>
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>

// RemoteXY connection settings 
#define REMOTEXY_WIFI_SSID      "NINJA COMBI"
#define REMOTEXY_WIFI_PASSWORD  "12345678"
#define REMOTEXY_SERVER_PORT    6377
int lau,rau;

// RemoteXY configurate  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =
  { 255,6,0,0,0,66,0,13,8,0,
  5,32,3,12,41,41,1,26,31,1,
  3,79,16,16,12,1,31,82,240,159,
  166,190,0,1,3,56,39,18,12,1,
  31,240,159,146,191,0,1,3,79,39,
  17,12,1,31,240,159,166,191,0,1,
  3,56,16,17,12,1,31,76,240,159,
  166,190,0 };
  
// this structure defines all the variables and events of your control interface 
struct {
  // input variables
  int8_t J_x; // =-100..100 x-coordinate joystick position 
  int8_t J_y; // =-100..100 y-coordinate joystick position 
  uint8_t button_B; // =1 if button pressed, else =0 
  uint8_t button_X; // =1 if button pressed, else =0 
  uint8_t button_Y; // =1 if button pressed, else =0 
  uint8_t button_A; // =1 if button pressed, else =0 
  // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 
} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

//CALIBRATION SETTINGS:
//LEFT FOOT
int LFFWRS  =  44;  // forward  walking rotation Speed 0 = Slowest 90 = Fastest  Default = 12
int LFBWRS  =  44;  // backward walking rotation Speed 0 = Slowest 90 = Fastest  Default = 12
int LFFRRS  =  20;  // forward  rolling rotation Speed 0 = SLOW    90 = FAST  somehow this servo runs slower
int LFBRRS  =  20;  // backward rolling rotation Speed 0 = SLOW    90 = FAST  somehow this servo runs slower  
//RIGHT FOOT
int RFFWRS  =  12;  // forward  walking rotation Speed 0 = Slowest 90 = Fastest  Default = 12
int RFBWRS  =  12;  // backward walking rotation Speed 0 = Slowest 90 = Fastest  Default = 12
int RFFRRS  =  10;  // forward  rolling rotation Speed 0 = SLOW    90 = FAST  
int RFBRRS  =  10;  // backward rolling rotation Speed 0 = SLOW    90 = FAST 
//LEFT LEG 
int LA0     =  60;  // standing Position               0 = Full Tilt Right 180 = Full Tilt Left Default = 60
int LATL    =  80;  // tilt left walking position      0 = Full Tilt Right 180 = Full Tilt Left Default BASIC = 85  Default HUMANOID = 80
int LATR    =  30;  // tilt right walking position     0 = Full Tilt Right 180 = Full Tilt Left Default BASIC = 5   Default HUMANOID = 30
int LA1     = 170;  // roll Position                   0 = Full Tilt Right 180 = Full Tilt Left Default = 170
//RIGHT LEG
int RA0     = 126;  // standing position               0 = Full Tilt Right 180 = Full Tilt Left Default = 120
int RATL    = 150;  // tilt left walking position      0 = Full Tilt Right 180 = Full Tilt Left Default BASIC = 175 Default HUMANOID = 150
int RATR    = 100;  // tilt right walking position     0 = Full Tilt Right 180 = Full Tilt Left Default BASIC = 95  Default HUMANOID = 100
int RA1     =  10;  // roll position                   0 = Full Tilt Right 180 = Full Tilt Left Default = 10

int currentmillis1 = 0;
int currentmillis2 = 0;
int currentmillis3 = 0;
int ModeCounter    = 0;// Mode counter for biped/wheel mode

const uint8_t   PwrLed             = 12;     //D6 NEW
const uint8_t   ServoLeftFootPin   = 13;     //D7
const uint8_t   ServoLeftLegPin    = 15;     //D8
const uint8_t   LedBuildIn         = 2;      //D4
const uint8_t   ServoRightFootPin  = 4;      //D2 NEW
const uint8_t   ServoRightLegPin   = 5;      //D1 NEW

//HC-SR04
const uint8_t   echoPin            = 14;     //D5 NEW pin Echo of HC-SR04
const uint8_t   trigPin            = 16;     //D0 NEW pin Trig of HC-SR04
long            duration;                    // variable for the duration of sound wave travel
int             distance;                    // variable for the distance measurement
int             lastdistances[5],ldcount;
bool            blinkstate;
unsigned long   startAuto;
unsigned long   startMillis;
unsigned long   currentMillis;
unsigned long   blinktime,oldblinktime;

Servo           myservoLeftFoot;
Servo           myservoLeftLeg;
Servo           myservoRightFoot;
Servo           myservoRightLeg;

int             ROLL=1,WALK=0,STEP=0;
bool            AUTOGO=false,USE_MILLIS=false,DETOUR=false;

void setup() 
{
  Serial.begin            (115200);
  while (!Serial)         {delay(20);}
  Serial.println          (__FILE__);
  Serial.println          ();
  pinMode                 (PwrLed, OUTPUT);
  pinMode                 (LedBuildIn,OUTPUT);
  pinMode                 (trigPin,OUTPUT); 
  pinMode                 (echoPin,INPUT);
  digitalWrite            (PwrLed, HIGH);   // turn the LED on (battery power is used)
  myservoLeftFoot.attach  (ServoLeftFootPin,  544, 2400);
  myservoRightFoot.attach (ServoRightFootPin, 544, 2400);  
  myservoLeftLeg.attach   (ServoLeftLegPin,   544, 2400);
  myservoRightLeg.attach  (ServoRightLegPin,  544, 2400); 
  for (int d=0;d<4;d++)   {Distance();delay(200);}//set average of first 5 distance counts
  RemoteXY_Init           (); 
  startMillis=millis      ();
}

void loop() 
{ 
  Distance();
  currentMillis =       millis();
  blinktime     =       0;
  if (distance < 50)    {blinktime=1000;}
  if (distance < 25)    {blinktime=500;}
  if (distance < 12)    {blinktime=100;DETOUR=true;}
  
  if (blinktime!=0){
    oldblinktime  =     blinktime;
    if (currentMillis - startMillis >= blinktime){
      blinkstate  =     !blinkstate;
      digitalWrite      (PwrLed, blinkstate); 
      startMillis =     millis();
    }
  } else {
      if (oldblinktime!=0){
      digitalWrite      (PwrLed,HIGH);
      } 
  } 
  
if (AUTOGO){
  if (DETOUR){
    if (ModeCounter==WALK){
      if (USE_MILLIS){    // USE_MILLIS somehow interferes with RemoteXY_Handler
        if (STEP==0){Serial.println("Object detected at "+String(distance)+" cm. Taking detour!");
                                            NinjaStepBackward();STEP++;startAuto=millis();}
        if (STEP==1&&millis()-startAuto>1000){NinjaStepRight()   ;STEP++;startAuto=millis();}
        if (STEP==2&&millis()-startAuto>700 ){NinjaStepForward() ;STEP=0;DETOUR=false;}
      } else { //use delay io millis
          Serial.println("Object detected at "+String(distance)+" cm. Taking detour!");
          NinjaStepBackward();
          NinjaStepRight();
          NinjaStepForward();
          DETOUR=false;
      }
    } else { // roll io walk
      if (USE_MILLIS){
        if (STEP==0){Serial.println("Object detected at "+String(distance)+" cm. Taking detour!");
                                              NinjaRollBackward();STEP++;startAuto=millis();}
        if (STEP==1&&millis()-startAuto>1000){NinjaRollRight()   ;STEP++;startAuto=millis();}
        if (STEP==2&&millis()-startAuto>700 ){NinjaRollForward() ;STEP=0;DETOUR=false;}        
      } else { //use delay io millis
          Serial.println    ("Object detected at "+String(distance)+" cm. Taking detour!");
          NinjaRollBackward ();
          delay(1000);
          NinjaRollRight    ();
          delay(700);
          NinjaRollForward  ();
          DETOUR=false;
      }
    } // end of roll or walk
  }  // end of detour
}   // end of AUTOGO

  RemoteXY_Handler ();
  
  Serial.println("Button X="+String(RemoteXY.button_X)+" Y="+String(RemoteXY.button_Y));
  Serial.println("joystk X="+String(RemoteXY.J_x)     +" Y="+String(RemoteXY.J_y));
  
  if (RemoteXY.button_X == HIGH)
  {        
      NinjaSetRoll(); 
      ModeCounter = 1;  //ROLL 
  }
  if (RemoteXY.button_Y == HIGH)
  {
      NinjaSetWalk();
      ModeCounter = 0;  //WALK
  }
  if (RemoteXY.button_A == HIGH)
  { 
      lau++;
      Serial.println          ("LAU "+String(lau));
//    NinjaLeftArm();
  }

  if (RemoteXY.button_A == LOW)
  { 
      if (lau>0){
        lau=0;       
        Serial.print            ("AUTO "); 
        if   (ModeCounter==WALK){Serial.print("WALK");}
        else                    {Serial.print("ROLL");}
        Serial.println          (" ON!"); 
        AUTOGO = true;
        NinjaRollForward();
        digitalWrite            (LedBuildIn,LOW);      
      }    
  }
   
  if (RemoteXY.button_B == HIGH)
  {            
      rau++;
      Serial.println          ("RAU "+String(rau));
//    NinjaRightArm();
  }

  if (RemoteXY.button_B == LOW)
  {   
      if (rau>0){
        rau=0;       
        Serial.print            ("AUTO "); 
        if   (ModeCounter==WALK){Serial.print("WALK");}
        else                    {Serial.print("ROLL");}
        Serial.println          (" OFF!"); 
        AUTOGO = false;
        digitalWrite            (LedBuildIn,HIGH);
      }   
  }

  if (ModeCounter == 0)
  {   
          //WALK NEUTRAL
          if ((RemoteXY.J_x >= -10)&&(RemoteXY.J_x <= 10)&&(RemoteXY.J_y >= -10)&&(RemoteXY.J_y <= 10))
          {
          if (!AUTOGO){NinjaWalkStop();Serial.println("NEUTRAL STAND");} 
          }
   
          if (RemoteXY.J_y > 0)
          {
             int lt= map(RemoteXY.J_x, 100, -100, 200, 700); 
             int rt= map(RemoteXY.J_x, 100, -100, 700, 200); 
             int Interval1 = 250;
             int Interval2 = 250 + rt;
             int Interval3 = 250 + rt + 250;
             int Interval4 = 250 + rt + 250 + lt;
             int Interval5 = 250 + rt + 250 + lt + 50;
             
             if(millis() > currentmillis1 + Interval5)
             {
              currentmillis1 = millis();
             }
             
             
             if(millis() - currentmillis1 <= Interval1)
             {   
                 myservoLeftLeg.attach  (ServoLeftLegPin,   544, 2400);
                 myservoRightLeg.attach (ServoRightLegPin,  544, 2400);
                 myservoRightFoot.attach(ServoRightFootPin, 544, 2400);  
                 myservoLeftFoot.attach (ServoLeftFootPin,  544, 2400); 
                 myservoLeftLeg.write   (LATR); 
                 myservoRightLeg.write  (RATR);
             }
                 
             if((millis() - currentmillis1 >= Interval1)&&(millis() - currentmillis1 <= Interval2))
             {      
                 myservoRightFoot.write (90-RFFWRS);
                 
             }

             if((millis() - currentmillis1 >= Interval2)&&(millis() - currentmillis1 <= Interval3))
             {  
                 myservoRightFoot.detach();
                 myservoLeftLeg.write(LATL); 
                 myservoRightLeg.write(RATL);
             }

             if((millis() - currentmillis1 >= Interval3)&&(millis() - currentmillis1 <= Interval4))
             {       
                 myservoLeftFoot.write(90+LFFWRS);      
             }  

             if((millis() - currentmillis1 >= Interval4)&&(millis() - currentmillis1 <= Interval5))
             {
              myservoLeftFoot.detach();  
             }
          }      

          if (RemoteXY.J_y < 0)
          {

             int lt= map(RemoteXY.J_x, 100, -100, 200, 700); 
             int rt= map(RemoteXY.J_x, 100, -100, 700, 200); 
             int Interval1 = 250;
             int Interval2 = 250 + rt;
             int Interval3 = 250 + rt + 250;
             int Interval4 = 250 + rt + 250 + lt;
             int Interval5 = 250 + rt + 250 + lt + 50;
             
             if(millis() > currentmillis1 + Interval5)
             {
              currentmillis1 = millis();
             }
             
             
             if(millis() - currentmillis1 <= Interval1)
             {   
                 myservoLeftLeg.attach(ServoLeftLegPin, 544, 2400);
                 myservoRightLeg.attach(ServoRightLegPin, 544, 2400);
                 myservoRightFoot.attach(ServoRightFootPin, 544, 2400);  
                 myservoLeftFoot.attach(ServoLeftFootPin, 544, 2400); 
                 
                 myservoLeftLeg.write(LATR); 
                 myservoRightLeg.write(RATR);
             }
                 
             if((millis() - currentmillis1 >= Interval1)&&(millis() - currentmillis1 <= Interval2))
             {      
                 myservoRightFoot.write(90+RFBWRS);
                 
             }

             if((millis() - currentmillis1 >= Interval2)&&(millis() - currentmillis1 <= Interval3))
             {  
                 myservoRightFoot.detach();
                 myservoLeftLeg.write(LATL); 
                 myservoRightLeg.write(RATL);
             }

             if((millis() - currentmillis1 >= Interval3)&&(millis() - currentmillis1 <= Interval4))
             {       
                 myservoLeftFoot.write(90-LFBWRS);      
             }  

             if((millis() - currentmillis1 >= Interval4)&&(millis() - currentmillis1 <= Interval5))
             {
              myservoLeftFoot.detach();  
             }
          }      
      }

  if (ModeCounter == 1)
  {  
      //ROLL NEUTRAL
      if ((RemoteXY.J_x >= -10)&&(RemoteXY.J_x <= 10)&&(RemoteXY.J_y >= -10)&&(RemoteXY.J_y <= 10))
      {
        if (!AUTOGO){NinjaRollStop();Serial.println("NEUTRAL ROLL");} 
      }  

      else
      //ROLL LEFT OR RIGHT
      {
        myservoLeftFoot.attach(ServoLeftFootPin, 544, 2400);  
        myservoRightFoot.attach(ServoRightFootPin, 544, 2400);  
        
        int LWS= map(RemoteXY.J_y, 100, -100, 135,    45); 
        int RWS= map(RemoteXY.J_y, 100, -100,  45,   135); 
        int LWD= map(RemoteXY.J_x, 100, -100,  45,     0); 
        int RWD= map(RemoteXY.J_x, 100, -100,   0,   -45);
        
        myservoLeftFoot.write(LWS+LWD);
        myservoRightFoot.write(RWS+RWD); 
      }
  }
  
} // end of loop

void NinjaStop()
{
  myservoLeftFoot.detach  ();
  myservoRightFoot.detach ();  
  myservoLeftLeg.detach   ();
  myservoRightLeg.detach  ();
}

void NinjaSetWalk()
{    
  myservoLeftLeg.attach   (ServoLeftLegPin, 544, 2400);
  myservoRightLeg.attach  (ServoRightLegPin, 544, 2400);                                                                          
  myservoLeftLeg.write    (LA0); 
  myservoRightLeg.write   (RA0); 
  delay(300);
  myservoLeftLeg.detach   ();
  myservoRightLeg.detach  ();
}   

void NinjaSetRoll()
{  
  myservoLeftLeg.attach   (ServoLeftLegPin, 544, 2400);
  myservoRightLeg.attach  (ServoRightLegPin, 544, 2400);                                                                          
  myservoLeftLeg.write    (LA1); 
  myservoRightLeg.write   (RA1); 
  delay(300);
  myservoLeftLeg.detach   ();
  myservoRightLeg.detach  ();
}

void NinjaWalkStop()
{
  myservoLeftFoot.write   (90);
  myservoRightFoot.write  (90);  
  myservoLeftLeg.write    (LA0); 
  myservoRightLeg.write   (RA0);  
}

void NinjaRollStop()
{
  myservoLeftFoot.write   (90);
  myservoRightFoot.write  (90);  
  myservoLeftFoot.detach  ();
  myservoRightFoot.detach ();  
}

//void NinjaWalkForward()
void NinjaStepForward()
{
  Serial.println          ("NinjaStepForward()");
  myservoLeftLeg.attach   (ServoLeftLegPin, 544, 2400);
  myservoRightLeg.attach  (ServoRightLegPin, 544, 2400);
  myservoLeftLeg.write    (LATR); 
  myservoRightLeg.write   (RATR);
  delay(300);
  myservoRightFoot.attach (ServoRightFootPin, 544, 2400);  
  myservoRightFoot.write  (90-RFFWRS);
  delay(300);
  myservoRightFoot.detach ();
  delay(100);
  myservoLeftLeg.write    (LATL); 
  myservoRightLeg.write   (RATL);
  delay(300);
  myservoLeftFoot.attach  (ServoLeftFootPin, 544, 2400); 
  myservoLeftFoot.write   (90+LFFWRS);
  delay(300);
  myservoLeftFoot.detach  ();
  delay(100);
}

//void NinjaWalkBackward()
void NinjaStepBackward()  
{
  Serial.println          ("NinjaStepBackward()");
  myservoLeftLeg.attach   (ServoLeftLegPin, 544, 2400);
  myservoRightLeg.attach  (ServoRightLegPin, 544, 2400);
  myservoLeftLeg.write    (LATR); 
  myservoRightLeg.write   (RATR);
  delay(300);
  myservoRightFoot.attach (ServoRightFootPin, 544, 2400);  
  myservoRightFoot.write  (90+RFBWRS);
  delay(300);
  myservoRightFoot.detach ();
  delay(100);
  myservoLeftLeg.write    (LATL); 
  myservoRightLeg.write   (RATL);
  delay(300);
  myservoLeftFoot.attach  (ServoLeftFootPin, 544, 2400); 
  myservoLeftFoot.write   (90-LFBWRS);
  delay(300);
  myservoLeftFoot.detach  ();
  delay(100);
}

//void NinjaWalkRight()
void NinjaStepRight()
{
  Serial.println          ("NinjaStepRight()");
  myservoLeftLeg.attach   (ServoLeftLegPin, 544, 2400);
  myservoRightLeg.attach  (ServoRightLegPin, 544, 2400);
  myservoLeftLeg.write    (LATR); 
  myservoRightLeg.write   (RATR);
  delay(300);
  myservoRightFoot.attach (ServoRightFootPin, 544, 2400);  
  myservoRightFoot.write  (90-RFFWRS);
  delay(300);
  myservoRightFoot.detach ();
  delay(100);
  myservoLeftLeg.write    (LATL); 
  myservoRightLeg.write   (RATL);
  delay(300);
  myservoLeftFoot.attach  (ServoLeftFootPin, 544, 2400); 
  myservoLeftFoot.write   (90+LFFWRS);
  delay(50);
  myservoLeftFoot.detach  ();
  delay(100);
}

//obsolete?
void NinjaLeftArm(){}
void NinjaRightArm(){}
void NinjaLeftArmDown(){}
void NinjaRightArmDown(){}
//

void Distance() 
{
  digitalWrite            (trigPin, LOW);
  delayMicroseconds       (2);
  digitalWrite            (trigPin, HIGH);
  delayMicroseconds       (10);
  digitalWrite            (trigPin, LOW);
  duration =  pulseIn     (echoPin, HIGH);
  distance =              duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

  // take the avarage distance of last 5 measurements to avoid invalid reaction
  lastdistances[ldcount]= distance;
  ldcount++;        
  if  (ldcount==5)        {ldcount=0;}
  distance    = 0;
  for (int d=0;d<5;d++){
    distance  +=          lastdistances[d];
  }
  distance    =           distance/5;
}

void NinjaRollForward()
{
  Serial.print            ("NinjaRollForward()");
  myservoLeftFoot.attach  (ServoLeftFootPin, 544, 2400);
  myservoRightFoot.attach (ServoRightFootPin, 544, 2400);
  myservoLeftFoot.write   (90+LFFRRS);
  myservoRightFoot.write  (90-RFFRRS);
  Serial.println          (String(90+LFBRRS)+" & "+String(90-RFBRRS));
}

void NinjaRollBackward()
{
  Serial.print            ("NinjaRollBackward()");
  myservoLeftFoot.attach  (ServoLeftFootPin, 544, 2400);
  myservoRightFoot.attach (ServoRightFootPin, 544, 2400);
  myservoLeftFoot.write   (90-LFBRRS);
  myservoRightFoot.write  (90+RFBRRS);
  Serial.println          (String(90-LFBRRS)+" & "+String(90+RFBRRS));
}

void NinjaRollLeft()
{
  Serial.print            ("NinjaRollLeft()");
  myservoLeftFoot.attach  (ServoLeftFootPin, 544, 2400);
  myservoRightFoot.attach (ServoRightFootPin, 544, 2400);
  myservoLeftFoot.write   (90-LFBRRS);
  myservoRightFoot.write  (90-RFFRRS);
  Serial.println          (String(90-LFBRRS)+" & "+String(90+RFBRRS));
}

void NinjaRollRight()
{
  Serial.print            ("NinjaRollRight()");
  myservoLeftFoot.attach  (ServoLeftFootPin, 544, 2400);
  myservoRightFoot.attach (ServoRightFootPin, 544, 2400);
  myservoLeftFoot.write   (90+LFFRRS);
  myservoRightFoot.write  (90+RFBRRS);
  Serial.println          (String(90+LFBRRS)+" & "+String(90+RFBRRS));
}
