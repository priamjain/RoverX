
int dataArray[6] = {2,100,2,100,2,100}; //get 6 int array
int updownfb = PA0; //updown feedback
int forbackfb = PA3; //forback feedback
int sweepfb = PA8 ; //updown feedback
#include <Wire_slave.h>
volatile long updowncount = 0;
int updownpwm=PA1,updowndir=PA2; 
volatile long forbackcount=0;
int forbackpwm=PA4 ,forbackdir=PA5; 
volatile long sweepcount=0;
int sweeppwm=PA9 ,sweepdir=PA10; 
void setup(){
  Serial.begin(9600);         // start serial for output
  pinMode(updownfb,INPUT_PULLUP);
  pinMode(updownpwm,PWM);
  pinMode(updowndir,OUTPUT);
  pinMode(forbackfb,INPUT_PULLUP);
  pinMode(forbackpwm,PWM);
  pinMode(forbackdir,OUTPUT);
  pinMode(sweepfb,INPUT_PULLUP);
  pinMode(sweeppwm,PWM);
  pinMode(sweepdir,OUTPUT);  
  attachInterrupt(digitalPinToInterrupt(updownfb),updownenc,FALLING);
  attachInterrupt(digitalPinToInterrupt(forbackfb),forbackenc,FALLING);
  attachInterrupt(digitalPinToInterrupt(sweepfb),sweepenc,FALLING);
  Wire.begin(4);               
  Wire.onReceive(receiveEvent); 
}

void loop(){
  updown();
}

void updown()
  {


  double updownreqcount=dataArray[0]*8652;
  if(updownreqcount>updowncount){
    digitalWrite(updowndir,HIGH);
    pwmWrite(updownpwm,dataArray[1]*257);
   // while(updownreqcount>updowncount){    Serial.println(updowncount);}
    pwmWrite(updownpwm, 0 );
  }
  else if(updownreqcount<updowncount){
    digitalWrite(updowndir,LOW);
    pwmWrite(updownpwm,dataArray[1]*257);
   // while(updownreqcount<updowncount){    Serial.println(updowncount);}
    pwmWrite(updownpwm, 0 );
    Serial.print(500);
  }
  else{
    pwmWrite(updownpwm, 0 );
  }
  
 }

void updownenc(){
  digitalRead(updowndir)?updowncount++:updowncount--;
}


void receiveEvent(int howMany){
for(int i=0; i<howMany; i++)
  {
    dataArray[i] = Wire.read();
  }         
}
////////////////////////////
void forback()
  {
    
  double forbackreqcount=dataArray[2]*8652;
  if(forbackreqcount>forbackcount){
    digitalWrite(forbackdir,HIGH);
    pwmWrite(forbackpwm,dataArray[3]*257);
   // while(forbackreqcount>forbackcount){}
    pwmWrite(forbackpwm, 0 );
  }
  else if(forbackreqcount<forbackcount){
    digitalWrite(forbackdir,LOW);
    pwmWrite(forbackpwm,dataArray[1]*257);
   // while(forbackreqcount<forbackcount){}
    pwmWrite(forbackpwm, 0 );
  }
  else{
    pwmWrite(forbackpwm, 0 );
  }
 }

void forbackenc(){
  digitalRead(forbackdir)?forbackcount--:forbackcount++;
}
////////////////////////////
void sweep()
  {
    
  double sweepreqcount=dataArray[3]/360*8652;
  if(sweepreqcount>sweepcount){
    digitalWrite(sweepdir,HIGH);
    pwmWrite(sweeppwm,dataArray[4]*257);
   // while(sweepreqcount>sweepcount){}
    pwmWrite(sweeppwm, 0 );
  }
  else if(sweepreqcount<sweepcount){
    digitalWrite(sweepdir,LOW);
    pwmWrite(sweeppwm,dataArray[3]*257);
  //  while(sweepreqcount<sweepcount){}
    pwmWrite(sweeppwm, 0 );
  }
  else{
    pwmWrite(sweeppwm, 0 );
  }
 }

void sweepenc(){
  digitalRead(sweepdir)?sweepcount--:sweepcount++;
}
