#include <Wire_slave.h>
int dataArray[3] = {1,225,225};
int claw1fb = PA7; //claw1 feedback
//int claw2fb = PB5; //claw2 feedback
volatile long clawudcount = 0;
int claw1pwm=PB6,claw1dir=PA5;
volatile long clawrotcount = 0;
int claw2pwm=PB8,claw2dir=PB7; 
int flag1 =0;
int flag2 =0;
int x=0;
void setup() {
  Serial.begin(9600);
  pinMode(claw1fb,INPUT);
  pinMode(claw1pwm,PWM);
  pinMode(claw1dir,OUTPUT);
 // pinMode(claw2fb,INPUT);
  pinMode(claw2pwm,PWM);
  pinMode(claw2dir,OUTPUT);  
  attachInterrupt(digitalPinToInterrupt(claw1fb),claw1enc,FALLING);
  //attachInterrupt(digitalPinToInterrupt(claw2fb),claw2enc,FALLING); 
  pwmWrite(claw1pwm,0);
  pwmWrite(claw2pwm,0); 
      digitalWrite(claw1dir,LOW);
    
    digitalWrite(claw2dir,LOW);
}

void loop() {
 if(Serial.available())
 {
  for(int i=0;i<3;i++)
  {
    dataArray[i]=Serial.parseInt();
  }
  if(dataArray[0]==0)
{
  endupdown();
}
else if(dataArray[0]==1)
{
  rotate();
}
}
  
  

}

void claw1enc(){
  if(dataArray[0]==0)
{
  digitalRead(claw1dir)?clawudcount++:clawudcount--;
}
else if(dataArray[0]==1)
{
  digitalRead(claw1dir)?clawrotcount++:clawrotcount--;
}
}
void endupdown()
  {
  int clawudcountreqcount=dataArray[1]*1365/360;
  if(clawudcountreqcount==clawudcount)
  {
    pwmWrite(claw1pwm,0);
    pwmWrite(claw2pwm,0); 
    flag1=1;   
  }
  else if(clawudcountreqcount>clawudcount){
    digitalWrite(claw1dir,HIGH);
    pwmWrite(claw1pwm,10000);
    digitalWrite(claw2dir,LOW);
    pwmWrite(claw2pwm,10000);
   while(clawudcountreqcount>clawudcount){Serial.println(clawudcount);}
       pwmWrite(claw1pwm,0);
    pwmWrite(claw2pwm,0); 
   
  }
  else if(clawudcountreqcount<clawudcount){
    digitalWrite(claw1dir,LOW);
    pwmWrite(claw1pwm,10000);
    digitalWrite(claw2dir,HIGH);
    pwmWrite(claw2pwm,10000);
   while(clawudcountreqcount<clawudcount){    Serial.println(clawudcount);}
        pwmWrite(claw1pwm,0);
    pwmWrite(claw2pwm,0); 
    
  }
  
 }

 void rotate()
 {
  
    Serial.println("clawrotcount");
    double clawrotreqcount=dataArray[1]*1365/360;
    if(clawrotreqcount==clawrotcount)
    {
    pwmWrite(claw1pwm,0);
    pwmWrite(claw2pwm,0); 
    flag1=0; 
    }
  else if(clawrotreqcount>clawrotcount){
    digitalWrite(claw1dir,HIGH);
    pwmWrite(claw1pwm,10000);
    digitalWrite(claw2dir,HIGH);
    pwmWrite(claw2pwm,10000);
   while(clawrotreqcount>clawrotcount){    Serial.println(clawrotcount);}
       pwmWrite(claw1pwm,0);
    pwmWrite(claw2pwm,0); 
  }
  else if(clawrotreqcount<clawrotcount){
    digitalWrite(claw1dir,LOW);
    pwmWrite(claw1pwm,10000);
    digitalWrite(claw2dir,LOW);
    pwmWrite(claw2pwm,10000);
   while(clawrotreqcount<clawrotcount){    Serial.println(clawrotcount);}
    pwmWrite(claw1pwm,0);
    pwmWrite(claw2pwm,0); 
  }
 }
