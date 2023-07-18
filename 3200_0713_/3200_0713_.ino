//3200motor+lidar
#include <SoftwareSerial.h>   // 引用程式庫
#include <MsTimer2.h>
//PID
float kp = 1;
float ki = 0.003;
float kd = 0.0;
//腳位
const byte interruptPinA = 2;
const byte interruptPinB = 3;
volatile unsigned long EncoderCount = 0;
const byte PWMPin = 6;
const byte DirPin1 = 7;
//時間
unsigned long t;
unsigned long t_prev = 0;
volatile unsigned long count = 0;
unsigned long count_prev = 0;
//PID coef
float Theta=0;
float RPM=0;
float RPM_d=0;
float Theta_prev = 0;
int dt=0;
float RPM_max = 10;
float e=0;
float e_prev = 0;
float inte=0;
float inte_prev = 0;
//low pass coef
float v1Filt = 0;
float v1Prev = 0;
//voltage coef
float Vmax = 100;
float Vmin = -100;
float V = 0.1;
//Lidar
volatile int dis=0;
unsigned long pulseWidth;
//constant
#define pi 3.1416
//**********FUNCTIONS******************
//     Void ISR_EncoderA
//     Void ISR_EncoderB
//     Void Motor Driver Write
//     Timer Interrupt
//***********************************
void ISR_EncoderA() {
  bool B = digitalRead(interruptPinB);
  bool A = digitalRead(interruptPinA);
  bool atemp,btemp;
  bool check=1;
  
  if (B == LOW) {
    if (A == HIGH) {
      for(int i=0;i++;i<2000){ 
        atemp=digitalRead(interruptPinA);
        btemp=digitalRead(interruptPinB);
        if(btemp!=B || atemp!=A){check=0;}  
      }
      if(check==1){
      EncoderCount--;}
    }
    else {
      for(int i=0;i++;i<2000){ 
        atemp=digitalRead(interruptPinA);
        btemp=digitalRead(interruptPinB);
        if(btemp!=B || atemp!=A){check=0;}  
      }
      if(check==1){
      EncoderCount++;}
    }
  }

  else {
    if (A == HIGH) {
      for(int i=0;i++;i<2000){ 
        atemp=digitalRead(interruptPinA);
        btemp=digitalRead(interruptPinB);
        if(btemp!=B || atemp!=A){check=0;}  
      }
      if(check==1){
      EncoderCount++;}
    }
    else {
      for(int i=0;i++;i<2000){ 
        atemp=digitalRead(interruptPinA);
        btemp=digitalRead(interruptPinB);
        if(btemp!=B || atemp!=A){check=0;}  
      }
      if(check==1){
      EncoderCount--;}
    }
  }
}
void ISR_EncoderB() {
  bool B = digitalRead(interruptPinA);
  bool A = digitalRead(interruptPinB);
  bool atemp,btemp;
  bool check=1;
  if (A == LOW) {
    if (B == HIGH) {
      for(int i=0;i++;i<2000){ 
        btemp=digitalRead(interruptPinA);
        atemp=digitalRead(interruptPinB);
        if(btemp!=B || atemp!=A){check=0;}  
      }
      if(check==1){
      EncoderCount++;}
    }
    else {
      for(int i=0;i++;i<2000){ 
        btemp=digitalRead(interruptPinA);
        atemp=digitalRead(interruptPinB);
        if(btemp!=B || atemp!=A){check=0;}  
      }
      if(check==1){
      EncoderCount--;}
      }
    }
  
  else {
    if (B == HIGH) {
      for(int i=0;i++;i<2000){ 
        btemp=digitalRead(interruptPinA);
        atemp=digitalRead(interruptPinB);
        if(btemp!=B || atemp!=A){check=0;}  
      }
      if(check==1){
      EncoderCount--;}
    }
    else {
      for(int i=0;i++;i<2000){ 
        btemp=digitalRead(interruptPinA);
        atemp=digitalRead(interruptPinB);
        if(btemp!=B || atemp!=A){check=0;}  
      }
      if(check==1){
      EncoderCount++;}
    }
  }
}
//***Motor Driver Functions*****
void WriteDriverVoltage(float V, float Vmax) {
  int PWMval = int(255 * abs(V) / Vmax);
  if (PWMval > 255) {   
    PWMval = 255;
  }
  if (V < 0) {
    digitalWrite(DirPin1, HIGH);
  }
  else if (V > 0) {
    digitalWrite(DirPin1, LOW);
  }
  else {
    analogWrite(PWMPin, 0);
  }
  analogWrite(PWMPin, PWMval);
}

void setup() {
  Serial.begin(9600);
  pinMode(interruptPinA, INPUT_PULLUP);
  pinMode(interruptPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinA), ISR_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinB), ISR_EncoderB, CHANGE);
  pinMode(PWMPin, OUTPUT);
  pinMode(DirPin1, OUTPUT);
  analogWrite (PWMPin,0);
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A =12499; //Prescaler = 64  (12499)
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11 | 1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  sei();

  pinMode(12, OUTPUT); // Set pin 2 as trigger pin
  digitalWrite(12, LOW); // Set trigger LOW for continuous read
  pinMode(11, INPUT); // Set pin 3 as monitor pin

  MsTimer2::set(300, Lidar_scan); 
  MsTimer2::start(); 
}

void Lidar_scan(){
  pulseWidth = pulseIn(11, HIGH); // Count how long the pulse is high in microseconds
  if(pulseWidth != 0){
    dis= pulseWidth / 20; // 10usec = 1 cm of distance
  }
}

int k; int j; int state;
void loop() {
  t = millis();   
  if(Serial.available()) {
    state=Serial.parseInt();
    if(state==1 || state== -2){ //直走 + 左轉右動
      k=1;
      analogWrite(PWMPin, 0);
      RPM_d=0;
      delay(3000);
    }
    else if(state==-1 || state==2){ //停
      k=-1;
      analogWrite(PWMPin, 0);
      RPM_d=0;    
      }
      
  }
    
  if(dis<30){ //避障
      analogWrite(PWMPin, 0);
      RPM_d=0;
      Serial.println(dis);
      Serial.println("stoppppppppppp");
  }
  else{
    Serial.print("dis:");
    Serial.println(dis);
    if(k==1){ //直走 + 左轉右動
      Theta = EncoderCount / 3200.0;
      dt = (t - t_prev);
      RPM = (Theta - Theta_prev) / (dt / 1000.0) * 60 ;
      if(RPM_d>=52){
        RPM_d=52;
      }
      else{
        RPM_d+=0.5;
      }
      //PID
      e = RPM_d - v1Filt;
      inte = inte_prev + (dt * (e + e_prev) / 2);
      V = kp * e + ki * inte + (kd * (e - e_prev) / dt) ;
      if (V > Vmax) {
        V = Vmax;
        inte = inte_prev;
      }
      if (V < Vmin) {
        V = Vmin;
        inte = inte_prev;
      }
      //give the correct V
      WriteDriverVoltage(V, Vmax);
      // Low-pass filter (25 Hz cutoff)
      v1Filt = 0.854*v1Filt + 0.0728*RPM + 0.0728*v1Prev;
      v1Prev = RPM;
      //output
      Serial.print(RPM_d); Serial.print(" \t");
      Serial.print(v1Filt); Serial.print(" \t ");
      //Serial.print(EncoderCount); Serial.print(" \t ");
      //Serial.print(V); Serial.print("\t  ");
      Serial.print(e); Serial.println("  ");
      
      Theta_prev = Theta;
      count_prev = count;
      t_prev = t;
      inte_prev = inte;
      e_prev = e;
    }
    else if(k==-1){ //stop + 右轉右不動
      analogWrite(PWMPin, 0);
      RPM_d=0;
    }
  } 
}

ISR(TIMER1_COMPA_vect) {
    count++; 
}
