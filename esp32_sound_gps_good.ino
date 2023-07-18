#include "Arduino.h"
#include "BluetoothSerial.h" 
#include<SoftwareSerial.h>
#include <stdlib.h>
BluetoothSerial BT;

//UART
HardwareSerial gps(1);     // 26 27
HardwareSerial Motor(2);   //16黃=>arduino TX 17白=>arduino RX

//map:home, park, library, school, toilet
const float point_x[5]={0.6, 1, 4, 4, 2}; 
const float point_y[5]={1.6, 3, 3, 0, 1.5};
int head=1;
float now_x=0;float now_y=0;
//gps
long hedgehog_x, hedgehog_y;// coordinates of hedgehog (X,Y), mm
long hedgehog_z;// height of hedgehog, mm
int hedgehog_pos_updated;// flag of new data from hedgehog received
bool high_resolution_mode;
#define HEDGEHOG_BUF_SIZE 40 
#define HEDGEHOG_CM_DATA_SIZE 0x10
#define HEDGEHOG_MM_DATA_SIZE 0x16
byte hedgehog_serial_buf[HEDGEHOG_BUF_SIZE];
byte hedgehog_serial_buf_ofs;
#define POSITION_DATAGRAM_ID 0x0001
#define POSITION_DATAGRAM_HIGHRES_ID 0x0011
unsigned int hedgehog_data_id;
typedef union {byte b[2]; unsigned int w;int wi;} uni_8x2_16;
typedef union {byte b[4];float f;unsigned long v32;long vi32;} uni_8x4_32;

void setup_hedgehog(){
  gps.begin(115200, SERIAL_8N1, 26, 27); // hedgehog transmits data on 9600bps  
  hedgehog_serial_buf_ofs= 0;
  hedgehog_pos_updated= 0;
}

void loop_hedgehog(){
  int incoming_byte;
  int total_received_in_loop;
  int packet_received;
  bool good_byte;
  byte packet_size;
  uni_8x2_16 un16;
  uni_8x4_32 un32;

  total_received_in_loop= 0;
  packet_received= 0;
  
  while(gps.available() > 0){
      if (hedgehog_serial_buf_ofs>=HEDGEHOG_BUF_SIZE) {
        hedgehog_serial_buf_ofs= 0;// restart bufer fill
        break;// buffer overflow
      }
      total_received_in_loop++;
      if (total_received_in_loop>100) break;// too much data without required header
      incoming_byte= gps.read();
      good_byte= false;
      switch(hedgehog_serial_buf_ofs){
        case 0:{
          good_byte= (incoming_byte = 0xff);
          break;
        }
        case 1:{
          good_byte= (incoming_byte = 0x47);
          break;
        }
        case 2:{
          good_byte= true;
          break;
        }
        case 3:{
          hedgehog_data_id= (((unsigned int) incoming_byte)<<8) + hedgehog_serial_buf[2];
          good_byte=   (hedgehog_data_id == POSITION_DATAGRAM_ID) ||
                       (hedgehog_data_id == POSITION_DATAGRAM_HIGHRES_ID);
          break;
        }
        case 4:{
          switch(hedgehog_data_id){
            case POSITION_DATAGRAM_ID:{
              good_byte= (incoming_byte == HEDGEHOG_CM_DATA_SIZE);
              break;
            }
            case POSITION_DATAGRAM_HIGHRES_ID:{
              good_byte= (incoming_byte == HEDGEHOG_MM_DATA_SIZE);
              break;
            }
          }
          break;
        }
        default:{
          good_byte= true;
          break;
        }
      }
      
      if (!good_byte){
          hedgehog_serial_buf_ofs= 0;// restart bufer fill         
          continue;
        }     
      hedgehog_serial_buf[hedgehog_serial_buf_ofs++]= incoming_byte; 
      if (hedgehog_serial_buf_ofs>5){
          packet_size=  7 + hedgehog_serial_buf[4];
          if (hedgehog_serial_buf_ofs == packet_size){// received packet with required header
              packet_received= 1;
              hedgehog_serial_buf_ofs= 0;// restart bufer fill
              break; 
            }
        }
    }

  if (packet_received)  {
      hedgehog_set_crc16(&hedgehog_serial_buf[0], packet_size);// calculate CRC checksum of packet
      if ((hedgehog_serial_buf[packet_size] == 0)&&(hedgehog_serial_buf[packet_size+1] == 0)){// checksum success
          switch(hedgehog_data_id){
            case POSITION_DATAGRAM_ID:{
              // coordinates of hedgehog (X,Y), cm ==> mm
              un16.b[0]= hedgehog_serial_buf[9];
              un16.b[1]= hedgehog_serial_buf[10];
              hedgehog_x= 10*long(un16.wi);
              un16.b[0]= hedgehog_serial_buf[11];
              un16.b[1]= hedgehog_serial_buf[12];
              hedgehog_y= 10*long(un16.wi);              
              // height of hedgehog, cm==>mm (FW V3.97+)
              un16.b[0]= hedgehog_serial_buf[13];
              un16.b[1]= hedgehog_serial_buf[14];
              hedgehog_z= 10*long(un16.wi);
              
              hedgehog_pos_updated= 1;// flag of new data from hedgehog received
              high_resolution_mode= false;
              break;
            }

            case POSITION_DATAGRAM_HIGHRES_ID:{
              // coordinates of hedgehog (X,Y), mm
              un32.b[0]= hedgehog_serial_buf[9];
              un32.b[1]= hedgehog_serial_buf[10];
              un32.b[2]= hedgehog_serial_buf[11];
              un32.b[3]= hedgehog_serial_buf[12];
              hedgehog_x= un32.vi32;

              un32.b[0]= hedgehog_serial_buf[13];
              un32.b[1]= hedgehog_serial_buf[14];
              un32.b[2]= hedgehog_serial_buf[15];
              un32.b[3]= hedgehog_serial_buf[16];
              hedgehog_y= un32.vi32;
              
              // height of hedgehog, mm 
              un32.b[0]= hedgehog_serial_buf[17];
              un32.b[1]= hedgehog_serial_buf[18];
              un32.b[2]= hedgehog_serial_buf[19];
              un32.b[3]= hedgehog_serial_buf[20];
              hedgehog_z= un32.vi32;
              
              hedgehog_pos_updated= 1;// flag of new data from hedgehog received
              high_resolution_mode= true;
              break;
            }
          }
        } 
    }
}
// Calculate CRC-16 of hedgehog packet
void hedgehog_set_crc16(byte *buf, byte size)
{uni_8x2_16 sum;
 byte shift_cnt;
 byte byte_cnt;

  sum.w=0xffffU;
  for(byte_cnt=size; byte_cnt>0; byte_cnt--){
   sum.w=(unsigned int) ((sum.w/256U)*256U + ((sum.w%256U)^(buf[size-byte_cnt])));
     for(shift_cnt=0; shift_cnt<8; shift_cnt++){
         if((sum.w&0x1)==1) sum.w=(unsigned int)((sum.w>>1)^0xa001U);
                       else sum.w>>=1;
       }
   }

  buf[size]=sum.b[0];
  buf[size+1]=sum.b[1];// little endian
}// hedgehog_set_crc16

void setup() {
  Serial.begin(38400);
  setup_hedgehog();//    Marvelmind hedgehog support initialize
  Motor.begin(9600);
  BT.begin("zxc");
}

float gps_target(float Target ,float now){
  return  abs(now-Target);
}

void head_direction_x(float target_x, float now_x){
  if((target_x-now_x)>0){//right 90
    if(head==1){
      Motor.write("2");
      delay(12300); 
      Motor.write("1");
      delay(4000);
      head=2;
    }
    else if(head==2){//X
      Motor.write("1");
      delay(4000);
      head=2;
    }
    else if(head==3){//left 90
      Motor.write("-2");
      delay(11700);
      Motor.write("1");
      delay(4000);
      head=2;
    }
    else if(head==4){//right 180
      Motor.write("2");
      delay(23000); 
      Motor.write("1");
      delay(4000);
      head=2;
    }
  }
  else if((target_x-now_x)<0){//left 90
    if(head==1){
      Motor.write("-2");
      delay(11700);
      Motor.write("1"); 
      delay(4000);
      head=4; 
    }
    else if(head==2){//right 180
      Motor.write("2");
      delay(23000);
      Motor.write("1"); 
      delay(4000);
      head=4;
    }
    else if(head==3){//right 90
      Motor.write("2");
      delay(12300); 
      Motor.write("1"); 
      delay(4000);
      head=4;
    }
    else if(head==4){//X      
      Motor.write("1"); 
      delay(4000);
      head=4;
    }
  }
}

void head_direction_y(float target_y, float now_y){
  if((target_y-now_y)>0){
    if(head==1){//X
      Motor.write("1"); 
      delay(4000);
      head=1;
    }
    else if(head==2){//left 90
      Motor.write("-2"); 
      delay(11700);    
      Motor.write("1"); 
      delay(4000);
      head=1;
    }
    else if(head==3){//right 180
      Motor.write("2"); 
      delay(23000);
      Motor.write("1"); 
      delay(4000);
      head=1;
    }
    else if(head==4){//right 90
      Motor.write("2");
      delay(12300); 
      Motor.write("1"); 
      delay(4000);
      head=1;
    }
  }
  else if((target_y-now_y)<0){
    if(head==1){//right 180
      Motor.write("2"); 
      delay(23000);
      Motor.write("1"); 
      delay(4000);
      head=3; 
    }
    else if(head==2){//right 90
      Motor.write("2");
      delay(12300); 
      Motor.write("1"); 
      delay(4000);
      head=3;
    }
    else if(head==3){//X 
      Motor.write("1"); 
      delay(4000);
      head=3;
    }
    else if(head==4){//left 90
      Motor.write("-2"); 
      delay(11700);  
      Motor.write("1"); 
      delay(4000);
      head=1;
    }
  }
}
float dataX=0;
float dataY=0;
int check_y=0; int spot=-1; int stop_x=1; int p=0;
void loop() {  
  loop_hedgehog();
  dataX=hedgehog_x*0.001;
  dataY=hedgehog_y*0.001;
  switch(spot){
    case 0:
      if(stop_x==1 && (gps_target(point_x[spot] ,dataX)<0.2)){
         Motor.write("-1"); 
         delay(1500);
         check_y=1;
         stop_x=0;
      }
      if(check_y==1){
         head_direction_y(point_y[spot], dataY);
//         Motor.write("2");
//         delay(12200);     
//         Motor.write("1"); 
//         delay(4000); 
         p=1;
         check_y=0;
      }
      if(p==1){
        if(gps_target(point_y[spot] ,dataY)<0.2){
           Motor.write("-1"); 
           delay(1500);
           spot=-1;
           p=0;           
        }  
      }
      break; 
  }
  
  if (BT.available()) {
    String value = BT.readString();
      if (value == "stop"){
        Motor.write("-1");  
        delay(100);
      }
      if (value == "home") {
        spot=0;
        stop_x=1;
        head_direction_x(point_x[0], dataX);
//        Motor.write("-2"); 
//        delay(11700); 
//        Motor.write("1"); 
//        delay(4000);          
      }
      /*
      if (value == "park") {
        Wire.requestFrom(ARDUINO_I2C_ADDRESS, 8);
        if (Wire.available() >= 😎 {
          Wire.readBytes((uint8_t*)&data1, sizeof(data1));  // 讀第一個浮點數
          Wire.readBytes((uint8_t*)&data2, sizeof(data2));  // 讀第二個浮點數
        } 
        now_x=data1;
        now_y=data2;
        head_direction_x(point_x[1], now_x);
        while(1){
          Wire.requestFrom(ARDUINO_I2C_ADDRESS, 8);
          if (Wire.available() >= 😎 {
            Wire.readBytes((uint8_t*)&data1, sizeof(data1));  // 讀第一個浮點數
            Wire.readBytes((uint8_t*)&data2, sizeof(data2));  // 讀第二個浮點數
          }
          if(gps_target(point_x[1] ,data2)<0.2){
           abc1.write("-1");
           abc2.write("-1");  
           delay(1500);
           break;
          }
        }
      head_direction_y(point_y[1], now_y);
        while(1){
          Wire.requestFrom(ARDUINO_I2C_ADDRESS, 8);
          if (Wire.available() >= 😎 {
            Wire.readBytes((uint8_t*)&data1, sizeof(data1));  // 讀第一個浮點數
            Wire.readBytes((uint8_t*)&data2, sizeof(data2));  // 讀第二個浮點數
          }
          if(gps_target(point_x[1] ,point_y[1])<0.2){
           abc1.write("-1");
           abc2.write("-1"); 
           delay(1500);
           break;
          }
        }          
      }
      if (value == "library") {
        Wire.requestFrom(ARDUINO_I2C_ADDRESS, 8);
        if (Wire.available() >= 😎 {
          Wire.readBytes((uint8_t*)&data1, sizeof(data1));  // 讀第一個浮點數
          Wire.readBytes((uint8_t*)&data2, sizeof(data2));  // 讀第二個浮點數
        } 
        now_x=data1;
        now_y=data2;
        head_direction_x(point_x[2], now_x);
        while(1){
          Wire.requestFrom(ARDUINO_I2C_ADDRESS, 8);
          if (Wire.available() >= 😎 {
            Wire.readBytes((uint8_t*)&data1, sizeof(data1));  // 讀第一個浮點數
            Wire.readBytes((uint8_t*)&data2, sizeof(data2));  // 讀第二個浮點數
          }
          if(gps_target(point_x[2] ,data2)<0.2){
           abc1.write("-1");
           abc2.write("-1"); 
           delay(1500);
           break;
          }
        }
      head_direction_y(point_y[2], now_y);
        while(1){
          Wire.requestFrom(ARDUINO_I2C_ADDRESS, 8);
          if (Wire.available() >= 😎 {
            Wire.readBytes((uint8_t*)&data1, sizeof(data1));  // 讀第一個浮點數
            Wire.readBytes((uint8_t*)&data2, sizeof(data2));  // 讀第二個浮點數
          }
          if(gps_target(point_x[2] ,point_y[2])<0.2){
           abc1.write("-1");
           abc2.write("-1");
           delay(1500);
           break;
          }
        }          
      }
      if (value == "school") {
        Wire.requestFrom(ARDUINO_I2C_ADDRESS, 8);
        if (Wire.available() >= 😎 {
          Wire.readBytes((uint8_t*)&data1, sizeof(data1));  // 讀第一個浮點數
          Wire.readBytes((uint8_t*)&data2, sizeof(data2));  // 讀第二個浮點數
        } 
        now_x=data1;
        now_y=data2;
        head_direction_x(point_x[3], now_x);
        while(1){
          Wire.requestFrom(ARDUINO_I2C_ADDRESS, 8);
          if (Wire.available() >= 😎 {
            Wire.readBytes((uint8_t*)&data1, sizeof(data1));  // 讀第一個浮點數
            Wire.readBytes((uint8_t*)&data2, sizeof(data2));  // 讀第二個浮點數
          }
          if(gps_target(point_x[3] ,data2)<0.2){
           abc1.write("-1");
           abc2.write("-1"); 
           delay(1500);
           break;
          }
        }
      head_direction_y(point_y[3], now_y);
        while(1){
          Wire.requestFrom(ARDUINO_I2C_ADDRESS, 8);
          if (Wire.available() >= 😎 {
            Wire.readBytes((uint8_t*)&data1, sizeof(data1));  // 讀第一個浮點數
            Wire.readBytes((uint8_t*)&data2, sizeof(data2));  // 讀第二個浮點數
          }
          if(gps_target(point_x[3] ,point_y[3])<0.2){
           abc1.write("-1");
           abc2.write("-1");
           delay(1500);
           break;
          }
        }          
      }
      if (value == "toilet") {
        Wire.requestFrom(ARDUINO_I2C_ADDRESS, 8);
        if (Wire.available() >= 😎 {
          Wire.readBytes((uint8_t*)&data1, sizeof(data1));  // 讀第一個浮點數
          Wire.readBytes((uint8_t*)&data2, sizeof(data2));  // 讀第二個浮點數
        } 
        now_x=data1;
        now_y=data2;
        head_direction_x(point_x[4], now_x);
        while(1){
          Wire.requestFrom(ARDUINO_I2C_ADDRESS, 8);
          if (Wire.available() >= 😎 {
            Wire.readBytes((uint8_t*)&data1, sizeof(data1));  // 讀第一個浮點數
            Wire.readBytes((uint8_t*)&data2, sizeof(data2));  // 讀第二個浮點數
          }
          if(gps_target(point_x[4] ,data2)<0.2){
           abc1.write("-1");
           abc2.write("-1");  
           delay(1500);
           break;
          }
        }
      head_direction_y(point_y[4], now_y);
        while(1){
          Wire.requestFrom(ARDUINO_I2C_ADDRESS, 8);
          if (Wire.available() >= 😎 {
            Wire.readBytes((uint8_t*)&data1, sizeof(data1));  // 讀第一個浮點數
            Wire.readBytes((uint8_t*)&data2, sizeof(data2));  // 讀第二個浮點數
          }
          if(gps_target(point_x[4] ,point_y[4])<0.2){
           abc1.write("-1");
           abc2.write("-1");  
           delay(1500);
           break;
          }
        }          
      }
    } 
//  }*/
  }
}
