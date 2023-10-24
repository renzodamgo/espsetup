#include <iostream>
#include <HardwareSerial.h>
#include <Arduino.h>
#include <stdarg.h>
#include <Stream.h>
#define GET_LOW_BYTE(A) (uint8_t)((A))
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)

#define FRAME_HEADER            0x55
#define CMD_SERVO_MOVE          0x03

HardwareSerial SerialPort(2);

const uint8_t rx2Pin = 16;
const uint8_t tx2Pin = 17;

uint16_t count=0;

struct BLKServo { 
  uint8_t  ID; 
  uint16_t Position;
};

void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time) {
//(servoID,Posicion de 0 a 1000,Tiempo en ms)
  uint8_t buf[11];
  if (servoID > 31 || !(Time > 0)) {
    return;
  }
  buf[0] = FRAME_HEADER;             
  buf[1] = FRAME_HEADER;
  buf[2] = 8;                             
  buf[3] = CMD_SERVO_MOVE;              
  buf[4] = 1;                         
  buf[5] = GET_LOW_BYTE(Time);        
  buf[6] = GET_HIGH_BYTE(Time);        
  buf[7] = servoID;             
  buf[8] = GET_LOW_BYTE(Position);        
  buf[9] = GET_HIGH_BYTE(Position);        
  SerialPort.write(buf,10);
//   for (int i = 0; i < 10; i++){
//     Serial.print("0X");
//     Serial.print(buf[i], HEX);
//     Serial.print(" ");
//     }
//   Serial.println();
 }


void moveServos(BLKServo servos[], uint8_t numServos, uint16_t Time)
{
	uint8_t buf[103]; 
	if (numServos < 1 || numServos > 32 || !(Time > 0)) {
		return; 
	}
	buf[0] = FRAME_HEADER; 
	buf[1] = FRAME_HEADER;
	buf[2] = numServos * 3 + 5; 
	buf[3] = CMD_SERVO_MOVE; 
	buf[4] = numServos;  
	buf[5] = GET_LOW_BYTE(Time); 
	buf[6] = GET_HIGH_BYTE(Time);
	uint8_t index = 7;
	for (uint8_t i = 0; i < numServos; i++) {
		buf[index++] = servos[i].ID;
		buf[index++] = GET_LOW_BYTE(servos[i].Position);
		buf[index++] = GET_HIGH_BYTE(servos[i].Position);
	}
	SerialPort.write(buf, buf[2] + 2);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  SerialPort.begin(9600, SERIAL_8N1, rx2Pin, tx2Pin);
}

void loop() {

  BLKServo servos[2];   
  
  servos[0].ID = 3;       
  servos[0].Position = 0;  

  moveServos(servos, sizeof(servos),2000);  
  delay(600);  

  servos[0].ID = 3;       
  servos[0].Position = 1000;  

  moveServos(servos, sizeof(servos), 2000);  
  delay(600);
}
