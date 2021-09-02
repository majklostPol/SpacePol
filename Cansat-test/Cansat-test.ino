#include <Arduino.h>


// HC-12 serial port
#define RXD2 16  //(RX2)
#define TXD2 4  //(TX2)

#define HC12 Serial2  //Hardware serial 2 on the ESP32

int x =0;
String sender = "Majkl";

bool sendData(String _data);


void setup() 
{
  
  /*pinMode(5, OUTPUT);
  digitalWrite(5, LOW);           //Normally HIGH, LOW for settings*/
  Serial.begin(115200);           // Serial port to computer
  HC12.begin(9600, SERIAL_8N1, RXD2, TXD2);      // Serial port to HC12
  Serial.write("INIT DONE");}


void loop() 
{


  while (HC12.available()) 
  {        
    // If HC-12 has data
    Serial.write(HC12.read());
    // Send the data to Serial monitor
    
  
  }
  while (Serial.available()) 
  {      
    // If we have data from Serial monitor
          // Send that data to HC-12
  }
  x+=1;
  HC12.write((sender+x+"\n").c_str());
  delay(1000);
}

bool sendData(String _data){

  HC12.write(_data.c_str());  //Send data to HC-12

}

