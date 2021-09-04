#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <ESP32Servo.h>

#define MOTOR_LEFT 4
#define MOTOR_RIGHT 5

Servo motor_left;
Servo motor_right;

#define FOR_LEFT 110
#define FOR_RIGHT 70

#define FALL 0x00
#define MOVE 0x01
#define WAIT 0X03

int state = WAIT;


/**** HC-12 serial port ****/
#define RXD2 16  //(RX2)
#define TXD2 4  //(TX2)

#define HC12 Serial2  //Hardware serial 2 on the ESP32


/**** BMP280 ****/
Adafruit_BMP280 bme; // I2C
float temp = 0;
float pressure = 0;


/**** AKCELEROMETR ****/
Adafruit_MPU6050 mpu;
float corr_x = 0;
float corr_y = 0; 
float corr_z = 0;



/**** FUNCTIONS ****/
bool sendData(String _data);    //send data to HC-12
void readBME(float *_temp, float *_press);    //read bme data
void getAccData(float* _x, float* _y, float* _z);   //read acceleration
void getGyroData(float* _x, float* _y, float* _z);  //read rotation
void getAccTemp(float* _temp);  //read acc. temp
void motorOn();
void motorOff();
void updateState();
bool isTimeMove();
void sendToServer();


void setup() 
{
  
  Serial.begin(115200);           // Serial port to computer
  HC12.begin(9600, SERIAL_8N1, RXD2, TXD2);      // Serial port to HC12
 
  if (!bme.begin(0x76)) {   //init bme280
  Serial.println("Could not find a valid BMP280 sensor, check wiring!");
  while (1);
  }

  if (!mpu.begin()) {   //init accelerometer
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);   //set accelerometer
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  //setup acc
  getAccData(&corr_x, &corr_y, &corr_z);

  //servo
  motor_left.attach(MOTOR_LEFT);
  motor_right.attach(MOTOR_RIGHT);
  motorOff();
  
  Serial.write("INIT DONE");}


void loop() 
{


  

  readBME(&temp, &pressure);

  Serial.print("Temperature = ");
  Serial.print(temp);
  Serial.println(" *C");
   
  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println(" Pa");


  float acc_x, acc_y, acc_z = 0;

  getAccData(&acc_x, &acc_y, &acc_z);

  Serial.print("Acceleration X: ");
  Serial.print(acc_x);
  Serial.print(", Y: ");
  Serial.print(acc_y);
  Serial.print(", Z: ");
  Serial.print(acc_z);
  Serial.println(" m/s^2");


  float g_x, g_y, g_z = 0;
  
  getGyroData(&g_x, &g_y, &g_z);

  Serial.print("Rotation X: ");
  Serial.print(g_x);
  Serial.print(", Y: ");
  Serial.print(g_y);
  Serial.print(", Z: ");
  Serial.print(g_z);
  Serial.println(" rad/s");

  float acc_temp = 0;
  
  getAccTemp(&acc_temp);

  Serial.print("Temperature: ");
  Serial.print(acc_temp);
  Serial.println(" degC");

  if(state == FALL){

    String _data = String(temp) + " " + String(pressure);
    sendData(_data);
    
  }else if(state == MOVE){

    
    
  }

  delay(2000);
  
}

bool sendData(String _data){    //send data to hc-12

  HC12.write(_data.c_str());  //Send data to HC-12

}

void readBME(float* _temp, float* _press){    //read values from bme280 and save it to variables

  *_temp = bme.readTemperature();
  *_press = bme.readPressure();
  
}

void getAccData(float* _x, float* _y, float* _z){

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);;

  *_x = a.acceleration.x - corr_x;
  *_y = a.acceleration.y - corr_y;
  *_z = a.acceleration.z - corr_z;
  
}

void getGyroData(float* _x, float* _y, float* _z){

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  *_x = g.gyro.x;
  *_y = g.gyro.y;
  *_z = g.gyro.z;
  
}

void getAccTemp(float* _temp){

  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  *_temp = t.temperature;

}

void motorOn(){
  
  motor_left.attach(MOTOR_LEFT);
  motor_right.attach(MOTOR_RIGHT);

  motor_left.write(FOR_LEFT);
  motor_right.write(FOR_RIGHT);
  
}

void motorOff(){
  
  motor_left.detach();
  motor_right.detach();
    
}

void updateState(){

  if(isTimeMove() == true){
    
      state = MOVE;
    
    }else{

      state = FALL;
      
    }
    
}
bool isTimeMove(){

  //get request -> time to move
  
}

void sendToServer(){

  //send data to database
  
}


