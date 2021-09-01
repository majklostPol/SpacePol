#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <U8g2lib.h>

#define SDA_2 32
#define SCL_2 33

U8G2_SH1106_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 33, /* data=*/ 32, /* reset=*/ U8X8_PIN_NONE);
Adafruit_MLX90614 mlx = Adafruit_MLX90614(0x5A);

double temp;
void setup() {
  
  u8g2.begin();
  Serial.begin(9600);
  mlx.begin();
  Serial.println(mlx.readObjectTempC());
  
}

void loop() {
  u8g2.clearBuffer();      
  temp = mlx.readObjectTempC();
  Serial.println(temp);
      // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  u8g2.drawStr(128/2,64/2, String(temp).c_str());  // write something to the internal memory
  u8g2.sendBuffer();
  
  delay(1000); 
}
