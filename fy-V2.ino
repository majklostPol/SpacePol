#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include "SH1106Wire.h"

 

Adafruit_MLX90614 mlx = Adafruit_MLX90614(0x5A);
double temp;
SH1106Wire display(0x3c, SDA, SCL); 
void setup() {
  
  display.init();

  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  Serial.begin(9600);
  mlx.begin();
  
  
}

void loop() {
  Serial.println(mlx.readObjectTempC());
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(128, 54, String(millis()));
  // write the buffer to the display
  display.display();
  
  delay(1000); 
}
