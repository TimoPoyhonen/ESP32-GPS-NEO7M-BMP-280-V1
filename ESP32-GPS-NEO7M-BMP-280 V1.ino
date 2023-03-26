#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <LiquidCrystal_I2C.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define BMP_SDA 21
#define BMP_SCL 22
#define GPS_TX 17 // ESP32 TX pin connected to GPS module RX pin
#define GPS_RX 16 // ESP32 RX pin connected to GPS module TX pin
#define LCD_SDA 23
#define LCD_SCL 19

Adafruit_BMP280 bmp;
LiquidCrystal_I2C lcd(0x27, 16, 4);
TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

byte Sat[8] = {
	0b00000,
	0b00000,
	0b00000,
	0b00000,
	0b01110,
	0b00100,
	0b00000,
	0b00000
};

byte Cel[8] =
{
0b01000,
0b10100,
0b01000,
0b00000,
0b00000,
0b00000,
0b00000,
0b00000
};

byte Spe[8] =
{
0b11000,
0b01100,
0b00110,
0b00011,
0b00110,
0b01100,
0b11000,
0b00000
};
byte Pha[8] =
{
0b00100,
0b00100,
0b00100,
0b00100,
0b10101,
0b01110,
0b00100,
0b00000
};
void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  Wire.begin(BMP_SDA, BMP_SCL);
  bmp.begin(0x76);
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, Sat);
  lcd.createChar(1, Cel);
  lcd.createChar(2, Spe);
  lcd.createChar(3, Pha);
}

void loop() {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      float latitude = gps.location.lat();
      float longitude = gps.location.lng();
      int satellites = 0;

      if (gps.location.isValid()) {
        satellites = gps.satellites.value();
      }
      float speed = gps.speed.kmph();
      float temperature = bmp.readTemperature();
      float pressure = bmp.readPressure() / 100.0F;
      float altitude = bmp.readAltitude(1013.25);
      
      lcd.clear();

      lcd.setCursor(0, 0);
      lcd.write(0);
      lcd.print(satellites);
  
      lcd.setCursor(4, 0);
      lcd.write(2);
      lcd.print(speed);

      lcd.setCursor(11, 0);
      lcd.write(3);
      lcd.print(pressure, 0);
      

      lcd.setCursor(17, 0);
      lcd.print(temperature, 0);
      lcd.write(1);

      lcd.setCursor(0, 1);
      lcd.print("Lat:");
      lcd.print(latitude, 6);

      lcd.setCursor(0, 2);
      lcd.print("Lng:");
      lcd.print(longitude, 6);

      lcd.setCursor(0, 3);
      lcd.print("Alt:");
      lcd.print(altitude);
      lcd.print("m");

      delay(500);


      
     

      Serial.print("Satellites: ");
      Serial.print(satellites);
      Serial.print(", Lat: ");
      Serial.print(latitude, 6);
      Serial.print(", Lng: ");
      Serial.print(longitude, 6);

      Serial.print(", Temp: ");
      Serial.print(temperature);
      Serial.print(" C");
      Serial.print(", Press: ");
      Serial.print(pressure);
      Serial.print(" hPa");

      Serial.println();
    }
  }
}
