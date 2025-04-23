#include <MCUFRIEND_kbv.h>
#define BLACK 0x0000
#define RED 0xF800
#define GREEN 0x00FF00
#define WHITE 0xFFFF

int ppmPrcnt;

MCUFRIEND_kbv tft;

void setup() {
  uint16_t ID = tft.readID();
  Serial.begin(115200);
  Serial.println(ID);
  tft.begin(ID);
  tft.fillScreen(BLACK);
  tft.setTextSize(2);
  tft.setTextColor(RED);
  tft.setCursor(10, 150);
  tft.println("Sensorler Hazirlaniyor");
  tft.println(" 15 Saniye Bekleyin");
  delay(14000);
  tft.fillScreen(BLACK);
}

void loop() {
  delay(1000);
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    data.trim();

    int commaIndex1 = data.indexOf(',');
    int commaIndex2 = data.indexOf(',', commaIndex1 + 1);
    String incTemp = data.substring(0, commaIndex1);
    String ppmStr = data.substring(commaIndex1 + 1, commaIndex2);
    // Convert ppmStr to an integer and then divide by 10,000
    int ppmValue = ppmStr.toInt();
    ppmPrcnt = ppmValue / 10000;
    String tempStr = data.substring(commaIndex2 + 1, commaIndex2 + 6);

    tft.setTextColor(BLACK);
    tft.fillRect(10, 150, 300, 30, BLACK);
    tft.fillRect(10, 200, 300, 30, BLACK);
    tft.fillRect(10, 250, 300, 30, BLACK);

    tft.setCursor(10, 100);
    tft.setTextColor(WHITE);
    tft.println("INCIBATOR BOX");


    tft.setCursor(10, 150);
    tft.setTextColor(GREEN);
    tft.println("TEMP: " + incTemp + " 'C");


    tft.setCursor(10, 175);
    tft.setTextColor(GREEN);
    tft.println("---------");


    tft.setCursor(10, 200);
    tft.setTextColor(WHITE);
    tft.println("SOLUTION BOX");



    tft.setCursor(10, 250);
    tft.setTextColor(GREEN);
    tft.println("CO2: " + ppmStr + " Ppm");


    tft.setCursor(10, 350);
    tft.setTextColor(GREEN);
    tft.println("TEMP: " + tempStr + " 'C");
  }
}
