/*
  An example showing rainbow colours on a 1.8" TFT LCD screen
  and to show a basic example of font use.

  Make sure all the display driver and pin connections are correct by
  editing the User_Setup.h file in the TFT_eSPI library folder.

  Note that yield() or delay(0) must be called in long duration for/while
  loops to stop the ESP8266 watchdog triggering.

  #########################################################################
  ###### DON'T FORGET TO UPDATE THE User_Setup.h FILE IN THE LIBRARY ######
  #########################################################################
*/

#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <SPI.h>
#include "pin_config.h"
#include "NotoSansBold36.h"
#include "NotoSansBold15.h"
#include "Unicode_Test_72.h"
#include <XBee.h>


#define BORDER 10
#define LCD_WIDTH  170
#define LCD_HEIGHT 320
#define SPRITE_WIDTH  (LCD_WIDTH - 2 * BORDER)
#define SPRITE_HEIGHT 180

#define AA_FONT_LARGE NotoSansBold36
#define AA_FONT_SMALL NotoSansBold15
#define AA_FONT_HUGE Unicode_Test_72

const uint8_t rx1_pin = 17;
const uint8_t tx1_pin = 18;

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);
XBee xbee = XBee();

uint8_t rssi = 0;
uint32_t noRSSIcount = 0;
uint32_t loopCount = 0;

void setup(void)
{
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, rx1_pin, tx1_pin);
    xbee.setSerial(Serial1);

    pinMode(PIN_POWER_ON, OUTPUT);
    digitalWrite(PIN_POWER_ON, HIGH);

    tft.init();
    tft.setRotation(0);
    tft.fillScreen(TFT_BLACK);
    for (int i = 0; i < LCD_HEIGHT; i+=2) {
        int width = random(LCD_WIDTH/6, LCD_WIDTH/2);
        tft.drawFastHLine(width, i, LCD_WIDTH-width*2, 0x1212);
    }

    spr.createSprite(SPRITE_WIDTH, SPRITE_HEIGHT);
    spr.setTextColor(TFT_BLACK, TFT_BLACK); // Set the font colour and the background colour

}

void loop()
{
    Serial.print("Current running (loop=");
    Serial.print(loopCount);
    Serial.print("). RSSI Age: ");
    Serial.print(noRSSIcount);
    Serial.print(", RSSI:");
    Serial.print(-rssi);
    
    xbee.readPacket();
    XBeeResponse & rx = xbee.getResponse();
    if (rx.isAvailable()) 
    {
        Serial.print(" / xbee.isAvailable()  / ");
        if (rx.getApiId() == RX_16_RESPONSE) 
        {
            Rx16Response rx16 = Rx16Response();
            rx.getRx16Response(rx16);
            noRSSIcount = 0;
            rssi = rx16.getRssi();
            Serial.print(-rssi);
        }
    }
    String rssiStr = String("-");
    rssiStr += String(rssi);
    //rssiStr += String(" dBm");

    Serial.println(" <<");

    spr.fillSprite(TFT_WHITE);
    spr.drawFastHLine(0, 0, SPRITE_WIDTH, TFT_GREENYELLOW);
    spr.drawFastHLine(0, SPRITE_HEIGHT-1, SPRITE_WIDTH, TFT_GREENYELLOW);
    spr.drawFastVLine(0, 3, SPRITE_HEIGHT-6, TFT_GREENYELLOW);
    spr.drawFastVLine(SPRITE_WIDTH-1, 3, SPRITE_HEIGHT-6, TFT_GREENYELLOW);

    if (loopCount % 2 == 0)
        spr.drawFastHLine(0, 3, SPRITE_WIDTH, TFT_RED);

/*
    spr.drawFastHLine(0, 0, SPRITE_WIDTH, TFT_RED);
    spr.drawFastHLine(0, 1, SPRITE_WIDTH, TFT_RED);
    spr.drawFastHLine(0, 2, SPRITE_WIDTH, TFT_RED);
    spr.drawFastHLine(0, 3, SPRITE_WIDTH, TFT_RED);

    spr.drawFastHLine(0, SPRITE_HEIGHT-1, SPRITE_WIDTH, TFT_RED);
    spr.drawFastHLine(0, SPRITE_HEIGHT-2, SPRITE_WIDTH, TFT_RED);
    spr.drawFastHLine(0, SPRITE_HEIGHT-3, SPRITE_WIDTH, TFT_RED);
    spr.drawFastHLine(0, SPRITE_HEIGHT-4, SPRITE_WIDTH, TFT_RED);

    spr.drawFastVLine(0, 3, SPRITE_HEIGHT-6, TFT_RED);
    spr.drawFastVLine(1, 3, SPRITE_HEIGHT-6, TFT_RED);
    spr.drawFastVLine(2, 3, SPRITE_HEIGHT-6, TFT_RED);
    spr.drawFastVLine(3, 3, SPRITE_HEIGHT-6, TFT_RED);

    spr.drawFastVLine(SPRITE_WIDTH-1, 3, SPRITE_HEIGHT-6, TFT_RED);
    spr.drawFastVLine(SPRITE_WIDTH-2, 3, SPRITE_HEIGHT-6, TFT_RED);
    spr.drawFastVLine(SPRITE_WIDTH-3, 3, SPRITE_HEIGHT-6, TFT_RED);
    spr.drawFastVLine(SPRITE_WIDTH-4, 3, SPRITE_HEIGHT-6, TFT_RED);
*/
    spr.setTextColor(TFT_BLACK, TFT_BLACK);
    spr.loadFont(AA_FONT_LARGE);
    spr.drawString("RSSI", 35, 20);

    int height = spr.fontHeight();

    spr.loadFont(AA_FONT_LARGE);
    String throbber = "";
    for (int i = 0; i < std::min(8u,noRSSIcount); i++)
    {
        throbber += "-";
    }
    spr.drawString(throbber, 35, height);
    height += spr.fontHeight() + 10;

    spr.loadFont(AA_FONT_HUGE);
    if (noRSSIcount > 60)
        spr.setTextColor(TFT_RED, TFT_RED);
    else if (noRSSIcount > 30)
        spr.setTextColor(TFT_ORANGE, TFT_ORANGE);
    else if (noRSSIcount > 10)
        spr.setTextColor(TFT_DARKGREY, TFT_DARKGREY);
    spr.drawString(rssiStr, 20, height);

    spr.pushSprite(10, 70);
    noRSSIcount++;
    loopCount++;
    delay(10);
}



// TFT Pin check
#if PIN_LCD_WR  != TFT_WR || \
    PIN_LCD_RD  != TFT_RD || \
    PIN_LCD_CS    != TFT_CS   || \
    PIN_LCD_DC    != TFT_DC   || \
    PIN_LCD_RES   != TFT_RST  || \
    PIN_LCD_D0   != TFT_D0  || \
    PIN_LCD_D1   != TFT_D1  || \
    PIN_LCD_D2   != TFT_D2  || \
    PIN_LCD_D3   != TFT_D3  || \
    PIN_LCD_D4   != TFT_D4  || \
    PIN_LCD_D5   != TFT_D5  || \
    PIN_LCD_D6   != TFT_D6  || \
    PIN_LCD_D7   != TFT_D7  || \
    PIN_LCD_BL   != TFT_BL  || \
    TFT_BACKLIGHT_ON   != HIGH  || \
    170   != TFT_WIDTH  || \
    320   != TFT_HEIGHT
#error  "Error! Please make sure <User_Setups/Setup206_LilyGo_T_Display_S3.h> is selected in <TFT_eSPI/User_Setup_Select.h>"
#error  "Error! Please make sure <User_Setups/Setup206_LilyGo_T_Display_S3.h> is selected in <TFT_eSPI/User_Setup_Select.h>"
#error  "Error! Please make sure <User_Setups/Setup206_LilyGo_T_Display_S3.h> is selected in <TFT_eSPI/User_Setup_Select.h>"
#error  "Error! Please make sure <User_Setups/Setup206_LilyGo_T_Display_S3.h> is selected in <TFT_eSPI/User_Setup_Select.h>"
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)
#error  "The current version is not supported for the time being, please use a version below Arduino ESP32 3.0"
#endif




