/**
 * @file      Factory.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2024  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2024-01-04
 * @note      Compatible with LilyGo TWR Rev2.0 and Rev2.1
 * * Currently, SA868 only supports NiceRF factory AT firmware and does not support OpenRTX firmware for the time being.
 * * Menu inspired by https://giltesa.com/2020/09/07/menu-grafico-para-pantalla-oled-en-arduino
 */
//#define U8X8_DO_NOT_SET_WIRE_CLOCK
//#include <LibAPRSesp.h>


#include <U8x8lib.h>
#include <U8g2lib.h>
#include <Rotary.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <BLEServer.h>
#include <WiFiAP.h>
#include <SD.h>
#include "BLE.h"
#include "fix_fft.h"
#include <AceButton.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_DPS310.h>
#include <Adafruit_ADXL345_U.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <SPIFFS.h>
#include "Constants.h"
#include "WebIndex.h"
#include <SD.h>
#include <driver/i2s.h>
// ESP8266Audio is not a mainline branch and uses SoftRF modified libraries.
#include <AudioGeneratorWAV.h>
#include <AudioOutputI2S.h>
#include <AudioFileSourceSD.h>


#define DEBUG_PORT Serial    // Output debugging information
#include "LilyGo_TWR.h"
#define WAV_FILE_PATH           "/POST.wav"
#define WAV_FILE_PATH2          "/TEST.wav"
#define WAV_FILE_PATH3          "/TESTING1.wav"
#define WAV_ZERO                "/NUMBERS/ZERO.wav"
#define WAV_ONE                 "/NUMBERS/ONE.wav"
#define WAV_TWO                 "/NUMBERS/TWO.wav"
#define WAV_THREE               "/NUMBERS/THREE.wav"
#define WAV_FOUR                "/NUMBERS/FOUR.wav"
#define WAV_FIVE                "/NUMBERS/FIVE.wav"
#define WAV_SIX                 "/NUMBERS/SIX.wav"
#define WAV_SEVEN               "/NUMBERS/SEVEN.wav"
#define WAV_EIGHT               "/NUMBERS/EIGHT.wav"
#define WAV_NINE                "/NUMBERS/NINE.wav"
#define WAV_MPS                "/UNITS/METPERSEC.wav"
#define WAV_MET                 "/UNITS/METERS.wav"
#define WAV_DEG                 "/UNITS/DEGREES.wav"
#define WAV_PERC                "/UNITS/P.wav"
#define WAV_TIME                "/UNITS/PM.wav"
#define WAV_CALLSIGN            "/DATA/CALLSIGN.wav"
#define WAV_TEMP                "/DATA/TEMPERATURE.wav"
#define WAV_ALT                 "/DATA/APOGEE.wav"
#define WAV_TOL                 "/DATA/TOLTA2.wav"
#define WAV_CREW                "/DATA/STEMCREW.wav"
#define WAV_MAXVELO             "/DATA/MAXVELO.wav"
#define WAV_BATPERC             "/DATA/BATPERC.wav"
#define WAV_ALEXPERC            "/UNITS/ALEXPERC.wav"
#define WAV_ALIPERC             "/UNITS/PERC.wav"
#define WAV_LAT                 "/UNITS/LAT.wav"
#define WAV_LONG                "/UNITS/LONG.wav"







using namespace ace_button;

enum Button {
    ShortPress,
    LongPress,
    Unknown
};

#define DECLARE_DEMO(function)      void function(uint8_t menuSelect)
#define U8G2_HOR_ALIGN_CENTER(t)    ((u8g2.getDisplayWidth() - (u8g2.getUTF8Width(t))) / 2)
#define U8G2_HOR_ALIGN_RIGHT(t)     (u8g2.getDisplayWidth()  -  u8g2.getUTF8Width(t))



DECLARE_DEMO(demoVelocitySensor);
DECLARE_DEMO(demoFlightTelemetry);
DECLARE_DEMO(demoGPS);
DECLARE_DEMO(demoPMU);
DECLARE_DEMO(demoTransFreq);
DECLARE_DEMO(demoRecvFreq);
DECLARE_DEMO(demoSquelchLevel);
DECLARE_DEMO(demoPowerLevel);
DECLARE_DEMO(demoFilter);
DECLARE_DEMO(demoBLE);
DECLARE_DEMO(demoSentData);
DECLARE_DEMO(demoDevInfo);
DECLARE_DEMO(drawError);
DECLARE_DEMO(broadcastAudioTelemetry);


uint32_t readRotary(uint32_t cur, uint32_t minOut, uint32_t maxOut, uint32_t steps = 1 );
void printMain();
void printPowerOFF();
Button readButton();

struct demo_struct {
    const char      *demo_item;
    const char      *demo_description;
    const uint8_t   demo_icon[32];
    void            (*demo_func)(uint8_t menuSelect);
} demo [] = {
    //* {
    //*         Title,
    //*         Description,
    //*         XBM ICON,
    //*         Callback,
    //* },

    {
        "Velocity Sensor",
        "EXTERNAL SENSOR",
        { 0x00, 0x00, 0xc0, 0x03, 0xe0, 0x07, 0x60, 0x06, 0x60, 0x06, 0x60, 0x06, 0x60, 0x06, 0xec, 0x37, 0xcc, 0x33, 0x1c, 0x38, 0xf8, 0x1f, 0xe0, 0x07, 0x80, 0x01, 0x80, 0x01, 0xe0, 0x07, 0xf0, 0x0f },
        demoVelocitySensor
    }, 
    {
        "BMP390 SENSOR",
        "EXTERNAL SENSOR",
        { 0xe0, 0x00, 0xf0, 0x01, 0x18, 0x03, 0x58, 0x7b, 0x58, 0x03, 0x58, 0x03, 0x58, 0x7b, 0x58, 0x03, 0x58, 0x03, 0x4c, 0x66, 0xe6, 0x0c, 0xf6, 0x0d, 0xe6, 0x6c, 0x0c, 0x06, 0xf8, 0x03, 0xf0, 0x01 },
        demoFlightTelemetry
    },
    {
        "POWER",
        "POWER MANAGEMENT",
        {0x1e, 0x7a, 0x1f, 0xfb, 0x83, 0xc3, 0xc3, 0xc1, 0xe3, 0xc1, 0xa0, 0x01, 0x90, 0x1f, 0x18, 0x10, 0xfc, 0x09, 0x80, 0x0d, 0x80, 0x07, 0x83, 0xc2, 0x83, 0xc3, 0xc3, 0xc1, 0xdf, 0xf8, 0xde, 0x78},
        demoPMU
    },
    {
        "GPS",
        "LOCATION INFORMATION",
        {0xe0, 0x07, 0xf0, 0x0f, 0x38, 0x1c, 0xcc, 0x33, 0xec, 0x37, 0x6e, 0x76, 0x26, 0x64, 0x66, 0x66, 0xee, 0x77, 0xcc, 0x33, 0x18, 0x18, 0x38, 0x1c, 0x70, 0x0e, 0xe0, 0x07, 0xc0, 0x03, 0x80, 0x01},
        demoGPS
    },
    {
        "TRANS FREQ",
        "RADIO TRANS FREQ",
        {0x00, 0x00, 0x00, 0x0c, 0x00, 0x1c, 0x00, 0x3c, 0xc0, 0x7f, 0x30, 0xe0, 0x38, 0xe0, 0xfc, 0x77, 0xee, 0x3f, 0x06, 0x1c, 0x06, 0x0c, 0xfc, 0x03, 0xf8, 0x01, 0x30, 0x00, 0x20, 0x00, 0x00, 0x00},
        demoTransFreq
    },

    {
        "RECV FREQ",
        "RADIO RECV FREQ",
        {0x00, 0x00, 0x00, 0x0c, 0x00, 0x1c, 0x00, 0x3c, 0xc0, 0x7f, 0x30, 0xe0, 0x38, 0xe0, 0xfc, 0x77, 0xee, 0x3f, 0x06, 0x1c, 0x06, 0x0c, 0xfc, 0x03, 0xf8, 0x01, 0x30, 0x00, 0x20, 0x00, 0x00, 0x00},
        demoRecvFreq
    },

    {
        "SQUELCH LEVEL",
        "RADIO SQUELCH LEVEL",
        {0x00, 0x00, 0x20, 0x20, 0x20, 0x20, 0x70, 0x3c, 0xf0, 0x7c, 0xf0, 0x7f, 0xf0, 0x1f, 0xf0, 0x3e, 0xfe, 0x3e, 0xfc, 0x3e, 0x7c, 0x1f, 0xfc, 0x0f, 0xf8, 0x01, 0xf0, 0x03, 0xc0, 0x03, 0x00, 0x00},
        demoSquelchLevel
    },
    {
        "POWER LEVEL",
        "RADIO POWER LEVEL",
        {0xf0, 0x0f, 0xf8, 0x1f, 0x1c, 0x18, 0x06, 0x40, 0x07, 0xc2, 0x03, 0xc7, 0x43, 0xc7, 0x63, 0xc7, 0xfb, 0xdf, 0xbb, 0xdd, 0x83, 0xc1, 0x87, 0xe0, 0x06, 0x60, 0x1c, 0x38, 0xf8, 0x1f, 0xf0, 0x0f},
        demoPowerLevel
    },
    {
        "FILTER",
        "RADIO FILTER",
        {0x00, 0x00, 0x00, 0x08, 0x00, 0x1e, 0xfe, 0x77, 0xfe, 0x7f, 0x00, 0x1c, 0x78, 0x00, 0xfe, 0x7f, 0xfe, 0x7f, 0x78, 0x00, 0x00, 0x1c, 0xfe, 0x7f, 0xfe, 0x7f, 0x00, 0x1e, 0x00, 0x08, 0x00, 0x00},
        demoFilter
    },
    {
        "WAV PLAY",
        "PLAYER WAV",
        {0x80, 0x01, 0x80, 0x03, 0x80, 0x07, 0x98, 0x0f, 0xb8, 0x1d, 0xf0, 0x0f, 0xe0, 0x07, 0xc0, 0x03, 0xc0, 0x03, 0xe0, 0x07, 0xf0, 0x0f, 0xb8, 0x1d, 0x98, 0x0f, 0x80, 0x07, 0x80, 0x03, 0x80, 0x01},
        broadcastAudioTelemetry
    }, 
    {
        "INFO",
        "DEVICE INFO",
        {0xf0, 0x0f, 0xf8, 0x1f, 0x3c, 0x3c, 0x8e, 0x71, 0x87, 0xe1, 0x07, 0xe0, 0x03, 0xc0, 0x83, 0xc1, 0x83, 0xc1, 0x83, 0xc1, 0x87, 0xe1, 0x87, 0xe1, 0x8e, 0x61, 0x3c, 0x3c, 0xf8, 0x1f, 0xf0, 0x0f},
        demoDevInfo
    },
};

const uint8_t itemsMENU = COUNT(demo);

struct RotarySetting {
    uint32_t cur;
    uint32_t max;
    uint32_t min;
    uint32_t steps;
};

AudioGeneratorWAV    *wav;
AudioGeneratorWAV    *wav2;
AudioFileSourceSD    *file;
AudioFileSourceSD    *file2;
AudioFileSourceSD    *file3;
AudioFileSourceSD    *ZEROF;
AudioFileSourceSD    *ONEF;
AudioFileSourceSD    *TWOF;
AudioFileSourceSD    *THREEF;
AudioFileSourceSD    *FOURF;
AudioFileSourceSD    *FIVEF;
AudioFileSourceSD    *SIXF;
AudioFileSourceSD    *SEVENF;
AudioFileSourceSD    *EIGHTF;
AudioFileSourceSD    *NINEF;
AudioOutputI2S       *out;
AsyncWebServer                      server(80);
U8G2_SH1106_128X64_NONAME_F_HW_I2C  u8g2(U8G2_R0, U8X8_PIN_NONE);
Rotary                              rotary = Rotary(ENCODER_A_PIN, ENCODER_B_PIN);
TinyGPSPlus                         gps;
Adafruit_BMP280                     bme;
Adafruit_BMP3XX                     bmp390;
Adafruit_DPS310                     dps310;
Adafruit_ADXL345_Unified            adxl345 = Adafruit_ADXL345_Unified(12345);     
AceButton                           buttons[3];
Button                              state = Unknown;
Adafruit_NeoPixel                   strip = Adafruit_NeoPixel(1, PIXELS_PIN, NEO_GRB + NEO_KHZ800);
QueueHandle_t                       rotaryMsg;
QueueHandle_t                       rotarySetting;
TaskHandle_t                        rotaryHandler;
bool                                inMenu = true;
bool                                isBMEOnline = false;
int                                 counter = -2;

const unsigned long MID = 4000;

const uint8_t                       buttonPins [] = {
    ENCODER_OK_PIN,
    BUTTON_PTT_PIN,
    BUTTON_DOWN_PIN
};

void endWeb()
{
    server.end();
    MDNS.end();
}

// Just for testing, no functionality
void setupWeb()
{
    server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
        request->send(200, "text/html", web_html );
    });

    MDNS.begin("twr");

    MDNS.addService("http", "tcp", 80);

    server.begin();
}

void setRotaryValue(uint32_t cur, uint32_t min, uint32_t max, uint32_t steps)
{
    static RotarySetting rSetting = {0};
    rSetting.cur = cur;
    rSetting.max = max;
    rSetting.min = min;
    rSetting.steps = steps;
    xQueueOverwrite(rotarySetting, (void *)&rSetting);
}


void rotaryTask(void *p)
{
    RotarySetting setting = {0};
    uint32_t pos = 0;

    rotary.begin();

    while (1) {

        uint8_t result = rotary.process();

        if (xQueueReceive(rotarySetting, (void *)&setting, ( TickType_t ) 2) == pdPASS) {
            pos = setting.cur;
            xQueueOverwrite(rotaryMsg, (void *)&pos);
            continue;
        }

        if (result) {
            if (result == DIR_CW) {
                pos += setting.steps;
                DBG("Up");
            } else {
                if (pos != 0) {
                    pos -= setting.steps;
                }
                DBG("Down");
            }
            if (pos <= setting.min) {
                pos = setting.min;
            }
            if (pos >= setting.max) {
                pos = setting.max;
            }
            xQueueOverwrite(rotaryMsg, (void *)&pos);
        }
        delay(2);
    }
}

void handleEvent(AceButton *button, uint8_t eventType, uint8_t buttonState)
{
    uint8_t id = button->getId();

    DBG(F("EventType: "), AceButton::eventName(eventType), F("; buttonState: "), buttonState, F("; ID: "), id);

    // rotary center button
    switch (id) {
    case 0: {
        switch (eventType) {
        case AceButton::kEventClicked:
            state = ShortPress;
            DBG("ShortPress");
            break;
        case AceButton::kEventLongPressed:
            state = LongPress;
            DBG("LongPress");
            break;
        default:
            break;
        }
    }
    break;

    // PTT button
    case 1:
        switch (eventType) {
        case AceButton::kEventPressed:
            DBG("transmit");
            strip.setPixelColor(0, strip.Color(255, 0, 0));
            strip.show();
            radio.transmit();
            break;
        case AceButton::kEventReleased:
            DBG("receive");
            strip.clear();
            strip.show();
            radio.receive();
            break;
        default:
            break;
        }
        break;

    // BOOT button
    case 2:
        switch (eventType) {
        case AceButton::kEventPressed:
            if (!inMenu) {
                state = LongPress;
                DBG("LongPress");

            } else {
                uint8_t v = radio.getVolume();
                radio.setVolume(--v);
            }
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}


//BMP thing
bool setupBMP390()
{
    uint8_t slaveAddress = 0x00;
    Wire.beginTransmission(0x76);
    if (Wire.endTransmission() == 0) {
        slaveAddress = 0x76;
    }
    Wire.beginTransmission(0x77);
    if (Wire.endTransmission() == 0) {
        slaveAddress = 0x77;
    }
    if (slaveAddress == 0) {
        return false;
    }
    if (!bmp390.begin_I2C(slaveAddress, &Wire)) {
        DBG("Could not find a valid BME280 sensor, check wiring!");
        isBMEOnline = false;
        return false;
    }
    bmp390.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp390.setPressureOversampling(BMP3_OVERSAMPLING_2X);
  bmp390.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp390.setOutputDataRate(BMP3_ODR_50_HZ);
    isBMEOnline = true;
    return isBMEOnline;
}

void setupOLED(uint8_t addr)
{
    u8g2.setI2CAddress(addr << 1);
    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setContrast(10);
    uint8_t b = 10;
    u8g2.firstPage();
    do {
        do {
            u8g2.setFont(u8g2_font_tenstamps_mu);
            u8g2.drawXBM(40, 0, 48, 48, xbmLOGO);
            u8g2.setCursor(22, 62);
            u8g2.print("LILYGO");
            u8g2.sendBuffer();
            u8g2.setContrast(b);
        } while (u8g2.nextPage());
        b += 10;
        delay(8);
    } while (b < twr.pdat.dispBrightness);
}

void setup()
{
    bool rslt = false;

#ifdef DEBUG_PORT
    DEBUG_PORT.begin(115200);
#endif

    // Create message queue
    rotaryMsg = xQueueCreate(1, sizeof(uint32_t));
    rotarySetting = xQueueCreate(1, sizeof(RotarySetting));

    // Initialize pixel lights
    strip.setBrightness(50);
    strip.begin();
    strip.setPixelColor(0, strip.Color(0, 255, 0));
    strip.show();

    //* Initializing PMU is related to other peripherals
    //* Automatically detect the revision version through GPIO detection.
    //* If GPIO2 has been externally connected to other devices, the automatic detection may not work properly.
    //* Please initialize with the version specified below.
    //* Rev2.1 is not affected by GPIO2 because GPIO2 is not exposed in Rev2.1
    //HUNTSVILLE
    //rslt = twr.begin();
   // rslt = twr.begin(LILYGO_TWR_REV2_1);
    //* If GPIO2 is already connected to other devices, please initialize it with the specified version.
    rslt =  twr.begin(LILYGO_TWR_REV2_0);

    while (!rslt) {
        DBG("PMU communication failed..");
        delay(1000);
    }


    //* Rev2.1 uses OLED to determine whether it is VHF or UHF.
    //* Rev2.0 cannot determine by device address.By default, the 0X3C device address is used.

    if (twr.getVersion() == TWRClass::TWR_REV2V1) {
        DBG("Detection using TWR Rev2.1");

        //* Initialize SA868
        radio.setPins(SA868_PTT_PIN, SA868_PD_PIN);
        rslt = radio.begin(RadioSerial, twr.getBandDefinition());

    } else {

        DBG("Detection using TWR Rev2.0");

        //* Rev2.0 cannot determine whether it is UHF or VHF, and can only specify the initialization BAND.
        //* Or modify it through the menu INFO -> BAND
        radio.setPins(SA868_PTT_PIN, SA868_PD_PIN, SA868_RF_PIN);

        

        //* Designated as VHF
         rslt = radio.begin(RadioSerial, SA8X8_VHF);

    }

    // If the display does not exist, it will block here
    uint8_t addr = twr.getOLEDAddress();
    if ((addr != 0x3C || addr != 0x3D) && !rslt) {
        // Initialize display
        u8g2.setI2CAddress(addr << 1);
        u8g2.begin();
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_tenstamps_mu);
        u8g2.setCursor(30, 28);
        u8g2.print("SA8X8");
        u8g2.setCursor(22, 52);
        u8g2.print("FAILED");
        u8g2.sendBuffer();
    }
    while (!rslt) {
        DBG("SA8x8 communication failed, please use ATDebug to check whether the module responds normally..");
        strip.setPixelColor(0, strip.Color(255, 0, 0));
        strip.show();
        delay(300);
        strip.clear();
        strip.show();
        delay(300);
    }

    strip.setBrightness(twr.pdat.pixelBrightness);
    strip.clear();
    strip.show();


    // Initialize the temperature and pressure sensor.
    // Note that this sensor is an external addition and is not integrated in TWR.
    setupBMP390();

    // Initialize display
    setupOLED(addr);

    delay(3000);

    // Initialize button
    for (uint8_t i = 0; i < COUNT(buttonPins); i++) {
        pinMode(buttonPins[i], INPUT_PULLUP);
        buttons[i].init(buttonPins[i], HIGH, i);
    }
    ButtonConfig *buttonConfig = ButtonConfig::getSystemButtonConfig();
    buttonConfig->setEventHandler(handleEvent);
    buttonConfig->setFeature(ButtonConfig::kFeatureClick);
    buttonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
    buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);
    buttonConfig->setFeature(ButtonConfig::kFeatureRepeatPress);

    // Disable the PMU long press shutdown function.
    twr.enablePowerOff(false);

    // Added display of shutdown animation after long pressing the PWR button
    twr.attachLongPressed([]() {
        printPowerOFF();
    });

    // Add a callback function after pressing the PWR button
    twr.attachClick([]() {
        if (inMenu) {
            uint8_t v = radio.getVolume();
            radio.setVolume(++v);
        }
    });

    // Create a rotary encoder processing task
    xTaskCreate(rotaryTask, "rotary", 10 * 1024, NULL, 10, &rotaryHandler);

} // END OF SETUP


void loop()
{
    static uint8_t menuSelect     = 0;
    static uint8_t prevMenuSelect = menuSelect;
    Button btnPressed;
    
    inMenu = true;
    do {
        btnPressed = readButton();
        printMain();

        // DeepSleep test , About ~660uA
        if (btnPressed == LongPress) {
            DBG("DeepSleep....");
            printDeepSleep();

            strip.clear();
            strip.show();

            radio.sleep();

            twr.sleep();

            esp_sleep_enable_ext1_wakeup(_BV(BUTTON_PTT_PIN), ESP_EXT1_WAKEUP_ALL_LOW);

            esp_deep_sleep_start();
        }

    } while ( btnPressed != ShortPress );

    inMenu = false;
    uint8_t lastItem = 0;

    if (twr.isEnableBeep()) {
        twr.routingSpeakerChannel(TWRClass::TWR_ESP_TO_SPK);
    }

    while (1) {
        do {
            menuSelect  = readRotary(lastItem, 0, itemsMENU - 1);
            btnPressed = readButton();
            if (btnPressed == LongPress) {
                menuSelect = 0;
                break;
            }
            if ( menuSelect != prevMenuSelect ) {
                prevMenuSelect = menuSelect;
                beep();
            }
            printMenu(menuSelect);

        } while ( btnPressed != ShortPress );

        beep();

        if (btnPressed == ShortPress) {
            demo[menuSelect].demo_func(menuSelect);
            lastItem = menuSelect;
        } else {
            break;
        }
    }
    menuSelect = 0;
    if (twr.isEnableBeep()) {
        twr.routingSpeakerChannel(TWRClass::TWR_RADIO_TO_SPK);
    }
}

void printPowerOFF()
{
    int b = twr.pdat.dispBrightness;
    do {
        u8g2.firstPage();
        do {
            u8g2.setFont(u8g2_font_tenstamps_mu);
            u8g2.drawXBM(40, 0, 48, 48, xbmOFF);
            u8g2.setCursor(5, 59);
            u8g2.print("POWEROFF");
            u8g2.setContrast(b);
        } while ( u8g2.nextPage() );
        b -= 10;
        delay(8);
    } while (b >= 0);
    twr.shutdown();
}

void printDeepSleep()
{
    int b = twr.pdat.dispBrightness;
    do {
        u8g2.firstPage();
        do {
            u8g2.setFont(u8g2_font_tenstamps_mu);
            u8g2.drawXBM(40, 0, 48, 48, xbmOFF);
            u8g2.setCursor(32, 59);
            u8g2.print("SLEEP");
            u8g2.setContrast(b);
        } while ( u8g2.nextPage() );
        b -= 10;
        delay(8);
    } while (b >= 0);

    u8g2.setPowerSave(true);
}

void printMain()
{
    uint8_t rssi_x = 95;
    uint8_t rssi_y = 12;
    uint8_t bat_x = 110;
    uint8_t bat_y = 12;
    static int rssi = 0;
    static uint32_t check_interval = 0;
    int percentage = 0;
    static bool  showStrip = false;

    u8g2.firstPage();
    do {
        drawFrame();

        bool isTransmit = radio.isTransmit();
        // Draw RSSI
        if (!isTransmit) {
            if (millis() > check_interval) {
                rssi = radio.getRSSI();
                check_interval = millis() + 1000;
            }
        }

        percentage = twr.getBatteryPercent();

        u8g2.setFont(u8g2_font_siji_t_6x10);

        if (rssi >= 80) {
            u8g2.drawGlyph(rssi_x, rssi_y, 0xe261);
        } else if (rssi >= 40) {
            u8g2.drawGlyph(rssi_x, rssi_y, 0xe260);
        } else if (rssi >= 20) {
            u8g2.drawGlyph(rssi_x, rssi_y, 0xe25f);
        } else if (rssi >= 10) {
            u8g2.drawGlyph(rssi_x, rssi_y, 0xe25e);
        } else {
            u8g2.drawGlyph(rssi_x, rssi_y, 0xe25d);
        }

        // Battery
        if (twr.isCharging()) {
            u8g2.drawGlyph(bat_x, bat_y, 0xe239);
        } else if (percentage < 0) {
            u8g2.drawGlyph(bat_x, bat_y, 0xe242);
        } else if (percentage >= 80) {
            u8g2.drawGlyph(bat_x, bat_y, 0xe251);
        } else if (percentage >= 40) {
            u8g2.drawGlyph(bat_x, bat_y, 0xe250);
        } else if (percentage >= 20) {
            u8g2.drawGlyph(bat_x, bat_y, 0xe24f);
        } else if (percentage >= 10) {
            u8g2.drawGlyph(bat_x, bat_y, 0xe24e);
        } else {
            u8g2.drawGlyph(bat_x, bat_y, 0xe24d);
        }

        // GPS
        u8g2.drawGlyph(80, 12, 0xe02c);

        int start_pos = 70;
        // Ble
        if (twr.getSetting().ble) {
            u8g2.drawGlyph(start_pos, 12, 0xe00b);
            start_pos -= 13;
        }
        // Wifi
        if (twr.getSetting().wifi) {
            u8g2.drawGlyph(start_pos, 12, 0xe21a);
            start_pos -= 10;
        }
        // SD
        if (SD.cardType() != CARD_NONE
                && SD.cardType() != CARD_UNKNOWN) {
            u8g2.drawGlyph(start_pos - 2, 11, 0xE120);
            start_pos -= 10;
        }

        if (isTransmit) {
            u8g2.drawGlyph(start_pos, 12, 0xe04C);
        }

        if (twr.isReceiving) { 
            u8g2.drawGlyph(start_pos, 12, 0xe275);
            if (!showStrip) {
                strip.setPixelColor(0, strip.Color(0, 255, 0));
                strip.show();
                showStrip = true;
            }
        } else {
            if (showStrip) {
                showStrip = false;
                strip.setPixelColor(0, strip.Color(0, 0, 0));
                strip.show();
            }
        }
        

        int volume = radio.getVolume();
        for (int i = 0; i < volume; ++i) {
            u8g2.drawGlyph(5 + (4 * i), 12, 0x007c);
        }

        float transFreq = radio.getStetting().transFreq / 1000000.0;
        float recvFreq  = radio.getStetting().recvFreq / 1000000.0;

        // Draw Freq
        u8g2.setFont(u8g2_font_pxplusibmvga8_mr    );
        u8g2.setCursor(22, 35);
        u8g2.print("TX:");
        u8g2.setCursor(22, 50);
        u8g2.print("RX:");

        u8g2.setFont(u8g2_font_9x6LED_mn);
        u8g2.setCursor( U8G2_HOR_ALIGN_RIGHT(String(transFreq, 4).c_str()) - 21, 35 );
        u8g2.print(transFreq, 4);
        u8g2.setCursor( U8G2_HOR_ALIGN_RIGHT(String(recvFreq, 4).c_str()) - 21, 50 );
        u8g2.print(recvFreq, 4);

    } while ( u8g2.nextPage() );
}

void printMenu( uint8_t menuSelect )
{
    u8g2.firstPage();
    do {
        //ROW 1:
        if ( menuSelect > 0 ) {
            u8g2.drawXBMP(4, 2,  14, 14, xMenuUp);
            u8g2.setFont(u8g2_font_haxrcorp4089_tr);
            u8g2.setCursor(22, 13);
            u8g2.print(demo[menuSelect - 1].demo_item);

        }
        //ROW 2:
        u8g2.setFont(u8g2_font_nokiafc22_tu);
        u8g2.drawBox(0, 18, 128, 27);

        u8g2.setColorIndex(0);
        u8g2.drawXBMP(3, 24, 16, 16, demo[menuSelect].demo_icon);

        if ( strlen(demo[menuSelect].demo_description) == 0 ) {
            u8g2.drawStr(22, 35, demo[menuSelect].demo_item);
        } else {
            u8g2.drawStr(22, 30, demo[menuSelect].demo_item);
            u8g2.setFont(u8g2_font_micro_mr);
            u8g2.drawStr(22, 40, demo[menuSelect].demo_description);
        }

        //ROW 3:
        u8g2.setColorIndex(1);
        if ( menuSelect < itemsMENU - 1 ) {
            u8g2.drawXBMP(4, 48, 14, 14, xMenuDown);
            u8g2.setFont(u8g2_font_haxrcorp4089_tr);
            u8g2.setCursor(22, 58);
            u8g2.print(demo[menuSelect + 1].demo_item);
        }
        drawFrame();
    } while ( u8g2.nextPage() );
}


void demoMic(uint8_t menuSelect )
{
    Button   btnPressed;
    uint32_t prevTime   = 0;
    int value = 600;
    twr.routingMicrophoneChannel(TWRClass::TWR_MIC_TO_ESP);
    radio.transmit();

    strip.setPixelColor(0, strip.Color(0, 255, 0));
    strip.show();

    do {
        value   = readRotary(value, 100, 2000, 100);
        btnPressed = readButton();

        u8g2.firstPage();
        do {
            drawFrame();
            drawHeader(menuSelect);

            u8g2.drawFrame(13, 30, 102, 12);
            u8g2.drawFrame(14, 31, 100, 10);

            uint32_t bar = map(value, 100, 2000, 0, 100);
            for ( int x = 0 ; x < bar ; x++ ) {
                u8g2.drawVLine(15 + x, 32, 8);
            }

            u8g2.setFont(u8g2_font_nokiafc22_tu);
            u8g2.setCursor(14, 51);
            u8g2.print(F("FREQ:"));

            u8g2.setCursor( U8G2_HOR_ALIGN_RIGHT(String(bar).c_str()) - 35, 51 );
            u8g2.print(value); u8g2.print("HZ");

        } while ( u8g2.nextPage() );

        if (millis() > prevTime) {
            tone(ESP2SA868_MIC, value, 100);
            prevTime = millis() + 1000;
        }

    } while (btnPressed != LongPress);

    radio.receive();

    twr.routingMicrophoneChannel(TWRClass::TWR_MIC_TO_RADIO);

    strip.clear();
    strip.show();
}


void demoVelocitySensor(uint8_t menuSelect) {   
    uint32_t intervalue = 0;
    Button btnPressed;
    sensors_event_t event;
    float previousAltitude = 0.0;
    float currentAltitude = 0.0;
    float velocity = 0.0;  // Vertical velocity in m/s
    unsigned long previousTime = 0;  // To track the time difference
    float maxVelocity = 0.0;
    float altitude = 0;
    float pressure = 0;
    float maxPressure = 0;

   
    do {
        // Read the acceleration data every second
        if (isBMEOnline && millis() > intervalue) {
            pressure = (bmp390.pressure / 100.0);
            
            currentAltitude = bmp390.readAltitude(1024.30);

            unsigned long currentTime = millis();
            float deltaTime = (currentTime - previousTime) / 1000.0;

            if(deltaTime > 0){
                float deltaAltitude = currentAltitude - previousAltitude;
                velocity = deltaAltitude / deltaTime;
            }

            if(velocity > maxVelocity){
                maxVelocity = velocity;
            }

            previousAltitude = currentAltitude;
            previousTime = currentTime;
            intervalue = millis() + 1000;
        }

        // Read button press
        btnPressed = readButton();

        // Display the data
        u8g2.firstPage();
        do {
            drawFrame();
            drawHeader(menuSelect);

            // Display acceleration (Z-axis)
            u8g2.setFont(u8g2_font_nokiafc22_tu);
            u8g2.setCursor(14, 31);
            u8g2.print(F("Z Velo:"));
            u8g2.setCursor(U8G2_HOR_ALIGN_RIGHT(String(velocity, 2).c_str()) - 22, 31);
            u8g2.print(velocity, 2);
            u8g2.setCursor(109, 31);
            u8g2.print(F("m/s")); 

            u8g2.setCursor(14, 44);
            u8g2.print(F("Max Velo:"));
            u8g2.setCursor( U8G2_HOR_ALIGN_RIGHT(String(maxVelocity, 2).c_str()) - 22, 44 );
            u8g2.print(maxVelocity, 2);
            u8g2.setCursor(109, 44);
            u8g2.print(F("m/s"));

            u8g2.setCursor(14, 57);
            u8g2.print(F(/*"TEMP:"*/"ALT:"));
            u8g2.setCursor( U8G2_HOR_ALIGN_RIGHT(String(/*temperature*/pressure, 5).c_str()) - 22, 57 );
            u8g2.print(/*temperature*/pressure, 5);
            u8g2.setCursor(109, 57);
            u8g2.print(F("M"));
 
        } while (u8g2.nextPage());

    } while (btnPressed != LongPress);
}


void demoFlightTelemetry(uint8_t menuSelect) {
    float altitude = 0;
    float pressure = 0;
    float temperature = 0;
    float maxAltitude = 0;
    float currentAltitude;
    float maxVelocity = 0;
    float previousAltitude = 0.0;
    float velocity = 0.0;        // Vertical velocity in m/s
    float voltageBattery; // Placeholder for battery percentage
    float survivePercentage = 100.0;
    int timeHour,timeMinutes,landingH,landingM;
    bool  altFlag, veloFlag, batFlag = false;
    uint32_t intervalue = 0;
    float previousTime = 0;
    bool landed = false;
    
    Button btnPressed;

    do {
        if (isBMEOnline && millis() > intervalue) {
            
            //For GPS data
            while (GPSSerial.available()){
                gps.encode(GPSSerial.read());
            }
            
            // Read sensor data
            temperature = bmp390.temperature;

            //GPS Time for TOL
            timeHour = gps.time.hour() + 10;
            timeMinutes = gps.time.minute();

            if(twr.isReceiving){
                strip.setPixelColor(0, strip.Color(0, 255, 0));
                    strip.show();
                    counter ++;
                
            }
            if(counter > 10){
                counter = 0;
                strip.clear();
                strip.show();
            }
            voltageBattery  = twr.getBatteryPercent();
            pressure = (bmp390.pressure / 100.0);
            altitude = bmp390.readAltitude(1030.0);

            currentAltitude = bmp390.readAltitude(1030.0); 

            
            float currentTime = millis();
            float deltaTime = (currentTime - previousTime) / 1000.0;

            if(deltaTime > 0.0f){
                float deltaAltitude = currentAltitude - previousAltitude;
                velocity = deltaAltitude / deltaTime;
            } 

            previousAltitude = currentAltitude;
            previousTime = currentTime;

            //For max Altitude
            if (altitude > maxAltitude) {
                maxAltitude = altitude;
            }

            //For max velocity ERROR at start
            if(velocity > 3000 || isinf(velocity)){
                velocity = 0;
            }

            if(velocity > maxVelocity){
                maxVelocity = velocity;
            }

            //For landing time
            // Landing is inferred from sustained radio idle time (counter),
            // not a direct altitude/velocity check -- see notes above.
            if (counter > 2) {
                landed = true;
                landingH = timeHour;
                landingM = timeMinutes;
            } else if(counter < 1){
                landed = false;
            }
            DBG(counter);
            //For Survivability
            if(altitude > 1410 && !altFlag){
            survivePercentage -= 10.0;
            altFlag = true;
            }
            if(velocity < -7.2 || velocity > 183.0 && !veloFlag ){
            survivePercentage -= 25.0;
            veloFlag = true;
            }
            if(voltageBattery < 75 && !batFlag){
            survivePercentage = survivePercentage - 30.0;
            batFlag = true;
            }
            
        

        intervalue = millis() + 500;
    
        }

        // Check for button press
        btnPressed = readButton();

        // Display data on screen
        u8g2.firstPage();
        do {
            drawFrame();
            drawHeader(menuSelect);

            u8g2.setFont(u8g2_font_5x7_tf);

            // Row 1: Max Velocity
            u8g2.setCursor(2, 28);
            u8g2.print(F("MAXV:"));
            u8g2.setCursor(27,28);
            u8g2.print(maxVelocity, 2);
            u8g2.setCursor(60, 28);
            u8g2.print(F("m/s"));

            // Row 2: Max Altitude
            u8g2.setCursor(2, 40);
            u8g2.print(F("MALT:"));
            u8g2.setCursor(27,40);
            u8g2.print(maxAltitude, 2);
            u8g2.setCursor(65, 40);
            u8g2.print(F("m"));

            // Row 3: Temperature
            u8g2.setCursor(2, 52);
            u8g2.print(F("TEMP:"));
            u8g2.setCursor(27, 52);
            u8g2.print(temperature, 2);
            u8g2.setCursor(54, 52);
            u8g2.print(F("C"));
            
            //Row 3: Survivability (Next to Temp)
            u8g2.setCursor(75, 52);
            u8g2.print(F("SOC:"));
            u8g2.setCursor(95, 52);
            u8g2.print(survivePercentage, 1);
            u8g2.setCursor(120, 52);
            u8g2.print(F("%")); 

            // Row 4: Battery
            u8g2.setCursor(2, 62);
            u8g2.print(F("BAT:"));
            u8g2.setCursor(21, 62);
            u8g2.print(voltageBattery, 1);
            u8g2.setCursor(50, 62);
            u8g2.print(F("%"));

            // Row 4: Time of Landing (Next to Battery)
            u8g2.setCursor(65, 62);
            u8g2.print(F("TOL:"));
            if(landed){
            u8g2.setCursor(85, 62); 
            u8g2.printf("%d:",-landingH);
            u8g2.setCursor(100, 62); 
            if(gps.time.minute() < 10) u8g2.print("0");
            u8g2.print(landingM);
              if(gps.time.hour() > 10){
                    u8g2.setCursor(110,62);
                    u8g2.print("AM");
                } else{
                    u8g2.setCursor(110,62);
                    u8g2.print("PM");
                }
    
            } else{
                u8g2.setCursor(90,62);
                u8g2.print(F("N/A"));
            }
            
            
        } while (u8g2.nextPage());

    } while (btnPressed != LongPress);
}



void demoGPS(uint8_t menuSelect)
{
    Button btnPressed;
    uint8_t ledState = LOW;
    bool enableNMEAOutSerial = false;
    const uint32_t debounceDelay = 50;
    uint32_t lastDebounceTime = 0;

    do {

        btnPressed = readButton();

#if ARDUINO_USB_CDC_ON_BOOT
        if (btnPressed == ShortPress) {
            // DBG("enableNMEAOutSerial.....");
            enableNMEAOutSerial = !enableNMEAOutSerial;
        }
#endif

        // GPS PPS
        int pps = digitalRead(GNSS_PPS_PIN);
        if (pps == HIGH && (millis() - lastDebounceTime) > debounceDelay) {
            ledState = !ledState;
            if (ledState) {
                strip.setPixelColor(0, strip.Color(0, 255, 0));
            } else {
                strip.clear();
            }
            strip.show();
            lastDebounceTime = millis();
            twr.ledToggle();
        }

        while (GPSSerial.available()) {
            int c = GPSSerial.read();
#if ARDUINO_USB_CDC_ON_BOOT
            if (enableNMEAOutSerial) {
                Serial.write(c);
            }
#endif
            gps.encode(c);
        }

        u8g2.firstPage();
        do {
            const uint8_t offset = 10;
            drawFrame();
            drawHeader(menuSelect);

            u8g2.setColorIndex(0);
            u8g2.setFont(u8g2_font_nokiafc22_tu);
            if (gps.location.isValid()) {
                u8g2.drawStr(100, 12, "3D");
            } else {
                u8g2.drawStr(100, 12, "N/A");
            }
            u8g2.setColorIndex(1);

            u8g2.setCursor(14, 31);
            u8g2.print(F("LONGITUDE:"));
            String lng = gps.location.isValid() ? String(gps.location.lng(), 5) : "N/A";
            u8g2.setCursor( U8G2_HOR_ALIGN_RIGHT(lng.c_str()) - offset, 31 );
            u8g2.print(lng);

            u8g2.setCursor(14, 44);
            u8g2.print(F("LATITUDE:"));
            String lat = gps.location.isValid() ? String(gps.location.lat(), 3).c_str() : "N/A";
            u8g2.setCursor( U8G2_HOR_ALIGN_RIGHT(lat.c_str()) - offset, 44 );
            u8g2.print(lat);

            u8g2.setCursor(14, 57);
            u8g2.print(F("SPEED:"));
            String kmph = gps.location.isValid() ? String(gps.speed.kmph(), 1).c_str() : "N/A";
            u8g2.setCursor( U8G2_HOR_ALIGN_RIGHT(kmph.c_str()) - 35, 57 );
            u8g2.print(kmph);
            u8g2.setCursor(99, 57);
            u8g2.print(F("KM/H"));

        } while ( u8g2.nextPage() );

    } while ( btnPressed != LongPress );

    strip.clear();
    strip.show();
}


void demoPMU(uint8_t menuSelect)
{
    float    voltageUSB;
    float    voltageBattery;
    float    tempPMU;
    Button btnPressed;

    do {
        voltageUSB      = twr.getVbusVoltage() / 1000.0;
        voltageBattery  = twr.getBatteryPercent();
        tempPMU         = twr.getTemperature();
        btnPressed      = readButton();

        u8g2.firstPage();
        do {
            drawFrame();
            drawHeader(menuSelect);

            u8g2.setFont(u8g2_font_nokiafc22_tu);

            u8g2.setCursor(14, 31);
            u8g2.print(F("USB:"));
            u8g2.setCursor( U8G2_HOR_ALIGN_RIGHT(String(voltageUSB).c_str()) - 22, 31 );
            u8g2.print(voltageUSB);
            u8g2.setCursor(109, 31);
            u8g2.print(F("V"));

            u8g2.setCursor(14, 44);
            u8g2.print(F("BATTERY:"));
            u8g2.setCursor( U8G2_HOR_ALIGN_RIGHT(String(voltageBattery).c_str()) - 22, 44 );
            u8g2.print(voltageBattery);
            u8g2.setCursor(109, 44);
            u8g2.print(F("%"));

            u8g2.setCursor(14, 57);
            u8g2.print(F("TEMP. PMU:"));
            u8g2.setCursor( U8G2_HOR_ALIGN_RIGHT(String(tempPMU).c_str()) - 22, 57 );
            u8g2.print(tempPMU);
            u8g2.setCursor(109, 57);
            u8g2.print(F("C"));
        } while ( u8g2.nextPage() );

    } while ( btnPressed != LongPress );
}

uint32_t demoFreqSetting(uint8_t menuSelect,
                         bool tx,
                         uint32_t curFreq,
                         uint8_t option,
                         uint32_t minOut,
                         uint32_t maxOut,
                         uint32_t steps
                        )
{
    Button btnPressed;
    uint32_t value = 0;
    uint32_t prevValue = 0;
    uint32_t bandwidth = radio.getStetting().bandwidth ;
    uint32_t  transFreq = tx ? radio.getStetting().transFreq : radio.getStetting().recvFreq;
    uint32_t CXCSS = tx ? radio.getStetting().txCXCSS : radio.getStetting().rxCXCSS;
    do {
        btnPressed = readButton();
        value = readRotary(curFreq, minOut, maxOut, steps);
        if ( btnPressed == ShortPress ) {
            beep();
        }
        if (prevValue != value) {
            beep();
            prevValue = value;
        }
        u8g2.firstPage();
        do {

            drawFrame();
            drawHeader(menuSelect);

            u8g2.setFont(u8g2_font_nokiafc22_tu);

            u8g2.setCursor(14, 31);
            u8g2.print(F("FREQ  :"));

            if (option == 0) {
                u8g2.drawRBox(55, 22, 58, 12, 2);
                u8g2.setColorIndex(0);
                u8g2.setCursor(U8G2_HOR_ALIGN_RIGHT(String(value / 1000000.0, 5).c_str()) - 22, 31 );
                u8g2.print(value / 1000000.0, 5);
                u8g2.setColorIndex(1);
            } else {
                u8g2.setCursor(U8G2_HOR_ALIGN_RIGHT(String(transFreq / 1000000.0, 5).c_str()) - 22, 31 );
                u8g2.print(transFreq / 1000000.0, 5);
            }

            u8g2.setCursor(14, 44);
            u8g2.print(F("STEPS:"));
            if (option == 1) {
                u8g2.drawRBox(55, 35, 58, 12, 2);
                u8g2.setColorIndex(0);
                u8g2.setCursor(U8G2_HOR_ALIGN_RIGHT(String(value / 1000.0, 1).c_str()) - 45, 44 );
                u8g2.print(value / 1000.0, 1);
                u8g2.print("KHZ");
                u8g2.setColorIndex(1);
            } else {
                u8g2.setCursor( U8G2_HOR_ALIGN_RIGHT(String(bandwidth / 1000.0, 1).c_str()) - 45, 44 );
                u8g2.print(bandwidth / 1000.0, 1);
                u8g2.print("KHZ");
            }

            u8g2.setCursor(14, 57);
            u8g2.print(F("CXCSS:"));
            if (option == 2) {
                u8g2.drawRBox(55, 48, 58, 12, 2);
                u8g2.setColorIndex(0);
                u8g2.setCursor( U8G2_HOR_ALIGN_RIGHT(String(cxcss[value]).c_str()) - 38, 57 );
                u8g2.print(cxcss[value]);
                u8g2.setColorIndex(1);
            } else {
                u8g2.setCursor( U8G2_HOR_ALIGN_RIGHT(String(cxcss[CXCSS]).c_str()) - 38, 57 );
                u8g2.print(cxcss[CXCSS]);
            }
        } while (u8g2.nextPage());

    } while ( btnPressed != ShortPress );

    return  value ;
}

void setTransFreq(uint8_t menuSelect, bool tx)
{
    Button btnPressed;
    uint32_t pos = 0;
    uint32_t prevPos = 0;
    uint32_t minOut = radio.getStetting().minFreq;
    uint32_t maxOut = radio.getStetting().maxFreq;
    uint32_t transFreq = tx ? radio.getStetting().transFreq : radio.getStetting().recvFreq;
    uint32_t bandwidth = radio.getStetting().bandwidth ;
    uint32_t CXCSS = tx ? radio.getStetting().txCXCSS : 
    radio.getStetting().rxCXCSS;
    DBG(CXCSS);

    do {
        btnPressed = readButton();
        pos = readRotary(0, 0, 2);

        if (prevPos != pos) {
            prevPos  = pos;
            beep();
        }

        u8g2.firstPage();
        do {
            drawFrame();
            drawHeader(menuSelect);
            u8g2.setFont(u8g2_font_nokiafc22_tu);

            switch (pos) {
            case 0:
                u8g2.drawRFrame(55, 22, 58, 12, 2);
                break;
            case 1:
                u8g2.drawRFrame(55, 35, 58, 12, 2);
                break;
            case 2:
                u8g2.drawRFrame(55, 48, 58, 12, 2);
                break;
            default:
                break;
            }
            u8g2.setCursor(14, 31);
            u8g2.print(F("FREQ  :"));
            u8g2.setCursor(U8G2_HOR_ALIGN_RIGHT(String(transFreq / 1000000.0, 5).c_str()) - 22, 31 );
            u8g2.print(transFreq / 1000000.0, 5);

            u8g2.setCursor(14, 44);
            u8g2.print(F("STEPS:"));
            u8g2.setCursor( U8G2_HOR_ALIGN_RIGHT(String(bandwidth / 1000.0, 1).c_str()) - 45, 44 );
            u8g2.print(bandwidth / 1000.0, 1);
            u8g2.print("KHZ");

            u8g2.setCursor(14, 57);
            u8g2.print(F("CXCSS:"));
            u8g2.setCursor( U8G2_HOR_ALIGN_RIGHT(String(cxcss[CXCSS]).c_str()) - 38, 57 );
            u8g2.print(cxcss[CXCSS]);

        } while ( u8g2.nextPage() );

        if (btnPressed == ShortPress ) {

            beep();

            uint32_t steps = bandwidth;
            uint32_t cur = transFreq;

            switch (pos) {
            case 0:
                minOut = radio.getStetting().minFreq;
                maxOut = radio.getStetting().maxFreq;
                steps = radio.getStetting().bandwidth ;
                transFreq = tx ? radio.getStetting().transFreq : radio.getStetting().recvFreq;
                break;
            case 1:
                minOut = 12500;
                maxOut = 25000;
                steps =  12500;
                bandwidth = radio.getStetting().bandwidth ;
                cur = bandwidth;
                break;
            case 2:
                minOut = 0;
                maxOut = COUNT(cxcss);
                steps =  1;
                cur = CXCSS;
                break;
            default:
                break;
            }

            uint32_t setValue = demoFreqSetting(menuSelect, tx, cur, pos, minOut, maxOut, steps);
            DBG("Set opt:", pos, "value:", setValue);

            switch (pos) {
            case 0:
                tx ? radio.setTxFreq(setValue) : radio.setRxFreq(setValue);
                break;
            case 1:
                if (radio.getStetting().bandwidth != setValue) {
                    radio.setBandWidth(setValue);
                    // Bandwidth change, reset frequency to the minimum frequency value
                    tx ? radio.setTxFreq(radio.getStetting().minFreq) : radio.setRxFreq(radio.getStetting().minFreq);
                }
                break;
            case 2:
                tx ? radio.setTxCXCSS(setValue) : radio.setRxCXCSS(setValue);
                break;
            default:
                break;
            }
            transFreq = tx ? radio.getStetting().transFreq : radio.getStetting().recvFreq;
            bandwidth = radio.getStetting().bandwidth ;
            CXCSS = tx ? radio.getStetting().txCXCSS : 
            radio.getStetting().rxCXCSS;
            DBG(CXCSS);
        }

    } while ( btnPressed != LongPress );

    radio.saveConfigure();
}

void demoTransFreq(uint8_t menuSelect)
{
    setTransFreq(menuSelect, true);
}

void demoRecvFreq(uint8_t menuSelect)
{
    setTransFreq(menuSelect, false);
}

void demoSquelchLevel(uint8_t menuSelect)
{
    int    value        = radio.getStetting().SQ;
    int    prevSQ       = value;
    Button btnPressed;
    do {
        value   = readRotary(value, 0, 8);
        btnPressed = readButton();

        if (prevSQ != value) {
            prevSQ  = value;
            beep();
        }

        u8g2.firstPage();
        do {
            drawFrame();
            drawHeader(menuSelect);

            u8g2.drawFrame(13, 30, 102, 12);
            u8g2.drawFrame(14, 31, 100, 10);

            uint32_t bar = map(value, 0, 8, 0, 100);
            for ( int x = 0 ; x < bar ; x++ ) {
                u8g2.drawVLine(15 + x, 32, 8);
            }

            u8g2.setFont(u8g2_font_nokiafc22_tu);
            u8g2.setCursor(14, 51);
            u8g2.print(F("SQUELCH LEVEL:"));
            u8g2.setCursor( U8G2_HOR_ALIGN_RIGHT(String(bar).c_str()) - 14, 51 );
            u8g2.print(value);
        } while ( u8g2.nextPage() );

    } while ( btnPressed != LongPress );

    radio.setSquelchLevel(value);

    radio.saveConfigure();
}

void demoPowerLevel(uint8_t menuSelect)
{
    Button btnPressed;
    int value = radio.getStetting().txPower;
    do {
        value           = readRotary(value, 0, 1);
        btnPressed      = readButton();
        u8g2.firstPage();
        do {
            drawFrame();
            drawHeader(menuSelect);

            u8g2.setFont(u8g2_font_nokiafc22_tu);
            u8g2.setCursor(45, 31);
            u8g2.print(F("LOW  1.6W"));
            u8g2.setCursor(45, 44);
            u8g2.print(F("HIGH 1.8W"));

            u8g2.setCursor(19, 31);
            // TX POWER: High and low power settings (0: high power; 1: low power)
            if (value) {
                u8g2.print(F("[ * ]"));
            } else {
                u8g2.print(F("[    ]"));
            }
            u8g2.setCursor(19, 44);
            if (!value) {
                u8g2.print(F("[ * ]"));
            } else {
                u8g2.print(F("[    ]"));
            }
        } while ( u8g2.nextPage() );

        if (btnPressed == ShortPress) {
            beep();
            value ? radio.lowPower() : radio.highPower();
            value = radio.getStetting().txPower;
        }

    } while ( btnPressed != LongPress );

    radio.saveConfigure();
}

void demoFilter(uint8_t menuSelect)
{
    Button btnPressed;
    int value = 0;
    int prevValue = 0;
    bool  emphasis = radio.getStetting().emphasis;
    bool  highPass = radio.getStetting().highPass;
    bool  lowPass = radio.getStetting().lowPass;

    do {
        value       = readRotary(0, 0, 2);
        btnPressed  = readButton();

        u8g2.firstPage();
        do {
            drawFrame();
            drawHeader(menuSelect);

            if (prevValue != value) {
                prevValue = value;
                beep();
            }
            u8g2.setFont(u8g2_font_nokiafc22_tu);

            u8g2.setCursor(14, 31);
            if (emphasis) {
                u8g2.print(F("   *    ")) ;
            } else {
                u8g2.print(F("   -    ")) ;
            }
            u8g2.setCursor(45, 31);
            u8g2.print(F("EMPHASIS")) ;

            u8g2.setCursor(14, 44);
            if (highPass) {
                u8g2.print(F("   *    ")) ;
            } else {
                u8g2.print(F("   -    ")) ;
            }
            u8g2.setCursor(45, 44);
            u8g2.print(F("HIGH PASS")) ;

            u8g2.setCursor(14, 57);
            if (lowPass) {
                u8g2.print(F("   *    ")) ;
            } else {
                u8g2.print(F("   -    ")) ;
            }
            u8g2.setCursor(45, 57);
            u8g2.print(F("LOW PASS")) ;

            switch (value) {
            case 0:
                u8g2.setCursor(14, 31);
                if (emphasis) {
                    u8g2.print(F("[  *  ]")) ;
                } else {
                    u8g2.print(F("[  -  ]")) ;
                }
                break;
            case 1:
                u8g2.setCursor(14, 44);
                if (highPass) {
                    u8g2.print(F("[  *  ]")) ;
                } else {
                    u8g2.print(F("[  -  ]")) ;
                }
                break;
            case 2:
                u8g2.setCursor(14, 57);
                if (lowPass) {
                    u8g2.print(F("[  *  ]")) ;
                } else {
                    u8g2.print(F("[  -  ]")) ;
                }
                break;
            default:
                break;
            }

            if (btnPressed == ShortPress) {
                beep();
                switch (value) {
                case 0:
                    radio.enableEmpHassis(!emphasis);
                    emphasis = radio.getStetting().emphasis;
                    break;
                case 1:
                    radio.enableHighPass(!highPass);
                    highPass = radio.getStetting().highPass;
                    break;
                case 2:
                    u8g2.setCursor(14, 57);
                    radio.enableLowPass(!lowPass);
                    lowPass = radio.getStetting().lowPass;
                    break;
                default:
                    break;
                }
            }

        } while ( u8g2.nextPage() );

    } while ( btnPressed != LongPress );

    radio.saveConfigure();
}
 void broadcastAudioTelemetry(uint8_t menuSelect)
{
    if(!SD.begin(SD_CS, SPI, 4000000, "/sd" , 20)){
            DBG("SD BAD");
    }
    Button btnPressed;
    int value = 0; // 0 for Stop, 1 for Play
    bool isPlaying = false;
    bool rslt = false;
   // bool landed = radio.isReceive(); DELETE AND REPLACE WITH twr.isReceiving

    int firstVal = 0;
    int secondVal = 0;
    int divVal = 0;
    int altitude = 0;
    int pressure = 0;
    int temperature = 0;
    int maxAltitude = 0;
    int currentAltitude;
    int maxVelocity = 0;
    int previousAltitude = 0.0;
    int velocity = 0.0;        // Vertical velocity in m/s
    int voltageBattery; // Placeholder for battery percentage
    int survivePercentage = 100.0;
    int timeHour,timeMinutes,landingH,landingM;
    bool  altFlag, veloFlag, batFlag = false;
    uint32_t intervalue = 0;
    float previousTime = 0;
    bool landed = false;
    
    radio.setTxFreq(169420000UL); //147500000UL                 //PRACTICE TRANSMISSION: 146500000UL
    radio.setRxFreq(147500000UL); //169420000UL
    radio.setTxCXCSS(0);
    radio.setRxCXCSS(0);
    radio.setBandWidth(12500000/1000);

    // Just debugging to see the values doesn't effect anything
    DBG(radio.getStetting().transFreq);
    DBG(radio.getStetting().recvFreq);
    DBG(radio.getStetting().bandwidth);
    DBG(radio.getStetting().txCXCSS);
    DBG(radio.getStetting().rxCXCSS);
    DBG(radio.getStetting().type);


    //TEST VARIABLES
    bool cool = true;
    int reset = 3;
    do {
        
        
        if (cool == true) {
            
            //For GPS data
            while (GPSSerial.available()){
                gps.encode(GPSSerial.read());
            }
            // Read sensor data
            temperature = bmp390.temperature;
            
            //GPS Time for TOL
            timeHour = gps.time.hour() - 5;
            timeMinutes = gps.time.minute();
            
            if(twr.isReceiving){
                strip.setPixelColor(0, strip.Color(0, 0, 255));
                    strip.show();
                    counter = counter + 1;
                    delay(500);
                    if(counter > 10){
                    strip.clear();
                    strip.show();
                    }
                
            }
            if(counter > 15){
                counter = 0;
            }
            
            
            voltageBattery  = twr.getBatteryPercent();
            pressure = (bmp390.pressure / 100);
            altitude = bmp390.readAltitude(1011.0);

            currentAltitude = bmp390.readAltitude(1011.0);

            float currentTime = millis();
            float deltaTime = (currentTime - previousTime) / 1000.0;

            if(deltaTime > 0.0f){
                float deltaAltitude = currentAltitude - previousAltitude;
                velocity = deltaAltitude / deltaTime;
            }

            previousAltitude = currentAltitude;
            previousTime = currentTime;

            //For max Altitude
            if (altitude > maxAltitude) {
                maxAltitude = altitude;
            }

            //For max velocity ERROR at start
            if(velocity > 3000 || isinf(velocity)){
                velocity = 0;
            }

            if(velocity > maxVelocity){
                maxVelocity = velocity;
            }

             //For landing time
            if (counter > 6) {
                landed = true;
                landingH = timeHour;
                landingM = timeMinutes;
                if(landingH < 0){
                    landingH = -landingH;
                }
            } else if(counter < 6){
                landed = false;
            }
            
            //For Survivability
            if(altitude > 1625 && !altFlag){
            survivePercentage -= 5.0;
            altFlag = true;
            }
            if(velocity < -10.0 || velocity > 200 && !veloFlag ){
            survivePercentage -= 20.0;
            veloFlag = true;
            }
            if(voltageBattery < 75 && !batFlag){
            survivePercentage = survivePercentage - 30.0;
            batFlag = true;
            }
            
        

        intervalue = millis() + 500;
        
    
        }
        //value = readRotary(value, 0, 1); // Navigate between Stop and Play
        if(counter < 0){
        btnPressed = readButton();
        }
        u8g2.firstPage();
        do {
            drawFrame();
            drawHeader(menuSelect);

            u8g2.setFont(u8g2_font_nokiafc22_tu);
            u8g2.setCursor(14, 31);
            u8g2.print("WAV Player");

            u8g2.setCursor(14, 44);
            u8g2.print(F("[   ]  Stop"));
            u8g2.setCursor(14, 57);
            u8g2.print(F("[   ]  Play"));

            u8g2.setCursor(75, 52);
            u8g2.print(F("C#"));
            u8g2.setCursor(95, 52);
            u8g2.print(counter);

        } while (u8g2.nextPage());

        if (counter > 3) {
            switch(counter){
            
            case 11: 
                    isPlaying = false;
                    rslt = false;
                    radio.receive();
                    twr.routingSpeakerChannel(TWRClass::TWR_RADIO_TO_SPK);
                    DBG("Done .");
                    //file->close();
                    strip.setPixelColor(0, strip.Color(255, 0, 0));
                    strip.show();
                    strip.clear();
                    strip.show();
                    break;
            case 9:
                    strip.setPixelColor(0, strip.Color(0, 0, 255));
                    strip.show();
                    strip.clear();
                    strip.show();
                    isPlaying = true;
                    DBG("Case 1 Selected");
                    
                 //twr.enableRadio(); I dont think this does anything

                // Route the ESP32 PDM PIN to the power amplifier
                twr.routingSpeakerChannel(TWRClass::TWR_ESP_TO_SPK);


                DBG("Initialize Audio");

            wav  = new AudioGeneratorWAV();
            file = new AudioFileSourceSD();
            out  = new AudioOutputI2S(0, AudioOutputI2S::INTERNAL_PDM);

            radio.setPins(SA868_PTT_PIN, SA868_PD_PIN);
            rslt = radio.begin(RadioSerial, twr.getBandDefinition());

            out->SetPinout(I2S_PIN_NO_CHANGE, I2S_PIN_NO_CHANGE, ESP32_PWM_TONE, I2S_PIN_NO_CHANGE);

            out->SetOutputModeMono(true);

            out->SetGain(.8);
            
            if(landed && reset != 0){
            radio.transmit();
            delay(1000);

            playWavFile(WAV_CALLSIGN);
            playWavFile(WAV_TEMP);
            playVariableDigits(temperature);  // Play temperature digits
            playWavFile(WAV_DEG);
            playWavFile(WAV_ALT);
            playVariableDigits(maxAltitude);  // Play altitude digits
            playWavFile(WAV_MET);
            playWavFile(WAV_MAXVELO);
            playVariableDigits(maxVelocity);  // Play velocity digits
            playWavFile(WAV_MPS);
            playWavFile(WAV_BATPERC);
            playVariableDigits(voltageBattery);  // Play battery digits
            playWavFile(WAV_ALIPERC);
            playWavFile(WAV_CREW);
            playVariableDigits(survivePercentage);  // Play SOC digits
            playWavFile(WAV_ALEXPERC);
            playWavFile(WAV_TOL); // Time of landing
            playVariableDigits(landingH);  // Play TOL digits
            playVariableDigits(landingM);
            playWavFile(WAV_CALLSIGN);
            
           
            radio.receive(); 
            delay(2000);
            --reset;
                    break;
            } else {
                counter = -2;
                reset = 1;
                break;
            }
            radio.saveConfigure();
        }
         
     } 

    } while (btnPressed != LongPress);
    
}  

// NEW FUNC
void playWavFile(const char* fileName){
bool rslt = false;

file->open(fileName);
rslt = wav->begin(file, out);
if (!rslt) {
        DBG("wav begin failed.");
        return;
    }

    twr.routingMicrophoneChannel(TWRClass:: TWR_MIC_TO_RADIO);

    while(1) {
        strip.setPixelColor(0, strip.Color(0, 255, 0));
        strip.show();
        
        // Start microphone routing (if required)
         
        
        if (!wav->loop()) {
            wav->stop();
            file->close();
            strip.clear();
            strip.show();
            break;
        }
    }

}

void playVariableDigits(int variable) {
    if (variable == 0) {
        playDigitWAV(0);  // Special case for zero
        return;
    }

    // Convert the number to a string to preserve its exact format
    std::string str = std::to_string(variable);

    // Play each digit in reversed order
    for (char digitChar : str) {
        int digit = digitChar - '0';  // Convert the character to an integer
        playDigitWAV(digit);          // Play the corresponding WAV file
    }
}


void playDigitWAV(int digit) {
    const char* wavFiles[] = {
        "/NUMBERS/ZERO.wav", "/NUMBERS/ONE.wav", "/NUMBERS/TWO.wav", "/NUMBERS/THREE.wav",
        "/NUMBERS/FOUR.wav", "/NUMBERS/FIVE.wav", "/NUMBERS/SIX.wav", "/NUMBERS/SEVEN.wav",
        "/NUMBERS/EIGHT.wav", "/NUMBERS/NINE.wav"
    };

    if (digit >= 0 && digit <= 9) {
        playWavFile(wavFiles[digit]);
    }
}


//
void demoDevInfo(uint8_t menuSelect)
{
    Button btnPressed;
    RadioType  type = radio.getStetting().type;
    String  radioType = type == SA8X8_UHF ? "UHF" : "VHF";
    String fwType = radio.firmwareType();
    int value = type;

    TWRClass::TWR_Version _ver = twr.getVersion() ;
    do {
        btnPressed  = readButton();
        value       = readRotary(type, 0, 2);

        u8g2.firstPage();
        do {
            drawFrame();
            drawHeader(menuSelect);
            u8g2.setFont(u8g2_font_nokiafc22_tu);

            u8g2.setCursor(14, 31);
            u8g2.print(F("REV:"));
            String version = _ver == TWRClass::TWR_REV2V0 ? "2.0" : "2.1";
            u8g2.setCursor( U8G2_HOR_ALIGN_RIGHT((version).c_str()) - 22, 31 );
            u8g2.print(version);

            u8g2.setCursor(14, 44);
            u8g2.print(F("FW:"));
            fwType.toUpperCase();
            u8g2.setCursor( U8G2_HOR_ALIGN_RIGHT(fwType.c_str()) - 15, 44 );
            u8g2.print(fwType);


            u8g2.setCursor(14, 57);
            u8g2.print(F("BAND:"));

            if (_ver == TWRClass::TWR_REV2V1) {
                u8g2.setCursor( U8G2_HOR_ALIGN_RIGHT((radioType).c_str()) - 22, 57);
                u8g2.print(radioType);
            } else {
                switch (value) {
                case SA8X8_VHF:
                    radioType = "VHF";
                    break;
                case SA8X8_UHF:
                    radioType = "UHF";
                    break;
                default:
                    break;
                }
                u8g2.drawRBox(83, 48, 30, 12, 2);
                u8g2.setColorIndex(0);
                u8g2.setCursor( U8G2_HOR_ALIGN_RIGHT((radioType).c_str()) - 22, 57);
                u8g2.print(radioType);
                u8g2.setColorIndex(1);
            }
        } while ( u8g2.nextPage() );

        if (btnPressed == ShortPress ) {
            beep();
            if (radioType == "UHF") {
                radio.setRadioType(SA8X8_UHF);
            } else if (radioType == "VHF") {
                radio.setRadioType(SA8X8_VHF);
            }
        }

    } while ( btnPressed != LongPress );
}


Button readButton()
{
    Button btnPressed   = Unknown;
    // Handle key events
    for (uint8_t i = 0; i < COUNT(buttonPins); i++) {
        buttons[i].check();
    }

    // Handling PMU events
    twr.tick();

    if (state != btnPressed) {
        btnPressed = state;
        state = Unknown;
    }
    return btnPressed;
}


uint32_t readRotary(uint32_t cur, uint32_t minOut, uint32_t maxOut, uint32_t steps)
{
    static uint32_t lastMin, lastMax, lastSteps;
    static uint32_t readPotValue;

    if (lastMin != minOut || lastMax != maxOut || lastSteps != steps) {
        setRotaryValue(cur, minOut, maxOut, steps);
        lastMin = minOut;
        lastMax = maxOut;
        lastSteps = steps;
        return cur;
    }

    xQueueReceive(rotaryMsg, &readPotValue, pdMS_TO_TICKS(50));
    return readPotValue;
}

void drawFrame()
{
    u8g2.drawRFrame(0, 0, 128, 64, 5);
}

void drawHeader( uint8_t menuSelect )
{
    u8g2.drawHLine(2, 1, 124);
    u8g2.drawHLine(1, 2, 126);
    u8g2.drawHLine(0, 3, 128);
    u8g2.drawBox(0, 4, 128, 14);

    u8g2.setColorIndex(0);
    u8g2.drawXBMP(4, 1,  16, 16, demo[menuSelect].demo_icon);
    u8g2.setFont(u8g2_font_nokiafc22_tu);
    u8g2.drawStr(22, 12, demo[menuSelect].demo_item);
    u8g2.setColorIndex(1);
}


void beep()
{
    if (twr.isEnableBeep()) {
        tone(45, 800, 1000);
    }
}


void drawError(uint8_t menuSelect)
{
    Button     btnPressed;
    do {
        btnPressed = readButton();
        u8g2.firstPage();
        do {
            drawFrame();
            drawHeader(menuSelect);
            u8g2.setFont(u8g2_font_nokiafc22_tu);
            u8g2.setCursor(14, 44);
            u8g2.print(F("REV2.0 NOT SUPPORT"));
        } while ( u8g2.nextPage() );
    } while ( btnPressed != LongPress );
}

