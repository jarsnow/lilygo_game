#include "Arduino.h"
#include <TFT_eSPI.h> /* Please use the TFT library provided in the library. */
#include "box2d/b2_math.h"
#include "box2d/b2_world.h"
#include "esp32-hal-cpu.h"
#include "esp32-hal.h"
#include "pin_config.h"
#include "box2d/box2d.h"

#include "vex_swapped.h"
#include "leblanc_swapped.h"


/* The product now has two screens, and the initialization code needs a small change in the new version. The LCD_MODULE_CMD_1 is used to define the switch macro. */
#define LCD_MODULE_CMD_1

// used for sizing, no matter the size of the orb, it will always be 1meter x 1meter in box2d
#define ORB_DIAMETER = 45

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite screen = TFT_eSprite(&tft);

#define WAIT 15
unsigned long targetTime = 0; // Used for testing draw times
size_t iterations = 0;

#if defined(LCD_MODULE_CMD_1)
typedef struct {
    uint8_t cmd;
    uint8_t data[14];
    uint8_t len;
} lcd_cmd_t;

lcd_cmd_t lcd_st7789v[] = {
    {0x11, {0}, 0 | 0x80},
    {0x3A, {0X05}, 1},
    {0xB2, {0X0B, 0X0B, 0X00, 0X33, 0X33}, 5},
    {0xB7, {0X75}, 1},
    {0xBB, {0X28}, 1},
    {0xC0, {0X2C}, 1},
    {0xC2, {0X01}, 1},
    {0xC3, {0X1F}, 1},
    {0xC6, {0X13}, 1},
    {0xD0, {0XA7}, 1},
    {0xD0, {0XA4, 0XA1}, 2},
    {0xD6, {0XA1}, 1},
    {0xE0, {0XF0, 0X05, 0X0A, 0X06, 0X06, 0X03, 0X2B, 0X32, 0X43, 0X36, 0X11, 0X10, 0X2B, 0X32}, 14},
    {0xE1, {0XF0, 0X08, 0X0C, 0X0B, 0X09, 0X24, 0X2B, 0X22, 0X43, 0X38, 0X15, 0X16, 0X2F, 0X37}, 14},
};
#endif

void setup()
{
    pinMode(PIN_POWER_ON, OUTPUT);
    digitalWrite(PIN_POWER_ON, HIGH);

    Serial.begin(115200);
    Serial.println("Hello T-Display-S3");

    // setCpuFrequencyMhz(80);

    tft.begin();

#if defined(LCD_MODULE_CMD_1)
    for (uint8_t i = 0; i < (sizeof(lcd_st7789v) / sizeof(lcd_cmd_t)); i++) {
        tft.writecommand(lcd_st7789v[i].cmd);
        for (int j = 0; j < (lcd_st7789v[i].len & 0x7f); j++) {
            tft.writedata(lcd_st7789v[i].data[j]);
        }

        if (lcd_st7789v[i].len & 0x80) {
            delay(120);
        }
    }
#endif


    tft.setRotation(3);
    tft.setSwapBytes(true);

    pinMode(PIN_LCD_BL, OUTPUT);
    digitalWrite(PIN_LCD_BL, HIGH);


    pinMode(PIN_LCD_BL, OUTPUT);
    digitalWrite(PIN_LCD_BL, HIGH);
    // First we test them with a background colour set

    // test, wait here so I have a chance to monitor
    delay(10 * 1000);

    screen.createSprite(tft.width(), tft.height());
    
    // test for box2d
    b2Vec2 gravity(0.0f, -9.8f);
    b2World world(gravity);

    // bodies are first defined with position
    b2BodyDef groundBodyDef;
    groundBodyDef.position.Set(0.0f, -10.0f);

    // then created using world object
    b2Body* groundBody = world.CreateBody(&groundBodyDef);

    // an object needs its fixture of its shape defined
    // a fixture is simply a subsection of an object
    b2PolygonShape groundBox;
    groundBox.SetAsBox(50.0f, 10.0f);
    // then these fixtures are created onto the body
    groundBody->CreateFixture(&groundBox, 0.0f);

    
    // creating dynamic body
    b2BodyDef bodyDef;
    // b2_dynamicBody is used if the body is to move in response to forces
    bodyDef.type = b2_dynamicBody;
    bodyDef.position.Set(0.0f, 4.0f);
    b2Body* body = world.CreateBody(&bodyDef);

    b2PolygonShape dynamicBox;
    dynamicBox.SetAsBox(1.0f, 1.0f);

    b2FixtureDef fixtureDef;
    fixtureDef.shape = &dynamicBox;
    fixtureDef.density = 1.0f;
    fixtureDef.friction = 0.3f;

    body->CreateFixture(&fixtureDef);


    // simulating world
    float timeStep = 1.0f / 60.0f;
    int32 velocityIterations = 6;
    int32 positionIterations = 2;

    for (int32 i = 0; i < 60; ++i)
    {
        world.Step(timeStep, velocityIterations, positionIterations);
        b2Vec2 position = body->GetPosition();
        float angle = body->GetAngle();
        Serial.print("x position: ");
        Serial.println(position.x);

        Serial.print("y position: ");
        Serial.println(position.y);

        Serial.print("angle: ");
        Serial.println(angle);
        delay(timeStep * 1000);
    }
}

void loop()
{
    targetTime = millis();

    for (size_t i = 0; i < 50; i++) {
        // write vex image to sprite, then to screen
        screen.fillSprite(TFT_BLACK);
        screen.fillRect(50 + 5 * i, 50, 50, 50, TFT_BLUE);
        //screen.pushSprite(0, 0);
        screen.pushSprite(50, 50, 50, 50, 200, 100);
        delay(WAIT);
    }


    //Serial.print("cpu frequency (MHz): ");
    // Serial.println(spiGet());

    iterations++;
}

void draw_circle()
{
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
#endif
