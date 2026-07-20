#include "Arduino.h"
#include <TFT_eSPI.h> /* Please use the TFT library provided in the library. */
#include "box2d/b2_body.h"
#include "box2d/b2_circle_shape.h"
#include "box2d/b2_common.h"
#include "box2d/b2_fixture.h"
#include "box2d/b2_math.h"
#include "box2d/b2_polygon_shape.h"
#include "box2d/b2_world.h"
#include "esp32-hal-cpu.h"
#include "esp32-hal.h"
#include "pin_config.h"
#include "box2d/box2d.h"


#include "orb.h"
#include "vex_swapped.h"

#include <cmath>
# include <cstdlib>


/* The product now has two screens, and the initialization code needs a small change in the new version. The LCD_MODULE_CMD_1 is used to define the switch macro. */
#define LCD_MODULE_CMD_1
#define SCREEN_WIDTH_PX 320
#define SCREEN_HEIGHT_PX 170
// box2d setup
// used for sizing, no matter the size of the orb, it will always be 1meter x 1meter in box2d
#define ORB_DIAMETER 45
#define ORB_COUNT 15
#define WALL_THICKNESS 5 // represents how thick the wall should be

// represents the size of the screen in box2d's meter dimensions
float SCREEN_WIDTH = (float) SCREEN_HEIGHT_PX / ORB_DIAMETER;
float SCREEN_HEIGHT = (float) SCREEN_WIDTH_PX / ORB_DIAMETER;

b2World* world;
b2Body* test_orbs[ORB_COUNT];

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

void setup_world()
{
    // create world
    b2Vec2 gravity(2.0f, -9.8f);
    world = new b2World(gravity);


    // left and right wall fixtures
    // remember that the given values for SetAsBox are half-values
    b2PolygonShape leftWall;
    b2PolygonShape rightWall;
    b2PolygonShape upperWall;
    b2PolygonShape lowerWall;

    float side_wall_width = 0.5 * WALL_THICKNESS;
    float side_wall_height = 0.5 * (SCREEN_HEIGHT) + WALL_THICKNESS;

    // also used for the top boundary but I can't think of a good name for both
    float floor_width = 0.5 * SCREEN_WIDTH;
    float floor_height = 0.5 * WALL_THICKNESS;

    leftWall.SetAsBox(side_wall_width, side_wall_height);
    rightWall.SetAsBox(side_wall_width, side_wall_height);
    upperWall.SetAsBox(floor_width, floor_height);
    lowerWall.SetAsBox(floor_width, floor_height);

    // create body to represent 4 borders
    // one body, four fixtures
    b2BodyDef leftWallDef;
    b2BodyDef rightWallDef;
    b2BodyDef upperWallDef;
    b2BodyDef lowerWallDef;

    // set the left/right wall's shape to left/right of the center of the screen
    leftWallDef.position.Set(-0.5 * (SCREEN_WIDTH + WALL_THICKNESS), 0);
    rightWallDef.position.Set(0.5 * (SCREEN_WIDTH + WALL_THICKNESS), 0);
    upperWallDef.position.Set(0, 0.5 * (SCREEN_HEIGHT + WALL_THICKNESS));
    lowerWallDef.position.Set(0, -0.5 * (SCREEN_HEIGHT + WALL_THICKNESS));

    b2Body* leftWallBody = world->CreateBody(&leftWallDef);
    b2Body* rightWallBody = world->CreateBody(&rightWallDef);
    b2Body* upperWallBody = world->CreateBody(&upperWallDef);
    b2Body* lowerWallBody = world->CreateBody(&lowerWallDef);

    leftWallBody->CreateFixture(&leftWall, 0.0f);
    rightWallBody->CreateFixture(&rightWall, 0.0f);
    upperWallBody->CreateFixture(&upperWall, 0.0f);
    lowerWallBody->CreateFixture(&lowerWall, 0.0f);

    // create dynamic bodies here
    b2BodyDef orbDef;
    orbDef.type = b2_dynamicBody;

    // create 5 balls
    for (size_t i = 0; i < ORB_COUNT; i++) {
        orbDef.position.Set(i * 0.1f, 0.1f);
        test_orbs[i] = world->CreateBody(&orbDef);
    }

    // radius must be 0.5
    b2CircleShape orbShape;
    orbShape.m_radius = 0.5f;

    // create fixture
    b2FixtureDef orbFixtureDef;
    orbFixtureDef.shape = &orbShape;
    orbFixtureDef.density = 1.0f;
    orbFixtureDef.friction = 0.1f;

    // create the orb
    for (size_t i = 0; i < ORB_COUNT; i++) {
        test_orbs[i]->CreateFixture(&orbFixtureDef);
    }
}

void draw_circle()
{
    // make sure when drawing things to the screen, the coordinates are translated over correctly
    // 0,0 is center of the display
    for (size_t i = 0; i < ORB_COUNT; i++) {
        // need to draw the orb image to a sprite, then draw that sprite to the main screen sprite

        // push image to temp sprite
        TFT_eSprite temp_orb_sprite = TFT_eSprite(&tft);
        temp_orb_sprite.createSprite(45, 45);
        temp_orb_sprite.pushImage(0, 0, 45, 45, orb);

        b2Vec2 position = test_orbs[i]->GetPosition();
        int px = position.x * ORB_DIAMETER + (SCREEN_HEIGHT_PX / 2) - (ORB_DIAMETER / 2);
        int py = position.y * ORB_DIAMETER + (SCREEN_WIDTH_PX / 2) - (ORB_DIAMETER / 2);

        // to translate a vertical px,py to horizontal r/c, px-> row, py -> col
        // push sprite to screen sprite
        const unsigned short pink = 0b1111100000011111;
        temp_orb_sprite.pushToSprite(&screen, py, px, pink);
    }
}

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


    screen.createSprite(tft.width(), tft.height());

    setup_world();
}

void loop()
{
    targetTime = millis();

    screen.fillSprite(TFT_BLACK);
    draw_circle();
    world->Step(1.0f/60.0f, 6, 2);

    screen.pushSprite(0, 0);
    iterations++;

    // test gravity by setting it to a random direction every once in a while
    if (iterations % 75 == 0){
        // get random number -10 to 10
        int random_x = (rand() % 21) - 10;
        int random_y = (rand() % 21) - 10;
        b2Vec2 new_gravity(random_x, random_y);
        world->SetGravity(new_gravity);

        // awake all orbs
        for (b2Body* orb : test_orbs) {
            orb->SetAwake(true);
        }
    }
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
