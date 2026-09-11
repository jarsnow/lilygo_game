#include "Adafruit_Sensor.h"
#include "Arduino.h"
#include <TFT_eSPI.h> /* Please use the TFT library provided in the library. */
#include "HWCDC.h"

#include "box2d/b2_body.h"
#include "box2d/b2_circle_shape.h"
#include "box2d/b2_common.h"
#include "box2d/b2_fixture.h"
#include "box2d/b2_math.h"
#include "box2d/b2_polygon_shape.h"
#include "box2d/b2_types.h"
#include "box2d/b2_world.h"
#include "box2d/box2d.h"

#include <Adafruit_MPU6050.h>

#include "esp32-hal-cpu.h"
#include "esp32-hal.h"

#include "pin_config.h"

#include "orb.h"
#include "drawn_orb.h"

#include <cmath>
#include <cstdint>
# include <cstdlib>
#include <cstring>
#include <iterator>
#include <vector>


/* The product now has two screens, and the initialization code needs a small change in the new version. The LCD_MODULE_CMD_1 is used to define the switch macro. */
#define LCD_MODULE_CMD_1
#define SCREEN_WIDTH_PX 320
#define SCREEN_HEIGHT_PX 170
// box2d setup
// used for sizing, no matter the size of the orb, it will always be 1meter x 1meter in box2d
#define IMAGE_DIAMETER 64
#define ORB_DIAMETER 45
#define ORB_COUNT 15
#define WALL_THICKNESS 5 // represents how thick the wall should be

#define LIGHT_RING_COUNT 4
const uint16 BACKGROUND_COLOR = 0b0001000010000010; // 0x20412c
const uint16 LIGHT_COLOR = 0b0100011010110000;//0x43d783
uint16 light_values[LIGHT_RING_COUNT] = {0};

// represents the size of the screen in box2d's meter dimensions
float SCREEN_WIDTH = (float) SCREEN_HEIGHT_PX / ORB_DIAMETER;
float SCREEN_HEIGHT = (float) SCREEN_WIDTH_PX / ORB_DIAMETER;

b2World* world;
b2Body* test_orbs[ORB_COUNT];
b2Body* walls[4];

b2Vec2 upper_wall_starting_point;

// testing only
b2Vec2 wall_velocity_test = b2Vec2(10, 10);

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite screen = TFT_eSprite(&tft);
TFT_eSprite temp_orb_sprite = TFT_eSprite(&tft);

// must make this pointers
TFT_eSprite* test_orb_animation[5];

// light map as 1d array
uint8* light_map = new uint8[SCREEN_WIDTH_PX * SCREEN_HEIGHT_PX];

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

    leftWallDef.type = b2_kinematicBody;
    rightWallDef.type = b2_kinematicBody;
    upperWallDef.type = b2_kinematicBody;
    lowerWallDef.type = b2_kinematicBody;

    // set the left/right wall's shape to left/right of the center of the screen
    leftWallDef.position.Set(-0.5 * (SCREEN_WIDTH + WALL_THICKNESS), 0);
    rightWallDef.position.Set(0.5 * (SCREEN_WIDTH + WALL_THICKNESS), 0);
    upperWallDef.position.Set(0, 0.5 * (SCREEN_HEIGHT + WALL_THICKNESS));
    lowerWallDef.position.Set(0, -0.5 * (SCREEN_HEIGHT + WALL_THICKNESS));

    upper_wall_starting_point = b2Vec2(0, 0.5 * (SCREEN_HEIGHT + WALL_THICKNESS));

    b2Body* leftWallBody = world->CreateBody(&leftWallDef);
    b2Body* rightWallBody = world->CreateBody(&rightWallDef);
    b2Body* upperWallBody = world->CreateBody(&upperWallDef);
    b2Body* lowerWallBody = world->CreateBody(&lowerWallDef);

    walls[0] = leftWallBody;
    walls[1] = rightWallBody;
    walls[2] = upperWallBody;
    walls[3] = lowerWallBody;

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
    orbFixtureDef.friction = 0.3f;

    // create the orb
    for (size_t i = 0; i < ORB_COUNT; i++) {
        test_orbs[i]->CreateFixture(&orbFixtureDef);
    }
}

int get_orb_center_pixel_x(b2Body* orb)
{
    b2Vec2 moved_diff = walls[2]->GetPosition() - upper_wall_starting_point;
    b2Vec2 position = orb->GetPosition() - moved_diff;
    return position.x * ORB_DIAMETER + (SCREEN_HEIGHT_PX / 2);
}

int get_orb_center_pixel_y(b2Body* orb)
{
    b2Vec2 moved_diff = walls[2]->GetPosition() - upper_wall_starting_point;
    b2Vec2 position = orb->GetPosition() - moved_diff;
    return position.y * ORB_DIAMETER + (SCREEN_WIDTH_PX / 2);
}

void draw_circle()
{
    // make sure when drawing things to the screen, the coordinates are translated over correctly
    // 0,0 is center of the display
    for (b2Body* single_orb : test_orbs) {
        // need to draw the orb image to a sprite, then draw that sprite to the main screen sprite

        // push image to temp sprite
        temp_orb_sprite.pushImage(0, 0, IMAGE_DIAMETER, IMAGE_DIAMETER, drawn_orb);

        b2Vec2 moved_diff = walls[2]->GetPosition() - upper_wall_starting_point;

        // subtract the distance that the wall has moved to get relative position
        b2Vec2 position = single_orb->GetPosition() - moved_diff;
        int px = position.x * ORB_DIAMETER + (SCREEN_HEIGHT_PX / 2) - (IMAGE_DIAMETER / 2);
        int py = position.y * ORB_DIAMETER + (SCREEN_WIDTH_PX / 2) - (IMAGE_DIAMETER / 2);

        // to translate a vertical px,py to horizontal r/c, px-> row, py -> col
        // push sprite to screen sprite
        const unsigned short pink = 0b1111100000011111;
        temp_orb_sprite.pushToSprite(&screen, py, px, pink);
    }
}

void setup_light_ring_colors()
{
    // get background color R,G,B values
    uint16 b_red = BACKGROUND_COLOR &   0b1111100000000000;
    b_red >>= 11;
    uint16 b_green = BACKGROUND_COLOR & 0b0000011111100000;
    b_green >>= 5;
    uint16 b_blue = BACKGROUND_COLOR &  0b0000000000011111;

    // get light color R,G,B values
    uint16 l_red = LIGHT_COLOR &   0b1111100000000000;
    l_red >>= 11;
    uint16 l_green = LIGHT_COLOR & 0b0000011111100000;
    l_green >>= 5;
    uint16 l_blue = LIGHT_COLOR &  0b0000000000011111;

    // create LIGHT_RING_COUNT colors
    // linearly interpolate between background to light
    for (size_t i = 0; i < LIGHT_RING_COUNT; i++) {
        float light_ratio = (float) (LIGHT_RING_COUNT - i) / LIGHT_RING_COUNT;

        uint16 r_red = b_red * (1 - light_ratio) + l_red * light_ratio;
        uint16 r_green = b_green * (1 - light_ratio) + l_green * light_ratio;
        uint16 r_blue = b_blue * (1 - light_ratio) + l_blue * light_ratio;

        uint16 result_color = (r_red << 11) + (r_green << 5) + r_blue;
        light_values[i] = result_color;
    }
}

// draw the area of light around each orb
// draw multiple circles around each orb, 
void draw_light()
{
    int orb_radius = ORB_DIAMETER / 2;
    screen.fillSprite(BACKGROUND_COLOR);

    const uint extra_light_radius = 40;
    uint radius_per_step = extra_light_radius / LIGHT_RING_COUNT;

    // get x,y center coords for each orb
    int orbs_x[ORB_COUNT];
    int orbs_y[ORB_COUNT];

    // store current positions for each orb
    for (size_t i = 0; i < ORB_COUNT; i++) {
        b2Body* orb = test_orbs[i];

        orbs_x[i] = get_orb_center_pixel_x(orb);
        orbs_y[i] = get_orb_center_pixel_y(orb);
    }

    // draw all large, darker light first, for each orb
    // then draw lighter orbs next
    for (size_t i = 0; i < LIGHT_RING_COUNT; i++) {
        for (size_t j = 0; j < ORB_COUNT; j++) {
            screen.fillCircle(orbs_y[j], orbs_x[j], orb_radius + (radius_per_step * (LIGHT_RING_COUNT - i)), light_values[LIGHT_RING_COUNT - i]);
        }
    }

}

void mpu_test()
{
    Wire.begin(43, 44);
    Wire.setClock(50000);
    Adafruit_MPU6050 mpu = Adafruit_MPU6050();

    bool res = mpu.begin(MPU6050_I2CADDR_DEFAULT);

    while (res) {
        sensors_event_t accel, gyro, temp;
        mpu.getEvent(&accel, &gyro, &temp);

        // Serial.print("X: "); Serial.println(accel.acceleration.x);
        // Serial.print("Y: "); Serial.println(accel.acceleration.y);
        // Serial.print("Z: "); Serial.println(accel.acceleration.z);

        float roll  = atan2(accel.acceleration.y, accel.acceleration.z) * 180.0 / PI;
        float pitch = atan2(-accel.acceleration.x, sqrt(accel.acceleration.y * accel.acceleration.y + accel.acceleration.z * accel.acceleration.z)) * 180.0 / PI;

        Serial.print("Roll: "); Serial.println(roll);
        Serial.print("Pitch: "); Serial.println(pitch);

        delay(100);
    }

    Serial.println("failed");
}

void setup()
{
    pinMode(PIN_POWER_ON, OUTPUT);
    digitalWrite(PIN_POWER_ON, HIGH);

    Serial.begin(115200);
    Serial.println("Hello T-Display-S3");
    delay(1000);

    // TODO: remove
    mpu_test();

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


    screen.createSprite(tft.width(), tft.height());
    temp_orb_sprite.createSprite(IMAGE_DIAMETER, IMAGE_DIAMETER);
    
    // init all sprites for animation testing
    for (TFT_eSprite* sprite_frame : test_orb_animation){
        sprite_frame = new TFT_eSprite(&tft);
    }
    setup_light_ring_colors();
    setup_world();

}

void loop()
{
    setup_light_ring_colors();
    ulong start_time = millis();

    const uint16 BACKGROUND_COLOR = 0b0001000010000010; // 0x20412c
    screen.fillSprite(BACKGROUND_COLOR);

    draw_light();
    draw_circle();


    world->Step(1.0f/40.0f, 6, 2);

    screen.pushSprite(0, 0);
    iterations++;


    // test gravity by setting it to a random direction every once in a while
    if (iterations % 15 == 0){
        // get random number -10 to 10
        int random_x = (rand() % 21) - 10;
        int random_y = (rand() % 21) - 10;
        b2Vec2 new_gravity(random_x, random_y);
        //world->SetGravity(new_gravity);

        // awake all orbs
        for (b2Body* orb : test_orbs) {
            orb->SetAwake(true);
        }

        // update wall velocity
        for (b2Body* wall : walls){
            wall->SetLinearVelocity(wall_velocity_test);
        }

        wall_velocity_test *= -1;
    }

    ulong end_time = millis();
    ulong draw_time = end_time - start_time;
    Serial.print("Draw time: ");
    Serial.println(draw_time);
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
