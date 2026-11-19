#include "Adafruit_Sensor.h"
#include "Arduino.h"

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

#include "orb.h"
#include "drawn_orb.h"

// maybe switch these files to a 2d matrix instead of just one row of sprites
#include "orb_3_sprites.h"
#include "orb_4_sprites.h"
#include "leblanc.h" // test TODO: remove

#include <cmath>
#include <cstdint>
# include <cstdlib>
#include <cstring>
#include <iterator>
#include <vector>

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

class LGFX : public lgfx::LGFX_Device {
    lgfx::Panel_ST7789 _panel;
    lgfx::Bus_SPI      _bus;
    lgfx::Light_PWM    _light;

public:
LGFX() {
    {
        auto cfg = _bus.config();
        cfg.spi_host   = SPI2_HOST;
        cfg.spi_mode   = 0;
        cfg.freq_write = 40000000;
        cfg.pin_sclk   = 7;     // D8
        cfg.pin_mosi   = 9;     // D10
        cfg.pin_miso   = -1;    // unused
        cfg.pin_dc     = 2;     // D1
        _bus.config(cfg);
        _panel.setBus(&_bus);
    }
    {
        auto cfg = _panel.config();
        cfg.pin_cs       = 3;   // D2
        cfg.pin_rst      = 1;   // D0
        cfg.panel_width  = 240;
        cfg.panel_height = 280;
        cfg.invert       = true;
        cfg.offset_y = 20; // get rid of pixel rows of noise at the bottom
        _panel.config(cfg);
    }
    {
        auto cfg = _light.config();
        cfg.pin_bl      = 4;    // D3
        cfg.invert      = false;
        cfg.freq        = 44100;
        cfg.pwm_channel = 7;
        _light.config(cfg);
        _panel.setLight(&_light);
    }
        setPanel(&_panel);
    }
};


/* The product now has two screens, and the initialization code needs a small change in the new version. The LCD_MODULE_CMD_1 is used to define the switch macro. */
#define LCD_MODULE_CMD_1
#define SCREEN_WIDTH_PX 240
#define SCREEN_HEIGHT_PX 280
// box2d setup
// used for sizing, no matter the size of the orb, it will always be 1meter x 1meter in box2d
#define IMAGE_DIAMETER 50
#define ORB_DIAMETER 45
#define ORB_COUNT 15
#define WALL_THICKNESS 5 // represents how thick the wall should be

Adafruit_MPU6050 mpu;

#define LIGHT_RING_COUNT 5
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

// replacement for tft_eSPI
LGFX tft;

LGFX_Sprite screen(&tft);
LGFX_Sprite temp_orb_sprite(&tft);
LGFX_Sprite leblanc_sprite(&tft);

// must make this pointers
// 2 sets of 4 frames per orb
LGFX_Sprite* orb_animation[2][4];

// light map as 1d array
uint8* light_map = new uint8[SCREEN_WIDTH_PX * SCREEN_HEIGHT_PX];


void setup_world()
{
    // create world
    b2Vec2 gravity(0.0f, -9.8f);
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
        b2Vec2 moved_diff = walls[2]->GetPosition() - upper_wall_starting_point;

        // subtract the distance that the wall has moved to get relative position
        b2Vec2 position = single_orb->GetPosition() - moved_diff;
        int px = position.x * ORB_DIAMETER + (SCREEN_HEIGHT_PX / 2) - (IMAGE_DIAMETER / 2);
        int py = position.y * ORB_DIAMETER + (SCREEN_WIDTH_PX / 2) - (IMAGE_DIAMETER / 2);

        // to translate a vertical px,py to horizontal r/c, px-> row, py -> col
        // push sprite to screen sprite
        const unsigned short pink = 0b1111100000011111;
        temp_orb_sprite.pushSprite(&screen, py, px, pink);
    }
}

// pass in arrays in orb_3_sprites.h, orb_4_sprites.h
void setup_single_sheet_matrix(const unsigned short *array, size_t sprite_num)
{
    // only used for a single header file
    // input is an 10_000 length array, in row major order
    // index of a single pixel is r * 200 + c, in global terms
    // index of a single pixel, relative to orb i is (r * 200) + (i * 50 + c)
    // for each orb in each sprite sheet, extract the sub-matrix and draw it to the sprite
    for (size_t i = 0; i < 4; i++) {

        unsigned short orb_image[2500] = {0};

        for (size_t r = 0; r < 50; r++) {
            for (size_t c = 0; c < 50; c++) {
                size_t pixel_index = (r * 200) + (i * 50 + c);
                orb_image[r * 50 + c] = array[pixel_index];
            }
        }

        // save to sprite
        orb_animation[sprite_num][i]->pushImage(0, 0, IMAGE_DIAMETER, IMAGE_DIAMETER, orb_image);
    }
}

void setup_sprite_animations()
{
    setup_single_sheet_matrix(orb_3_sprites, 0);
    setup_single_sheet_matrix(orb_4_sprites, 0);
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

void mpu_setup()
{
    Wire.begin(5, 6);
    Wire.setClock(50000);
    mpu = Adafruit_MPU6050();

    bool res = mpu.begin(MPU6050_I2CADDR_DEFAULT);

    if (!res) {
        Serial.println("mpu setup failed");
        exit(1);
    }
}

float roll_from_mpu_accel(sensors_event_t accel) {
    float roll = atan2(accel.acceleration.y, accel.acceleration.z);
    return roll;
}

float pitch_from_mpu_accel(sensors_event_t accel) {
    float pitch = atan2(-accel.acceleration.x, sqrt(accel.acceleration.y * accel.acceleration.y + accel.acceleration.z * accel.acceleration.z));
    return pitch;
}

float horizontal_accel_from_mpu_accel(sensors_event_t accel) {
    return accel.acceleration.y;
}

float vertical_accel_from_mpu_accel(sensors_event_t accel) {
    return accel.acceleration.z - 9.8;
}

void setup()
{

    Serial.begin(115200);
    Serial.println("Hello now xiao esp32s3 seeed");

    tft.init();
    tft.setRotation(0);
    tft.setBrightness(255);

    mpu_setup();
    delay(1000);

    // // setCpuFrequencyMhz(80);
    screen.createSprite(SCREEN_WIDTH_PX, SCREEN_HEIGHT_PX);

    leblanc_sprite.createSprite(SCREEN_WIDTH_PX, SCREEN_HEIGHT_PX);
    leblanc_sprite.pushImage(0, 0, SCREEN_WIDTH_PX, SCREEN_HEIGHT_PX, leblanc);

    // temp_orb_sprite.createSprite(IMAGE_DIAMETER, IMAGE_DIAMETER);
    //
    // // push image to temp sprite
    // temp_orb_sprite.pushImage(0, 0, IMAGE_DIAMETER, IMAGE_DIAMETER, drawn_orb);
    //
    // // init all sprites for animation testing
    // for (auto& row: orb_animation){
    //     for (auto& sprite : row) {
    //         sprite = new LGFX_Sprite(&tft);
    //     }
    // }
    //
    //
    // setup_sprite_animations();
    // setup_light_ring_colors();
    setup_world();
}

void loop()
{
    ulong start_time = millis();

    const uint16 BACKGROUND_COLOR = 0b0001000010000010; // 0x20412c
    screen.fillSprite(BACKGROUND_COLOR);


    draw_light();
    draw_circle();

    world->Step(1.0f/10.0f, 6, 4);

    leblanc_sprite.pushSprite(&screen, 0, 0);
    screen.pushSprite(0, 0);

    // update gravity using mpu
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    float roll = roll_from_mpu_accel(accel);

    Serial.print("pitch: ");
    Serial.println(pitch_from_mpu_accel(accel));
    Serial.print("roll: ");
    Serial.println(roll_from_mpu_accel(accel));

    float x_grav = -9.8f * sin(roll);
    float y_grav = -9.8f * cos(roll);

    // TODO: multiply gravity by tilt
    float pitch = pitch_from_mpu_accel(accel);
    float tilt_amount = abs(pitch) / (PI / 2);

    b2Vec2 mpu_grav(x_grav, y_grav);
    mpu_grav *= (1 - tilt_amount);
    world->SetGravity(mpu_grav);

    // awake all orbs
    for (b2Body* orb : test_orbs) {
        orb->SetAwake(true);
    }

    // update wall velocity
    float vx = horizontal_accel_from_mpu_accel(accel);
    float vy = vertical_accel_from_mpu_accel(accel);
    b2Vec2 mpu_velocity = b2Vec2(vx, vy);
    for (b2Body* wall : walls){
        wall->SetLinearVelocity(mpu_velocity);
    }


    ulong end_time = millis();
    ulong draw_time = end_time - start_time;
    Serial.print("Draw time: ");
    Serial.println(draw_time);
}
