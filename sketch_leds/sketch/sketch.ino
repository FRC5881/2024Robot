#include <Adafruit_NeoPixel.h>

#define SLOW_RAINBOW 0
#define SOLID_RED 1
#define SOLID_GREEN 2
#define SOLID_BLUE 3
#define BREATHING_RED 4
#define BREATHING_GREEN 5
#define BREATHING_BLUE 6
#define SLOW_FLASH_GREEN 7
#define CHASING_UP_RED 8
#define CHASING_UP_GREEN 9
#define CHASING_UP_BLUE 10
#define CHASING_DOWN_RED 11
#define CHASING_DOWN_GREEN 12
#define CHASING_DOWN_BLUE 13
#define FAST_FLASH_RED 14
#define FAST_FLASH_GREEN 15
#define FAST_FLASH_BLUE 16
#define SOLID_PURPLE 17
#define FAST_RAINBOW_FLASH 18

#define NUM_PATTERNS 19

#define PIN_NEO_PIXEL 9 // Arduino pin that connects to NeoPixel
#define NUM_PIXELS 87   // The number of LEDs (pixels) on NeoPixel (robot LED)

Adafruit_NeoPixel NeoPixel(NUM_PIXELS, PIN_NEO_PIXEL, NEO_GRB + NEO_KHZ800);

// Robot settings
int max_pixel = 84;
int right_pixel_count = 29;
int left_pixel_count = 29;
int side_pixel_count = 29;
int top_pixel_count = 26;

// Pattern delays
int breath_delay = 50;
int fast_flash_delay = 200;
int slow_flash_delay = 1000;
int power_off_delay = 100;
int chase_delay = 50;
int slow_rainbow_delay = 10;

// Default RGB values
const uint8_t value = 70;
const uint32_t RED = NeoPixel.color(value, 0, 0);
const uint32_t GREEN = NeoPixel.color(0, value, 0);
const uint32_t BLUE = NeoPixel.color(0, 0, value);
const uint32_t PURPLE = NeoPixel.color(value, 0, value);

void setup()
{
    NeoPixel.begin();
    Serial.begin(9600);
}

void loop()
{
    // Read the voltage from the AnalogOutput port
    int voltage = (analogRead(A0) * NUM_PATTERNS) / 1024;

    switch (voltage)
    {
    case SLOW_RAINBOW:
        slowRainbow();
        delay(slow_rainbow_delay);
        break;

    case SOLID_RED:
        solidPattern(RED);
        break;
    case SOLID_GREEN:
        solidPattern(GREEN);
        break;
    case SOLID_BLUE:
        solidPattern(BLUE);
        break;

    case BREATHING_RED:
        breathingPattern(RED);
        break;
    case BREATHING_GREEN:
        breathingPattern(GREEN);
        break;
    case BREATHING_BLUE:
        breathingPattern(BLUE);
        break;

    case SLOW_FLASH_GREEN:
        flash(0, green, 0);
        delay(slow_flash_delay);
        break;

    case CHASING_UP_RED:
        chasingUp(red, 0, 0);
        delay(chase_delay);
        break;
    case CHASING_UP_GREEN:
        chasingUp(0, green, 0);
        delay(chase_delay);
        break;
    case CHASING_UP_BLUE:
        chasingUp(0, 0, blue);
        delay(chase_delay);
        break;

    case FAST_FLASH_RED:
        flash(red, 0, 0);
        delay(fast_flash_delay);
        break;
    case FAST_FLASH_GREEN:
        flash(0, green, 0);
        delay(fast_flash_delay);
        break;
    case FAST_FLASH_BLUE:
        flash(0, 0, blue);
        delay(fast_flash_delay);
        break;

    case SOLID_PURPLE:
        solidPattern(PURPLE);
        break;

    case FAST_RAINBOW_FLASH:
        noise();
        delay(fast_flash_delay);
        break;
    }
}

/**
 * Function: solidPattern
 * ----------------------
 * Description: This function displays a solid color pattern on the LEDs.
 */
void solidPattern(uint32_t color)
{
    NeoPixel.fill(color, 0, 0);
    NeoPixel.show();
}

/**
 * Function: offPattern
 * ----------------------
 * Description: This function turns off the LEDs by setting all pixels to black.
 */
void offPattern()
{
    NeoPixel.clear();
    NeoPixel.show();
}

const int rainbow_hue_increase = 45;
const int brightness = 50;
const int saturation = 255;
uint16_t hue = 0;

/**
 * Function: slowRainbow
 * ----------------------
 * Description: This function displays a slow rainbow pattern on the LEDs.
 *              It gradually transitions between different colors to create a smooth rainbow effect.
 *              The rate of this transition can be configured using `rainbow_hue_increase`
 */
void slowRainbow()
{
    for (int pixel = 0; pixel < max_pixel; pixel++)
    {
        uint16_t pixel_color = hue + (pixel * rainbow_hue_increase);
        NeoPixel.setPixelColor(pixel, NeoPixel.ColorHSV(pixel_color, saturation, brightness));
    }
    hue = hue + rainbow_hue_increase;
    NeoPixel.show();
}

const int chase_length = 6;
int chase_state = 0;

/**
 * Function: chasingUp
 * ----------------------
 * Description: This function displays a chasing pattern on the LEDs.
 *              A group of 6 LEDs move up both sides of the robot meeting at the top.
 */
void chasingUp(uint32_t color)
{
    NeoPixel.clear();
    NeoPixel.fill(color, chase_state, chase_state + chase_length);
    NeoPixel.fill(color, NUM_PIXELS - chase_state, NUM_PIXELS - (chase_state + chase_length));
    NeoPixel.show();

    chase_state = (chase_state + 1) % ((max_pixel + chase_length) / 2);
}

/**
 * Function: chasingDown
 * ----------------------
 * Description: This function displays a chasing pattern on the LEDs.
 *              A group of 6 LEDs move up both sides of the robot meeting at the top.
 */
void chasingDown(uint32_t color)
{
    NeoPixel.clear();
    NeoPixel.fill(color, chase_state, chase_state + chase_length);
    NeoPixel.fill(color, NUM_PIXELS - chase_state, NUM_PIXELS - (chase_state + chase_length));
    NeoPixel.show();

    chase_state = (chase_state - 1) % ((max_pixel + chase_length) / 2);
}

bool flash_state = false;

/**
 * Function: flash
 * ------------------
 * Description: This function flashes all pixels on, then all pixels off
 * State Machine:
 *              false : pixels off
 *              true  : pixels on
 */
void flash(uint32_t color)
{
    flash_state = !flash_state;
    if (flash_state)
    {
        offPattern();
    }
    else
    {
        solidPattern(color);
    }
}

/**
 * Function: noise
 * ------------------
 * Description: This function sets all pixels to random noise
 */
void noise()
{
    for (int pixel = 0; pixel < NUM_PIXELS; pixel++)
    {
        uint8_t random_red = random(0, red);
        uint8_t random_blue = random(0, blue);
        uint8_t random_green = random(0, green);

        NeoPixel.setPixelColor(pixel, random_red, random_blue, random_green);
    }
    NeoPixel.show();
}

const int breathing_steps = 35;
int breathing_state = 0;
int breathing_direction = 1;

/**
 * Function: breathing
 * ------------------
 * Description: Pixels gradually turn on and turn off
 * State Machine:
 *             i for 0..35 : pixels are i/35 * max_value
 */
void breathing(uint16_t hue, uint8_t max_value)
{
    if (breathing_state == 0)
    {
        breathing_direction = 1;
    }
    else if (breathing_state == breathing_steps - 1)
    {
        breathing_direction = -1;
    }

    uint8_t value = breathing_state * max_value / breathing_steps;
    solidPattern(NeoPixel.ColorHSV(hue, 255, value));

    breathing_state += breathing_direction;
}
