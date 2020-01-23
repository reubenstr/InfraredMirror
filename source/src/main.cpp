// Infrared Mirror
//
// Reuben Strangelove
// Winter 2020
//


#include "Arduino.h"
#include "SPI.h"
#include <Adafruit_MLX90640.h>
#include <Adafruit_NeoPixel.h>
#include <main.h>
#include <math.h>

#include <FastLED.h>

#define kMatrixWidth 24
#define kMatrixHeight 32
#define NUM_LEDS 768
uint8_t ledBrightness = 192;
CRGB leds[NUM_LEDS];
CRGB motionLed[1];

Adafruit_MLX90640 mlx;
float frame[32 * 24]; // buffer for full frame of temperatures

#define PIN_LED_STRIP_GRID 8
#define PIN_LED_STRIP_NAMEPLATE 9
#define PIN_LED_STRIP_MOTION 10

Adafruit_NeoPixel stripMotion = Adafruit_NeoPixel(1, PIN_LED_STRIP_MOTION, NEO_GRB + NEO_KHZ800);

void setup()
{

  // LED Init
  FastLED.addLeds<NEOPIXEL, PIN_LED_STRIP_GRID>(leds, NUM_LEDS).setDither(0);
  FastLED.setBrightness(ledBrightness);
  FastLED.show();

  stripMotion.begin();
  stripMotion.setBrightness(127);
  stripMotion.show();

  //while (!Serial)
  //  delay(10);
  Serial.begin(115200);
  Serial.println("Infrared Mirror startup...");

  if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire))
  {
    Serial.println("MLX90640 not found!");
    while (1)
    {
      stripMotion.setPixelColor(0, Color(255, 0, 0));
      stripMotion.show();
      delay(200);
      stripMotion.setPixelColor(0, Color(0, 0, 0));
      stripMotion.show();
      delay(200);
    }
  }

  Serial.println("Found Adafruit MLX90640.");

  mlx.setMode(MLX90640_CHESS);           // MLX90640_CHESS or MLX90640_INTERLEAVED
  mlx.setResolution(MLX90640_ADC_18BIT); // 16, 17, 18, 19
  mlx.setRefreshRate(MLX90640_16_HZ);    // Modification was required in the library to increase I2C speed.
}

void loop()
{
  //stripMotion.setPixelColor(0, Color(255, 0, 0));
  // stripMotion.show();

  MatrixEffect();

  //UpdateMLX90640();
}

bool UpdateMLX90640()
{
  static bool statusFlag = true;
  static unsigned int oldMillis;

  if (millis() > oldMillis + 3000)
  {
    statusFlag = false;
  }

  if (mlx.getFrame(frame) != 0)
  {
    return statusFlag;
  }

  // Data received: apply frame to led grid.

  statusFlag = true;
  oldMillis = millis();
  float minTemperature = 100.0f;
  float maxTemperature = 0;

  for (uint8_t y = 0; y < 32; y++)
  {
    for (uint8_t x = 0; x < 24; x++)
    {

      // Rotate the camera pixels -90 degress and extract
      // the pixels by the matrix represented as an array.
      int xRotated = 32 - y;
      int yRotated = x;
      float temperature = frame[xRotated + yRotated * 32];

      if (temperature > 15)
      {
        if (minTemperature > temperature)
          minTemperature = temperature;
      }
      if (maxTemperature < temperature)
        maxTemperature = temperature;         

      const int red[] = {0, 0, 0, 0, 0, 0, 16, 32, 64, 92, 127, 192, 252, 255, 255, 255, 255};
      const int green[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 127, 212};
      const int blue[] = {8, 16, 32, 64, 127, 192, 255, 192, 127, 64, 32, 0, 0, 0, 0, 127, 212};

      int index = round(temperature - 20);
      if (index < 0)
      {
        index = 0;
      }
      if (index > 16)
      {
        index = 16;
      }
     
      leds[getIndexOfPixel(x, y)] = CRGB(red[index], green[index], blue[index]);
    }
  }

  Serial.print(minTemperature);
  Serial.print(" | ");
  Serial.println(maxTemperature);

  FastLED.show();
  return statusFlag;
}

void MatrixEffect()
{

  EVERY_N_MILLIS(75) // falling speed
  {
    const CRGB spawnColor = CRGB(125, 255, 125);

    // move code downward
    // start with lowest row to allow proper overlapping on each column
    for (int8_t row = kMatrixHeight - 1; row >= 0; row--)
    {
      for (int8_t col = 0; col < kMatrixWidth; col++)
      {
        if (leds[getIndexOfPixel(col, row)] == spawnColor)
        {
          leds[getIndexOfPixel(col, row)] = CRGB(27, 130, 39); // create trail
          if (row < kMatrixHeight - 1)
            leds[getIndexOfPixel(col, row + 1)] = spawnColor;
        }
      }
    }

    // fade all leds
    for (int i = 0; i < NUM_LEDS; i++)
    {
      if (leds[i].g != spawnColor.g)
        leds[i].nscale8(192); // only fade trail
    }

    // spawn new falling code
    if (random8(3) == 0) // lower number == more frequent spawns
    {
      int8_t spawnX = random8(kMatrixWidth);
      leds[getIndexOfPixel(spawnX, 0)] = spawnColor;
    }

    FastLED.show();
  }
}


// Convert x/y cordinates into pixel index inside the matrix of matrix.
// x/y cordinates are expected to start in the upper left and end in the lower right.
// Matrix of matrix consists of 12 8x8 LED matrix.
// Matrix order (11 upper left, 0 lower right):
// 11  10   9
//  8   7   6
//  5   4   3
//  2   1   0
// Pixels start on the lower right of each matrix and end in the upper left.
uint16_t getIndexOfPixel(int x, int y)
{

  int xMatrix = x / 8;
  int yMatrix = y / 8;
  int matrixIndex = 11 - (xMatrix + 3 * yMatrix);

  // inside matrix x, y
  int xL = 7 - (x % 8);
  int yL = 7 - (y % 8);
  uint16_t pixel = (uint16_t)(64 * matrixIndex + (xL + (8 * yL)));

  return pixel;
}

/*

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos)
{
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85)
  {
    return stripMotion.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170)
  {
    WheelPos -= 85;
    return stripMotion.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return stripMotion.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

*/
// Pack color data into 32 bit unsigned int (copied from Neopixel library).
uint32_t Color(uint8_t r, uint8_t g, uint8_t b)
{
  return (uint32_t)((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}
