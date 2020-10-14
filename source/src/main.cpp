/*
  Infrared Mirror

  Displays an infrared camara image using a matrix of LEDs.

  Reuben Strangelove
  Winter 2020

  MCU: Teensy 3.2 (required for speed, memory capacity, and library support).
  SENSOR: Adafruit MLX90640 over I2C.

  Notes: Double tap brightness pot into zero postion (min brightness) to
          enter into animation mode that displays noise effects on the led matrix.

          LED brightness limited in software due to LED panel overheating issues.


  Library notes: In order to use 16hz refresh rate a line needed to be added to the MLX60640 library.
                  on line 22 of Adafruit_MLX90640.cpp in the boolean Adafruit_MLX90640::begin() method, 
                  add the follow line:
                  wire->setClock(1000000); //1MHz I2C clock
*/

#include "Arduino.h"
#include "SPI.h"
#include <Adafruit_MLX90640.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>
#include <FastLED.h>
#include <main.h>
#include <msTimer.h> // local library.

// LED matrix and LED strips parameters.
#define kMatrixWidth 24
#define kMatrixHeight 32
#define NUM_LEDS_IN_LED_MATRIX 768
CRGB leds[NUM_LEDS_IN_LED_MATRIX];
#define NUM_LEDS_IN_NAMEPLATE 12

#define PIN_LED_STRIP_GRID 8
#define PIN_LED_STRIP_NAMEPLATE 9
#define PIN_LED_STRIP_MOTION 10
#define PIN_BRIGHTNESS_POT A0

Adafruit_NeoPixel stripNameplate = Adafruit_NeoPixel(NUM_LEDS_IN_NAMEPLATE, PIN_LED_STRIP_NAMEPLATE, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripMotion = Adafruit_NeoPixel(1, PIN_LED_STRIP_MOTION, NEO_RGB + NEO_KHZ800);

// LED brightness
#define MAX_LED_MATRIX_BRIGHTNESS 105 // Max brightness causes overheating issues with the LEDs and they burn out.

// NoiseEffect parameters.
#define MAX_DIMENSION ((kMatrixWidth > kMatrixHeight) ? kMatrixWidth : kMatrixHeight)
uint16_t speed = 20; // 1 very slow, or 60 for fast water.
uint16_t scale = 30; // A value of 1 will be so zoomed in, you'll mostly see solid colors.
uint8_t colorLoop = 1;
uint8_t noise[MAX_DIMENSION][MAX_DIMENSION];
CRGBPalette16 currentPalette(PartyColors_p);

// Adafruit MLX90640 parameters.
Adafruit_MLX90640 mlx;
float temperatureFrame[32 * 24];

// Heat colors. Configured by trial and error for best visual effect, does not conform to data representation standards.
const int redHeatColor[] = {0, 0, 0, 0, 0, 0, 16, 32, 64, 92, 127, 192, 252, 255, 255, 255, 255};
const int greenHeatColor[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 127, 192};
const int blueHeatColor[] = {8, 16, 32, 64, 127, 192, 255, 192, 127, 64, 32, 0, 0, 0, 0, 127, 192};

int cumTempInsideMatrix[12];

#define RED 0xFF0000
#define GREEN 0x00FF00
#define BLUE 0x0000FF
#define OFF 0x000000

enum state
{
  DisplayTemperature,
  DisplayAnimations
} State;

void setup()
{
  FastLED.addLeds<NEOPIXEL, PIN_LED_STRIP_GRID>(leds, NUM_LEDS_IN_LED_MATRIX).setDither(0);
  FastLED.setBrightness(0);
  FastLED.show();

  stripNameplate.begin();
  stripNameplate.setBrightness(185);
  stripNameplate.show();

  stripMotion.begin();
  stripMotion.setBrightness(60);
  stripMotion.show();

  Serial.begin(115200);
  Serial.println("Infrared Mirror startup...");

  delay(500);

  if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire))
  {
    Serial.println("MLX90640 not found!");
    while (1)
    {
      SetMotionLED(RED);
      delay(200);
      SetMotionLED(OFF);
      delay(200);
    }
  }

  Serial.println("Found Adafruit MLX90640.");

  mlx.setMode(MLX90640_CHESS);           // MLX90640_CHESS or MLX90640_INTERLEAVED
  mlx.setResolution(MLX90640_ADC_16BIT); // 16, 17, 18, 19
  mlx.setRefreshRate(MLX90640_16_HZ);    // Modification was required in the library to increase I2C speed, see top comments.

  // State = DisplayAnimations; // TEMP

  // Start up name plate animation.
  for (int i = 0; i < NUM_LEDS_IN_NAMEPLATE; i++)
  {
    int colorIndex = i + 4;
    stripNameplate.setPixelColor(NUM_LEDS_IN_NAMEPLATE - i - 1, Color(redHeatColor[colorIndex], greenHeatColor[colorIndex], blueHeatColor[colorIndex]));
    stripNameplate.show();
    delay(75);
  }
}

void loop()
{

  static msTimer noMotionTimer(15000);

  if (noMotionTimer.elapsed())
  {
    State = DisplayAnimations;
  }

  // Adjust LED matrix brightness by reference from user input potentiometer.
  int adcValue = 1023 - analogRead(PIN_BRIGHTNESS_POT);
  int ledBrightness = map(adcValue, 0, 1024, 1, MAX_LED_MATRIX_BRIGHTNESS);
  FastLED.setBrightness(ledBrightness);

  if (checkForTriggerActionOnPot(adcValue))
  {
    State = DisplayAnimations;
  }

  if (State == DisplayTemperature)
  {
    if (UpdateMLX90640())
    {
      if (DetectMotion())
      {
        SetMotionLED(GREEN);
        noMotionTimer.resetDelay();
      }
      else
      {
        SetMotionLED(OFF);
      }
    }
    else
    {
      // Error
      SetMotionLED(RED);
    }
  }
  else if (State == DisplayAnimations)
  {
    MatrixEffect();
    //noiseEffect();
  }
}

// Detect if user taps pot far left multiple times (taps to zero value).
bool checkForTriggerActionOnPot(int adcValue)
{
  static int tapCount = 0;
  static unsigned long atHighMillis = 0;
  static bool newAtZeroFlag = false;

  if (adcValue == 0)
  {
    if (newAtZeroFlag == true)
    {
      newAtZeroFlag = false;
      tapCount++;
      if (tapCount == 2)
      {
        return true;
      }
    }
    atHighMillis = millis();
  }

  if (adcValue > 20)
  {
    if (millis() > (atHighMillis + 1500))
    {
      newAtZeroFlag = false;
      tapCount = 0;
    }
    else
    {
      newAtZeroFlag = true;
    }
  }
  return false;
}

bool UpdateMLX90640()
{
  static bool statusFlag = true;
  static msTimer timeoutTimer(3000);

  // Allow for x seconds of errors before flaging an error state.
  if (timeoutTimer.elapsed())
  {
     statusFlag = false;
  }

  int ret = mlx.getFrame(temperatureFrame);
  if (ret != 0)
  {
    Serial.printf("MLX90640 error: %d\n", ret);
    return false;
  }

  // Data received success: apply frame to led grid.

  statusFlag = true;
  timeoutTimer.resetDelay();
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
      float temperature = temperatureFrame[xRotated + yRotated * 32];

      // Store min and max temperature for debugging.
      if (temperature > 15)
      {
        if (minTemperature > temperature)
          minTemperature = temperature;
      }
      if (maxTemperature < temperature)
        maxTemperature = temperature;

      // Convert temperature into heat color index.
      int index = round(temperature - 20);
      if (index < 0)
      {
        index = 0;
      }
      if (index > 16)
      {
        index = 16;
      }

      // Set the color of the LEDs.
      leds[getIndexOfPixel(x, y)] = CRGB(redHeatColor[index], greenHeatColor[index], blueHeatColor[index]);

      // Store temperature values.
      cumTempInsideMatrix[GetMatrixIndexOfPixel(x, y)] += (int)round(temperature);
    }
  }

  // Calculate average temperature.
  float avgTemperature;
  float cumTemperature = 0;
  for (int i = 0; i < 32 * 24; i++)
  {
    cumTemperature += temperatureFrame[i];
  }
  avgTemperature = cumTemperature / (32 * 24);

  Serial.printf("Min temperature: %0.1f. Max temperature: %0.1f. Average temperature: %0.2f.\n", minTemperature, maxTemperature, avgTemperature);

  FastLED.show();
  return statusFlag;
}

bool DetectMotion()
{
  static int curAvgTempInsideMatrix[12];
  static int prevAvgTempInsideMatrix[12];
  bool motionDetected = false;

  // Calculator average temperature inside each matrix.
  for (int i = 0; i < 12; i++)
  {
    curAvgTempInsideMatrix[i] = cumTempInsideMatrix[i] / (8 * 8);
    cumTempInsideMatrix[i] = 0;
  }

  // Compare cur with prev temperature average.
  for (int i = 0; i < 12; i++)
  {
    int threasholdDelta = 1;

    // Serial.printf("%u - %u - %u - %u\n", i, threasholdDelta, prevAvgTempInsideMatrix[i], curAvgTempInsideMatrix[i]);

    if (prevAvgTempInsideMatrix[i] + threasholdDelta < curAvgTempInsideMatrix[i])
      motionDetected = true;
    if (prevAvgTempInsideMatrix[i] - threasholdDelta > curAvgTempInsideMatrix[i])
      motionDetected = true;
  }

  // Store cur as prev temperature average.
  for (int i = 0; i < 12; i++)
  {
    prevAvgTempInsideMatrix[i] = curAvgTempInsideMatrix[i];
  }

  return motionDetected;
}

// Converts pixel index into maxtrix index.
// See comments on other methods.
int GetMatrixIndexOfPixel(int x, int y)
{
  const int w = 3;
  const int h = 4;
  const int pixelPerDim = 8;

  int m = 12 - (int)((x / pixelPerDim) + ((y / pixelPerDim) * w));

  return m;
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
    for (int i = 0; i < NUM_LEDS_IN_LED_MATRIX; i++)
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
// x/y cordinates of the data to be displated are expected to start in the upper left corner
// and end in the lower right corner.
// Matrix of matrix consists of 12 8x8 LED matrix.
// LED Matrix order (11 upper left, 0 lower right):
// 11  10   9
//  8   7   6
//  5   4   3
//  2   1   0
// Pixels start on the lower right of each matrix and end in the upper left.
// LEDs within the 8x8 matrix do not zig-zag.
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

// Noise Effect Functions
void noiseEffect()
{
  ChangePaletteAndSettingsPeriodically();

  // The 16 bit version of our coordinates
  static uint16_t x = random16();
  static uint16_t y = random16();
  static uint16_t z = random16();

  // Smooth out artifacts at low speeds.
  uint8_t dataSmoothing = 0;
  if (speed < 50)
  {
    dataSmoothing = 200 - (speed * 4);
  }

  for (int i = 0; i < MAX_DIMENSION; i++)
  {
    int ioffset = scale * i;

    for (int j = 0; j < MAX_DIMENSION; j++)
    {
      int joffset = scale * j;

      uint8_t data = inoise8(x + ioffset, y + joffset, z);

      // The range of the inoise8 function is roughly 16-238.
      // These two operations expand those values out to roughly 0..255
      // You can comment them out if you want the raw noise data.
      data = qsub8(data, 16);
      data = qadd8(data, scale8(data, 39));

      if (dataSmoothing)
      {
        uint8_t olddata = noise[i][j];
        uint8_t newdata = scale8(olddata, dataSmoothing) + scale8(data, 256 - dataSmoothing);
        data = newdata;
      }

      noise[i][j] = data;
    }
  }

  z += speed;

  // apply slow drift to X and Y, just for visual variation.
  x += speed / 8;
  y -= speed / 16;

  mapNoiseToLEDsUsingPalette();
}

void mapNoiseToLEDsUsingPalette()
{
  static uint8_t ihue = 0;

  for (int i = 0; i < kMatrixWidth; i++)
  {
    for (int j = 0; j < kMatrixHeight; j++)
    {
      uint8_t index = noise[j][i];
      uint8_t bri = noise[i][j];

      if (colorLoop)
      {
        index += ihue;
      }

      // brighten up.
      if (bri > 127)
      {
        bri = 255;
      }
      else
      {
        bri = dim8_raw(bri * 2);
      }

      CRGB color = ColorFromPalette(currentPalette, index, bri);
      leds[getIndexOfPixel(i, j)] = color;
    }
  }

  ihue += 1;

  FastLED.show();
}

void ChangePaletteAndSettingsPeriodically()
{
  static int noiseColors = 0;

  EVERY_N_SECONDS(5)
  {
    noiseColors++;
    if (noiseColors > 8)
      noiseColors = 0;

    if (noiseColors == 0)
    {
      currentPalette = RainbowColors_p;
      speed = 5;
      scale = 30;
      colorLoop = 1;
    }
    else if (noiseColors == 1)
    {
      SetupPurpleAndGreenPalette();
      speed = 5;
      scale = 50;
      colorLoop = 1;
    }
    else if (noiseColors == 2)
    {
      currentPalette = CloudColors_p;
      speed = 5;
      scale = 30;
      colorLoop = 0;
    }
    else if (noiseColors == 3)
    {
      currentPalette = LavaColors_p;
      speed = 5;
      scale = 50;
      colorLoop = 0;
    }
    else if (noiseColors == 4)
    {
      currentPalette = PartyColors_p;
      speed = 5;
      scale = 30;
      colorLoop = 1;
    }
    else if (noiseColors == 5)
    {
      SetupRandomPalette();
      speed = 5;
      scale = 20;
      colorLoop = 1;
    }
    else if (noiseColors == 6)
    {
      SetupRandomPalette();
      speed = 5;
      scale = 50;
      colorLoop = 1;
    }
    else if (noiseColors == 7)
    {
      SetupRandomPalette();
      speed = 20;
      scale = 90;
      colorLoop = 1;
    }
    else if (noiseColors == 8)
    {
      currentPalette = RainbowStripeColors_p;
      speed = 5;
      scale = 20;
      colorLoop = 1;
    }
  }
}

// This function generates a random palette that's a gradient
// between four different colors.  The first is a dim hue, the second is
// a bright hue, the third is a bright pastel, and the last is
// another bright hue.  This gives some visual bright/dark variation
// which is more interesting than just a gradient of different hues.
void SetupRandomPalette()
{
  currentPalette = CRGBPalette16(
      CHSV(random8(), 255, 32),
      CHSV(random8(), 255, 255),
      CHSV(random8(), 128, 255),
      CHSV(random8(), 255, 255));
}

// This function sets up a palette of purple and green stripes.
void SetupPurpleAndGreenPalette()
{
  CRGB purple = CHSV(HUE_PURPLE, 255, 255);
  CRGB green = CHSV(HUE_GREEN, 255, 255);
  CRGB black = CRGB::Black;

  currentPalette = CRGBPalette16(
      green, green, black, black,
      purple, purple, black, black,
      green, green, black, black,
      purple, purple, black, black);
}

// Pack color data into 32 bit unsigned int (copied from Neopixel library).
uint32_t Color(uint8_t r, uint8_t g, uint8_t b)
{
  return (uint32_t)((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

void SetMotionLED(uint32_t color)
{
  stripMotion.setPixelColor(0, color);
  stripMotion.show();
}

void PrintRefeshRate()
{
  mlx90640_refreshrate_t rate = mlx.getRefreshRate();
  Serial.print("MLX90640 refresh rate: ");
  switch (rate)
  {
  case MLX90640_0_5_HZ:
    Serial.println("0.5 Hz");
    break;
  case MLX90640_1_HZ:
    Serial.println("1 Hz");
    break;
  case MLX90640_2_HZ:
    Serial.println("2 Hz");
    break;
  case MLX90640_4_HZ:
    Serial.println("4 Hz");
    break;
  case MLX90640_8_HZ:
    Serial.println("8 Hz");
    break;
  case MLX90640_16_HZ:
    Serial.println("16 Hz");
    break;
  case MLX90640_32_HZ:
    Serial.println("32 Hz");
    break;
  case MLX90640_64_HZ:
    Serial.println("64 Hz");
    break;
  }
}