




bool checkForTriggerActionOnPot(int adcValue);

bool UpdateMLX90640();

void MatrixEffect();

uint16_t getIndexOfPixel(int x, int y);

uint32_t Color(uint8_t r, uint8_t g, uint8_t b);

void PrintRefeshRate();

int GetMatrixIndexOfPixel(int x, int y);

bool DetectMotion();

void SetMotionLED(uint32_t color);


// Noise Effect Functions
void noiseEffect();
void ChangePaletteAndSettingsPeriodically();
void mapNoiseToLEDsUsingPalette();
void SetupRandomPalette();
void SetupPurpleAndGreenPalette();