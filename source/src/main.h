bool GetTemperatureFrame();
void ApplyTemperatureFrameToLEDs();
uint16_t getIndexOfPixel(int x, int y);
int GetMatrixIndexOfPixel(int x, int y);
bool DetectMotion();
void SetMotionLED(uint32_t color);
void PrintRefeshRate();
uint32_t Color(uint8_t r, uint8_t g, uint8_t b);

// Noise Effect Functions
void noiseEffect();
void ChangePaletteAndSettingsPeriodically();
void mapNoiseToLEDsUsingPalette();
void SetupRandomPalette();
void SetupPurpleAndGreenPalette();

void MatrixEffect();