
#include <Arduino.h>

#include "zippies/hardware/ZippyFace.h"
#include "zippies/displays/FaceGraphics.h"

ZippyFace::ZippyFace()
  : display(TinyScreenPlus)
{
  display.begin();
  display.setBrightness(4);
  display.setFlip(true);
  display.setFont(thinPixel7_10ptFontInfo);
}

void ZippyFace::displayFace()
{
  //displaying the face is a bit processor-intensive, so we just do it once at the start, since it doesn't currently change
  display.setBitDepth(TSBitDepth8);
  display.setColorMode(TSColorModeBGR);
  display.setX(0, TinyScreen::xMax);
  display.setY(0, TinyScreen::yMax);
  display.startData();
  display.writeBuffer((uint8_t*)FACE_HAPPY, 6144);
  display.endTransfer();
}

void ZippyFace::clearScreen()
{
  display.clearScreen();
}

void ZippyFace::displayText(uint8_t x, uint8_t y, const char* text)
{
  if (text == NULL)
    return;

  //label
  // /*
  display.fontColor(TS_8b_Blue, TS_8b_Black);
  display.setCursor(x, y);
  display.print(text);
  // */
}

void ZippyFace::displayLabelAndData(uint8_t x, uint8_t y, const char* label, double data)
{
  displayLabelAndData(x, y, label, data, 1);
}

void ZippyFace::displayLabelAndData(uint8_t x, uint8_t y, const char* label, double data, uint8_t digits)
{
  displayText(x, y, label);

  //data
  display.fontColor(TS_8b_White, TS_8b_Black);
  String s = String(data, digits);
  uint8_t printWidth = display.getPrintWidth((char*)s.c_str());
  display.setCursor(x+SCREEN_WIDTH_PIXELS_2-printWidth, y);
  display.print(s);
}

void ZippyFace::displayLabelAndData(uint8_t x, uint8_t y, const char* label, int data)
{
  displayText(x, y, label);

  //data
  display.fontColor(TS_8b_White, TS_8b_Black);
  String s = String(data);
  uint8_t printWidth = display.getPrintWidth((char*)s.c_str());
  display.setCursor(x+SCREEN_WIDTH_PIXELS_2-printWidth, y);
  display.print(s);
}

void ZippyFace::displayData(uint8_t x, uint8_t y, int data)
{
  display.setCursor(x, y);
  display.print(data);
}

void ZippyFace::displayTextCentered(const char* text)
{
  uint8_t printWidth = display.getPrintWidth((char*)text);
  display.setCursor(
      SCREEN_WIDTH_PIXELS_2-(printWidth/2),
      SCREEN_HEIGHT_PIXELS_2 - (display.getFontHeight()/2));
  display.print(text);
}

/*
void ZippyFace::loop()
{
  uint8_t modeColor = bluetooth.isConnected() ? TS_8b_Green : TS_8b_Red;

  display.startData();

  drawBattery();

  //debugging; show position info
  uint8_t printHeight = display.getFontHeight();
  uint8_t screenCenter = SCREEN_WIDTH_PIXELS/2;

  uint8_t nextRow = SCREEN_HEIGHT_PIXELS-(5.0f*printHeight);
  LighthouseSensor* sensorRight = lighthouse.getSensorRight();
  LighthouseSensor* sensorLeft = lighthouse.getSensorLeft();
  drawCoordinate(0, nextRow, "U: ", sensorRight->getXSyncTickCount(), 0);
  drawCoordinate(screenCenter, nextRow, "U: ", sensorLeft->getXSyncTickCount(), 0);
  nextRow += printHeight;
  //show the x,y position of each sensor
  drawCoordinate(0, nextRow, "X: ", sensorRight->getXPosition(), 1);
  drawCoordinate(screenCenter, nextRow, "X: ", sensorLeft->getXPosition(), 1);
  nextRow += printHeight;
  drawCoordinate(0, nextRow, "Y: ", sensorRight->getYPosition(), 1);
  drawCoordinate(screenCenter, nextRow, "Y: ", sensorLeft->getYPosition(), 1);

  nextRow += printHeight;
  drawCoordinate(0, nextRow, "V: ", sensorRight->getYSyncTickCount(), 0);
  drawCoordinate(screenCenter, nextRow, "V: ", sensorLeft->getYSyncTickCount(), 0);

  nextRow += printHeight;
  //show the calculated separation between each of the sensor positions; ideally, this will be exactly the same everywhere on the floor
  drawCoordinate(0, nextRow, "D: ", 0.0d, 1);
  //show the orientation
  drawCoordinate(screenCenter, nextRow, "O: ", (lighthouse.atan2() / M_PI) * 180.0d, 1);

  //show the calculated x,y position of the center of the Zippy
//  nextRow += printHeight;
//  drawCoordinate(0, nextRow, "X: ", lighthouse.getXPosition(), 1);
//  drawCoordinate(screenCenter, nextRow, "Y: ", lighthouse.getYPosition(), 1);

  //draw the mode we're currently in; red == manhandled, blue == auto-drive, green == user control
  drawModeIndicator(modeColor);

  display.endTransfer();
}

void ZippyFace::drawBattery() {
  //determine the current battery level
  int batteryLevel = getBatteryLevel();
  uint8_t red, green;
  uint8_t batteryLevelWidth;
//  SerialUSB.print("Battery Level: ");
//  SerialUSB.println(batteryLevel);
  if (batteryLevel > BATTERY_FULLY_CHARGED_VOLTAGE) {
    red = 0;
    green = 0x3F;
    batteryLevelWidth = BATTERY_DISPLAY_WIDTH;
  }
  else {
    float batteryLevelNormalized = (((float)batteryLevel) - BATTERY_FULLY_DISCHARGED_VOLTAGE) / (BATTERY_FULLY_CHARGED_VOLTAGE - BATTERY_FULLY_DISCHARGED_VOLTAGE);
    if (batteryLevelNormalized < 0.0f)
      batteryLevelNormalized = 0.0f;
    red = (1.0f - batteryLevelNormalized) * 0x3F;
    green = batteryLevelNormalized * 0x3F;
    batteryLevelWidth =  batteryLevelNormalized * BATTERY_DISPLAY_WIDTH;
  }
  //draw the outline
  display.drawRect(BATTERY_DISPLAY_X - 1, BATTERY_DISPLAY_Y - 1,
                   BATTERY_DISPLAY_WIDTH + 2, BATTERY_DISPLAY_HEIGHT + 2,
                   false, TS_8b_White);

  //right side of outline battery top notch
  display.drawLine(
      BATTERY_DISPLAY_X + BATTERY_DISPLAY_WIDTH + 1,
      BATTERY_DISPLAY_Y,
      BATTERY_DISPLAY_X + BATTERY_DISPLAY_WIDTH + 1,
      BATTERY_DISPLAY_Y + BATTERY_DISPLAY_HEIGHT - 2,
      COLOR8_WHITE);

  //draw the battery meter
  display.drawRect(BATTERY_DISPLAY_X, BATTERY_DISPLAY_Y,
                   BATTERY_DISPLAY_WIDTH, BATTERY_DISPLAY_HEIGHT,
                   true, red, green, 0);

  display.setCursor(0,0);
  display.print(batteryLevel);
}

int ZippyFace::getBatteryLevel()
{
  //http://atmel.force.com/support/articles/en_US/FAQ/ADC-example
  SYSCTRL->VREF.reg |= SYSCTRL_VREF_BGOUTEN;
  while (ADC->STATUS.bit.SYNCBUSY == 1);
  ADC->SAMPCTRL.bit.SAMPLEN = 0x1;
  while (ADC->STATUS.bit.SYNCBUSY == 1);
  ADC->INPUTCTRL.bit.MUXPOS = 0x19;         // Internal bandgap input
  while (ADC->STATUS.bit.SYNCBUSY == 1);
  ADC->CTRLA.bit.ENABLE = 0x01;             // Enable ADC
  // Start conversion
  while (ADC->STATUS.bit.SYNCBUSY == 1);
  ADC->SWTRIG.bit.START = 1;
  // Clear the Data Ready flag
  ADC->INTFLAG.bit.RESRDY = 1;
  // Start conversion again, since The first conversion after the reference is changed must not be used.
  while (ADC->STATUS.bit.SYNCBUSY == 1);
  ADC->SWTRIG.bit.START = 1;
  // Store the value
  while ( ADC->INTFLAG.bit.RESRDY == 0 );   // Waiting for conversion to complete
  uint32_t valueRead = ADC->RESULT.reg;
  while (ADC->STATUS.bit.SYNCBUSY == 1);
  ADC->CTRLA.bit.ENABLE = 0x00;             // Disable ADC
  while (ADC->STATUS.bit.SYNCBUSY == 1);
  SYSCTRL->VREF.reg &= ~SYSCTRL_VREF_BGOUTEN;
  return (((1100L * 1024L) / valueRead) + 5L) / 10L;
}

void ZippyFace::drawCoordinate(uint8_t x, uint8_t y, char* label, float value, int precision)
{
  //move the cursor to the desired position
  display.setCursor(x, y);
  //print the label
  display.fontColor(TS_16b_Green, TS_8b_Black);
  display.print(label);

  String text(value, precision);
//  SerialUSB.println((SCREEN_WIDTH_PIXELS/2)-display.getPrintWidth(charArray));
  uint8_t labelWidth = display.getPrintWidth(label);

  //calculate the display width of the value
  char* charArray = (char*)malloc(sizeof(char)*(text.length()+1));
  text.toCharArray(charArray, text.length()+1);
  uint8_t valueWidth = display.getPrintWidth(charArray);
  free(charArray);

  //calculate the x location to display the value
  uint8_t labelEnd = x+labelWidth;
  uint8_t valueX = x + (SCREEN_WIDTH_PIXELS/2)-valueWidth;
  if (valueX > labelEnd) {
    //clear the area between the label and the value
    display.drawRect(labelEnd, y, valueX-labelEnd, display.getFontHeight(), true, TS_8b_Black);
  }
  display.setCursor(valueX, y);
  display.fontColor(TS_16b_White, TS_8b_Black);
  display.print(text);
}

void ZippyFace::drawModeIndicator(uint8_t modeColor)
{
  //display an indicator for which mode we are in: user-driven, auto-driven, or manhandled (idle)
  display.drawRect((SCREEN_WIDTH_PIXELS-MODE_INDICATOR_WIDTH)/2, 0,
                   MODE_INDICATOR_WIDTH, MODE_INDICATOR_WIDTH,
                   true, modeColor);
}
*/
