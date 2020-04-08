
#include <Arduino.h>

#define DEFAULT_PID_UPDATE_INTERVAL                   17
#define DEFAULT_PID_OUTPUT_LIMIT                   40000.0d

//PID CONFIGURATION
#define DEFAULT_MOTORS_10_1_PID_KP                    70.0d
#define DEFAULT_MOTORS_10_1_PID_KI                     0.0d
#define DEFAULT_MOTORS_10_1_PID_KD                     5.0d
#define DEFAULT_MOTORS_15_1_PID_KP                   160.0d
#define DEFAULT_MOTORS_15_1_PID_KI                     0.0d
#define DEFAULT_MOTORS_15_1_PID_KD                     7.0d

#define UUID_MEMORY_LOCATION_A 0x0080A00C
#define UUID_MEMORY_LOCATION_B 0x0080A040

uint32_t MCU_ID[4];

int ZIPPY_ID = 0;
double MOTOR_DEAD_ZONE_LEFT = 3000.0d;
double MOTOR_DEAD_ZONE_RIGHT = 3000.0d;
extern double MOTOR_DEAD_ZONES[][2];
// extern double MOTOR_DEAD_ZONE_CONFIGS[][2];

extern uint32_t KNOWN_MCU_IDS[][4];
extern int KNOWN_MCU_ID_COUNT;

void readUniqueID();
void printUniqueID();

void initZippyConfiguration()
{
  readUniqueID();
  // printUniqueID();
  for (int i = 0; i < KNOWN_MCU_ID_COUNT; i++) {
    if (!memcmp(MCU_ID, KNOWN_MCU_IDS[i], 16)) {
      // SerialUSB.print("Found known Zippy config: ");
      // SerialUSB.println(i);
      ZIPPY_ID = i;
      // MOTOR_DEAD_ZONE_LEFT = MOTOR_DEAD_ZONE_CONFIGS[i][0];
      // MOTOR_DEAD_ZONE_RIGHT = MOTOR_DEAD_ZONE_CONFIGS[i][1];
      double deadZoneSpread = MOTOR_DEAD_ZONES[i][0] * MOTOR_DEAD_ZONES[i][1];
      MOTOR_DEAD_ZONE_LEFT = MOTOR_DEAD_ZONES[i][0] - deadZoneSpread;
      MOTOR_DEAD_ZONE_RIGHT = MOTOR_DEAD_ZONES[i][0] + deadZoneSpread;
      break;
    }
  }
}

int getCurrentZippyNumber()
{
  return ZIPPY_ID;
}

int getTotalZippyCount()
{
  return KNOWN_MCU_ID_COUNT;
}

uint32_t KNOWN_MCU_IDS[][4]
{
  //red
  { 0x3BCD3F11, 0x504B3233, 0x372E3120, 0xFF022719 },
  //orange
  { 0xA69E657C, 0x50515946, 0x392E3120, 0xFF192E1B },
  //green
  { 0x82C95D68, 0x504D5257, 0x352E3120, 0xFF152F28 },
  //blue
  { 0x8A95E7ED, 0x504E5452, 0x372E3120, 0xFF181D37 },
  //purple
  { 0x27A3B213, 0x50533336, 0x372E3120, 0xFF142915 },
};
int KNOWN_MCU_ID_COUNT = sizeof(KNOWN_MCU_IDS) / 16;

double MOTOR_DEAD_ZONES[][2]
{
  //motor dead zones are configured for the performance characteristics of each individual Zippy, since
  //the behavior of each DC motor can vary even for motors of the same model
  //the dead zone is configured with a minimum PCM value and a balance ratio
  //the minimum PCM value is the point beyond which the motors begin to have a linear relationship between
  //input and output
  //negative balance ratios give stronger turns on the left wheel (turns right harder)
  //positive balance ratios give stronger turns on the right wheel (turns left harder)
  //red
  // { 2300.0d,  0.2000d },
  { 2200.0d,  0.0500d },
  //orange
  // { 1650.0d, -0.3200d },
  { 1800.0d, -0.3200d },
  //green
  { 2800.0d,  0.1800d },
  //blue
  { 2700.0d,  0.2600d },
  //purple
  { 1900.0d,  0.0500d },
};

void readUniqueID()
{
  volatile uint32_t *ptr = (volatile uint32_t*)UUID_MEMORY_LOCATION_A;
  MCU_ID[0] = (*ptr);
  ptr = (volatile uint32_t*)UUID_MEMORY_LOCATION_B;
  MCU_ID[1] = (*(ptr));
  MCU_ID[2] = (*(ptr+1));
  MCU_ID[3] = (*(ptr+2));
}

void printUniqueID()
{
  //format the ID into a 128-bit hexadecimal string
  char formattedID[33];
  sprintf(formattedID, "%08X", (unsigned int)MCU_ID[0]);
  sprintf(&(formattedID[8]), "%08X", (unsigned int)MCU_ID[1]);
  sprintf(&(formattedID[16]), "%08X", (unsigned int)MCU_ID[2]);
  sprintf(&(formattedID[24]), "%08X", (unsigned int)MCU_ID[3]);

  SerialUSB.print("Zippy ID: ");
  SerialUSB.println(formattedID);
}
