
#include "zippies/config/ZippyConfig.h"

#include <Arduino.h>

#define UUID_MEMORY_LOCATION_A 0x0080A00C
#define UUID_MEMORY_LOCATION_B 0x0080A040

int ZIPPY_ID = 0;
double MOTOR_DEAD_ZONE = 3000.0;
double MOTOR_BALANCE = 0.0;
extern double MOTOR_DEAD_ZONES[][2];

extern uint32_t KNOWN_MCU_IDS[][4];
extern int KNOWN_MCU_ID_COUNT;

void printUniqueID(uint32_t* mcuID);

void initZippyConfiguration()
{
    uint32_t mcuID[4];
    readMCUID(mcuID);
    // printUniqueID(mcuID);
    for (int i = 0; i < KNOWN_MCU_ID_COUNT; i++) {
        if (!memcmp(mcuID, KNOWN_MCU_IDS[i], 16)) {
            // SerialUSB.print("Found known Zippy config: ");
            // SerialUSB.println(i);
            ZIPPY_ID = i;
            MOTOR_DEAD_ZONE = MOTOR_DEAD_ZONES[i][0];
            MOTOR_BALANCE = MOTOR_DEAD_ZONES[i][1];
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
    { 0x4AC29873, 0x50533748, 0x392E3120, 0xFF192F33 },
    //orange
    { 0x9E104EE4, 0x5052534A, 0x352E3120, 0xFF091527 },
    //yellow
    { 0x4596FFCD, 0x50533748, 0x392E3120, 0xFF19103F },
    //green
    { 0x82C95D68, 0x504D5257, 0x352E3120, 0xFF152F28 },
    //blue
    { 0x8A95E7ED, 0x504E5452, 0x372E3120, 0xFF181D37 },
    //purple
    { 0x5A252B0E, 0x5052534A, 0x352E3120, 0xFF0C3222 },

    //GRAVEYARD BELOW THIS LINE --------

    //RIP previous red
    // { 0x3BCD3F11, 0x504B3233, 0x372E3120, 0xFF022719 },
    //RIP previous orange
    // { 0xA69E657C, 0x50515946, 0x392E3120, 0xFF192E1B },
    //RIP previous purple
    // { 0x27A3B213, 0x50533336, 0x372E3120, 0xFF142915 },
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
    { 3000.0,  0.0800 },
    //orange
    {  600.0,  0.1100 },
    //yellow
    { 2000.0,  0.0000 },
    //green
    { 2000.0, -0.0100 },
    //blue
    { 2200.0,  0.0200 },
    //purple
    {  600.0,  0.0000 },
};

void readMCUID(uint32_t* outID)
{
    volatile uint32_t *ptr = (volatile uint32_t*)UUID_MEMORY_LOCATION_A;
    outID[0] = (*ptr);
    ptr = (volatile uint32_t*)UUID_MEMORY_LOCATION_B;
    outID[1] = (*(ptr));
    outID[2] = (*(ptr+1));
    outID[3] = (*(ptr+2));
}

void printUniqueID(uint32_t* mcuID)
{
    //format the ID into a 128-bit hexadecimal string
    char formattedID[33];
    sprintf(formattedID, "%08X", (unsigned int)mcuID[0]);
    sprintf(&(formattedID[8]), "%08X", (unsigned int)mcuID[1]);
    sprintf(&(formattedID[16]), "%08X", (unsigned int)mcuID[2]);
    sprintf(&(formattedID[24]), "%08X", (unsigned int)mcuID[3]);

    SerialUSB.print("Zippy ID: ");
    SerialUSB.println(formattedID);
}
