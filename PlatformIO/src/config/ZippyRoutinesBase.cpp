
#include <cstdint>
#include <string.h>
#include "zippies/config/ZippyRoutines.h"

#define TEST_CONFIG_SEPARATION 100.0

extern ZippyRoutineData routines[];
extern int routinesCount;
extern ZippyConfig KNOWN_ZIPPIES[];
extern int KNOWN_ZIPPY_COUNT;

extern ZippyConfig testConfig;

#ifdef WEBOTS_SUPPORT
#define UUID_MEMORY_LOCATION_A &KNOWN_ZIPPIES[2].mcuId[0]
#define UUID_MEMORY_LOCATION_B &KNOWN_ZIPPIES[2].mcuId[1]
#else
#define UUID_MEMORY_LOCATION_A 0x0080A00C
#define UUID_MEMORY_LOCATION_B 0x0080A040
#endif

void readMCUID(uint32_t* outID)
{
    volatile uint32_t *ptr = (volatile uint32_t*)UUID_MEMORY_LOCATION_A;
    outID[0] = (*ptr);
    ptr = (volatile uint32_t*)UUID_MEMORY_LOCATION_B;
    outID[1] = (*(ptr));
    outID[2] = (*(ptr+1));
    outID[3] = (*(ptr+2));
}

int getZippyNumber()
{
    uint32_t mcuID[4];
    readMCUID(mcuID);
    // printUniqueID(mcuID);
    int zippyNumber = 0;
    for (int i = 0; i < KNOWN_ZIPPY_COUNT; i++)
    {
        // printUniqueID((uint32_t*)(&routines[i]));
        if (!memcmp(mcuID, &KNOWN_ZIPPIES[i].mcuId, 16)) {
            //found a matching MCU ID; this is the one
            zippyNumber = i;
            break;
        }
    }
    return zippyNumber;
}

const ZippyConfig* getZippyConfig()
{
    return getZippyConfig(getZippyNumber());
}

const ZippyConfig* getZippyConfig(int zippyNumber)
{
    return &KNOWN_ZIPPIES[zippyNumber];
}

const ZippyConfig* getZippyTestConfig()
{
    return getZippyTestConfig(getZippyNumber());
}

const ZippyConfig* getZippyTestConfig(int zippyNumber)
{
    testConfig.zippyNumber = zippyNumber;
    int zippyOffset = ((KNOWN_ZIPPY_COUNT-1) * TEST_CONFIG_SEPARATION / 2.0) -
        (testConfig.zippyNumber * TEST_CONFIG_SEPARATION);
    testConfig.routine->startingPosition[0] += zippyOffset;
    int i = 0;
    while (i < testConfig.routine->routineMovementCount) {
        if (testConfig.routine->routineMovements[i] >= 0) {
            testConfig.routine->routineMovements[i+1] += zippyOffset;
            i += 4;
        }
        else
            i++;
    }
    //also capture the motor gain for each robot; could affect direction of motor
    testConfig.leftMotorConfig.motorGain = KNOWN_ZIPPIES[zippyNumber].leftMotorConfig.motorGain;
    testConfig.rightMotorConfig.motorGain = KNOWN_ZIPPIES[zippyNumber].rightMotorConfig.motorGain;
    return &testConfig;
}

ZippyConfig KNOWN_ZIPPIES[]
{
    //red
    {
        0,
        { 0x4AC29873, 0x50533748, 0x392E3120, 0xFF192F33 },
        //as tested
        // { 7500.0, 3200.0, -1.0 },
        // { 7400.0, 2900.0, 1.0 },
        { 7400.0, 1000.0, -1.0 },
        { 7400.0, 1000.0, 1.0 },
        &routines[0]
    },
    //orange
    {
        1,
        { 0x9E104EE4, 0x5052534A, 0x352E3120, 0xFF091527 },
        //as tested
        // { 7800.0, 3600.0, 1.0 },
        // { 7200.0, 2900.0, 1.0 },
        //experimental
        { 7800.0, 1000.0, 1.0 },
        { 7200.0, 1000.0, 1.0 },
        &routines[1]
    },
    //yellow
    {
        2,
        { 0x4596FFCD, 0x50533748, 0x392E3120, 0xFF19103F },
        // { 7700.0, 3100.0, 1.0 },
        // { 7100.0, 6500.0, 1.0 },
        { 7700.0, 1000.0, 1.0 },
        { 7100.0, 1000.0, 1.0 },
        &routines[2]
    },
    //green
    {
        3,
        { 0x82C95D68, 0x504D5257, 0x352E3120, 0xFF152F28 },
        // { 0.0, 0.0, 1.0 },
        // { 0.0, 0.0, 1.0 },
        { 7500.0, 1000.0, 1.0 },
        { 7500.0, 1000.0, 1.0 },
        &routines[3]
    },
    //blue
    {
        4,
        { 0x8A95E7ED, 0x504E5452, 0x372E3120, 0xFF181D37 },
        // { 7600.0, 3200.0, 1.0 },
        // { 8300.0, 3500.0, 1.0 },
        { 7600.0, 1000.0, 1.0 },
        { 8300.0, 1000.0, 1.0 },
        &routines[4]
    },
    //purple
    {
        5,
        { 0x5A252B0E, 0x5052534A, 0x352E3120, 0xFF0C3222 },
        // { 6900.0, 3300.0, 1.0 },
        // { 8100.0, 3300.0, 1.0 },
        { 6900.0, 1000.0, 1.0 },
        { 8100.0, 1000.0, 1.0 },
        &routines[5]
    },
};
int KNOWN_ZIPPY_COUNT = sizeof(KNOWN_ZIPPIES) / sizeof(ZippyConfig);

void printUniqueID(uint32_t* mcuID)
{
    /*
    //format the ID into a 128-bit hexadecimal string
    char formattedID[33];
    sprintf(formattedID, "%08X", (unsigned int)mcuID[0]);
    sprintf(&(formattedID[8]), "%08X", (unsigned int)mcuID[1]);
    sprintf(&(formattedID[16]), "%08X", (unsigned int)mcuID[2]);
    sprintf(&(formattedID[24]), "%08X", (unsigned int)mcuID[3]);

    SerialUSB.print("Zippy ID: ");
    SerialUSB.println(formattedID);
    */
}

int testRoutineData[] =
{
    //circular figure 8
    //forward right
     1000,        200,        200,         90,
     1000,        400,          0,        180,
     1000,        200,       -200,        -90,
     1000,          0,          0,          0,
    // -1000,
    //forward left
     1000,       -200,        200,        -90,
     1000,       -400,          0,        180,
     1000,       -200,       -200,         90,
     1000,          0,          0,          0,
    -1000,
    //backward right
     1000,        200,       -200,        -90,
     1000,        400,          0,        180,
     1000,        200,        200,         90,
     1000,          0,          0,          0,
    // -1000,
    //backward left
     1000,       -200,       -200,         90,
     1000,       -400,          0,        180,
     1000,       -200,        200,        -90,
     1000,          0,          0,          0,
    -1000,
};

ZippyRoutineData testRoutine =
{
    0, 0, 0,
    &testRoutineData[0],
    sizeof(testRoutineData) / sizeof(int),
};

ZippyConfig testConfig
{
    0,
    { 0x00000000, 0x00000000, 0x00000000, 0x00000000 },
    { 7500.0,    0.0, 1.00 },
    { 7400.0,    0.0, 1.00 },
    &testRoutine,
};
