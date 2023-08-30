
#include "zippies/config/ZippyRoutines.h"
#include "zippies/config/ZippyConfig.h"

extern ZippyRoutineData routines[];
extern int routinesCount;

const ZippyRoutineData* getZippyRoutineData()
{
    uint32_t mcuID[4];
    readMCUID(mcuID);
    // printUniqueID(mcuID);
    int routineIndex = 0;
    for (int i = 0; i < routinesCount; i++)
    {
        // printUniqueID((uint32_t*)(&routines[i]));
        if (!memcmp(mcuID, &routines[i], 16)) {
            //found a matching MCU ID; this is the one
            routineIndex = i;
            break;
        }
    }
    return &routines[routineIndex];
}
