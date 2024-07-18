using System;
using ZippyMath;

namespace RoutineModelV3
{

    public class ZRoutineV3 : ZRoutinePathV3
    {

        public readonly int StartingPositionIndex;
        public readonly ZMatrix2 StartingPosition;
        // public readonly Guid zippyUUID;

        // public ZRoutineV3(int startingPositionIndex, ZMatrix2 startingPosition, Guid zippyUUID)
        public ZRoutineV3(int startingPositionIndex, ZMatrix2 startingPosition)
        {
            this.StartingPositionIndex = startingPositionIndex;
            this.StartingPosition = startingPosition;
            // this.zippyUUID = zippyUUID;
        }

    }

}
