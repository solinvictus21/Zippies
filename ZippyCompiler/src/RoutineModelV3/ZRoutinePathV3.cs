
using System;
using System.Collections.Generic;
using ZippyMath;

namespace RoutineModelV3
{

    public class ZRoutinePathV3
    {

        private List<int> timings = new List<int>();
        private List<ZRoutinePathPointV3?> pathPoints = new List<ZRoutinePathPointV3?>();

        public ZRoutinePathV3()
        {
        }

        public void AddPause(int timing)
        {
            timings.Add(timing);
            pathPoints.Add(null);
        }

        public void AddPointIndex(int timing, int pointIndex, int multiplierX, int multiplierY)
        {
            timings.Add(timing);
            pathPoints.Add(new ZRoutinePathPointV3(pointIndex, multiplierX, multiplierY));
        }

        public void AddSubPath(int timing, string pathName, int multiplierX, int multiplierY)
        {
            timings.Add(timing);
            pathPoints.Add(new ZRoutinePathPointV3(pathName, multiplierX, multiplierY));
        }

        public int PointCount
        {
            get
            {
                return timings.Count;
            }
        }

        public int GetTiming(int i)
        {
            return timings[i];
        }

        public ZRoutinePathPointV3? GetPoint(int i)
        {
            return pathPoints[i];
        }

    }

}