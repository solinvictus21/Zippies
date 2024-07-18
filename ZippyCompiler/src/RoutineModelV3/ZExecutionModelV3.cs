
using System;
using System.Collections.Generic;
using ZippyMath;

namespace RoutineModelV3
{

    public class ZExecutionModelV3
    {

        public readonly ZMatrix2 StartingPosition;
        // public readonly Guid zippyUUID;

        private List<int> timings = new List<int>();
        private List<ZMatrix2?> points = new List<ZMatrix2?>();

        // public ZExecutionModelV3(ZMatrix2 StartingPosition, Guid zippyUUID)
        public ZExecutionModelV3(ZMatrix2 StartingPosition)
        {
            this.StartingPosition = StartingPosition;
            // this.zippyUUID = zippyUUID;
        }

        public static List<ZExecutionModelV3> FlattenRoutines(ZRoutineSetV3 routineSet)
        {
            List<ZExecutionModelV3> flattenedRoutines = new List<ZExecutionModelV3>();
            for (int i = 0; i < routineSet.RoutineCount; i++)
            {
                ZRoutineV3 nextRoutine = routineSet.GetRoutine(i);
                Console.WriteLine("Routine: " + i);
                //create a new execution model that will represent the flattened version of this routine
                ZExecutionModelV3 nextModel = new ZExecutionModelV3(nextRoutine.StartingPosition);//, nextRoutine.zippyUUID);
                //keep track of the current position after each movement
                ZMatrix2 currentPosition = new ZMatrix2(nextRoutine.StartingPosition);
                for (int j = 0; j < nextRoutine.PointCount; j++)
                {
                    //get the timing of the next movement
                    int nextTiming = nextRoutine.GetTiming(j);

                    //get the next movement
                    ZRoutinePathPointV3? nextPathPoint = nextRoutine.GetPoint(j);
                    if (nextPathPoint == null)
                    {
                        //no point or subpath, so just pause in place
                        Console.WriteLine(String.Format("  T: {0,10:0}", nextTiming));
                        nextModel.AddPoint(nextTiming, null);
                    }
                    else if (nextPathPoint.pointIndex != -1)
                    {
                        //this is a reference to a single point relative to the anchor point of the routine, which is (0, 0, 0)
                        ZMatrix2 nextPoint = routineSet.GetPoint(nextPathPoint.pointIndex);
                        nextModel.AddPoint(nextTiming, nextPoint);
                        debugDistance(currentPosition, nextPoint, nextTiming);
                        currentPosition.Set(nextPoint);
                    }
                    else
                    {
                        //now it gets more complicated, because we have to flatten a hierarchy of paths into the routine, relative
                        //to the current position
                        Console.WriteLine("Path: " + nextPathPoint.pathName!);
                        FlattenPathToModel(nextModel, routineSet, routineSet.GetPath(nextPathPoint.pathName!),
                            currentPosition, nextTiming, nextPathPoint.multiplierX, nextPathPoint.multiplierY);
                    }
                }
                flattenedRoutines.Add(nextModel);
            }

            return flattenedRoutines;
        }

        private static void FlattenPathToModel(ZExecutionModelV3 model, ZRoutineSetV3 routineSet,
            ZRoutinePathV3 path, ZMatrix2 anchorPosition,
            int multiplierT, int multiplierX, int multiplierY)
        {
            ZMatrix2 currentPosition = anchorPosition;
            ZMatrix2 previousPosition = new ZMatrix2(currentPosition);
            for (int i = 0; i < path.PointCount; i++)
            {
                int nextTiming = path.GetTiming(i);
                ZRoutinePathPointV3? nextPathPoint = path.GetPoint(i);
                //Debug.Log("Getting next point #" + i);
                if (nextPathPoint == null)
                {
                    //just a pause in place
                    Console.Write("  " + currentPosition);
                    Console.WriteLine(String.Format("  T: {0,10:0}", multiplierT * nextTiming));
                    model.AddPoint(multiplierT * nextTiming, null);
                }
                else if (nextPathPoint.pointIndex != -1)
                {
                    //this is a reference to a single point relative to the anchor point of the routine, which is (0, 0, 0)
                    ZMatrix2 nextPoint = routineSet.GetPoint(nextPathPoint.pointIndex);
                    previousPosition.Set(currentPosition);
                    currentPosition = new ZMatrix2(
                        nextPoint.x * multiplierX * nextPathPoint.multiplierX,
                        nextPoint.y * multiplierY * nextPathPoint.multiplierY,
                        (multiplierX * nextPathPoint.multiplierX * multiplierY * nextPathPoint.multiplierY >= 0) ? nextPoint.o : -nextPoint.o);
                    currentPosition.ConcatTo(anchorPosition);
                    debugDistance(previousPosition, currentPosition, multiplierT * nextTiming);
                    model.AddPoint(multiplierT * nextTiming, currentPosition);
                }
                else
                {
                    //flatten the path points and the next level down in the hierarchy of paths
                    Console.WriteLine("Subpath: " + nextPathPoint.pathName!);
                    // Console.Write("  " + currentPosition);
                    FlattenPathToModel(model,
                        routineSet,
                        routineSet.GetPath(nextPathPoint.pathName!),
                        currentPosition,
                        multiplierT * nextTiming,
                        multiplierX * nextPathPoint.multiplierX,
                        multiplierY * nextPathPoint.multiplierY);
                    Console.WriteLine();
                    // Console.WriteLine("  " + currentPosition);
                }
            }
            if (currentPosition != anchorPosition)
                anchorPosition.Set(currentPosition);
        }

        private static void debugDistance(ZMatrix2 start, ZMatrix2 end, int timing)
        {
            Console.Write("  " + start);
            Console.Write(String.Format("  T: {0,10:0}", timing));

            /*
            double estimatedDistance = estimateDistance(start, end);  
            Console.Write(String.Format("  Df: {0,10:F2}", estimatedDistance));
            Console.Write(String.Format("  Df/T: {0,10:F2}", estimatedDistance * 1000 / timing));

            double estimatedDistanceI = estimateDistance(end, start);  
            Console.Write(String.Format("  Di: {0,10:F2}", estimatedDistanceI));
            Console.Write(String.Format("  Di/T: {0,10:F2}", estimatedDistanceI * 1000 / timing));
            */

            double estimatedDistance2 = estimateDistance2(start, end);  
            Console.Write(String.Format("  D: {0,10:F2}", estimatedDistance2));
            Console.WriteLine(String.Format("  D/T: {0,10:F2}", estimatedDistance2 * 1000 / timing));
        }
        
        /*
        private static double estimateDistance(ZMatrix2 start, ZMatrix2 end)
        {
            ZVector2 shadowToTargetPosition = new ZVector2(
                end.Position.x - start.Position.x,
                end.Position.y - start.Position.y);
            double delta = ZMath.SubtractAngles(start.Orientation, shadowToTargetPosition.Atan2);
            if (isReverseDirection(start, end))
                delta = ZMath.AddAngles(delta, Math.PI);
            return delta == 0.0 ? shadowToTargetPosition.D :
                (shadowToTargetPosition.D * delta / Math.Sin(delta));
        }

        private static bool isReverseDirection(ZMatrix2 previousTargetPosition, ZMatrix2 targetPosition)
        {
            double forwardTurn = 2.0 * ZMath.SubtractAngles(
                Math.Atan2(
                    targetPosition.Position.x - previousTargetPosition.Position.x,
                    targetPosition.Position.y - previousTargetPosition.Position.y),
                previousTargetPosition.Orientation);
            //from our arrival orientation, add the difference to the next target orientation to calculate the total turn required
            forwardTurn += ZMath.SubtractAngles(targetPosition.Orientation,
                ZMath.AddAngles(previousTargetPosition.Orientation, forwardTurn));
            //if we need to do more than a 180 degree turn to arrive at the target orientation, then move backwards instead
            return Math.Abs(forwardTurn) > Math.PI;
        }
        */

        private static double estimateDistance2(ZMatrix2 start, ZMatrix2 end)
        {
            ZVector2 shadowToTargetPosition = new ZVector2(
                end.Position.x - start.Position.x,
                end.Position.y - start.Position.y);
            double thetaMoveO = ZMath.SubtractAngles(shadowToTargetPosition.Atan2, start.Orientation);
            double thetaMoveI = ZMath.SubtractAngles(shadowToTargetPosition.Atan2, end.Orientation);
            bool reverseMotion = Math.Abs(thetaMoveO - thetaMoveI) > Math.PI;
            if (reverseMotion)
            {
                thetaMoveO = ZMath.AddAngles(thetaMoveO, Math.PI);
                thetaMoveI = ZMath.AddAngles(thetaMoveI, Math.PI);
            }
            double length = (arcLength(shadowToTargetPosition.D, thetaMoveO) +
                arcLength(shadowToTargetPosition.D, thetaMoveI)) / 2.0;
            return reverseMotion ? -length : length;
        }

        private static double arcLength(double distance, double theta)
        {
            double sinTheta = Math.Sin(theta);
            if (sinTheta == 0.0)
                return distance;
            return distance * theta / sinTheta;
        }

        public void AddPoint(int timing, ZMatrix2? point)
        {
            timings.Add(timing);
            points.Add(point);
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

        public ZMatrix2? GetPoint(int i)
        {
            return points[i];
        }

    }

}
