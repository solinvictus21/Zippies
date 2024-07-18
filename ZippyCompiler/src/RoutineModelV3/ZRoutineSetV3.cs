
// using System.Text.RegularExpressions;
using ZippyMath;

namespace RoutineModelV3
{

    public class ZRoutineSetV3
    {

        public const string DEFAULT_ROUTINE_FILE_EXTENSION = ".zrtn";

        private const string ROUTINE_COMMENT = "//";
        private const string COMMAND_INCLUDE = "include";
        private const string COMMAND_POINT = "point";
        private const string COMMAND_TIMING = "timing";
        private const string COMMAND_PATH = "path";
        private const string COMMAND_ROUTINE = "routine";

        private string routineName;
        private List<ZRoutineV3> routines = new List<ZRoutineV3>();
        private Dictionary<string, ZRoutinePathV3> pathsByName = new Dictionary<string, ZRoutinePathV3>();
        private List<ZMatrix2> pointList = new List<ZMatrix2>();
        private Dictionary<int, int> pointIndexesByHashCode = new Dictionary<int, int>();
        private Dictionary<string, int> pointIndexesByName = new Dictionary<string, int>();
        private Dictionary<string, int> timingsByName = new Dictionary<string, int>();

        public ZRoutineSetV3(FileInfo fileInfo)
        {
            routineName = fileInfo.Name;
            if (routineName.EndsWith(DEFAULT_ROUTINE_FILE_EXTENSION))
                routineName = routineName.Substring(0, routineName.Length - DEFAULT_ROUTINE_FILE_EXTENSION.Length);
            HashSet<string> includedFiles = new HashSet<string>();
            Parse(includedFiles, fileInfo);
        }

        public string RoutineName
        {
            get
            {
                return routineName;
            }
        }

        public int RoutineCount
        {
            get
            {
                return routines.Count;
            }
        }

        public ZRoutineV3 GetRoutine(int i)
        {
            return routines[i];
        }

        public int TotalPointCount
        {
            get
            {
                return pointList.Count;
            }
        }

        public ZMatrix2 GetPoint(int i)
        {
            return pointList[i];
        }

        public ZRoutinePathV3 GetPath(string pathName)
        {
            return pathsByName[pathName];
        }

        private void Parse(HashSet<string> includedFiles, FileInfo fileInfo)
        {
            if (includedFiles.Contains(fileInfo.FullName))
            {
                //log when duplicates or circularities occur
                //Debug.Log("The file '" + fileInfo.FullName + "' is already included. Ignoring duplicate.");
                return;
            }

            //Debug.Log("Loading routine file: " + fileInfo.FullName);
            includedFiles.Add(fileInfo.FullName);

            ZRoutineV3? currentRoutine = null;
            ZRoutinePathV3? currentPath = null;
            StreamReader streamReader = new StreamReader(fileInfo.FullName);

            string? nextLine;
            while ((nextLine = streamReader.ReadLine()) != null)
            {
                // Console.WriteLine("Line: " + nextLine);
                //read the next line
                nextLine = nextLine.Trim();
                //skip comments and blank lines
                if (nextLine.Length == 0 || nextLine.StartsWith(ROUTINE_COMMENT))
                    continue;
                //Debug.Log("Line: " + nextLine);

                string[] splitCommand = nextLine.Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
                switch (splitCommand[0])
                {
                    //include
                    case COMMAND_INCLUDE:
                        //close current routine and path
                        currentRoutine = null;
                        currentPath = null;
                        //Debug.Log("Parsing included file: " + fileInfo.Directory.FullName + Path.DirectorySeparatorChar + splitCommand[1] + ".zrtn");
                        Parse(includedFiles, new FileInfo(fileInfo.Directory!.FullName + Path.DirectorySeparatorChar + splitCommand[1] + ".zrtn"));
                        break;

                    //named point
                    case COMMAND_POINT:
                        //close current routine and path
                        currentRoutine = null;
                        currentPath = null;
                        //TODO: check for naming collisions (points, timings and paths)
                        pointIndexesByName[splitCommand[1]] = AddPoint(
                            int.Parse(splitCommand[2]), int.Parse(splitCommand[3]), int.Parse(splitCommand[4]));
                        break;

                    //named timing
                    case COMMAND_TIMING:
                        //close current routine and path
                        currentRoutine = null;
                        currentPath = null;
                        //TODO: check for naming collisions (points, timings and paths)
                        timingsByName[splitCommand[1]] = int.Parse(splitCommand[2]);
                        break;

                    //start of a new named path
                    case COMMAND_PATH:
                        //close current routine
                        currentRoutine = null;
                        //TODO: check for naming collisions (points, timings and paths)
                        currentPath = new ZRoutinePathV3();
                        pathsByName[splitCommand[1]] = currentPath;
                        break;

                    //start of a new routine
                    case COMMAND_ROUTINE:
                        //close current path
                        currentPath = null;
                        int pointIndex = pointIndexesByName[splitCommand[1]];
                        // Guid guid = Guid.Parse(splitCommand[2]);
                        //TODO: check for routine collisions (based on routine name)
                        currentRoutine = new ZRoutineV3(
                            pointIndex,
                            pointList[pointIndex]);//,
                            // guid);
                        //routinesByGUID[guid] = currentRoutine;
                        routines.Add(currentRoutine);
                        break;

                    default:
                        MergePoints(currentRoutine != null ? currentRoutine! : currentPath!, splitCommand);
                        break;
                }
            }

            streamReader.Close();
        }

        private int AddPoint(int x, int y, int o)
        {
            o = SnapAngleDegrees(o);
            int hashCode = GetPointHashCode(x, y, o);
            int pointIndex;
            if (pointIndexesByHashCode.TryGetValue(hashCode, out pointIndex))
                return pointIndex;

            int newPositionIndex = pointList.Count;
            pointList.Add(new ZMatrix2(x, y, ZMath.ANGLES_TO_RADIANS * o));
            pointIndexesByHashCode[hashCode] = newPositionIndex;
            return newPositionIndex;
        }

        private static int SnapAngleDegrees(int angle)
        {
            while (angle <= -180)
                angle += 360;
            while (angle > 180)
                angle -= 360;
            return angle;
        }

        private static int GetPointHashCode(int x, int y, int o)
        {
            //allow arithmetic overflow; hash codes can just wrap around
            unchecked
            {
                int hashcode = 1430287;
                hashcode = hashcode * 7302013 ^ x.GetHashCode();
                hashcode = hashcode * 7302013 ^ y.GetHashCode();
                hashcode = hashcode * 7302013 ^ o.GetHashCode();
                return hashcode;
            }
        }

        private void MergePoints(ZRoutinePathV3 path, string[] splitCommand)
        {
            //parse the first parameter, which could be either a named timing or a literal timing
            string timingParam = splitCommand[0];
            int timing;
            if (!timingsByName.TryGetValue(timingParam, out timing))
            {
                //TODO: if the timing doesn't parse into a number, it's a syntax error
                timing = int.Parse(timingParam);
            }

            //check for a pause
            if (splitCommand.Length == 1)
            {
                //this is just a pause in place
                path.AddPause(timing);
                return;
            }


            //now parse the second parameter, which could be a named point, a named path or the start of a literal point
            string namedAlias = splitCommand[1];
            int multiplierX = splitCommand.Length > 2 ? int.Parse(splitCommand[2]) : 1;
            int multiplierY = splitCommand.Length > 3 ? int.Parse(splitCommand[3]) : 1;
            if (pointIndexesByName.TryGetValue(namedAlias, out int pointIndex))
            {
                //it's a named point
                path.AddPointIndex(timing, pointIndex, multiplierX, multiplierY);
            }
            else if (pathsByName.TryGetValue(namedAlias, out ZRoutinePathV3? subPath))
            {
                //it's a named path
                path.AddSubPath(timing, namedAlias, multiplierX, multiplierY);
            }
            else
            {
                //it had better be a literal point
                int newPointIndex = AddPoint(int.Parse(namedAlias), multiplierX, multiplierY);
                path.AddPointIndex(timing, newPointIndex, 1, 1);
            }
        }

    }

}