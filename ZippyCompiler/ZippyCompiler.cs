
using RoutineModelV3;
using ZippyMath;

class ZippyCompiler
{

    private const string DEFAULT_ROUTINE_CPP_FILE_EXTENSION = ".cpp";
    private const string DEFAULT_ROUTINE_OUTPUT_DIRECTORY = "out";

    private const string FORMAT_TIMING           = "    {0,10:0},";
    private const string FORMAT_TIMING_AND_POINT = "    {0,10:0}, {1,10:0}, {2,10:0}, {3,10:0},";
    private const string FORMAT_POINT            = "    {0,10:0}, {1,10:0}, {2,10:0},";
    // private const string FORMAT_UUID_COMPONENT   = "    0x{0:X2}{1:X2}{2:X2}{3:X2},";

    static void Main(string[] args)
    {
        if (args == null || args.Length != 1) {
            //TODO: support binary output format
            Console.WriteLine("Usage: ZippyCompiler <routine_file>");
            return;
        }

        FileInfo fileInfo = new FileInfo(args[0]);
        ZRoutineSetV3 routineSet = new ZRoutineSetV3(fileInfo);
        DirectoryInfo outputDirectory = new DirectoryInfo(fileInfo.Directory!.FullName + Path.DirectorySeparatorChar + DEFAULT_ROUTINE_OUTPUT_DIRECTORY);
        WriteRoutines(routineSet, outputDirectory);
    }

    private static void WriteRoutines(ZRoutineSetV3 routineSet, DirectoryInfo outputDirectory)
    {
        //create the output directory and file path
        outputDirectory.Create();
        StreamWriter streamWriter = new StreamWriter(outputDirectory.FullName + Path.DirectorySeparatorChar +
            routineSet.RoutineName + DEFAULT_ROUTINE_CPP_FILE_EXTENSION);

        //write the standard header
        streamWriter.WriteLine();
        streamWriter.WriteLine("#include \"zippies/config/ZippyRoutines.h\"");
        streamWriter.WriteLine();

        List<ZExecutionModelV3> routines = ZExecutionModelV3.FlattenRoutines(routineSet);
        int[] routineArrayLengths = new int[routines.Count];
        for (int i = 0; i < routines.Count; i++)
        {
            ZExecutionModelV3 nextRoutine = routines[i];
            ZMatrix2 startingPosition = nextRoutine.StartingPosition;
            streamWriter.WriteLine("int routine" + i + "[] {");
            for (int j = 0; j < nextRoutine.PointCount; j++)
            {
                int nextTiming = nextRoutine.GetTiming(j);
                ZMatrix2? nextPoint = nextRoutine.GetPoint(j);
                if (nextPoint == null)
                {
                    streamWriter.WriteLine(String.Format(FORMAT_TIMING, -nextTiming));
                    routineArrayLengths[i]++;
                }
                else
                {
                    streamWriter.WriteLine(String.Format(FORMAT_TIMING_AND_POINT,
                        nextTiming,
                        (Int32)Math.Round(nextPoint.x),
                        (Int32)Math.Round(nextPoint.y),
                        (Int32)Math.Round(ZMath.RADIANS_TO_ANGLES * nextPoint.o)));
                    routineArrayLengths[i] += 4;
                }
            }
            streamWriter.WriteLine("};");
            streamWriter.WriteLine();
        }

        //now build the overall routine list
        streamWriter.WriteLine("ZippyRoutineData routines[] = {");
        for (int i = 0; i < routines.Count; i++)
        {
            //get the next routine
            ZExecutionModelV3 nextRoutine = routines[i];

            //start the routine line
            streamWriter.WriteLine("  {");

            //routine starting position
            streamWriter.WriteLine(String.Format(FORMAT_POINT,
                (Int32)Math.Round(nextRoutine.StartingPosition.x),
                (Int32)Math.Round(nextRoutine.StartingPosition.y),
                (Int32)Math.Round(ZMath.RADIANS_TO_ANGLES * nextRoutine.StartingPosition.o)));

            //routine points
            streamWriter.WriteLine("    &routine" + i + "[0],");

            //routine point count
            streamWriter.WriteLine("    " + routineArrayLengths[i] + ",");

            //end the routine line
            streamWriter.WriteLine("  },");
        }
        streamWriter.WriteLine("};");
        streamWriter.WriteLine("int routinesCount = " + routines.Count + ";");

        streamWriter.Close();
    }

}
