
using System;

namespace RoutineModelV3
{

	public class ZRoutinePathPointV3
	{
        //three types of subcommand formats to support here
        //         Beats1        FullCircle12      -100       100
        //  pause:
        //       <timing>
        //  move to absolute point:
        //       <timing>                 <x>       <y>       <z>
        //  move to named point (with optional x/y multipliers?):
        //       <timing>       <named_point>  <x_mult>  <y_mult>
        //  follow a named path (with optional x/y multipliers?):
        //       <timing>        <named_path>  <x_mult>  <y_mult>

        public readonly int pointIndex;
        public readonly string? pathName;
        public readonly int multiplierX;
        public readonly int multiplierY;

        public ZRoutinePathPointV3(int pointIndex, int multiplierX, int multiplierY)
        {
            this.pointIndex = pointIndex;
            this.pathName = null;
            this.multiplierX = multiplierX;
            this.multiplierY = multiplierY;
        }

        public ZRoutinePathPointV3(string pathName, int multiplierX, int multiplierY)
        {
            this.pointIndex = -1;
            this.pathName = pathName;
            this.multiplierX = multiplierX;
            this.multiplierY = multiplierY;
        }

    }

}
