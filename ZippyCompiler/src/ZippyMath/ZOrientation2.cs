using System;

namespace ZippyMath
{

    public class ZOrientation2
    {

        public ZOrientation2(ZOrientation2 other)
        {
            this.o = other.o;
            this.Cos = other.Cos;
            this.Sin = other.Sin;
        }

        public ZOrientation2(double orientation)
        {
            o = ZMath.SnapAngle(orientation);
            Cos = Math.Cos(o);
            Sin = Math.Sin(o);
        }

        public void Reset()
        {
            this.o = 0.0;
            this.Cos = 1.0;
            this.Sin = 0.0;
        }

        public double o
        { get; private set; }

        public double Cos
        { get; private set; }

        public double Sin
        { get; private set; }

        public void Add(double orientation)
        {
            o = ZMath.AddAngles(o, orientation);
            Cos = Math.Cos(o);
            Sin = Math.Sin(o);
        }

        public void Subtract(double orientation)
        {
            o = ZMath.SubtractAngles(o, orientation);
            Cos = Math.Cos(o);
            Sin = Math.Sin(o);
        }

        public void Set(ZOrientation2 other)
        {
            this.o = other.o;
            this.Cos = other.Cos;
            this.Sin = other.Sin;
        }

        public void Set(double orientation)
        {
            o = orientation;
            Cos = Math.Cos(o);
            Sin = Math.Sin(o);
        }

        public static implicit operator double(ZOrientation2 o) => o.o;

    }

}
