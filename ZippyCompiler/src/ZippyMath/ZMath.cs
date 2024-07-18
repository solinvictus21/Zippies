using System;

namespace ZippyMath
{

    public class ZMath
    {

        public const double ANGLES_TO_RADIANS = Math.PI / 180.0d;
        public const double RADIANS_TO_ANGLES = 180.0d / Math.PI;

        public const double PI_2 = Math.PI / 2.0d;
        public const double PI_4 = Math.PI / 4.0d;

        private ZMath()
        {
        }

        public static double Clamp(double value, double min, double max)
        {
            return (value < min) ? min : (value > max) ? max : value;
        }

        public static double AddAngles(double a1, double a2)
        {
            return SnapAngle(a1 + a2);
        }

        public static double SubtractAngles(double a1, double a2)
        {
            return SnapAngle(a1 - a2);
        }

        public static double SnapAngle(double angle)
        {
            while (angle <= -Math.PI)
                angle += 2.0d * Math.PI;
            while (angle > Math.PI)
                angle -= 2.0d * Math.PI;
            return angle;
        }

        public static double EaseInOutBezier(double t, double a, double b)
        {
            double t2 = t * t;
            double t3 = t2 * t;
            double mt = 1.0d - t;
            double mt2 = mt * mt;
            return (a * 3.0d * mt2 * t) + (b * 3.0d * mt * t2) + t3;
        }

        public static double EaseInOutSin(double t, double b, double c, double d)
        {
            return -c / 2.0 * (Math.Cos(Math.PI * t / d) - 1.0) + b;
        }

        public static double EaseInSin(double t, double b, double c, double d)
        {
            return -c * Math.Cos(t / d * (Math.PI / 2.0)) + c + b;
        }

        public static double EaseInSin(double t, double c)
        {
            return -c * Math.Cos(t * (Math.PI / 2.0)) + c;
        }

        public static double EaseOutSin(double t, double b, double c, double d)
        {
            return c * Math.Sin(t / d * (Math.PI / 2.0)) + b;
        }

        public static double EaseOutSin(double t, double c)
        {
            return c * Math.Sin(t * (Math.PI / 2.0));
        }

        public static double EaseInOutSin(double t, double c)
        {
            return -c / 2.0 * (Math.Cos(Math.PI * t) - 1.0);
        }

        public static double EaseInOutCubic(double t, double b, double c, double d)
        {
            t /= d / 2.0;
            if (t < 1.0)
                return c / 2.0 * t * t * t + b;

            t -= 2.0;
            return c / 2.0d * (t * t * t + 2.0) + b;
        }

        public static double EaseInOutCubic(double t, double c)
        {
            t /= 0.5;
            if (t < 1.0)
                return c / 2.0 * t * t * t;

            t -= 2.0d;
            return c / 2.0 * (t * t * t + 2.0);
        }

        public static double EaseInOutExpo(double t, double b, double c, double d)
        {
            t /= d / 2.0;
            if (t < 1.0) return c / 2.0 * Math.Pow(2.0, 10.0 * (t - 1.0)) + b;
            t--;
            return c / 2.0 * (-Math.Pow(2.0, -10.0 * t) + 2.0) + b;
        }

        public static double EaseInOutExpo(double t, double c)
        {
            t /= 0.5;
            if (t < 1.0) return c / 2.0 * Math.Pow(2.0, 10.0 * (t - 1.0));
            t--;
            return c / 2.0 * (-Math.Pow(2.0, -10.0 * t) + 2.0);
        }

        public static double SafeAtan(double x, double y)
        {
            if (y == 0.0)
            {
                if (x == 0.0)
                    return 0.0;

                return x < 0.0 ? -ZMath.PI_2 : ZMath.PI_2;
            }

            return Math.Atan(x / y);
        }

    }

}
