using System;

namespace ZippyMath
{

    public class ZMatrix2
    {

        public ZMatrix2()
        {
            this.Position = new ZVector2();
            this.Orientation = new ZOrientation2(0);
        }

        public ZMatrix2(ZMatrix2 m)
        {
            this.Position = new ZVector2(m.Position);
            this.Orientation = new ZOrientation2(m.Orientation);
        }

        public ZMatrix2(double x, double y, double orientation)
        {
            this.Position = new ZVector2(x, y);
            this.Orientation = new ZOrientation2(orientation);
        }

        public void Reset()
        {
            this.Position.Reset();
            this.Orientation.Reset();
        }

        public ZVector2 Position
        { get; }

        public ZOrientation2 Orientation
        { get; }

        public double x
        {
            get
            {
                return Position.x;
            }
        }

        public double y
        {
            get
            {
                return Position.y;
            }
        }

        public double o
        {
            get
            {
                return Orientation;
            }
        }

        public double D
        {
            get
            {
                return Position.D;
            }
        }

        public double D2
        {
            get
            {
                return Position.D2;
            }
        }

        public double Atan
        {
            get
            {
                return Position.Atan;
            }
        }

        public double Atan2
        {
            get
            {
                return Position.Atan2;
            }
        }

        public void Set(ZMatrix2 m)
        {
            Position.Set(m.Position);
            Orientation.Set(m.Orientation);
        }

        public void Set(double x, double y, double o)
        {
            Position.Set(x, y);
            Orientation.Set(o);
        }

        public void Concat(ZMatrix2 m)
        {
            Position.Add(
                (m.Position.x * this.Orientation.Cos) + (m.Position.y * this.Orientation.Sin),
                (m.Position.x * -this.Orientation.Sin) + (m.Position.y * this.Orientation.Cos));
            Orientation.Add(m.Orientation);
        }

        public void Concat(double x, double y, double o)
        {
            Position.Add(
                (x * this.Orientation.Cos) + (y * this.Orientation.Sin),
                (x * -this.Orientation.Sin) + (y * this.Orientation.Cos));
            Orientation.Add(o);
        }

        public void Concat(ZVector2 v)
        {
            Position.Add(
                (v.x * this.Orientation.Cos) + (v.y * this.Orientation.Sin),
                (v.x * -this.Orientation.Sin) + (v.y * this.Orientation.Cos));
            Orientation.Add(2.0 * v.Atan);
        }

        public void Concat(double x, double y)
        {
            Position.Add(
                (x * this.Orientation.Cos) + (y * this.Orientation.Sin),
                (x * -this.Orientation.Sin) + (y * this.Orientation.Cos));
            Orientation.Add(2.0 * ZMath.SafeAtan(x, y));
        }

        public void Unconcat(ZMatrix2 m)
        {
            Orientation.Subtract(m.Orientation);
            Position.Subtract(
                (m.Position.x * this.Orientation.Cos) + (m.Position.y * this.Orientation.Sin),
                (m.Position.x * -this.Orientation.Sin) + (m.Position.y * this.Orientation.Cos));
            Orientation.Add(m.Orientation);
        }

        public void ConcatTo(ZMatrix2 m)
        {
            Position.Rotate(m.Orientation);
            Position.Add(m.Position);
            Orientation.Add(m.Orientation);
        }

        public void UnconcatFrom(ZMatrix2 m)
        {
            Orientation.Subtract(m.Orientation);
            Position.Subtract(m.Position);
            Position.Unrotate(m.Orientation);
        }

        /*
        public void Concat(ZMatrix2 m)
        {
            Concat(m._x, m._y, m._o);
        }

        public void Concat(double x, double y, double o)
        {
            //Debug.Log("Concat: " + x + ", " + y + ", " + o);
            double cosO = Math.Cos(_o);
            double sinO = Math.Sin(_o);
            add((x * cosO) + (y * sinO),
                (x * -sinO) + (y * cosO));
            _o = ZMath.AddAngles(_o, o);
            arctanValid = false;
            arctan2Valid = false;
        }

        public void ConcatTo(ZMatrix2 m)
        {
            //unrotate and move
            unrotate(m._o);
            add(m._x, m._y);
            _o = ZMath.AddAngles(_o, m._o);
            arctanValid = false;
            arctan2Valid = false;
        }

        public void Unconcat(ZMatrix2 m)
        {
            Unconcat(m._x, m._y, m._o);
        }

        public void Unconcat(double x, double y, double o)
        {
            _o = ZMath.SubtractAngles(_o, o);
            double cosO = Math.Cos(_o);
            double sinO = Math.Sin(_o);
            subtract((x * cosO) + (y * sinO),
                     (x * -sinO) + (y * cosO));
            arctanValid = false;
            arctan2Valid = false;
        }

        public void UnconcatFrom(ZMatrix2 m)
        {
            //move and rotate
            _o = ZMath.SubtractAngles(_o, m._o);
            subtract(m._x, m._y);
            rotate(m._o);
            arctanValid = false;
            arctan2Valid = false;
        }

        private void rotate(double rotation)
        {
            double cosO = Math.Cos(rotation);
            double sinO = Math.Sin(rotation);
            double newX = (this._x *  cosO) + (this._y * -sinO);
            double newY = (this._x *  sinO) + (this._y *  cosO);
            this._x = newX;
            this._y = newY;
        }

        private void unrotate(double rotation)
        {
            double cosO = Math.Cos(rotation);
            double sinO = Math.Sin(rotation);
            double newX = (this._x *  cosO) + (this._y *  sinO);
            double newY = (this._x * -sinO) + (this._y *  cosO);
            this._x = newX;
            this._y = newY;
        }

        private void add(double x, double y)
        {
            this._x += x;
            this._y += y;
        }

        private void subtract(double x, double y)
        {
            this._x -= x;
            this._y -= y;
        }
        */

        public override string ToString()
        {
            return String.Format("({0,10:F2}, {1,10:F2}, {2,10:F2})", Position.x, Position.y, (ZMath.RADIANS_TO_ANGLES * Orientation));
        }

    }

}
