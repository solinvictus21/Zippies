using System;

namespace ZippyMath
{

    public class ZVector2
    {

        private double _x, _y;

        private double _d;
        private bool dValid;
        private double _d2;
        private bool d2Valid;
        private double _arctan;
        private bool arctanValid;
        private double _arctan2;
        private bool arctan2Valid;

        public ZVector2()
        {
        }

        public ZVector2(ZVector2 m)
        {
            this._x = m._x;
            this._y = m._y;
            this._arctan = m._arctan;
            this.arctanValid = m.arctanValid;
            this._arctan2 = m._arctan2;
            this.arctan2Valid = m.arctan2Valid;
        }

        public ZVector2(double x, double y)
        {
            this._x = x;
            this._y = y;
        }

        public void Reset()
        {
            this._x = 0.0;
            this._y = 0.0;
            this._d = 0.0;
            this.dValid = true;
            this._d2 = 0.0;
            this.d2Valid = true;
            this._arctan = 0.0;
            this.arctanValid = true;
            this._arctan2 = 0.0;
            this.arctan2Valid = true;
        }

        public void Add(ZVector2 v)
        {
            Add(v._x, v._y);
        }

        public void Add(double x, double y)
        {
            this._x += x;
            this._y += y;
            dValid = false;
            d2Valid = false;
            arctanValid = false;
            arctan2Valid = false;
        }

        public void Subtract(ZVector2 v)
        {
            Subtract(v._x, v._y);
        }

        public void Subtract(double x, double y)
        {
            this._x -= x;
            this._y -= y;
            dValid = false;
            d2Valid = false;
            arctanValid = false;
            arctan2Valid = false;
        }

        public void Rotate(ZOrientation2 o)
        {
            double newX = (this._x * o.Cos) + (this._y * o.Sin);
            double newY = (this._x * -o.Sin) + (this._y * o.Cos);
            this._x = newX;
            this._y = newY;
            arctanValid = false;
            arctan2Valid = false;
        }

        public void Unrotate(ZOrientation2 o)
        {
            double newX = (this._x * o.Cos) + (this._y * -o.Sin);
            double newY = (this._x * o.Sin) + (this._y * o.Cos);
            this._x = newX;
            this._y = newY;
            arctanValid = false;
            arctan2Valid = false;
        }

        public double x
        {
            get
            {
                return _x;
            }
            set
            {
                _x = value;
                dValid = false;
                d2Valid = false;
                arctanValid = false;
                arctan2Valid = false;
            }
        }

        public double y
        {
            get
            {
                return _y;
            }
            set
            {
                _y = value;
                dValid = false;
                d2Valid = false;
                arctanValid = false;
                arctan2Valid = false;
            }
        }

        public double D
        {
            get
            {
                if (!dValid)
                {
                    _d = Math.Sqrt(D2);
                    dValid = true;
                }

                return _d;
            }
        }

        public double D2
        {
            get
            {
                if (!d2Valid)
                {
                    if (dValid)
                        _d2 = _d * _d;
                    else
                        _d2 = (_x * _x) + (_y * _y);
                    d2Valid = true;
                }

                return _d2;
            }
        }

        public double Atan
        {
            get
            {
                if (!arctanValid)
                {
                    _arctan = ZMath.SafeAtan(_x, _y);
                    arctanValid = true;
                }

                return _arctan;
            }
        }

        public double Atan2
        {
            get
            {
                if (!arctan2Valid)
                {
                    _arctan2 = Math.Atan2(this._x, this._y);
                    arctan2Valid = true;
                }

                return _arctan2;
            }
        }

        public void Set(ZVector2 m)
        {
            this._x = m._x;
            this._y = m._y;
            this._d = m._d;
            this.dValid = m.dValid;
            this._d2 = m._d2;
            this.d2Valid = m.d2Valid;
            this._arctan = m._arctan;
            this.arctanValid = m.arctanValid;
            this._arctan2 = m._arctan2;
            this.arctan2Valid = m.arctan2Valid;
        }

        public void Set(double x, double y)
        {
            this._x = x;
            this._y = y;
            dValid = false;
            d2Valid = false;
            arctanValid = false;
            arctan2Valid = false;
        }

        public double DotVector(double x2, double y2)
        {
            return (this._x * x2) + (this._y * y2);
        }

        public double CrossVector(double x2, double y2)
        {
            return (this._x * y2) - (this._y * x2);
        }

        /*
        public void rotate(double rotation)
        {
            double cosO = Math.Cos(rotation);
            double sinO = Math.Sin(rotation);
            double newX = (this._x * cosO) + (this._y * -sinO);
            double newY = (this._x * sinO) + (this._y * cosO);
            this._x = newX;
            this._y = newY;
        }

        public void unrotate(double rotation)
        {
            double cosO = Math.Cos(rotation);
            double sinO = Math.Sin(rotation);
            double newX = (this._x * cosO) + (this._y * sinO);
            double newY = (this._x * -sinO) + (this._y * cosO);
            this._x = newX;
            this._y = newY;
        }

        public void add(double x, double y)
        {
            this._x += x;
            this._y += y;
        }

        public void subtract(double x, double y)
        {
            this._x -= x;
            this._y -= y;
        }
        */

        public override string ToString()
        {
            return "(" + _x + ", " + _y + ")";
        }

    }

}
