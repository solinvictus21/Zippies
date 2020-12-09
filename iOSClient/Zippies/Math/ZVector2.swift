
import Foundation

let EPSILON: Double = 0.0001

class ZVector2: NSObject
{
    
    fileprivate var x: Double
    fileprivate var y: Double
    fileprivate var d: Double
    fileprivate var dValid: Bool
    fileprivate var d2: Double
    fileprivate var d2Valid: Bool
    fileprivate var _atan: Double
    fileprivate var atanValid: Bool
    fileprivate var _atan2: Double
    fileprivate var atan2Valid: Bool

    override init()
    {
        self.x = 0.0
        self.y = 0.0
        self.d = 0.0
        self.dValid = true
        self.d2 = 0.0
        self.d2Valid = true
        self._atan = 0.0
        self.atanValid = true
        self._atan2 = 0.0
        self.atan2Valid = true
    }

    init(_ v: ZVector2)
    {
        self.x = v.x
        self.y = v.y
        self.d = v.d
        self.dValid = v.dValid
        self.d2 = v.d2
        self.d2Valid = v.d2Valid
        self._atan = v._atan
        self.atanValid = v.atanValid
        self._atan2 = v._atan2
        self.atan2Valid = v.atan2Valid
    }

    init(_ x: Double, _ y: Double)
    {
        self.x = x
        self.y = y
        self.d2 = 0.0
        self.d2Valid = false
        self.d = 0.0
        self.dValid = false
        self._atan = 0.0
        self.atanValid = false
        self._atan2 = 0.0
        self.atan2Valid = false
    }
    
    init(_ x: Double, _ y: Double, _ ofLength: Double)
    {
        if ofLength == 0.0 || (x == 0.0 && y == 0.0) {
            self.x = 0.0
            self.y = 0.0
            d = 0.0
            d2 = 0.0
        }
        else {
            //calculate the actual values from the unit vector
            let vd = sqrt(x*x+y*y)
            d = abs(ofLength)
            self.x = (x*d)/vd
            self.y = (y*d)/vd
            d2 = d*d
        }
        dValid = true
        d2Valid = true
        self._atan = 0.0
        self.atanValid = false
        self._atan2 = 0.0
        self.atan2Valid = false
    }

    var atan: Double {
        get {
            if !atanValid {
                if self.y == 0.0 {
                    if self.x == 0.0 {
                        _atan = 0.0
                    }
                    else if self.x < 0.0 {
                        _atan = -Double.pi / 2.0
                    }
                    else {
                        _atan = Double.pi / 2.0
                    }
                }
                else {
                    _atan = Double(Foundation.atan(Double(self.x / self.y)))
                }
                self.atanValid = true
            }
            
            return self._atan
        }
    }
    
    var atan2: Double {
        get {
            if (!atan2Valid) {
                _atan2 = Double(Foundation.atan2(Double(self.x), Double(self.y)))
                atan2Valid = true
            }
            
            return _atan2
        }
    }
    
    var sin: Double {
        get {
            guard getD() != 0.0 else {
                return 0.0
            }
            
            return x / d
        }
    }

    var cos: Double {
        get {
            guard getD() != 0.0 else {
                return 1.0
            }
            
            return y / d
        }
    }

    func getD() -> Double
    {
        if (!dValid) {
            d = sqrt(self.getD2())
            dValid = true
        }
        return d
    }

    func getD2() -> Double
    {
        if (!d2Valid) {
            if (dValid) {
                d2 = d*d
            }
            else {
                d2 = self.x*self.x + self.y*self.y
            }
            d2Valid = true
        }
    
        return d2
    }

    func equalsVector(_ v: ZVector2) -> Bool
    {
        return abs(v.x-x) < EPSILON && abs(v.y-y) < EPSILON
    }

    /**
     * Given two vectors, returns the cos(ϴ) * L1 * L2, where ϴ is the angle between
     * the vectors and L1 and L2 are the lengths of each vector. For unit vectors, the
     * result will be just cos(ϴ).
     */
    func dotVector(_ v: ZVector2) -> Double
    {
        return (x*v.x) + (y*v.y)
    }

    func dotVector(_ x2: Double, _ y2: Double) -> Double
    {
        return (x*x2) + (y*y2)
    }

    func dotOrientation(_ o: Double) -> Double
    {
        return (x * Double(Foundation.sin(Double(o)))) + (y * Double(Foundation.cos(Double(o))))
    }

    func multiply(_ m: Double)
    {
        x *= m
        y *= m
        if (dValid) {
            d *= m
        }
        d2Valid = false
    }

    func reset()
    {
        x = 0.0
        y = 0.0
        d = 0.0
        dValid = true
        d2 = 0.0
        d2Valid = true
        _atan = 0.0
        atanValid = true
        _atan2 = 0.0
        atan2Valid = true
    }

    func setX(_ newX: Double)
    {
        if (x == newX) {
            return
        }
    
        x = newX
        dValid = false
        d2Valid = false
        atanValid = false
        atan2Valid = false
    }
    
    func getX() -> Double
    {
        return x
    }

    func setY(_ newY: Double)
    {
        if (y == newY) {
            return
        }
    
        y = newY
        dValid = false
        d2Valid = false
        atanValid = false
        atan2Valid = false
    }

    func getY() -> Double
    {
        return y
    }
    
    func set(_ v: ZVector2)
    {
        self.x = v.x
        self.y = v.y
        self.d = v.d
        self.dValid = v.dValid
        self.d2 = v.d2
        self.d2Valid = v.d2Valid
        self._atan = v._atan
        self.atanValid = v.atanValid
        self._atan2 = v._atan2
        self.atan2Valid = v.atan2Valid
    }

    func set(_ x: Double, _ y: Double)
    {
        self.x = x
        self.y = y
        dValid = false
        d2Valid = false
        atanValid = false
        atan2Valid = false
    }

    func set(_ x: Double, _ y: Double, _ ofLength: Double)
    {
        if (ofLength == 0.0 || (x == 0.0 && y == 0.0)) {
            self.x = 0.0
            self.y = 0.0
            d = 0.0
            dValid = true
            d2 = 0.0
            d2Valid = true
            _atan = 0.0
            atanValid = true
            _atan2 = 0.0
            atan2Valid = true
            return
        }
    
        //calculate the actual values from the unit vector
        let vd = sqrt(x*x+y*y)
        self.x = (x*ofLength)/vd
        self.y = (y*ofLength)/vd
        d = ofLength
        dValid = true
        d2 = d*d
        d2Valid = true
        atanValid = false
        atan2Valid = false
    }

    func setD(_ newD: Double)
    {
        set(self.x, self.y, newD)
    }

    func add(_ v: ZVector2)
    {
        add(v.x, v.y)
    }

    func add(_ x: Double, _ y: Double)
    {
        self.x += x
        self.y += y
        dValid = false
        d2Valid = false
        atanValid = false
        atan2Valid = false
    }

    func subtract(_ v: ZVector2)
    {
        subtract(v.x, v.y)
    }
    
    func subtract(_ x: Double, _ y: Double)
    {
        self.x -= x
        self.y -= y
        dValid = false
        d2Valid = false
        atanValid = false
        atan2Valid = false
    }

    func projectAlong(_ orientation: Double) -> Double
    {
        let sinTheta = Double(Foundation.sin(Double(orientation)))
        let cosTheta = Double(Foundation.cos(Double(orientation)))
        let dotProduct = (x * sinTheta) + (y * cosTheta)
        self.x = dotProduct * sinTheta
        self.y = dotProduct * cosTheta
        let length = dotProduct / cosTheta
        self.d = abs(length)
        dValid = true
        atanValid = false
        //the result could be either the orientation specified or +M_PI, exactly the opposite direction
        self._atan2 = dotProduct >= 0.0 ? orientation : addAngles(orientation, Double.pi)
        atan2Valid = true
        d2Valid = false
    
        return length
    }

    func projectToward(_ orientation: Double) -> Double
    {
        let sinTheta = Double(Foundation.sin(Double(orientation)))
        let cosTheta = Double(Foundation.cos(Double(orientation)))
        let dotProduct = (x * sinTheta) + (y * cosTheta)
        let absDotProduct = abs(dotProduct)
        self.x = absDotProduct * sinTheta
        self.y = absDotProduct * cosTheta
        let length = dotProduct / cosTheta
        self.d = abs(length)
        dValid = true
        // dValid = false
        atanValid = false
        self._atan2 = orientation
        atan2Valid = true
        // orientationValid = false
        d2Valid = false
    
        return length
    }

    func getOrientation() -> Double
    {
        return atan2
    }

    func unrotate(_ angleRadians: Double)
    {
        let currentLength = getD()
        _atan2 = addAngles(getOrientation(), angleRadians)
    
        self.x = currentLength * Foundation.sin(_atan2)
        self.y = currentLength * Foundation.cos(_atan2)
        atanValid = false
        atan2Valid = true
    }
    
    func unrotate(_ rotation: ZRotation2) {
        let newX = (self.x *  rotation.cos) + (self.y * -rotation.sin)
        let newY = (self.x *  rotation.sin) + (self.y *  rotation.cos)
        self.x = newX
        self.y = newY
        atanValid = false
        atan2Valid = false
    }
    
    func rotate(_ rotation: ZRotation2) {
        let newX = (self.x *  rotation.cos) + (self.y *  rotation.sin)
        let newY = (self.x * -rotation.sin) + (self.y *  rotation.cos)
        self.x = newX
        self.y = newY
        atanValid = false
        atan2Valid = false
    }
    
    public override var description: String
    {
        get { return "(\(self.x), \(self.y))" }
    }
    
}

func distanceBetween(_ v1: ZVector2, _ v2: ZVector2) -> Double
{
    return distanceBetween(v1.getX(), v1.getY(), v2.getX(), v2.getY())
}

func distanceBetween(_ x1: Double, _ y1: Double, _ x2: Double, _ y2: Double) -> Double
{
    return sqrt(pow(x2 - x1, 2.0) + pow(y2 - y1, 2.0))
}

