
import Foundation

let EPSILON: Double = 0.0001

class KVector2: NSObject
{
    
    fileprivate var x: Double
    fileprivate var y: Double
    fileprivate var d: Double
    fileprivate var dValid: Bool
    fileprivate var d2: Double
    fileprivate var d2Valid: Bool
    fileprivate var orientation: Double
    fileprivate var orientationValid: Bool

    override init()
    {
        self.x = 0.0
        self.y = 0.0
        self.d = 0.0
        self.dValid = true
        self.d2 = 0.0
        self.d2Valid = true
        self.orientation = 0.0
        self.orientationValid = true
    }

    init(_ v: KVector2)
    {
        self.x = v.x
        self.y = v.y
        self.d = v.d
        self.dValid = v.dValid
        self.d2 = v.d2
        self.d2Valid = v.d2Valid
        self.orientation = v.orientation
        self.orientationValid = v.orientationValid
    }

    init(_ x: Double, _ y: Double)
    {
        self.x = x
        self.y = y
        self.d2 = 0.0
        self.d2Valid = false
        self.d = 0.0
        self.dValid = false
        self.orientation = 0.0
        self.orientationValid = false
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

    func equalsVector(_ v: KVector2) -> Bool
    {
        return abs(v.x-x) < EPSILON && abs(v.y-y) < EPSILON
    }

    /**
     * Given two vectors, returns the cos(ϴ) * L1 * L2, where ϴ is the angle between
     * the vectors and L1 and L2 are the lengths of each vector. For unit vectors, the
     * result will be just cos(ϴ).
     */
    func dotVector(_ v: KVector2) -> Double
    {
        return (x*v.x) + (y*v.y)
    }

    func dotOrientation(_ o: Double) -> Double
    {
        return (x * sin(o)) + (y * cos(o))
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
        orientation = 0.0
        orientationValid = true
    }

    func setX(_ newX: Double)
    {
        if (x == newX) {
            return
        }
    
        x = newX
        dValid = false
        d2Valid = false
        orientationValid = false
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
        orientationValid = false
    }

    func getY() -> Double
    {
        return y
    }
    
    func set(v: KVector2)
    {
        self.x = v.x
        self.y = v.y
        self.d = v.d
        self.dValid = v.dValid
        self.d2 = v.d2
        self.d2Valid = v.d2Valid
        self.orientation = v.orientation
        self.orientationValid = v.orientationValid
    }

    func set(_ x: Double, _ y: Double)
    {
        self.x = x
        self.y = y
        dValid = false
        d2Valid = false
        orientationValid = false
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
            orientation = 0.0
            orientationValid = true
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
        orientationValid = false
    }

    func setD(_ newD: Double)
    {
        set(self.x, self.y, newD)
    }

    func addVector(_ v: KVector2)
    {
        self.x += v.x
        self.y += v.y
        dValid = false
        d2Valid = false
        orientationValid = false
    }

    func subtractVector(_ v: KVector2)
    {
        self.x -= v.x
        self.y -= v.y
        dValid = false
        d2Valid = false
        orientationValid = false
    }

    func projectAlong(_ orientation: Double) -> Double
    {
        let sinTheta = sin(orientation)
        let cosTheta = cos(orientation)
        let dotProduct = (x * sinTheta) + (y * cosTheta)
        self.x = dotProduct * sinTheta
        self.y = dotProduct * cosTheta
        let length = dotProduct / cosTheta
        self.d = abs(length)
        dValid = true
        //the result could be either the orientation specified or +M_PI, exactly the opposite direction
        self.orientation = dotProduct >= 0.0 ? orientation : addAngles(orientation, Double.pi)
        orientationValid = true
        d2Valid = false
    
        return length
    }

    func projectToward(_ orientation: Double) -> Double
    {
        let sinTheta = sin(orientation)
        let cosTheta = cos(orientation)
        let dotProduct = (x * sinTheta) + (y * cosTheta)
        let absDotProduct = abs(dotProduct)
        self.x = absDotProduct * sinTheta
        self.y = absDotProduct * cosTheta
        let length = dotProduct / cosTheta
        self.d = abs(length)
        dValid = true
        // dValid = false
        self.orientation = orientation
        orientationValid = true
        // orientationValid = false
        d2Valid = false
    
        return length
    }

    func getOrientation() -> Double
    {
        if (!orientationValid) {
            orientation = atan2(self.x, self.y)
            orientationValid = true
        }
    
        return orientation
    }

    func rotate(_ angleRadians: Double)
    {
        let currentLength = getD()
        orientation = addAngles(getOrientation(), angleRadians)
    
        self.x = currentLength * sin(orientation)
        self.y = currentLength * cos(orientation)
        orientationValid = true
    }

}

func snapAngle(_ angle: Double) -> Double
{
    if (angle <= -Double.pi) {
        return angle + (2.0 * Double.pi)
    }
    else if (angle > Double.pi) {
        return angle - (2.0 * Double.pi)
    }
    return angle
}

func subtractAngles(_ a1: Double, _ a2: Double) -> Double
{
    return snapAngle(a1 - a2)
}

func addAngles(_ a1: Double, _ a2: Double) -> Double
{
    return snapAngle(a1 + a2)
}

func distanceBetween(_ x1: Double, _ y1: Double, _ x2: Double, _ y2: Double) -> Double
{
    return sqrt(pow(x2 - x1, 2.0) + pow(y2 - y1, 2.0))
}

func distanceBetween(_ v1: KVector2, _ v2: KVector2) -> Double
{
    return distanceBetween(v1.getX(), v1.getY(), v2.getX(), v2.getY())
}

