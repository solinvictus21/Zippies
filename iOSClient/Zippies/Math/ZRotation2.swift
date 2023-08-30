
import Foundation

class ZRotation2: NSObject
{
    
    fileprivate var _rotation: Double
    fileprivate var _sin: Double?
    fileprivate var _cos: Double?

    override init() {
        self._rotation = 0.0
        self._sin = nil
        self._cos = nil
    }
    
    init(_ rotation: Double) {
        self._rotation = rotation
    }
    
    init(_ other: ZRotation2) {
        self._rotation = other._rotation
        self._sin = other._sin
        self._cos = other._cos
    }
    
    var rotation: Double
    {
        get {
            return self._rotation
        }
        set(rotation) {
            self._rotation = rotation
            self._cos = nil
            self._sin = nil
        }
    }
    
    var sin: Double
    {
        get {
            if _sin == nil {
                _sin = Double(Foundation.sin(Double(_rotation)))
            }
            
            return _sin!
        }
    }
    
    var cos: Double
    {
        get {
            if _cos == nil {
                _cos = Double(Foundation.cos(Double(_rotation)))
            }
            
            return _cos!
        }
    }
    
    func set(_ other: ZRotation2)
    {
        self._rotation = other._rotation
        self._sin = other._sin
        self._cos = other._cos
    }
    
    func get() -> Double { return self._rotation }
    
    func add(_ other: ZRotation2)
    {
        add(other._rotation)
    }
    
    func add(_ orientation: Double)
    {
        self._rotation = addAngles(self._rotation, orientation)
        self._sin = nil
        self._cos = nil
    }

    func subtract(_ other: ZRotation2)
    {
        subtract(other._rotation)
    }

    func subtract(_ orientation: Double)
    {
        self._rotation = subtractAngles(self._rotation, orientation)
        self._sin = nil
        self._cos = nil
    }
    
}
