
import Foundation
import UIKit

class ZRotation2: NSObject
{
    
    fileprivate var _rotation: CGFloat
    fileprivate var _sin: CGFloat?
    fileprivate var _cos: CGFloat?

    override init() {
        self._rotation = 0.0
        self._sin = nil
        self._cos = nil
    }
    
    init(_ rotation: CGFloat) {
        self._rotation = rotation
    }
    
    init(_ other: ZRotation2) {
        self._rotation = other._rotation
        self._sin = other._sin
        self._cos = other._cos
    }
    
    var rotation: CGFloat {
        get {
            return self._rotation
        }
        set(rotation) {
            self._rotation = rotation
            self._cos = nil
            self._sin = nil
        }
    }
    
    var sin: CGFloat {
        get {
            if _sin == nil {
                _sin = CGFloat(Foundation.sin(Double(_rotation)))
            }
            
            return _sin!
        }
    }
    
    var cos: CGFloat {
        get {
            if _cos == nil {
                _cos = CGFloat(Foundation.cos(Double(_rotation)))
            }
            
            return _cos!
        }
    }
    
    func set(_ other: ZRotation2) {
        self._rotation = other._rotation
        self._sin = other._sin
        self._cos = other._cos
    }
    
    func get() -> CGFloat { return self._rotation }
    
    func add(_ other: ZRotation2) {
        self._rotation = addAngles(self._rotation, other._rotation)
        self._sin = nil
        self._cos = nil
    }
    
    func subtract(_ other: ZRotation2) {
        self._rotation = subtractAngles(self._rotation, other._rotation)
        self._sin = nil
        self._cos = nil
    }
    
}
