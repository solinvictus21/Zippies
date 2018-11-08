
import Foundation

class Zippy2 : NSObject
{
    
    fileprivate let id : UUID
    fileprivate var _x : Double = 0
    fileprivate var _y : Double = 0
    fileprivate var _o : Double = 0

    var x : Double
    {
        get {
            return _x
        }
    }
    
    var y : Double
    {
        get {
            return _y
        }
    }
    
    var orientation : Double
    {
        get {
            return _o
        }
    }
    
    init(_ id: UUID, x: Double, y: Double, o: Double)
    {
        self.id = id
        self._x = x
        self._y = y
        self._o = o
    }
    
    func setPosition(x: Double, y: Double, o: Double)
    {
        self._x = x
        self._y = y
        self._o = o
    }

}
