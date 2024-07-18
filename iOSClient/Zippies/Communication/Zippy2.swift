
import Foundation
import UIKit

class Zippy2 : NSObject
{
    
    fileprivate var _x : CGFloat = 0
    fileprivate var _y : CGFloat = 0
    fileprivate var _o : CGFloat = 0

    var x : CGFloat
    {
        get {
            return _x
        }
    }
    
    var y : CGFloat
    {
        get {
            return _y
        }
    }
    
    var orientation : CGFloat
    {
        get {
            return _o
        }
    }
    
    override init()
    {
    }
    
    func setPosition(x: CGFloat, y: CGFloat, o: CGFloat)
    {
        self._x = x
        self._y = y
        self._o = o
    }

}
