
import Foundation
import UIKit

class ZMatrix2: NSObject
{
    
    var position: ZVector2
    var orientation: ZRotation2

    override init()
    {
        position = ZVector2()
        orientation = ZRotation2()
    }
    
    init(_ p: ZMatrix2)
    {
        position = ZVector2(p.position)
        orientation = ZRotation2(p.orientation)
    }
    
    init(_ v: ZVector2, _ o: CGFloat)
    {
        position = ZVector2(v)
        self.orientation = ZRotation2(o)
    }
    
    init(_ x: CGFloat, _ y: CGFloat, _ o: CGFloat)
    {
        position = ZVector2(x, y)
        self.orientation = ZRotation2(o)
    }
    
    func set(_ m: ZMatrix2)
    {
        position.set(m.position)
        orientation.set(m.orientation)
    }
    
    func set(_ x: CGFloat, _ y: CGFloat, _ o: CGFloat)
    {
        position.set(x, y)
        orientation.rotation = o
    }
    
    func concat(_ m: ZMatrix2)
    {
        //rotate and move
        self.position.unrotate(m.orientation)
        self.position.addVector(m.position)
        self.orientation.add(m.orientation)
    }
    
    func unconcat(_ m: ZMatrix2)
    {
        //rotate and move
        self.orientation.subtract(m.orientation)
        self.position.subtractVector(m.position)
        self.position.rotate(m.orientation)
    }
    
}
