
import Foundation

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
    
    init(_ v: ZVector2, _ o: Double)
    {
        position = ZVector2(v)
        self.orientation = ZRotation2(o)
    }
    
    init(_ x: Double, _ y: Double, _ o: Double)
    {
        position = ZVector2(x, y)
        self.orientation = ZRotation2(o)
    }
    
    func set(_ m: ZMatrix2)
    {
        position.set(m.position)
        orientation.set(m.orientation)
    }
    
    func set(_ x: Double, _ y: Double, _ o: Double)
    {
        position.set(x, y)
        orientation.rotation = o
    }
    
    func concat(_ m: ZMatrix2)
    {
        self.position.add(
            (m.position.getX() * self.orientation.cos) + (m.position.getY() * -self.orientation.sin),
            (m.position.getX() * self.orientation.sin) + (m.position.getY() * self.orientation.cos))
        self.orientation.add(m.orientation)
    }
    
    func concat(_ x: Double, _ y: Double, _ o: Double)
    {
        self.position.add(
            (x * self.orientation.cos) + (y * -self.orientation.sin),
            (x * self.orientation.sin) + (y * self.orientation.cos))
        self.orientation.add(o)
    }
    
    func concatTo(_ m: ZMatrix2)
    {
        //rotate and move
        self.position.rotate(m.orientation)
        self.position.add(m.position)
        self.orientation.add(m.orientation)
    }
    
    func unconcatFrom(_ m: ZMatrix2)
    {
        //rotate and move
        self.orientation.subtract(m.orientation)
        self.position.subtract(m.position)
        self.position.unrotate(m.orientation)
    }
    
}
