
import Foundation

class KPosition: NSObject
{
    
    var vector: KVector2
    var orientation: Double
    
    override init()
    {
        vector = KVector2()
        orientation = 0.0
    }
    
    init(_ p: KPosition)
    {
        vector = KVector2(p.vector)
        orientation = p.orientation
    }
    
    init(v: KVector2, o: Double)
    {
        vector = KVector2(v)
        orientation = o
    }
    
    init(x: Double, y: Double, o: Double)
    {
        vector = KVector2(x: x, y: y)
        orientation = o
    }
    
    func set(_ p: KPosition)
    {
        vector.set(v: p.vector)
        orientation = p.orientation
    }
    
    func set(x: Double, y: Double, o: Double)
    {
        vector.set(x: x, y: y);
        orientation = o;
    }
    
}
