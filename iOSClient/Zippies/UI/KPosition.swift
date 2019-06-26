
import Foundation
import UIKit

class KPosition
{
    
    var vector: KVector2
    var orientation: Double

    init()
    {
        vector = KVector2()
        orientation = 0.0
    }
    
    init(_ p: KPosition)
    {
        vector = KVector2(p.vector)
        orientation = p.orientation
    }
    
    init(_ v: KVector2, _ o: Double)
    {
        vector = KVector2(v)
        orientation = o
    }
    
    init(_ x: Double, _ y: Double, _ o: Double)
    {
        vector = KVector2(x, y)
        orientation = o
    }
    
    func set(_ p: KPosition)
    {
        vector.set(v: p.vector)
        orientation = p.orientation
    }
    
    func set(_ x: Double, _ y: Double, _ o: Double)
    {
        vector.set(x, y);
        orientation = o;
    }
    
}
