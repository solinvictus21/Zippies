
import Foundation
import UIKit

class KMatrix2: NSObject
{
    
    var position: KVector2
    var orientation: KRotation2

    override init()
    {
        position = KVector2()
        orientation = KRotation2()
    }
    
    init(_ p: KMatrix2)
    {
        position = KVector2(p.position)
        orientation = KRotation2(p.orientation)
    }
    
    init(_ v: KVector2, _ o: Double)
    {
        position = KVector2(v)
        self.orientation = KRotation2(o)
    }
    
    init(_ x: Double, _ y: Double, _ o: Double)
    {
        position = KVector2(x, y)
        self.orientation = KRotation2(o)
    }
    
    func set(_ m: KMatrix2)
    {
        position.set(m.position)
        orientation.set(m.orientation)
    }
    
    func set(_ x: Double, _ y: Double, _ o: Double)
    {
        position.set(x, y);
        orientation.rotation = o
    }
    
    func concat(_ m: KMatrix2)
    {
        //rotate and move
        self.position.unrotate(m.orientation)
        self.position.addVector(m.position)
        self.orientation.add(m.orientation)
    }
    
    func unconcat(_ m: KMatrix2)
    {
        //rotate and move
        self.orientation.subtract(m.orientation)
        self.position.subtractVector(m.position)
        self.position.rotate(m.orientation)
    }
    
}
