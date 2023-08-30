
import Foundation
import UIKit

class ZVector3: NSObject
{
    
    var x: CGFloat
    var y: CGFloat
    var z: CGFloat

    override init()
    {
        x = 0
        y = 0
        z = 0
    }
    
    init(_ x: CGFloat, _ y: CGFloat, _ z: CGFloat)
    {
        self.x = x
        self.y = y
        self.z = z
    }
    
}
