
import Foundation
import UIKit

class Turn: ZPath
{
    
    let startX: Double
    let startY: Double
    let startO: Double
    let deltaO: Double
    
    init(_ x: Double, _ y: Double, _ o: Double, _ deltaO: Double)
    {
        self.startX = x
        self.startY = y
        self.startO = o
        self.deltaO = deltaO
    }
    
    func interpolate(_ t: Double, _ p: KPosition)
    {
        p.orientation = addAngles(startO, deltaO * t);
    }

}
