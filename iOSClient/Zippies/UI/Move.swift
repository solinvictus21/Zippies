
import Foundation
import UIKit

class Move: ZPath
{
    
    let startX: Double
    let startY: Double
    let startO: Double
    let deltaDistance: Double
    
    init(_ startX: Double, _ startY: Double, _ startO: Double, _ deltaDistance: Double)
    {
        self.startX = startX
        self.startY = startY
        self.startO = startO
        self.deltaDistance = deltaDistance
    }

    func interpolate(_ t: Double, _ p: KMatrix2)
    {
        let currentDistance = deltaDistance * t
        p.position.set(startX + (currentDistance * sin(startO)),
                     startY + (currentDistance * cos(startO)))
        p.orientation.rotation = self.startO
    }
}
