
import Foundation
import UIKit

class Move: ZPath
{
    
    let start: ZMatrix2
    let deltaDistance: CGFloat
    
    init(_ start: ZMatrix2, _ deltaDistance: CGFloat)
    {
        self.start = ZMatrix2(start)
        self.deltaDistance = deltaDistance
    }
    
    func updatesPosition() -> Bool { return true }
    func getLength() -> CGFloat { return deltaDistance }
    func getDrawable(_ color: UIColor) -> ZDrawable
    {
        let endX = start.position.getX() + (deltaDistance * sin(start.orientation.get()))
        let endY = start.position.getY() + (deltaDistance * cos(start.orientation.get()))
//        print("move", start.position.getX(), start.position.getY(), endX, endY)
        return ZDrawableLine(
            color,
            start.position,
            endX, endY)
    }

    func interpolate(_ t: CGFloat, _ p: ZMatrix2)
    {
        let currentDistance = deltaDistance * t
        p.position.set(start.position.getX() + (currentDistance * sin(start.orientation.get())),
                       start.position.getY() + (currentDistance * cos(start.orientation.get())))
        p.orientation.rotation = start.orientation.get()
    }
    
}
