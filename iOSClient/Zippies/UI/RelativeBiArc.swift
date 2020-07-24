
import Foundation
import UIKit

class RelativeBiArc
{

    private(set) var relativeTargetPosition: ZMatrix2
    private var knotPosition = ZVector2()
    
    init(_ relativeTargetPosition: ZMatrix2)
    {
        self.relativeTargetPosition = relativeTargetPosition
    }
    
    func recalculateKnot()
    {
        if relativeTargetPosition.orientation.rotation == 0.0 {
            knotPosition.set(
                relativeTargetPosition.position.getX() / 2.0,
                relativeTargetPosition.position.getY() / 2.0)
            return
        }

        let t2x = relativeTargetPosition.orientation.sin
        let t2y = relativeTargetPosition.orientation.cos
        
        //v dot v = (v.x * v.x) + (v.y * v.y)
        //        = distanceToTarget ^ 2
        //v dot t = (v.x * t.x)                               + (v.y * t.y)
        //        = ((p2.x - p1.x) * (t1.x + t2.x))           + ((p2.y - p1.y) * (t1.y + t2.y))
        //        = ((p2.x - p1.x) * (sin(p1.o) + sin(p2.o))) + ((p2.y - p1.y) * (cos(p1.o) + cos(p2.o)))
        //t1 dot t2 = (t1.x * t2.x)           + (t1.y * t2.y)
        //          = (sin(p1.o) * sin(p2.o)) + (cos(p1.o) * cos(p2.o))
        //          = t2.y
        
        //use these calculations to find the appropriate d value for the bi-arc
        let vDotT = (relativeTargetPosition.position.getX() * t2x) +
            (relativeTargetPosition.position.getY() * (1.0 + t2y))
        //precalc = 2 * (1 - (t1 dot t2))
        let t1DotT2Inv2 = 2.0 * (1.0 - t2y)
        let discrim = sqrt( CGFloat(pow(Double(vDotT), 2.0)) + ( t1DotT2Inv2 * relativeTargetPosition.position.getD2() ) )
        
        //now find the smallest d value of the bi-arc to create the shortest bi-arc to the target
        var d = -vDotT + discrim
        let altD = -vDotT - discrim
        if abs(d) > abs(altD) {
            d = altD
        }
        d /= t1DotT2Inv2

        //now we can use the d value to calculation the position of the "knot", which is the intersection
        //of the two arcs which make up the bi-arc
        knotPosition.set(
            ( relativeTargetPosition.position.getX() + (d * (-t2x)) ) / 2.0,
            ( relativeTargetPosition.position.getY() + (d * (1.0 - t2y)) ) / 2.0)
    }

}
