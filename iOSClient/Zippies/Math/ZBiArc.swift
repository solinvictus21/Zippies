
import Foundation

class ZBiArc: NSObject
{

    let startPosition: ZVector2
    let startOrientation: Double
    let d1: Double
    let knotPosition: ZVector2
    let knotOrientation: Double
    let targetPosition: ZVector2
    let d2: Double
    let targetVelocity: ZVector2
    
    /*
    let autoD: Double
    let autoAltD: Double
     */

    convenience init(
        relativeTargetPosition: ZVector2,
        relativeTargetOrientation: Double)
    {
        let t2x = sin(relativeTargetOrientation)
        let t2y = cos(relativeTargetOrientation)

        guard relativeTargetOrientation != 0.0 else {
            let d = relativeTargetPosition.getY() / 2.0
            let relativeTargetVelocity = ZVector2(t2x * d, t2y * d)
            self.init(
                relativeTargetPosition: relativeTargetPosition,
                d1: d,
                relativeTargetVelocity: relativeTargetVelocity,
                d2: d,
                knotPosition: ZVector2(
                    relativeTargetPosition.getX() / 2.0,
                    relativeTargetPosition.getY() / 2.0))
            return
        }
        
        //v = p2 - p1
        //v dot v = (v.x * v.x) + (v.y * v.y)
        //        = distanceToTarget ^ 2
        let vDotV = pow(relativeTargetPosition.getX(), 2.0) + pow(relativeTargetPosition.getY(), 2.0)

        //v dot t1 = (v.x * t1.x)      + (v.y * t1.y)
        //         = (v.x * sin(p1.o)) + (v.y * cos(p1.o))
        //         = (v.x * 0) + (v.y * 1)
        //         = v.y
        
        //t = t1 + t2
        //v dot t  = (v.x * t.x)                     + (v.y * t.y)
        //         = (v.x * (t1.x + t2.x))           + (v.y * (t1.y + t2.y))
        //         = (v.x * (sin(p1.o) + sin(p2.o))) + (v.y * (cos(p1.o) + cos(p2.o)))
        //         = (v.x * sin(p2.o)) + (v.y * (1.0 + cos(p2.o))
        let vDotT =
            (relativeTargetPosition.getX() * t2x) +
            (relativeTargetPosition.getY() * (1.0 + t2y))

        //t1 dot t2 = (t1.x * t2.x)           + (t1.y * t2.y)
        //          = (sin(p1.o) * sin(p2.o)) + (cos(p1.o) * cos(p2.o))
        //          = t2.y
        let t1DotT2Inv2 = 2.0 * (1.0 - t2y)
        let discrim = sqrt( pow(vDotT, 2.0) + ( t1DotT2Inv2 * vDotV ) )
        let d = (-vDotT + discrim) / t1DotT2Inv2

        let relativeTargetVelocity = ZVector2(t2x * d, t2y * d)
        self.init(
            relativeTargetPosition: relativeTargetPosition,
            d1: d,
            relativeTargetVelocity: relativeTargetVelocity,
            d2: d)
    }
    
    convenience init(
        relativeTargetPosition: ZVector2,
        relativeTargetVelocity: ZVector2)
    {
        self.init(
            relativeTargetPosition: relativeTargetPosition,
            relativeTargetVelocity: relativeTargetVelocity,
            d2: relativeTargetVelocity.getD())
    }
    
    convenience init(
        relativeTargetPosition: ZVector2,
        d1: Double,
        relativeTargetVelocity: ZVector2)
    {
        let vDotV = relativeTargetPosition.getD2()
        let vDotT1 = relativeTargetPosition.getY()
        let vDotT2 =
            (relativeTargetPosition.getX() * relativeTargetVelocity.sin) +
            (relativeTargetPosition.getY() * relativeTargetVelocity.cos)
        let d2 = ((vDotV / 2.0) - (d1 * vDotT1)) / (vDotT2 - (d1 * (relativeTargetVelocity.cos - 1.0)))
        //((vDotV / 2.0) - (d1 * vDotT1)) / (vDotT2 - (d1 * (relativeTargetVelocity.cos - 1.0))) = d2
        //((vDotV / 2.0) - (d1 * vDotT1)) / (vDotT2 - (d1 * (relativeTargetVelocity.cos - 1.0))) = d2
        self.init(
            relativeTargetPosition: relativeTargetPosition,
            d1: d1,
            relativeTargetVelocity: relativeTargetVelocity,
            d2: d2)
    }
    
    convenience init(
        relativeTargetPosition: ZVector2,
        relativeTargetVelocity: ZVector2,
        d2: Double)
    {
        let vDotV = relativeTargetPosition.getD2()
        let vDotT1 = relativeTargetPosition.getY()
        let vDotT2 =
            (relativeTargetPosition.getX() * relativeTargetVelocity.sin) +
            (relativeTargetPosition.getY() * relativeTargetVelocity.cos)
        let d1 = ((vDotV / 2.0) - (d2 * vDotT2)) / (vDotT1 - (d2 * (relativeTargetVelocity.cos - 1.0)))
        self.init(
            relativeTargetPosition: relativeTargetPosition,
            d1: d1,
            relativeTargetVelocity: relativeTargetVelocity,
            d2: d2)
    }
    
    convenience fileprivate init(
        relativeTargetPosition: ZVector2,
        d1: Double,
        relativeTargetVelocity: ZVector2,
        d2: Double)
    {
        let d1Ratio = d1 / (d1 + d2)
        //knot.x = ( (p1.x + (d1 * t1.x) * (d2 / (d1+d2)) ) + ( (p2.x - (d2 * t2.x) * (d1 / (d1+d2)) )
        //       = (p2.x - (d2 * t2.x)) * d1Ratio
        //knot.y = ( (p1.y + (d1 * t1.y) * (d2 / (d1+d2)) ) + ( (p2.y - (d2 * t2.y) * (d1 / (d1+d2)) )
        //       = ( ( d1 * (d2 / (d1+d2)) ) + ( (p2.y - (d2 * t2.y) * (d1 / (d1+d2)) )
        //       = (d2 * d1Ratio) + ( (p2.y - (d2 * t2.y)) * d1Ratio )
        self.init(
            relativeTargetPosition: relativeTargetPosition,
            d1: d1,
            relativeTargetVelocity: relativeTargetVelocity,
            d2: d2,
            knotPosition: ZVector2(
                (relativeTargetPosition.getX() - (d2 * relativeTargetVelocity.sin)) * d1Ratio,
                (d2 * d1Ratio) + ((relativeTargetPosition.getY() - (d2 * relativeTargetVelocity.cos)) * d1Ratio)))
    }
    
    fileprivate init(
        relativeTargetPosition: ZVector2,
        d1: Double,
        relativeTargetVelocity: ZVector2,
        d2: Double,
        knotPosition: ZVector2)
    {
        self.startPosition = ZVector2()
        self.startOrientation = 0.0
        self.d1 = d1
        self.knotPosition = knotPosition
        self.knotOrientation = 2.0 * knotPosition.atan2
        self.targetPosition = relativeTargetPosition
        self.d2 = d2
        self.targetVelocity = relativeTargetVelocity
    }
    
    /*
    fileprivate func calculateBiArcD() -> Double
    {
        let deltaX = targetPosition.getX() - startPosition.getX()
        let deltaY = targetPosition.getY() - startPosition.getY()
        let t1x = sin(startOrientation)
        let t1y = cos(startOrientation)
        let t2x = targetVelocity.sin
        let t2y = targetVelocity.cos
        
        //t = t1 + t2
        //  = (sin(p1.o) + sin(p2.o), cos(p1.o) + cos(p2.o))
        let tx = t1x + t2x
        let ty = t1y + t2y
        
        //v dot v = (v.x * v.x) + (v.y * v.y)
        //v dot t = (v.x * t.x)                               + (v.y * t.y)
        //        = ((p2.x - p1.x) * (t1.x + t2.x))           + ((p2.y - p1.y) * (t1.y + t2.y))
        //        = ((p2.x - p1.x) * (sin(p1.o) + sin(p2.o))) + ((p2.y - p1.y) * (cos(p1.o) + cos(p2.o)))
        //t1 dot t2 = (t1.x * t2.x)           + (t1.y * t2.y)
        //          = (sin(p1.o) * sin(p2.o)) + (cos(p1.o) * cos(p2.o))
        
        //use these calculations to find the appropriate d value for the bi-arc
        return calculateBiArcD(
            pow(deltaX, 2.0) + pow(deltaY, 2.0),
            (deltaX * tx) + (deltaY * ty),
            (t1x * t2x) + (t1y * t2y))
    }

    fileprivate func calculateBiArcD(
        _ vDotV: Double,
        _ vDotT: Double,
        _ t1DotT2: Double) -> Double
    {
        //precalc = 2 * (1 - (t1 dot t2))
        let t1DotT2Inv2 = 2.0 * (1.0 - t1DotT2)
        let discrim = sqrt( pow(vDotT, 2.0) + ( t1DotT2Inv2 * vDotV ) )
        
        //now find the smallest d value of the bi-arc to create the shortest bi-arc to the target
        let d = -vDotT + discrim
        let altD = -vDotT - discrim
        if abs(d) > abs(altD) {
            return altD / t1DotT2Inv2
        }
        return d / t1DotT2Inv2
    }
    
    fileprivate func calculateAutoDs() -> Double
    {
        let deltaX = targetPosition.getX() - startPosition.getX()
        let deltaY = targetPosition.getY() - startPosition.getY()
        let t1x = sin(startOrientation)
        let t1y = cos(startOrientation)
        let t2x = targetVelocity.sin
        let t2y = targetVelocity.cos
        let tx = t1x + t2x
        let ty = t1y + t2y

        let vDotV = pow(deltaX, 2.0) + pow(deltaY, 2.0)
        let vDotT = (deltaX * tx) + (deltaY * ty)
        let t1DotT2 = (t1x * t2x) + (t1y * t2y)

        //precalc = 2 * (1 - (t1 dot t2))
        let t1DotT2Inv2 = 2.0 * (1.0 - t1DotT2)
        let discrim = sqrt( pow(vDotT, 2.0) + ( t1DotT2Inv2 * vDotV ) )
        
        //now find the smallest d value of the bi-arc to create the shortest bi-arc to the target
        let d = -vDotT + discrim
        let altD = -vDotT - discrim
        if abs(d) > abs(altD) {
            return altD / t1DotT2Inv2
        }
        return d / t1DotT2Inv2
    }
     */
    
}
