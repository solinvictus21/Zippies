
import Foundation

class BiArc: NSObject
{
    let endingPosition: KPosition
    
    var startingPosition = KPosition()
    
    var arc1: Arc?
    var arc2: Arc?

    /*
    var centerPoint1 = KPosition()
    var radius1: Double = 0.0
    fileprivate var deltaOrientation1: Double = 0.0
    fileprivate var arcLength1: Double = 0.0
    
    var centerPoint2 = KPosition()
    var radius2: Double = 0.0
    fileprivate var deltaOrientation2: Double = 0.0
    fileprivate var arcLength2: Double = 0.0
    
    fileprivate var totalArcLength: Double = 0.0
     */

    init(_ endingPosition: KPosition)
    {
        self.endingPosition = KPosition(endingPosition)
    }
    
    init(x: Double, y: Double, o: Double)
    {
        self.endingPosition = KPosition(x: x, y: y, o: o)
    }

    func start(_ startingPosition: KPosition)
    {
        //derived from formulae and helpful explanations provided from the following site
        //  http://www.ryanjuckett.com/programming/biarc-interpolation/
        
        //p1 = starting position
        self.startingPosition.set(startingPosition)
        let p1x = startingPosition.vector.getX()
        let p1y = startingPosition.vector.getY()
        //t1 = starting orientation vector normal
        //   = (sin(p1.o), cos(p1.o))
        let t1x = sin(startingPosition.orientation)
        let t1y = cos(startingPosition.orientation)
        
        //p2 = target position
        let p2x = endingPosition.vector.getX()
        let p2y = endingPosition.vector.getY()
        //t2 = target orientation vector normal
        //   = (sin(p2.o), cos(p2.o))
        let t2x = sin(endingPosition.orientation)
        let t2y = cos(endingPosition.orientation)
        
        //t1 dot t2 = (t1.x * t2.x)           + (t1.y * t2.y)
        //          = (sin(p1.o) * sin(p2.o)) + (cos(p1.o) * cos(p2.o))
        let t1DotT2 = (t1x * t2x) + (t1y * t2y)
        
        let d: Double
        /*
         if (relativeDirectionOfMotion == 0.0) {
         if (y == 0.0) {
         *linearVelocity = x / 2.0
         *angularVelocity = x > 0.0 ? M_PI_2 : -M_PI_2
         return
         }
         
         //in this situation, our denominator becomes zero and d goes to infinity handle this with different formulae
         double vDotT2 = (x * sinO) + (y * cosO)
         d = relativeTargetPosition.getD2() / (4.0 * vDotT2)
         }
         else {
         */
        //v = p2 - p1
        //  = (p2.x - p1.x, p2.y - p1.y))
        let vx = p2x - p1x
        let vy = p2y - p1y
        //v dot v = (v.x * v.x) + (v.y * v.y)
        let vDotV = pow(vx, 2.0) + pow(vy, 2.0)
        //t = t1 + t2
        //  = (sin(p1.o) + sin(p2.o), cos(p1.o) + cos(p2.o))
        let tx = t1x + t2x
        let ty = t1y + t2y
        //v dot t = (v.x * t.x)                               + (v.y * t.y)
        //        = ((p2.x - p1.x) * (t1.x + t2.x))           + ((p2.y - p1.y) * (t1.y + t2.y))
        //        = ((p2.x - p1.x) * (sin(p1.o) + sin(p2.o))) + ((p2.y - p1.y) * (cos(p1.o) + cos(p2.o)))
        let vDotT = (vx * tx) + (vy * ty)
        //precalc = 2 * (1 - (t1 dot t2))
        let t1DotT2Inv2 = 2.0 * (1.0 - t1DotT2)
        d = ( -vDotT + sqrt( pow(vDotT, 2.0) + ( t1DotT2Inv2 * vDotV ) ) ) / t1DotT2Inv2
        // }
        
        //now find the location of the "knot" (the connection point between the arcs)
        //pm = ( p1 + p2 + (d * (t1 - t2)) ) / 2
        let pmx = ( p1x + p2x + (d * (t1x - t2x)) ) / 2.0
        let pmy = ( p1y + p2y + (d * (t1y - t2y)) ) / 2.0
        
        arc1 = Arc(startX: p1x, startY: p1y, endX: pmx, endY: pmy, tangentX: t1x, tangentY: t1y)
        let knotO = addAngles(a1: startingPosition.orientation, a2: arc1!.deltaAngle)
        arc2 = Arc(startX: pmx, startY: pmy, endX: p2x, endY: p2y, tangentX: sin(knotO), tangentY: cos(knotO))
    }
   
    /*
    func calculateArc(
        startX: Double, startY: Double,
        endX: Double, endY: Double,
        tangentX: Double, tangentY: Double) -> (KPosition, Double)
    {
        //pmp = the vector from the starting point to the ending point
        let deltaX = endX - startX
        let deltaY = endY - startY
        let deltaDotDelta = (deltaX * deltaX) + (deltaY * deltaY)
        let n2DotDelta = ((2.0 * -tangentY) * deltaX) + ((2.0 * tangentX) * deltaY)

        //the radius; a negative value indicates a turn forward to the right or backward to the left
        let radius = deltaDotDelta / n2DotDelta
        print("r = ", radius)

        //c = the center point
        let cx = startX + (radius * -tangentY)
        let cy = startY + (radius * tangentX)
        
        //op = the unit vector from the center point to the target position
        let opx = (px - cx) / radius
        let opy = (py - cy) / radius
        //opm = the unit vector from the center point to the knot
        let opmx = (pmx - cx) / radius
        let opmy = (pmy - cy) / radius
        let opCrossOpm = (opx * opmy) - (opy * opmx)
        print("o = ", opCrossOpm)
        let opDotOpm = (opx * opmx) + (opy * opmy)
        let arco = acos(opDotOpm)

        //now calculate the change in orientation
        let deltaOrientation: Double
        if d > 0 {
            deltaOrientation = (opCrossOpm > 0.0) ? arco : -arco
        }
        else {
            deltaOrientation = (opCrossOpm > 0.0) ? (-2.0 * Double.pi) + arco : (2.0 * Double.pi) - arco
        }
        return (KPosition(x: cx, y: cy, o: deltaOrientation), abs(radius))
    }
    */
    
    /*
    func loopTimed(normalizedTime: Double) -> KPosition
    {
        let distance = normalizedTime * totalArcLength
        let x, y, tangent: Double
        if distance < arcLength1 {
            let deltaAngle = deltaOrientation1 * normalizedTime
            let angleOnArc = addAngles(a1: centerPoint1.orientation, a2: deltaAngle)
            x = centerPoint1.vector.getX() + (radius1 * sin(angleOnArc))
            y = centerPoint1.vector.getY() + (radius1 * cos(angleOnArc))
            tangent = addAngles(a1: startingPosition.orientation, a2: deltaAngle)
        }
        else {
            let normalizedTime2 = normalizedTime - (arcLength1 / totalArcLength)
            let deltaAngle = deltaOrientation2 * normalizedTime2
            let angleOnArc = addAngles(a1: centerPoint2.orientation, a2: deltaAngle)
            x = centerPoint2.vector.getX() + (radius2 * sin(angleOnArc))
            y = centerPoint2.vector.getY() + (radius2 * cos(angleOnArc))
            tangent = addAngles(a1: addAngles(a1: startingPosition.orientation, a2: deltaOrientation1), a2: deltaAngle)
        }
        return KPosition(x: x, y: y, o: tangent)
    }
     */
    
}
