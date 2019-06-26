
import Foundation
import UIKit

fileprivate let LINEAR_EPSILON: Double = 2.0
fileprivate let LINEAR2_EPSILON: Double = 4.0
fileprivate let ANGULAR_EPSILON: Double = 0.017453292519943
//the delta angle within which is to be considered "pointing at the desired orientation" (1 degree)
fileprivate let SEMICIRCLE_QUARTER = Double.pi/2

fileprivate let ZIPPY_ORIENTATION_LENGTH: Double = 60
fileprivate let ZIPPY_POINT_INDICATOR_RADIUS: Double = 12

let lightGreen = UIColor.green.withAlphaComponent(0.375)
let lightLightGreen = UIColor.green.withAlphaComponent(0.075)
let lightRed = UIColor.red.withAlphaComponent(0.25)
let lightLightRed = UIColor.red.withAlphaComponent(0.05)

func distanceZero(_ distance: Double) -> Bool
{
    return abs(distance) <= LINEAR_EPSILON;
}

func distance2Zero(_ distance2: Double) -> Bool
{
    return abs(distance2) <= LINEAR2_EPSILON;
}

func angleZero(_ angle: Double) -> Bool
{
    return abs(angle) <= ANGULAR_EPSILON;
}

func anglesEquivalent(_ angle1: Double, _ angle2: Double) -> Bool
{
    return abs(subtractAngles(angle2, angle1)) <= ANGULAR_EPSILON;
}

func planPath(
    _ startX: Double, _ startY: Double, _ startO: Double,
    _ endX: Double, _ endY: Double, _ endO: Double) -> ZDrawable
{
    //the following code is derived from formulae and helpful explanations provided from the following site
    //  http://www.ryanjuckett.com/programming/biarc-interpolation
    
    let deltaX = endX - startX
    let deltaY = endY - startY
    let relativeTargetPosition = KPosition(deltaX, deltaY, subtractAngles(endO, startO))
    relativeTargetPosition.vector.rotate(-startO)
    
    if (distance2Zero(relativeTargetPosition.vector.getD2())) {
        //no movement
        if (angleZero(relativeTargetPosition.orientation)) {
            //no turn do nothing
            print("Planned stop.")
            return ZDrawableCircle(UIColor.green, startX, startY, ZIPPY_POINT_INDICATOR_RADIUS)
        }
        
        //linear turn
        print("Planned linear turn.")
        return ZCompoundDrawable(
            ZDrawableArrow(UIColor.red, endX, endY, endO, ZIPPY_POINT_INDICATOR_RADIUS),
            ZDrawableArrow(UIColor.green, startX, startY, startO, ZIPPY_POINT_INDICATOR_RADIUS))
    }
    
    if (angleZero(relativeTargetPosition.orientation)) {
        //orientation at target is equivalent to the starting orientation
        if (distanceZero(relativeTargetPosition.vector.getX())) {
            //target is directly in front or behind
            //linear move
            print("Planned linear move.")
            return ZCompoundDrawable(
                ZDrawableArrow(UIColor.green, startX, startY, startO, ZIPPY_POINT_INDICATOR_RADIUS),
                ZDrawablePath(UIColor.green, Move(startX, startY, endO, relativeTargetPosition.vector.getY())),
                ZDrawableArrow(UIColor.green, endX, endY, endO, ZIPPY_POINT_INDICATOR_RADIUS))
        }
        
        /*
        if (distanceZero(relativeTargetPosition.vector.getY())) {
            //target is directly to the left or right
            //simplified version of a bi-arc move
            let pmx = startX + (deltaX / 2.0)
            let pmy = startY + (deltaY / 2.0)
            let arc1 = Arc(startX, startY, startO, pmx, pmy)
            let pmo = addAngles(startO, arc1.deltaAngle)
            let arc2 = Arc(pmx, pmy, pmo, endX, endY)
            print("Planned half-circle bi-arc.")
            return ZCompoundDrawable(
                ZDrawableCircle(lightLightGreen, arc1.center.getX(), arc1.center.getY(), arc1.radius, true),
                ZDrawableCircle(lightLightRed, arc2.center.getX(), arc2.center.getY(), arc2.radius, true),
                ZDrawablePath(UIColor.green, arc1),
                ZDrawablePath(UIColor.red, arc2),
                ZDrawableArrow(UIColor.red, endX, endY, endO, ZIPPY_POINT_INDICATOR_RADIUS),
                ZDrawableArrow(UIColor.green, startX, startY, startO, ZIPPY_POINT_INDICATOR_RADIUS))
        }
         */
        
        //everything else must be a bi-arc move in this particular situation, our denominator for our default formula to
        //calculate d would become zero and d would go to infinity, so we handle this with a different formula
        print("Planned complex bi-arc to matching orientation.")
        /*
        tx = sin(endO)
        ty = cos(endO)
        let vDotT = (deltaX * tx) + (deltaY * ty)
        d = ((deltaX * deltaX) + (deltaY * deltaY)) / (4.0 * vDotT)
        invD = -d
         */
        
        //now find the "knot" (the connection point between the arcs)
        //pm = ( p1 + p2 + (d * (t1 - t2)) ) / 2
        let pmx = ( startX + endX ) / 2.0
        let pmy = ( startY + endY ) / 2.0
        let arc1 = Arc(startX, startY, startO, pmx, pmy)
        let arc2 = Arc(pmx, pmy, addAngles(startO, arc1.deltaAngle), endX, endY)
        return drawBiArc(
            startX, startY, startO,
            endX, endY, endO,
            arc1, arc2)
    }
    
    let orientationAtTarget = addAngles(
        relativeTargetPosition.vector.getOrientation(),
        relativeTargetPosition.vector.getOrientation())
//        print(" td:", relativeTargetPosition.vector.getOrientation())
//        print("oat:", orientationAtTarget)
//        print(" to:", relativeTargetPosition.orientation)
    if anglesEquivalent(orientationAtTarget, relativeTargetPosition.orientation) {
        print("Planned simple arc.")
        return ZCompoundDrawable(
            ZDrawableArrow(UIColor.green, startX, startY, startO, ZIPPY_POINT_INDICATOR_RADIUS),
            ZDrawablePath(UIColor.green, Arc(startX, startY, startO, endX, endY)),
            ZDrawableArrow(UIColor.green, endX, endY, endO, ZIPPY_POINT_INDICATOR_RADIUS))
    }
    
    //v dot v = (v.x * v.x) + (v.y * v.y)
    let vDotV = pow(deltaX, 2.0) + pow(deltaY, 2.0)
    
    let t1x = sin(startO)
    let t1y = cos(startO)
    let t2x = sin(endO)
    let t2y = cos(endO)
    
    //t = t1 + t2
    //  = (sin(p1.o) + sin(p2.o), cos(p1.o) + cos(p2.o))
    let tx = t1x + t2x
    let ty = t1y + t2y
    //v dot t = (v.x * t.x)                               + (v.y * t.y)
    //        = ((p2.x - p1.x) * (t1.x + t2.x))           + ((p2.y - p1.y) * (t1.y + t2.y))
    //        = ((p2.x - p1.x) * (sin(p1.o) + sin(p2.o))) + ((p2.y - p1.y) * (cos(p1.o) + cos(p2.o)))
    let vDotT = (deltaX * tx) + (deltaY * ty)
    
    //t1 dot t2 = (t1.x * t2.x)           + (t1.y * t2.y)
    //          = (sin(p1.o) * sin(p2.o)) + (cos(p1.o) * cos(p2.o))
    let t1DotT2 = (t1x * t2x) + (t1y * t2y)
    
    print("Planned complex bi-arc.")
    //precalc = 2 * (1 - (t1 dot t2))
    let t1DotT2Inv2 = 2.0 * (1.0 - t1DotT2)
    let discrim = sqrt( pow(vDotT, 2.0) + ( t1DotT2Inv2 * vDotV ) )
    
    let d = ( -vDotT + discrim ) / t1DotT2Inv2
    let invD = ( -vDotT - discrim ) / t1DotT2Inv2
    
    //now find the "knot" (the connection point between the arcs)
    //pm = ( p1 + p2 + (d * (t1 - t2)) ) / 2
    let pmx = ( startX + endX + (d * (t1x - t2x)) ) / 2.0
    let pmy = ( startY + endY + (d * (t1y - t2y)) ) / 2.0
//    print("d:", d, "   knot:", pmx, ",", pmy)

    //calculate the the two arcs
    if distance2Zero(pow(pmx - startX, 2.0) + pow(pmy - startY, 2.0)) {
//        print("Second arc is sufficient.")
        print("Planned simple second arc.")
        return ZCompoundDrawable(
            ZDrawableArrow(UIColor.green, startX, startY, startO, ZIPPY_POINT_INDICATOR_RADIUS),
            ZDrawablePath(UIColor.green, Arc(pmx, pmy, startO, endX, endY)),
            ZDrawableArrow(UIColor.green, endX, endY, endO, ZIPPY_POINT_INDICATOR_RADIUS))
    }

    if distance2Zero(pow(pmx - endX, 2.0) + pow(pmy - endY, 2.0)) {
//        print("First arc is sufficient.")
        print("Planned simple first arc.")
        return ZCompoundDrawable(
            ZDrawableArrow(UIColor.green, startX, startY, startO, ZIPPY_POINT_INDICATOR_RADIUS),
            ZDrawablePath(UIColor.green, Arc(startX, startY, startO, pmx, pmy)),
            ZDrawableArrow(UIColor.green, endX, endY, endO, ZIPPY_POINT_INDICATOR_RADIUS))
    }

    let arc1 = Arc(startX, startY, startO, pmx, pmy)
    let arc2 = Arc(pmx, pmy, addAngles(startO, arc1.deltaAngle), endX, endY)

//    print(" dp:", 180 * (relativeTargetPosition.vector.getOrientation() / Double.pi))
//    print(" do:", 180 * (relativeTargetPosition.orientation / Double.pi))

    let pmxr = ( startX + endX + (invD * (t1x - t2x)) ) / 2.0
    let pmyr = ( startY + endY + (invD * (t1y - t2y)) ) / 2.0
    let arc1r = Arc(startX, startY, startO, pmxr, pmyr)
    let arc2r = Arc(pmxr, pmyr, addAngles(startO, arc1r.deltaAngle), endX, endY)

    //move in reverse if the turn from moving forward is greater than the turn would be from moving in reverse
    let turnTowardTarget = 2.0 * relativeTargetPosition.vector.getOrientation()
    let turnTowardOrientation = subtractAngles(relativeTargetPosition.orientation, turnTowardTarget)
    if abs(turnTowardTarget + turnTowardOrientation) > Double.pi {
        //move the reverse way around the arc paths
        return drawBiArc(
            startX, startY, startO,
            endX, endY, endO,
            arc1r, arc2r,
            arc1, arc2)
    }
    
    //default use case; move forward toward target position and orientation
    return drawBiArc(
        startX, startY, startO,
        endX, endY, endO,
        arc1, arc2,
        arc1r, arc2r)
}

func drawBiArc(
    _ startX: Double, _ startY: Double, _ startO: Double,
    _ endX: Double, _ endY: Double, _ endO: Double,
    _ primaryArc1: Arc,
    _ primaryArc2: Arc,
    _ alternateArc1: Arc,
    _ alternateArc2: Arc) -> ZDrawable
{
    return ZCompoundDrawable(
        ZDrawableCircle(lightLightGreen, primaryArc1.center.getX(), primaryArc1.center.getY(), primaryArc1.radius, true),
        ZDrawableCircle(lightLightRed, primaryArc2.center.getX(), primaryArc2.center.getY(), primaryArc2.radius, true),
        ZDrawablePath(UIColor.green, primaryArc1),
        ZDrawablePath(UIColor.red, primaryArc2),
        ZDrawablePath(lightGreen, alternateArc1),
        ZDrawablePath(lightRed, alternateArc2),
        ZDrawableArrow(UIColor.green, startX, startY, startO, ZIPPY_POINT_INDICATOR_RADIUS),
        ZDrawableArrow(UIColor.red, endX, endY, endO, ZIPPY_POINT_INDICATOR_RADIUS))
}

func drawBiArc(
    _ startX: Double, _ startY: Double, _ startO: Double,
    _ endX: Double, _ endY: Double, _ endO: Double,
    _ primaryArc1: Arc,
    _ primaryArc2: Arc) -> ZDrawable
{
    return ZCompoundDrawable(
        ZDrawableCircle(lightLightGreen, primaryArc1.center.getX(), primaryArc1.center.getY(), primaryArc1.radius, true),
        ZDrawableCircle(lightLightRed, primaryArc2.center.getX(), primaryArc2.center.getY(), primaryArc2.radius, true),
        ZDrawablePath(UIColor.green, primaryArc1),
        ZDrawablePath(UIColor.red, primaryArc2),
        ZDrawableArrow(UIColor.green, startX, startY, startO, ZIPPY_POINT_INDICATOR_RADIUS),
        ZDrawableArrow(UIColor.red, endX, endY, endO, ZIPPY_POINT_INDICATOR_RADIUS))
}

func createTestPath() -> ZDrawable
{
//    return planPath(
//        -50.0, 500.0, Double.pi,
//         50.0, 500.0, Double.pi)
    
    //    let startPosition = KPosition(
    //        Double.random(in: -400...400),
    //        Double.random(in: -400...400),
    //        Double.random(in: -Double.pi..<Double.pi))
    let startPosition = KPosition(0, 0, 0)
    let randomPlan = Int.random(in: 0...20)
    let endPosition: KPosition
    switch (randomPlan) {
        
    case 0:
        print("Plotting stop.")
        endPosition = startPosition
        break
        
    case 1:
        print("Plotting linear turn.")
        endPosition = KPosition(
            startPosition.vector.getX(),
            startPosition.vector.getY(),
            Double.random(in: -Double.pi..<Double.pi))
        break
        
    case 2:
        print("Plotting linear move.")
        let distanceDelta = Double.random(in: -200...200)
        endPosition = KPosition(
            startPosition.vector.getX() + (distanceDelta * sin(startPosition.orientation)),
            startPosition.vector.getY() + (distanceDelta * cos(startPosition.orientation)),
            startPosition.orientation)
        break
        
    case 3:
        print("Plotting simple arc.")
        let radius = Double.random(in: -200...200)
        let centerX = startPosition.vector.getX() + (radius * -cos(startPosition.orientation))
        let centerY = startPosition.vector.getY() + (radius * sin(startPosition.orientation))
        let orientationDelta = Double.random(in: -Double.pi..<Double.pi)
        let arcEndAngle = addAngles(radius < 0 ? -Double.pi/2 : Double.pi/2, orientationDelta)
        endPosition = KPosition(
            centerX + (abs(radius) * sin(arcEndAngle)),
            centerY + (abs(radius) * cos(arcEndAngle)),
            addAngles(startPosition.orientation, orientationDelta))
        break
        
    case 4:
        print("Plotting simple arc to inverted orientation.")
        let radius = Double.random(in: -200...200)
        let centerX = startPosition.vector.getX() + (radius * -cos(startPosition.orientation))
        let centerY = startPosition.vector.getY() + (radius * sin(startPosition.orientation))
        let orientationDelta = Double.random(in: -Double.pi..<Double.pi)
        let arcEndAngle = addAngles(radius < 0 ? -Double.pi/2 : Double.pi/2, orientationDelta)
        endPosition = KPosition(
            centerX + (abs(radius) * sin(arcEndAngle)),
            centerY + (abs(radius) * cos(arcEndAngle)),
            addAngles(addAngles(startPosition.orientation, orientationDelta), Double.pi))
        break
        
    case 5:
        print("Plotting half-circle bi-arc.")
        let distanceDelta = Double.random(in: -200...200)
        endPosition = KPosition(
            startPosition.vector.getX() + (distanceDelta * -cos(startPosition.orientation)),
            startPosition.vector.getY() + (distanceDelta * sin(startPosition.orientation)),
            startPosition.orientation)
        break
        
    case 6:
        print("Plotting complex bi-arc to matching orientation.")
        endPosition = KPosition(
            Double.random(in: -400...400),
            Double.random(in: -400...400),
            startPosition.orientation)
        break
        
    default:
        print("Plotting random move.")
        endPosition = KPosition(Double.random(in: -400...400),
                                Double.random(in: -400...400),
                                Double.random(in: -Double.pi..<Double.pi))
        break
        
    }
    //    print("From:", startPosition.vector.getX(), ",", startPosition.vector.getY(),
    //          " at", 180 * (startPosition.orientation / Double.pi))
    //    print("  To:", endPosition.vector.getX(), ",", endPosition.vector.getY(),
    //          " at", 180 * (endPosition.orientation / Double.pi))
    let moveToDisplay = planPath(
        startPosition.vector.getX(), startPosition.vector.getY(), startPosition.orientation,
        endPosition.vector.getX(), endPosition.vector.getY(), endPosition.orientation)
    print()
    return moveToDisplay
}

