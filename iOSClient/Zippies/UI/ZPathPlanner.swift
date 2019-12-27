
import Foundation
import UIKit

fileprivate let LINEAR_EPSILON: Double = 2.0
fileprivate let LINEAR2_EPSILON: Double = 4.0
fileprivate let ANGULAR_EPSILON: Double = 0.017453292519943
fileprivate let DOUBLE_EPSILON: Double = 0.000001
//the delta angle within which is to be considered "pointing at the desired orientation" (1 degree)
fileprivate let SEMICIRCLE_QUARTER = Double.pi/2

fileprivate let ZIPPY_ORIENTATION_LENGTH: Double = 60
fileprivate let ZIPPY_POINT_INDICATOR_RADIUS: Double = 10
fileprivate let ZIPPY_POINT_INDICATOR_ORTHOGONAL: Double = 50
fileprivate let ZIPPY_ORIENTATION_LINE: Double = 150

fileprivate let ZIPPY_RANGE_X: Double = 600
fileprivate let ZIPPY_RANGE_Y: Double = 400

let lightRed = UIColor.red.withAlphaComponent(0.20)
let lightLightRed = UIColor.red.withAlphaComponent(0.10)
let lightOrange = UIColor.orange.withAlphaComponent(0.25)
let lightYellow = UIColor.yellow.withAlphaComponent(0.70)
let lightGreen = UIColor.green.withAlphaComponent(0.30)
let lightLightGreen = UIColor.green.withAlphaComponent(0.20)
let lightBlue = UIColor.blue.withAlphaComponent(0.20)
let lightLightBlue = UIColor.blue.withAlphaComponent(0.10)
let lightPurple = UIColor.blue.withAlphaComponent(0.15)

//start of conversion
func planTestDrawable() -> ZDrawable
{
    print()
    
    let start = KMatrix2(
        Double.random(in: -ZIPPY_RANGE_X...ZIPPY_RANGE_X),
        Double.random(in: -ZIPPY_RANGE_Y...ZIPPY_RANGE_Y),
        Double.random(in: -Double.pi..<Double.pi))
    let target = generateRandomTarget(start)
    return planPathDrawable(start, target)
}

func planPathDrawable(_ start: KMatrix2, _ toPosition: KMatrix2) -> ZDrawable
{
    let path = planPath(start, toPosition)
    guard path != nil else {
        return ZDrawableCircle(UIColor.green, start.position.getX(), start.position.getY(), ZIPPY_POINT_INDICATOR_RADIUS, true)
    }
    
    return ZCompoundDrawable(
        ZDrawableArrow(UIColor.green, start, ZIPPY_POINT_INDICATOR_RADIUS),
        path!.getDrawable(UIColor.green),
        ZDrawableArrow(UIColor.red, toPosition, ZIPPY_POINT_INDICATOR_RADIUS))
}

func planPath(_ start: KMatrix2, _ toPosition: KMatrix2) -> ZPath?
{
    //determine if we need to plan a bi-arc move first
    let relativeTarget = KMatrix2(toPosition)
    relativeTarget.unconcat(start)
    if !requiresBiArcMove(relativeTarget) {
        return planRelativePath(start, relativeTarget)
    }
    
    //plan a bi-arc path move
    print("Planned bi-arc path subdivision.")
    let knot = KMatrix2()
    calculateRelativeBiArcKnot(relativeTarget, knot)
    
    let firstSegment = planRelativePath(start, knot)
    
    relativeTarget.unconcat(knot)
    knot.concat(start)
    let secondSegment = planRelativePath(knot, relativeTarget)
    
    if firstSegment == nil {
        return secondSegment
    }
    else if secondSegment == nil {
        return firstSegment
    }
    
    return CompositePath(firstSegment!, secondSegment!)
}

func planRelativePath(_ start: KMatrix2, _ relativeTarget: KMatrix2) -> ZPath?
{
    if relativeTarget.position.getD2() < DOUBLE_EPSILON {
        let deltaAngle = relativeTarget.orientation.get()
        if abs(deltaAngle) < DOUBLE_EPSILON {
            print("Planned stop.")
            return nil
        }
        
        print("Planned turn.")
        return Turn(start, deltaAngle)
    }
    
    if abs(relativeTarget.position.atan) < DOUBLE_EPSILON {
        print("Planned linear move.")
        return Move(start, relativeTarget.position.getY())
    }
    
    print("Planned simple arc.")
    return Arc(start, relativeTarget)
}

func requiresBiArcMove(_ relativeTarget: KMatrix2) -> Bool
{
    //a simple turn in place does not need a bi-arc
    if relativeTarget.position.getD2() < DOUBLE_EPSILON {
        return false
    }
    
    //a simple linear move forward or backward does not need a bi-arc
    let directionToTarget = relativeTarget.position.atan
    if abs(directionToTarget) < DOUBLE_EPSILON {
        return false
    }
    
    //a simple arc move that arrives at the torget position in the correct orientation also
    //does not require a bi-arc move
    let subtendedAngle = 2.0 * directionToTarget
    if abs(subtractAngles(subtendedAngle, relativeTarget.orientation.get())) < DOUBLE_EPSILON {
        return false
    }
    
    return true
}
//end of conversion

func createTestDrawable() -> ZDrawable
{
    print()

    let startPosition = KMatrix2(
        Double.random(in: -ZIPPY_RANGE_X...ZIPPY_RANGE_X),
        Double.random(in: -ZIPPY_RANGE_Y...ZIPPY_RANGE_Y),
        Double.random(in: -Double.pi..<Double.pi))
    let startpointDrawable = ZDrawableArrow(UIColor.green,
                                            startPosition,
                                            ZIPPY_POINT_INDICATOR_RADIUS)
    let targetPosition = generateRandomTarget(startPosition)
//    let targetPosition = KMatrix2(
//        Double.random(in: -ZIPPY_RANGE_X...ZIPPY_RANGE_X),
//        Double.random(in: -ZIPPY_RANGE_Y...ZIPPY_RANGE_Y),
//        Double.random(in: -Double.pi..<Double.pi))
    
    let targetIndicator = createTargetIndicator(targetPosition)
    let endpointDrawable = ZDrawableArrow(UIColor.red,
                                          targetPosition,
                                          ZIPPY_POINT_INDICATOR_RADIUS)

    //check if we can do a single-motion move first
    let relativeTargetPosition = KMatrix2(targetPosition)
    relativeTargetPosition.unconcat(startPosition)
    if distance2Zero(relativeTargetPosition.position.getD2()) {
        return ZCompoundDrawable(
            ZDrawableCircle(UIColor.green, startPosition.position, ZIPPY_POINT_INDICATOR_RADIUS),
            endpointDrawable,
            startpointDrawable)
    }
    
    let arrivalAngleAtTarget = 2.0 * atan(relativeTargetPosition.position.getX() / relativeTargetPosition.position.getY())
    if angleZero(arrivalAngleAtTarget) {
        let (distance, rotation) = createMove(relativeTargetPosition.position)
        return ZCompoundDrawable(
            convertToDrawable(UIColor.green, startPosition, distance, rotation),
            endpointDrawable,
            startpointDrawable)
    }
    
    let knot = KMatrix2()
    calculateRelativeBiArcKnot(relativeTargetPosition, knot)
    knot.orientation.rotation = 2.0 *  atan(knot.position.getX() / knot.position.getY())
    let (distance1, rotation1) = createRelativeMove(knot)
    knot.concat(startPosition)

    targetPosition.unconcat(knot)
    let (distance2, rotation2) = createRelativeMove(targetPosition)
    
    //now convert it all to drawables
    let drawable1 = convertToDrawable(UIColor.green, startPosition, distance1, rotation1)
    let drawable2 = convertToDrawable(UIColor.red, knot, distance2, rotation2)
    return ZCompoundDrawable(
        targetIndicator,
        drawable1,
        drawable2,
        endpointDrawable,
        ZDrawableArrow(UIColor.blue, knot, ZIPPY_POINT_INDICATOR_RADIUS),
        startpointDrawable)
}

fileprivate func createRelativeMove(_ relativeTargetPosition: KMatrix2) -> (Double?, KRotation2?)
{
    if distance2Zero(relativeTargetPosition.position.getD2()) {
        //no linear movement
        return (nil, createTurn(relativeTargetPosition.orientation.rotation))
    }

    return createMove(relativeTargetPosition.position)
}

fileprivate func createTurn(_ angle: Double) -> KRotation2?
{
    if angleZero(angle) {
        //no linear turn do nothing
        return nil
    }
    
    //linear turn
    return KRotation2(angle)
}

fileprivate func createMove(_ relativeTargetPosition: KVector2) -> (Double?, KRotation2?)
{
    let subtendedAngle = 2.0 * atan(relativeTargetPosition.getX() / relativeTargetPosition.getY())
    if angleZero(subtendedAngle) {
        //target is directly in front or behind. linear move
        return (relativeTargetPosition.getY(), nil)
    }
    
    //fallback in the simple use case is a simple arc
    let radius = relativeTargetPosition.getD() / (2.0 * sin(relativeTargetPosition.getOrientation()))
    return (radius * subtendedAngle, KRotation2(subtendedAngle))
}

fileprivate func convertToDrawable(_ color: UIColor,
                                   _ startPosition: KMatrix2,
                                   _ distanceToMove: Double?,
                                   _ subtendedAngle: KRotation2?) -> ZDrawable
{
    //check if we can do a single-motion move first
    guard distanceToMove != nil else {
        //no linear movement
        guard subtendedAngle != nil else {
            //no linear turn do nothing
            print("Planned stop.")
            return ZDrawableCircle(color, startPosition.position, ZIPPY_POINT_INDICATOR_RADIUS)
        }
        
        //linear turn
        print("Planned turn.")
        return ZDrawableCircle(color, startPosition.position, ZIPPY_POINT_INDICATOR_RADIUS)
    }
    
    guard subtendedAngle != nil else {
        //linear move
        print("Planned move.")
        let targetPosition = KMatrix2(0.0, distanceToMove!, 0.0)
        targetPosition.concat(startPosition)
        return ZDrawableLine(color, startPosition.position, targetPosition.position)
    }
    
    //fallback in the simple use case is a simple arc
    print("Planned arc.")
    return ZDrawablePath(color,
                         Arc(startPosition, distanceToMove!, subtendedAngle!.rotation))
}

func calculateRelativeBiArcKnot(_ relativeTargetPosition: KMatrix2,
                                _ knotPosition: KMatrix2)
{
    if angleZero(relativeTargetPosition.orientation.get()) {
        knotPosition.position.set(
            relativeTargetPosition.position.getX() / 2.0,
            relativeTargetPosition.position.getY() / 2.0)
        knotPosition.orientation.rotation = 2.0 * knotPosition.position.atan
        return
    }
    
    let t2x = sin(relativeTargetPosition.orientation.get())
    let t2y = cos(relativeTargetPosition.orientation.get())
    
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
    let d = calculateBiArcD(
        relativeTargetPosition.position.getD2(),
        vDotT,
        t2y)
    
    //now we can use the d value to calculation the position of the "knot", which is the intersection
    //of the two arcs which make up the bi-arc
    knotPosition.position.set(
        ( relativeTargetPosition.position.getX() + (d * (-t2x)) ) / 2.0,
        ( relativeTargetPosition.position.getY() + (d * (1.0 - t2y)) ) / 2.0)
    knotPosition.orientation.rotation = 2.0 * knotPosition.position.atan
}

fileprivate func calculateBiArcKnotAbsolute(_ startPosition: KMatrix2,
                                            _ endPosition: KMatrix2,
                                            _ targetVector: KVector2)
{
    //the following code is derived from formulae and helpful explanations provided from the following site
    //  http://www.ryanjuckett.com/programming/biarc-interpolation
    
    let deltaX = endPosition.position.getX() - startPosition.position.getX()
    let deltaY = endPosition.position.getY() - startPosition.position.getY()
    if anglesEquivalent(startPosition.orientation.rotation, endPosition.orientation.rotation) {
        //this would create an asymptote in the regular bi-arc equation because the dot product of our orientation with
        //the target orientation would be 1. so handle it as a special use case
        print("Planned simple bi-arc to matching orientation.")
        targetVector.set(startPosition.position.getX() + (deltaX / 2.0),
                         startPosition.position.getY() + (deltaY / 2.0))
        return
    }
    
    print("Planned complex bi-arc.")
    let t1x = sin(startPosition.orientation.rotation)
    let t1y = cos(startPosition.orientation.rotation)
    let t2x = sin(endPosition.orientation.rotation)
    let t2y = cos(endPosition.orientation.rotation)
    
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
    let d = calculateBiArcD(
        pow(deltaX, 2.0) + pow(deltaY, 2.0),
        (deltaX * tx) + (deltaY * ty),
        (t1x * t2x) + (t1y * t2y))
    
    //now we can use the d value to calculation the position of the "knot", which is the intersection
    //of the two arcs which make up the bi-arc
    targetVector.set(
        ( startPosition.position.getX() + endPosition.position.getX() + (d * (t1x - t2x)) ) / 2.0,
        ( startPosition.position.getY() + endPosition.position.getY() + (d * (t1y - t2y)) ) / 2.0)
}

func calculateBiArcD(_ startPosition: KMatrix2,
                     _ endPosition: KMatrix2) -> Double
{
    let deltaX = endPosition.position.getX() - startPosition.position.getX()
    let deltaY = endPosition.position.getY() - startPosition.position.getY()
    let t1x = sin(startPosition.orientation.rotation)
    let t1y = cos(startPosition.orientation.rotation)
    let t2x = sin(endPosition.orientation.rotation)
    let t2y = cos(endPosition.orientation.rotation)
    
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

fileprivate func calculateBiArcD(_ vDotV: Double,
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

func generateRandomTarget(_ startPosition: KMatrix2) -> KMatrix2
{
    let randomPlan = Int.random(in: 0...10)
//    let randomPlan = 4
    let endPosition: KMatrix2
    switch (randomPlan) {
        
    case 0:
        print("Plotted stop.")
        endPosition = startPosition
        break
        
    case 1:
        print("Plotted linear turn.")
        endPosition = KMatrix2(
            startPosition.position.getX(),
            startPosition.position.getY(),
            Double.random(in: -Double.pi..<Double.pi))
        break
        
    case 2:
        print("Plotted linear move.")
        let distanceDelta = Double.random(in: -ZIPPY_RANGE_X...ZIPPY_RANGE_X)
        endPosition = KMatrix2(
            startPosition.position.getX() + (distanceDelta * sin(startPosition.orientation.rotation)),
            startPosition.position.getY() + (distanceDelta * cos(startPosition.orientation.rotation)),
            startPosition.orientation.rotation)
        break
        
    case 3:
        print("Plotted simple arc.")
        let radius = Double.random(in: -ZIPPY_RANGE_X...ZIPPY_RANGE_X)
        let center = KMatrix2(radius, 0.0, -Double.pi / 2.0)
        center.concat(startPosition)
        
        let deltaAngle = Double.random(in: -Double.pi..<Double.pi)
        let arcEndAngle = addAngles(center.orientation.get(), deltaAngle)
        endPosition = KMatrix2(
            center.position.getX() + (radius * sin(arcEndAngle)),
            center.position.getY() + (radius * cos(arcEndAngle)),
            addAngles(startPosition.orientation.get(), deltaAngle))
        break
        
    case 4:
        print("Plotted simple arc to inverted orientation.")
        let radius = Double.random(in: -ZIPPY_RANGE_X...ZIPPY_RANGE_X)
        let center = KMatrix2(radius, 0.0, -Double.pi / 2.0)
        center.concat(startPosition)

        let deltaAngle = Double.random(in: -Double.pi..<Double.pi)
        let arcEndAngle = addAngles(center.orientation.get(), deltaAngle)
        endPosition = KMatrix2(
            center.position.getX() + (radius * sin(arcEndAngle)),
            center.position.getY() + (radius * cos(arcEndAngle)),
            addAngles(addAngles(startPosition.orientation.get(), deltaAngle), Double.pi))
        break
        
    case 5:
        //there is an asymptote when the target orientation is the same as the starting orientation because the
        //denominator in our default formula will go to zero, causing an undefined calculation of the d value
        print("Plotted simple bi-arc to matching orientation.")
        endPosition = KMatrix2(
            Double.random(in: -ZIPPY_RANGE_X...ZIPPY_RANGE_X),
            Double.random(in: -ZIPPY_RANGE_Y...ZIPPY_RANGE_Y),
            startPosition.orientation.rotation)
        break
        
    case 6:
        //the web page describing bi-arc interpolation says this is another special use case (i.e. "Case 3"), but
        //from my analysis of this use case, the optimal know should just be computed the same was as above
        print("Plotted simple bi-arc to matching orientation at y=0.")
        let distanceDelta = Double.random(in: -ZIPPY_RANGE_X...ZIPPY_RANGE_X)
        endPosition = KMatrix2(
            startPosition.position.getX() + (distanceDelta * -cos(startPosition.orientation.rotation)),
            startPosition.position.getY() + (distanceDelta * sin(startPosition.orientation.rotation)),
            startPosition.orientation.rotation)
        break
        
    default:
        print("Plotted random move.")
        endPosition = KMatrix2(
            Double.random(in: -ZIPPY_RANGE_X...ZIPPY_RANGE_X),
            Double.random(in: -ZIPPY_RANGE_Y...ZIPPY_RANGE_Y),
            Double.random(in: -Double.pi..<Double.pi))
        break
        
    }
    
    return endPosition
}

fileprivate func createTargetIndicator(_ targetPosition: KMatrix2) -> ZDrawable
{
    let sinTO = sin(targetPosition.orientation.rotation)
    let cosTO = cos(targetPosition.orientation.rotation)
    return ZCompoundDrawable(
        //the orientation line
        ZDrawableLine(
            lightYellow,
            targetPosition.position.getX() + (ZIPPY_ORIENTATION_LINE * sinTO),
            targetPosition.position.getY() + (ZIPPY_ORIENTATION_LINE * cosTO),
            targetPosition.position.getX() + (ZIPPY_ORIENTATION_LINE * -sinTO),
            targetPosition.position.getY() + (ZIPPY_ORIENTATION_LINE * -cosTO)),
        //the orthogonal across the line of orientation through the target position
        ZDrawableLine(
            lightOrange,
            targetPosition.position.getX() + (ZIPPY_POINT_INDICATOR_ORTHOGONAL * -cosTO),
            targetPosition.position.getY() + (ZIPPY_POINT_INDICATOR_ORTHOGONAL * sinTO),
            targetPosition.position.getX() + (ZIPPY_POINT_INDICATOR_ORTHOGONAL * cosTO),
            targetPosition.position.getY() + (ZIPPY_POINT_INDICATOR_ORTHOGONAL * -sinTO)))
}

func drawPrimaryBiArc(
    _ startX: Double, _ startY: Double, _ startO: Double,
    _ endX: Double, _ endY: Double, _ endO: Double,
    _ primaryArc1: Arc,
    _ primaryArc2: Arc,
    _ reversed: Bool) -> ZDrawable
{
    return ZCompoundDrawable(
        //        ZDrawableCircle(lightLightGreen, primaryArc1.center.getX(), primaryArc1.center.getY(), primaryArc1.radius, true),
        //        ZDrawableCircle(lightLightRed, primaryArc2.center.getX(), primaryArc2.center.getY(), primaryArc2.radius, true),
        ZDrawablePath(reversed ? UIColor.red : UIColor.green, primaryArc1),
        ZDrawablePath(reversed ? UIColor.green : UIColor.red, primaryArc2),
        ZDrawableArrow(reversed ? UIColor.red : UIColor.green, startX, startY, startO, ZIPPY_POINT_INDICATOR_RADIUS),
        ZDrawableArrow(UIColor.blue, endX, endY, endO, ZIPPY_POINT_INDICATOR_RADIUS))
}

func drawSecondaryBiArc(
    _ secondaryArc1: Arc,
    _ secondaryArc2: Arc) -> ZDrawable
{
    return ZCompoundDrawable(
        ZDrawablePath(lightGreen, secondaryArc1),
        ZDrawablePath(lightRed, secondaryArc2))
}

func distanceZero(_ distance: Double) -> Bool
{
    return abs(distance) <= LINEAR_EPSILON
}

func distance2Zero(_ distance2: Double) -> Bool
{
    return abs(distance2) <= LINEAR2_EPSILON
}

func angleZero(_ angle: Double) -> Bool
{
    return abs(angle) <= ANGULAR_EPSILON
}

func anglesEquivalent(_ angle1: Double, _ angle2: Double) -> Bool
{
    return abs(subtractAngles(angle2, angle1)) <= ANGULAR_EPSILON
}

