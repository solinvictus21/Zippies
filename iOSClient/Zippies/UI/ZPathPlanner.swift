
import Foundation
import UIKit

fileprivate let LINEAR_EPSILON: Double = 2.0
fileprivate let LINEAR2_EPSILON: Double = 4.0
fileprivate let ANGULAR_EPSILON: Double = 0.017453292519943
//the delta angle within which is to be considered "pointing at the desired orientation" (1 degree)
fileprivate let SEMICIRCLE_QUARTER = Double.pi/2

fileprivate let ZIPPY_ORIENTATION_LENGTH: Double = 60
fileprivate let ZIPPY_POINT_INDICATOR_RADIUS: Double = 10
fileprivate let ZIPPY_POINT_INDICATOR_ORTHOGONAL: Double = 50
fileprivate let ZIPPY_ORIENTATION_LINE: Double = 150

let lightRed = UIColor.red.withAlphaComponent(0.20)
let lightLightRed = UIColor.red.withAlphaComponent(0.10)
let lightOrange = UIColor.orange.withAlphaComponent(0.25)
let lightYellow = UIColor.yellow.withAlphaComponent(0.70)
let lightGreen = UIColor.green.withAlphaComponent(0.30)
let lightLightGreen = UIColor.green.withAlphaComponent(0.20)
let lightBlue = UIColor.blue.withAlphaComponent(0.20)
let lightLightBlue = UIColor.blue.withAlphaComponent(0.10)
let lightPurple = UIColor.blue.withAlphaComponent(0.15)

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

func createTestDrawable() -> ZDrawable
{
    print()

    let startPosition = KMatrix2(
        Double.random(in: -300...300),
        Double.random(in: -200...200),
        Double.random(in: -Double.pi..<Double.pi))
    let startpointDrawable = ZDrawableArrow(UIColor.green,
                                            startPosition,
                                            ZIPPY_POINT_INDICATOR_RADIUS)
    let targetPosition = generateTarget(startPosition)
    
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
    calculateBiArcKnotRelative(relativeTargetPosition, knot.position)
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
        //target is directly in front or behind; linear move
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
        print("Planned linear turn.")
        return ZDrawableCircle(color, startPosition.position, ZIPPY_POINT_INDICATOR_RADIUS)
    }
    
    guard subtendedAngle != nil else {
        //linear move
        print("Planned linear move.")
        let targetPosition = KMatrix2(0.0, distanceToMove!, 0.0)
        targetPosition.concat(startPosition)
        return ZDrawableLine(color, startPosition.position, targetPosition.position)
    }
    
    //fallback in the simple use case is a simple arc
    print("Planned simple arc.")
    return ZDrawablePath(color,
                         Arc(startPosition, distanceToMove!, subtendedAngle!.rotation))
}

fileprivate func calculateBiArcKnotRelative(_ relativeTargetPosition: KMatrix2,
                                            _ targetVector: KVector2)
{
    if angleZero(relativeTargetPosition.orientation.rotation) {
        //this would create an asymptote in the regular bi-arc equation because the dot product of our orientation with
        //the target orientation would be 1; so handle it as a special use case
        print("Planned simple bi-arc to matching orientation.")
        targetVector.set(relativeTargetPosition.position.getX() / 2.0, relativeTargetPosition.position.getY() / 2.0)
        return
    }
    
    print("Planned complex bi-arc.")
    let t2x = sin(relativeTargetPosition.orientation.rotation)
    let t2y = cos(relativeTargetPosition.orientation.rotation)
    
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
    targetVector.set(
        ( relativeTargetPosition.position.getX() + (d * (-t2x)) ) / 2.0,
        ( relativeTargetPosition.position.getY() + (d * (1.0 - t2y)) ) / 2.0)
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
        //the target orientation would be 1; so handle it as a special use case
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

fileprivate func generateTarget(_ startPosition: KMatrix2) -> KMatrix2
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
        let distanceDelta = Double.random(in: -200...200)
        endPosition = KMatrix2(
            startPosition.position.getX() + (distanceDelta * sin(startPosition.orientation.rotation)),
            startPosition.position.getY() + (distanceDelta * cos(startPosition.orientation.rotation)),
            startPosition.orientation.rotation)
        break
        
    case 3:
        print("Plotted simple arc.")
        let radius = Double.random(in: -300...300)
        let centerX = startPosition.position.getX() + (radius * -cos(startPosition.orientation.rotation))
        let centerY = startPosition.position.getY() + (radius * sin(startPosition.orientation.rotation))
        let orientationDelta = Double.random(in: -Double.pi..<Double.pi)
        let endingOrientation = addAngles(startPosition.orientation.rotation, orientationDelta)
//        let arcEndAngle = addAngles(subtractAngles(startPosition.orientation, Double.pi/2), orientationDelta)
        let arcEndAngle = addAngles(startPosition.orientation.rotation, addAngles(Double.pi/2, orientationDelta))
        endPosition = KMatrix2(
            centerX + (radius * sin(arcEndAngle)),
            centerY + (radius * cos(arcEndAngle)),
            addAngles(startPosition.orientation.rotation, endingOrientation))
        break
        
    case 4:
        print("Plotted simple arc to inverted orientation.")
        let radius = Double.random(in: -300...300)
        let centerX = startPosition.position.getX() + (radius * -cos(startPosition.orientation.rotation))
        let centerY = startPosition.position.getY() + (radius * sin(startPosition.orientation.rotation))
        let orientationDelta = Double.random(in: -Double.pi..<Double.pi)
        let arcEndAngle = addAngles(radius < 0 ? -Double.pi/2 : Double.pi/2, orientationDelta)
        endPosition = KMatrix2(
            centerX + (abs(radius) * sin(arcEndAngle)),
            centerY + (abs(radius) * cos(arcEndAngle)),
            addAngles(addAngles(startPosition.orientation.rotation, orientationDelta), Double.pi))
        break
        
    case 5:
        //there is an asymptote when the target orientation is the same as the starting orientation because the
        //denominator in our default formula will go to zero, causing an undefined calculation of the d value
        print("Plotted simple bi-arc to matching orientation.")
        endPosition = KMatrix2(
            Double.random(in: -400...400),
            Double.random(in: -400...400),
            startPosition.orientation.rotation)
        break
        
    case 6:
        //the web page describing bi-arc interpolation says this is another special use case (i.e. "Case 3"), but
        //from my analysis of this use case, the optimal know should just be computed the same was as above
        print("Plotted simple bi-arc to matching orientation at y=0.")
        let distanceDelta = Double.random(in: -200...200)
        endPosition = KMatrix2(
            startPosition.position.getX() + (distanceDelta * -cos(startPosition.orientation.rotation)),
            startPosition.position.getY() + (distanceDelta * sin(startPosition.orientation.rotation)),
            startPosition.orientation.rotation)
        break
        
    default:
        print("Plotted random move.")
        endPosition = KMatrix2(Double.random(in: -400...400),
                                Double.random(in: -400...400),
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

/*
 fileprivate func createSimpleMove(_ color: UIColor,
 _ startPosition: KMatrix2,
 _ relativeTargetPosition: KVector2) -> ZDrawable
 {
 //check if we can do a single-motion move first
 if distance2Zero(relativeTargetPosition.getD2()) {
 //no linear movement
 if angleZero(relativeTargetPosition.getOrientation()) {
 //no linear turn do nothing
 print("Planned stop.")
 return ZDrawableCircle(color, startPosition.position.getX(), startPosition.position.getY(), ZIPPY_POINT_INDICATOR_RADIUS)
 }
 
 //linear turn
 print("Planned linear turn.")
 return ZDrawableCircle(color, startPosition.position.getX(), startPosition.position.getY(), ZIPPY_POINT_INDICATOR_RADIUS)
 }
 
 let turnTowardTarget = atan(relativeTargetPosition.getX() / relativeTargetPosition.getY())
 //    print("target orientation:", relativeOrientation)
 //    print("arrival angle     :", arrivalAngleAtTarget)
 if angleZero(2.0 * turnTowardTarget) {
 //target is directly in front or behind
 //linear move
 print("Planned linear move.")
 return ZDrawableLine(color,
 startPosition.position.getX(), startPosition.position.getY(),
 startPosition.position.getX() + (relativeTargetPosition.getD() * sin(turnTowardTarget)),
 startPosition.position.getY() + (relativeTargetPosition.getD() * cos(turnTowardTarget)))
 }
 
 //fallback in the simple use case is a simple arc
 print("Planned simple arc.")
 return ZDrawablePath(color, Arc(startPosition.position.getX(), startPosition.position.getY(), startPosition.orientationOld,
 startPosition.position.getX() + (relativeTargetPosition.getD() * sin(relativeTargetPosition.getOrientation())),
 startPosition.position.getY() + (relativeTargetPosition.getD() * cos(relativeTargetPosition.getOrientation())),
 relativeTargetPosition.getY() < 0.0))
 }
 */

/*
fileprivate func createTestPathExecution(_ relativeTargetPosition: KPosition) -> ZDrawable
{
    //first determine the single arc which would move us to be directly on the ray represented by the vector
    //to the target position and the target orientation we are trying to achieve from that position; since
    //the change in our current orientation will be twice the angle to the goal, that means that our goal
    //should be along the line represented by half of the change in orientation from our current orientation
    //toward our target orientation
//    let subtendedArc = relativeTargetPosition.orientation
    /*
    let turnTowardPosition = atan(relativeTargetPosition.vector.getX() / relativeTargetPosition.vector.getY());
    let normalArrivalOrientation = addAngles(
        turnTowardPosition,
        turnTowardPosition)
    let relativePositionToOrientation = subtractAngles(relativeTargetPosition.orientation, normalArrivalOrientation)
    let turnTowardOrientation = subtractAngles(
        relativeTargetPosition.vector.getOrientation(),
        relativePositionToOrientation / 2.0)
     */
    /*
    let subtendedArc = addAngles(
//        relativeTargetPosition.vector.getOrientation(),
        turnTowardPosition,
        subtractAngles(relativeTargetPosition.orientation, relativeTargetPosition.vector.getOrientation()) / 2.0)
     */
    //now we have a radius, distance, and angle for an ideal arc from our current position and orientation
    //to a position along the line of target orientation which will arrive precisely aligned with that line
    
    //but our primary goal is to maintain alignment with the line from the target position that is orthogonal
    //to the orientation line; the goal is for the target position along the orientation line to "sweep" the
    //robot forward along the orientation line
    let idealArc = calculateConvergingArc(relativeTargetPosition)
    let idealEndpoint = KPosition()
    idealArc.interpolate(1.0, idealEndpoint)
    let forward = (idealArc.deltaAngle * idealArc.radius) > 0.0

    /*
    //alternate execution plan; curve onto the line 45 degrees toward the target
    let turnTowardTarget = pow(
        atan(relativeTargetPosition.vector.getX() / relativeTargetPosition.vector.getY()),
        2.0)
    let alternativeTurn = addAngles(
        turnTowardTarget,
        subtractAngles(relativeTargetPosition.orientation, turnTowardTarget)) / 2.0
     */
    return ZCompoundDrawable(
        //the alternative arc path
        /*
        ZDrawablePath(
            UIColor.lightGray,
            Arc(0.0, 0.0, 0.0,
                relativeTargetPosition.vector.getD() * sin(alternativeTurn),
                relativeTargetPosition.vector.getD() * cos(alternativeTurn),
                false)),
         */
        //arc range lines
        //line from center of arc to start of arc
        /*
        ZDrawableLine(
            lightLightBlue,
            KVector2(0, 0),
            arcCenterOffset,
            0.0),
         */
        //line from center of arc to target position
        /*
        ZDrawableLine(
            lightYellow,
            arcCenterOffset,
            0.0,
            relativeTargetPosition.vector),
         */
        //line from center of arc to end of arc
        ZDrawableLine(
            lightLightBlue,
            idealArc.radius,
            0.0,
            idealEndpoint.vector.getX(),
            idealEndpoint.vector.getY()),
        //dot at center of arc
        ZDrawableCircle(
            lightBlue,
            idealArc.center,
            ZIPPY_POINT_INDICATOR_RADIUS / 2.0,
            true),
        //line from current position to target position
        /*
        ZDrawableLine(
            lightLightGreen,
            KVector2(0, 0),
            relativeTargetPosition.vector),
         */
        //line from intersection point to the target position
        /*
        ZDrawableLine(
            lightYellow,
            intersectionX,
            intersectionY,
            relativeTargetPosition.vector.getX(),
            relativeTargetPosition.vector.getY()),
         */
        /*
        ZDrawableArrow(
            lightLightRed,
            orientationIntersection.getX(),
            orientationIntersection.getY(),
            relativeTargetPosition.orientation,
            ZIPPY_POINT_INDICATOR_RADIUS),
         */
        //the ideal arc curve
        ZDrawablePath(
            forward ? UIColor.green : UIColor.red,
            idealArc),
        //portion of arc that we ARE traveling
        /*
        ZDrawablePath(
            UIColor.green,
            Arc(arcCenterOffset, angleDelta)),
        ZDrawableArrow(
            UIColor.red,
            targetIntersection.getX(),
            targetIntersection.getY(),
            angleDelta,
            ZIPPY_POINT_INDICATOR_RADIUS),
         */
        //arrow at starting position and orientation
        ZDrawableArrow(
            forward ? UIColor.green : UIColor.red,
            0.0,
            0.0,
            0.0,
            ZIPPY_POINT_INDICATOR_RADIUS),
        //arrow at the ideal arc curve endpoint
        ZDrawableArrow(
            UIColor.blue,
            idealEndpoint,
            ZIPPY_POINT_INDICATOR_RADIUS))
}
 */

/*
fileprivate func calculateConvergingArc(_ relativeTargetPosition: KPosition) -> Arc
{
    //now determine the intersection point from the direction of our goal arc to that ray
    //the distance along that arc angle is (tv X to) / (aa X to), where
    //  tv = vector to the current target position
    //  to = the target orientation
    //  aa = the angle of the ideal arc to the ray of the target orientation from the target position
    //in a more generalized use case, we would first calculate (aa X to) and check if this value
    //is zero before proceeding, but that can only happen when the current orientation is parallel
    //to the target orientation; the assumption here is that we've already eliminated that condition
    let subtendedArc = relativeTargetPosition.orientation
    let sinO = sin(subtendedArc)
    let cosO = cos(subtendedArc)
    let sinA = sin(subtendedArc / 2.0)
    let cosA = cos(subtendedArc / 2.0)
//    print("target orientation   :", 180.0 * (relativeTargetPosition.orientation / Double.pi))
//    print("subtended arc        :", 180.0 * (subtendedArc / Double.pi))

    //the distance to the intersection point from vector A to ray bB is calculated by dividing the cross
    //product of b and B with the cross product of A and B
    let intersectionDistance =
        ( (relativeTargetPosition.vector.getX() * cosO) - (relativeTargetPosition.vector.getY() * sinO) ) /
            ( (sinA * cosO) - (cosA * sinO) )
    /*
     let intersectionDistance =
     ( (relativeTargetPosition.vector.getX() * sinO) + (relativeTargetPosition.vector.getY() * cosO) ) /
     ( (sinA * sinO) + (cosA * cosO) )
     */
//    print("intersection distance:", intersectionDistance)
    
    //once we have the distance from A to the intersection point with bB, then the intersection point is...
    let intersectionX = intersectionDistance * sinA
    let deltaDotDelta = pow(intersectionX, 2.0) + pow(intersectionDistance * cosA, 2.0)
    let n2DotDelta = (-2.0 * intersectionX)
    let arcCenterOffset = -deltaDotDelta / n2DotDelta
//    print("intersection radius  :", arcCenterOffset)
//    print()
    
    return Arc(arcCenterOffset, subtendedArc)
}
 */

/*
func planArc(
    _ color: UIColor,
    _ startX: Double,
    _ startY: Double,
    _ startO: Double,
    _ endX: Double,
    _ endY: Double) -> ZDrawable
{
    let deltaX = endX - startX
    let deltaY = endY - startY
    if distance2Zero(pow(deltaX, 2.0) + pow(deltaY, 2.0)) {
        print("Planned turn instead of arc.")
        return ZDrawableTurn(UIColor.blue, startX, startY, Turn(startO, Double.pi))
    }
    
    return ZDrawablePath(color, Arc(startX, startY, startO, deltaX, deltaY))
}
 */

/*
func calculateArc(
    _ startX: Double,
    _ startY: Double,
    _ startO: Double,
    _ deltaX: Double,
    _ deltaY: Double) -> Arc
{
    let tangentX = sin(startO)
    let tangentY = cos(startO)
    let deltaDotDelta = (deltaX * deltaX) + (deltaY * deltaY)
    let n2DotDelta = ((2.0 * -tangentY) * deltaX) + ((2.0 * tangentX) * deltaY)
    let s = deltaDotDelta / n2DotDelta
    let centerX = startX + (s * -tangentY)
    let centerY = startY + (s * tangentX)
    let startAngle = atan2(startX - centerX, startY - centerY);
    let endAngle = atan2((startX+deltaX) - centerX, (startY+deltaY) - centerY);
    let deltaAngle = subtractAngles(endAngle, startAngle);
    return Arc(centerX, centerY, abs(s),
               startO, startAngle, deltaAngle)
}
*/

/*
func createTestPathPlan(
    _ startPosition: KPosition,
    _ targetPosition: KPosition,
    _ displaySecondaryBiArc: Bool) -> ZDrawable
{
    return createTestPathPlan(
        startPosition.vector.getX(),
        startPosition.vector.getY(),
        startPosition.orientation,
        targetPosition.vector.getX(),
        targetPosition.vector.getY(),
        targetPosition.orientation,
        displaySecondaryBiArc)
}

func createTestPathPlan(
    _ startX: Double, _ startY: Double, _ startO: Double,
    _ endX: Double, _ endY: Double, _ endO: Double,
    _ displaySecondaryBiArc: Bool) -> ZDrawable
{
    //the following code is derived from formulae and helpful explanations provided from the following site
    //  http://www.ryanjuckett.com/programming/biarc-interpolation
    
    let deltaX = endX - startX
    let deltaY = endY - startY

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
    
//    print("Planned complex bi-arc.")
    //precalc = 2 * (1 - (t1 dot t2))
    let t1DotT2Inv2 = 2.0 * (1.0 - t1DotT2)
    let discrim = sqrt( pow(vDotT, 2.0) + ( t1DotT2Inv2 * vDotV ) )
    
    //now find the "knot" (the connection point between the arcs)
    //pm = ( p1 + p2 + (d * (t1 - t2)) ) / 2
    let d = ( -vDotT + discrim ) / t1DotT2Inv2
    let pmx = ( startX + endX + (d * (t1x - t2x)) ) / 2.0
    let pmy = ( startY + endY + (d * (t1y - t2y)) ) / 2.0
    let arc1 = Arc(startX, startY, startO, pmx, pmy, false)
    let arc2 = Arc(pmx, pmy, addAngles(startO, arc1.deltaAngle), endX, endY, false)
    
    //also calculate the inverted path
    let invD = ( -vDotT - discrim ) / t1DotT2Inv2
    let pmxr = ( startX + endX + (invD * (t1x - t2x)) ) / 2.0
    let pmyr = ( startY + endY + (invD * (t1y - t2y)) ) / 2.0
    let arc1r = Arc(startX, startY, startO, pmxr, pmyr, true)
    let arc2r = Arc(pmxr, pmyr, addAngles(startO, arc1r.deltaAngle), endX, endY, true)
    
    //move in reverse if the turn from moving forward is greater than the turn would be from moving in reverse
    let relativeTargetPosition = KPosition(deltaX, deltaY, subtractAngles(endO, startO))
    relativeTargetPosition.vector.rotate(-startO)
    let turnTowardTarget = 2.0 * relativeTargetPosition.vector.getOrientation()
    let turnTowardOrientation = subtractAngles(relativeTargetPosition.orientation, turnTowardTarget)
    let reverseMotion = abs(turnTowardTarget + turnTowardOrientation) > Double.pi
    if reverseMotion {
        //move the reverse way around the arc paths
        var drawable = drawPrimaryBiArc(
            startX, startY, startO,
            endX, endY, endO,
            arc1r, arc2r,
            true)
        if displaySecondaryBiArc {
            drawable = ZCompoundDrawable(
                drawSecondaryBiArc(arc1, arc2),
                drawable)
        }
        return drawable
    }
    
    //default use case; move forward toward target position and orientation
    var drawable = drawPrimaryBiArc(
        startX, startY, startO,
        endX, endY, endO,
        arc1, arc2,
        false)
    if displaySecondaryBiArc {
        drawable = ZCompoundDrawable(
            drawSecondaryBiArc(arc1r, arc2r),
            drawable)
    }
    return drawable
}
 */

/*
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
        ZDrawableArrow(UIColor.blue, endX, endY, endO, ZIPPY_POINT_INDICATOR_RADIUS))
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
        ZDrawableArrow(UIColor.blue, endX, endY, endO, ZIPPY_POINT_INDICATOR_RADIUS))
}
 */
