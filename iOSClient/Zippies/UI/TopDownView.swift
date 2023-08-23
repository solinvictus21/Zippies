
import Foundation
import UIKit
import os

let ZIPPY_WIDTH: Double = 29.4
let ZIPPY_HEIGHT: Double = 24.8
let ZIPPY_HALF_WIDTH: Double = ZIPPY_WIDTH / 2.0
let ZIPPY_HALF_HEIGHT: Double = ZIPPY_HEIGHT / 2.0
let ZIPPY_AREA_WIDTH: Double = 2000.0
let ZIPPY_AREA_HEIGHT: Double = 1000.0

let TARGET_UPDATE_TIMER_INTERVAL = 100
//let TARGET_DISTANCE: CGFloat = 300.0
let TARGET_DISTANCE_VELOCITY_TOTAL: Double = 280.0

let TARGET_POSITION_ANGULAR_INCREMENT: Double = 0.008
let TARGET_ORIENTATION_ANGULAR_INCREMENT: Double = 0.1
//let TARGET_DISTANCE_VELOCITY_INCREMENT: Double = 0.10

let TARGET_FPS: Double = 20.0
let TARGET_SPF: Double = 1 / TARGET_FPS
let TARGET_MSPF: Double = 1000.0 / TARGET_FPS

fileprivate let pathKeyFrames: [PathKeyFrame] =
[
    PathKeyFrame(x:  200, y:  200, orientation:   90, time: 2),
    PathKeyFrame(x:  400, y:    0, orientation:  180, time: 2),
    PathKeyFrame(x:  200, y: -200, orientation:  -90, time: 2),
    PathKeyFrame(x: -200, y:  200, orientation:  -90, time: 4),
    PathKeyFrame(x: -400, y:    0, orientation:  180, time: 2),
    PathKeyFrame(x: -200, y: -200, orientation:   90, time: 2),
    PathKeyFrame(x:    0, y:    0, orientation:    0, time: 2),
]

class TopDownView : UIView
{
    let logger = Logger()
    var targetUpdateTimer: Timer?
    var targetDirection: Double = 0.0
    var targetOrientation: Double = -0.1
    var targetDistanceVelocityRatio: Double = 0.6
//    var targetDistanceVelocityIncrement: Double = 0.023
    var targetDistanceVelocityIncrement: Double = 0.04

    var zippyManager: ZippyManager?
    
    let EASING: CGFloat = 1/3
    
    let spline: ZCubicHermiteSpline
    var currentSplineTime: Double = 0.0
    var currentSplineTargetPosition = ZVector2()
    var currentSplineTargetVelocity = ZVector2()
    var relativeTargetPosition = ZVector2()
    var relativeTargetVelocity = ZVector2()
    var relativeEndPoint = ZVector2()
    var relativeMovement = ZVector2()

    var previousPosition = ZMatrix2()
    var currentTargetPosition = ZMatrix2()

    init(s: String, i: Int)
    {
        self.spline = BuildCubicHermiteSpline(keyframes: pathKeyFrames)
        super.init(frame: CGRect(x: 0, y: 0, width: 100, height: 100))
    }
    
    required init?(coder aDecoder: NSCoder)
    {
        self.spline = BuildCubicHermiteSpline(keyframes: pathKeyFrames)
        super.init(coder: aDecoder)
    }
    
    override func touchesBegan(_ touches: Set<UITouch>, with event: UIEvent?)
    {
        super.touchesBegan(touches, with: event)
        if event!.modifierFlags.contains(.shift) {
            costToDisplay = (costToDisplay + 1) % 3
        }
        else {
            if !spline.started {
                start()
            }
            else {
                stop()
            }
        }
        setNeedsDisplay()
    }
    
    fileprivate func start()
    {
        ///*
        guard !spline.started else {
            return
        }

        spline.Start(startTime: 0.0)
        currentSplineTime = 0.0
        previousPosition.set(0, 0, 0)
        spline.Interpolate(
            currentTime: 0.0,
            position: currentSplineTargetPosition,
            tangent: currentSplineTargetVelocity)
        //*/
        targetUpdateTimer = Timer.scheduledTimer(withTimeInterval: TARGET_SPF, repeats: true, block: { timer in
            self.update()
            })
    }
    
    fileprivate func update()
    {
        /*
        guard spline.started else {
            if targetUpdateTimer != nil {
                stop()
            }
            return
        }

        currentSplineTime += TARGET_SPF
        spline.Interpolate(
            currentTime: currentSplineTime,
            position: currentSplineTargetPosition,
            tangent: currentSplineTargetVelocity)
        currentSplineTargetVelocity.multiply(TARGET_SPF)

        previousPosition.set(currentTargetPosition)
        relativeTargetPosition.set(currentSplineTargetPosition)
        relativeTargetPosition.subtract(previousPosition.position)
        relativeTargetPosition.unrotate(previousPosition.orientation)
        relativeTargetVelocity.set(currentSplineTargetVelocity)
        relativeTargetVelocity.unrotate(previousPosition.orientation)
        relativeEndPoint.set(relativeTargetPosition)
        relativeEndPoint.add(relativeTargetVelocity)

        //display the path to the target
        calculateRelativeMonoArcPath()

        currentTargetPosition.set(relativeMovement.getX(), relativeMovement.getY(), snapAngle(2.0 * relativeMovement.atan2))
        currentTargetPosition.concatTo(previousPosition)
        */

        ///*
        targetDistanceVelocityRatio += targetDistanceVelocityIncrement
        if targetDistanceVelocityRatio > 1.0 {
            targetDistanceVelocityRatio = 2.0 - targetDistanceVelocityRatio
            targetDistanceVelocityIncrement = -targetDistanceVelocityIncrement
        }
        else if targetDistanceVelocityRatio < 0.0 {
            targetDistanceVelocityRatio = -targetDistanceVelocityRatio
            targetDistanceVelocityIncrement = -targetDistanceVelocityIncrement
        }
        //*/
        ///*
        targetDirection = addAngles(targetDirection, TARGET_POSITION_ANGULAR_INCREMENT)
        targetOrientation = addAngles(targetOrientation, TARGET_ORIENTATION_ANGULAR_INCREMENT)
        //*/
        
        setNeedsDisplay()
    }
    
    fileprivate func stop()
    {
        guard targetUpdateTimer != nil else {
            return
        }

        targetUpdateTimer!.invalidate()
        targetUpdateTimer = nil
        spline.Stop()
    }

    override func draw(_ rect: CGRect)
    {
        guard let context = UIGraphicsGetCurrentContext() else {
            return
        }

        let interpolationX = self.frame.width / CGFloat(ZIPPY_AREA_WIDTH)
        let interpolationY = self.frame.height / CGFloat(ZIPPY_AREA_HEIGHT)
        context.translateBy(x: self.frame.width/2, y: self.frame.height/2)
        context.scaleBy(x: interpolationX, y: -interpolationY)

        let targetDistance = TARGET_DISTANCE_VELOCITY_TOTAL * targetDistanceVelocityRatio
        let targetVelocity = TARGET_DISTANCE_VELOCITY_TOTAL * (1.0 - targetDistanceVelocityRatio)
        
        /*
        let sqRatio1 = pow(targetDistanceVelocityRatio, 2.0)
        let sqRatio2 = pow(1.0 - targetDistanceVelocityRatio, 2.0)
        let totalSqRatio = sqRatio1 + sqRatio2
        let sqRatio3 = sqRatio1 / totalSqRatio
        let sqRatio4 = sqRatio2 / totalSqRatio
        logger.debug("Square Ratios of \(self.targetDistanceVelocityRatio):  \(sqRatio3) / \(sqRatio4)")
         */
        
        let currentPositionVector = ZVector2()
        let targetVector = ZVector2(
            targetDistance * sin(targetDirection),
            targetDistance * cos(targetDirection))
        let velocityVector = ZVector2(
            targetVelocity * sin(targetOrientation),
            targetVelocity * cos(targetOrientation))
        /*
        let targetEndVector = ZVector2(
            targetVector.getX() + velocityVector.getX(),
            targetVector.getY() + velocityVector.getY())
         */

        //newest approach; prioritize orientation over Y movement
        //first determine how closely we will match the desired orientation by moving directly to the target point
        let angleAtTarget = snapAngle(2.0 * targetVector.atan)
        let angleAtTargetToVelocity = subtractAngles(velocityVector.atan2, angleAtTarget)
        let cosAngle = cos(angleAtTargetToVelocity)
        let linearMoveRatio = (cosAngle + 1.0) / 2.0
        
        let scaledVelocityVector = ZVector2(velocityVector)
        scaledVelocityVector.multiply(linearMoveRatio)
        let scaledEndVector = ZVector2(targetVector)
        scaledEndVector.add(scaledVelocityVector)

        //display the path of our brand new algorithm
        /*
        drawElasticPath(
            currentPositionVector,
            targetVector,
            velocityVector,
            false)
         */
        /*
        drawScissorPath(
            currentPositionVector,
            targetVector,
            velocityVector,
            false)
         */
        /*
        drawScissorPath(
            currentPositionVector,
            targetVector,
            velocityVector,
            true)
         */
        /*
        drawElasticPath(
            currentPositionVector,
            targetVector,
            velocityVector,
            false)
         */
        drawCostMap(targetVector, velocityVector)

//        /*
        //display the direct path to the target on top of all other paths
        drawDirectPath(
            currentPositionVector,
            targetVector,
            velocityVector)
//         */

        /*
        drawArcPath(
            currentPositionVector,
            targetVector,
            velocityVector)
         */
        
        calculateStanleyPath(
            currentPositionVector,
            targetVector,
            velocityVector)
    }
    
    let COST_MAP_WIDTH = 600.0
    let COST_MAP_INTERVAL = 10.0
    var costToDisplay = 0
    fileprivate func drawCostMap(
        _ toPoint: ZVector2,
        _ velocity: ZVector2)
    {
        let startX = -(COST_MAP_WIDTH) / 2.0
        let startY = startX
        
        let currentPosition = ZVector2()
//        let startingCosts = calculateCost(currentPosition, 0.0, toPoint, velocity)
        let currentInterval = (targetUpdateTimer == nil ? COST_MAP_INTERVAL / 2.0 : COST_MAP_INTERVAL)
        var currentY = startY
        while (currentY - startY <= COST_MAP_WIDTH)
        {
            var currentX = startX
            while (currentX - startX <= COST_MAP_WIDTH)
            {
                currentPosition.set(currentX, currentY)
                let costs = calculateCost(
                    currentPosition,
                    2.0 * currentPosition.atan,
                    toPoint,
                    velocity)
//                logger.debug("Costs: \(costs.0) \(costs.1) \(costs.2)")
                /*
                //show changes to
                let deltaCost = costs[costToDisplay] - startingCosts[costToDisplay]
                let pointColor = UIColor(
                    red: deltaCost > 0.0 ? deltaCost / (1.0 - startingCosts[costToDisplay]) : 0.0,
                    green: deltaCost < 0.0 ? -deltaCost / startingCosts[costToDisplay] : 0.0,
                    blue: 0.0,
                    alpha: 1.0)
                 */
//                /*
                //show cost directly
                let pointColor = UIColor(
//                    red: 0.0,
                    red: costs[costToDisplay] > 0.5 ? 2.0 * (costs[costToDisplay] - 0.5) : 0.0,
//                    green: 0.0,
                    green: costs[costToDisplay] < 0.5 ? 2.0 * (0.5 - costs[costToDisplay]) : 0.0,
                    blue: 0.0,
//                    blue: 1.0 - costs[2],
                    alpha: 1.0)
//                    alpha: 1.0 - costs[costToDisplay])
//                    alpha: 1.0 - pow(costs.0, 2.0))
//                 */
                pointColor.setStroke()
                pointColor.setFill()
//                drawPoint(center: currentPosition)
                drawRect(center: currentPosition, width: COST_MAP_INTERVAL, height: COST_MAP_INTERVAL)
                currentX += currentInterval
            }
            currentY += currentInterval
        }
    }
    
    fileprivate func calculateCost(
        _ fromPoint: ZVector2,
        _ fromOrientation: Double,
        _ toPoint: ZVector2,
        _ velocity: ZVector2) -> [Double]
    {
        //direction from the current point to the target endpoint
        let fromToTarget = ZVector2(
            toPoint.getX() - fromPoint.getX(),
            toPoint.getY() - fromPoint.getY())
        let fromToVelocity = ZVector2(
            fromToTarget.getX() + velocity.getX(),
            fromToTarget.getY() + velocity.getY())
        /*
//        let linearCost = fromToTarget.getD() / (fromToTarget.getD() + velocity.getD())
        let linearCost = fromToTarget.getD2() / (fromToTarget.getD2() + velocity.getD2())
*/
        //cost is ideal when the distance remaining to the target is equal to the velocity length
//        let linearCost = fromToTarget.getD() / (fromToTarget.getD() + velocity.getD())
//        let linearCost = abs(fromToVelocity.getD() - velocity.getD()) / (fromToVelocity.getD() + velocity.getD())
//        let linearCost = abs(subtractAngles(fromToTarget.atan2, fromOrientation)) / Double.pi
//        let linearCost = abs(subtractAngles(velocity.atan2, fromOrientation)) / Double.pi
        let linearCost = abs(subtractAngles(velocity.atan2, fromOrientation)) / Double.pi

//        /*
        //direction from the current point to the velocity endpoint
        let angularCost = abs(subtractAngles(fromToVelocity.atan2, fromOrientation)) / Double.pi
//         */
        /*
        //direction from the current orientation to the velocity direction
        let angularCost = abs(subtractAngles(fromOrientation, velocity.atan2)) / Double.pi
         */
        let totalCost = (linearCost + angularCost) / 2.0
//        let totalCost = linearCost * angularCost

        return [angularCost, linearCost, totalCost]
    }

    fileprivate func drawArcPath(
        _ fromPoint: ZVector2,
        _ toPoint: ZVector2,
        _ endPoint: ZVector2)
    {
        UIColor.blue.setStroke()
        UIColor.blue.setFill()
        /*
        let turnPoint1 = ZVector2(0, moveY)
        drawLine(
            start: fromPoint,
            end: turnPoint1)
        drawLine(
            start: turnPoint1,
            end: toPoint)
         */
        drawLine(
            start: fromPoint,
            end: toPoint)
        
        UIColor.magenta.setStroke()
        UIColor.magenta.setFill()
        drawLine(
            start: toPoint,
            end: endPoint)
        drawArc(relativeTargetPosition: toPoint)
        drawPoint(center: toPoint)
        
        drawCircle(
            center: endPoint,
            radius: sqrt(pow(endPoint.getX() - toPoint.getX(), 2.0) + pow(endPoint.getY() - toPoint.getY(), 2.0)))
    }
    
    fileprivate func drawScissorPath(
        _ fromPoint: ZVector2,
        _ toPoint: ZVector2,
        _ velocity: ZVector2,
        _ reverseDirection: Bool)
    {
        let targetLinearDistance = (toPoint.getY() / 2.0) + (velocity.getY() / 4.0)
        //vector that represents movement from current position to the midpoint on the velocity vector
        let velocityConvergencePoint = ZVector2(
            toPoint.getX() + (velocity.getX() / 2.0),
            toPoint.getY() + (velocity.getY() / 2.0))

        let velocityConvergenceVector = ZVector2(
            velocityConvergencePoint.getX(),
//            velocityConvergencePoint.getY() - targetLinearDistance)
            targetLinearDistance)

        //toPoint.getY() + (velocity.getY() / 2.0) - (toPoint.getY() / 2.0)  + (velocity.getY() / 4.0)
        //toPoint.getY() + (velocity.getY() / 2.0) - (toPoint.getY() / 2.0)  - (velocity.getY() / 4.0)
        //toPoint.getY() - (toPoint.getY() / 2.0)  + (velocity.getY() / 2.0) - (velocity.getY() / 4.0)
        //(toPoint.getY() / 2.0)                   + (velocity.getY() / 4.0)
        
        let targetPoint: ZVector2
        if reverseDirection {
            targetPoint = ZVector2(
                fromPoint.getX() - (targetLinearDistance * velocityConvergenceVector.sin),
                targetLinearDistance - (targetLinearDistance * velocityConvergenceVector.cos))
        }
        else {
            targetPoint = ZVector2(
                fromPoint.getX() + (targetLinearDistance * velocityConvergenceVector.sin),
                targetLinearDistance + (targetLinearDistance * velocityConvergenceVector.cos))
        }
//        let targetPoint = ZVector2(
//            fromPoint.getX() - (targetLinearDistance * velocityConvergenceVector.sin),
//            targetLinearDistance - (targetLinearDistance * velocityConvergenceVector.cos))

        let convergencePoint = ZVector2(
            fromPoint.getX(),
            targetLinearDistance)
        UIColor.red.setStroke()
        drawLine(start: fromPoint, end: convergencePoint)
        drawLine(start: convergencePoint, end: targetPoint)
        drawLine(start: convergencePoint, end: velocityConvergencePoint)
        if reverseDirection {
            UIColor.cyan.setStroke()
            UIColor.cyan.setFill()
        }
        else {
            UIColor.blue.setStroke()
            UIColor.blue.setFill()
        }
//        drawPoint(center: targetPoint)
        drawArrow(
            position: targetPoint,
            orientation: 2.0 * targetPoint.atan2)
        drawArc(relativeTargetPosition: targetPoint)

//        UIColor.cyan.setStroke()
//        UIColor.cyan.setFill()
//        drawPoint(center: velocityConvergenceVector)
    }

    let Klat = 1.0
    fileprivate func drawElasticPath(
        _ fromPoint: ZVector2,
        _ toPoint: ZVector2,
        _ velocity: ZVector2,
        _ reverseDirection: Bool)
    {
        let targetY = (toPoint.getY() / 2.0)// + (velocity.getY() / 4.0)

        //now derive X
        let distancePrime = (toPoint.getX() * velocity.cos) - (toPoint.getY() * velocity.sin)
        let distanceDot = (toPoint.getX() * velocity.cos) - ((toPoint.getY() - targetY) * velocity.sin)
//        let distanceDot2 = targetY * velocity.sin
        let distanceDotHat = -Klat * distancePrime
        let targetX = (distancePrime - distanceDot) + distanceDotHat
//        let distanceToVelocityLine2 = (toPoint.getX() * velocity.cos) - ((toPoint.getY() - targetY) * velocity.sin)
//        let targetX = distanceToVelocityLine - distanceToVelocityLine2
        logger.debug("tY: \(targetY)  d: \(distancePrime)  dDot: \(distanceDot)  dDotHat: \(distanceDotHat)  tX: \(targetX)")
//        logger.debug("dd2: \(distanceDot2)")
        let targetPoint = ZVector2(
            targetX,
            targetY)
//            -targetX * velocity.cos,
//            targetY + targetX * velocity.sin)
        
        //calculate the normalized cost of a movement, 0.0 - 1.0, where...
        //    angular cost
        //        0.0 = current orientation is directly toward the velocity endpoint
        //        1.0 = current orientation is facing direction away from the velocity endpoint
        //    linear cost
        //        0.0 = distance to target position is zero
        //        1.0 = target velocity is zero
        //    combined cost =
//        calculateCost(fromPoint, 0.0, toPoint, velocity)
//        calculateCost(targetPoint, 2.0 * targetPoint.atan, toPoint, velocity)

        UIColor.black.setStroke()
//        drawLine(start: fromPoint, end: targetPoint)
        drawArrow(
            position: targetPoint,
            orientation: 2.0 * targetPoint.atan2)
        drawArc(relativeTargetPosition: targetPoint)
        drawLine(
            start: fromPoint,
            end: ZVector2(
                distancePrime * velocity.cos,
                -distancePrime * velocity.sin))
        UIColor.cyan.setStroke()
        /*
        drawLine(
            start: fromPoint,
            end: ZVector2(
                distanceDot * velocity.cos,
                -distanceDot * velocity.sin))
        */
        drawLine(
            start: ZVector2(
                0.0,
                targetY),
            end: ZVector2(
                distanceDot * velocity.cos,
                targetY - (distanceDot * velocity.sin)))
//                -distanceDot,
//                targetY))
    }
    
    fileprivate func drawDirectPath(
        _ fromPoint: ZVector2,
        _ toPoint: ZVector2,
        _ velocity: ZVector2)
    {
        let velocityEndPoint = ZVector2(
            fromPoint.getX() + toPoint.getX() + velocity.getX(),
            fromPoint.getY() + toPoint.getY() + velocity.getY())

        //1 - current position
        UIColor.cyan.setStroke()

        //draw an arrow at the current position in the direction of the current orientation
        drawArrow(
            position: fromPoint,
            orientation: fromPoint.atan2,
            arrowRadius: 10.0)

        //draw a point at the current position
//        drawPoint(center: fromPoint)
        
        //2 - transition to target position

        UIColor.white.setStroke()
        UIColor.white.setFill()

        //draw the direct line from our current position to the target position
        drawLine(
            start: fromPoint,
            end: toPoint)

        //draw an arc from the current position to the target position
//        drawArc(relativeTargetPosition: toPoint)
        
        //draw an arc from the current position to the velocity endpoint
//        drawArc(relativeTargetPosition: velocityEndPoint)

        //3 - target position

        //draw a point at the target position
        drawPoint(center: toPoint)

        //draw an arrow at the target position in the direction of the target
//        drawArrow(
//            position: toPoint,
//            orientation: toPoint.atan2)
        
        //target position to velocity endpoint

        UIColor.lightGray.setStroke()
        UIColor.lightGray.setFill()
        
        //draw a line from the target position to the velocity endpoint
        drawLine(
            start: toPoint,
            end: velocityEndPoint)

        /*
        //draw an arc from the target position to the velocity endpoint
        drawArc(
            fromPoint: toPoint,
            fromOrientation: 2.0 * toPoint.atan,
            toPoint: velocityEndPoint)
         */
        
        //velocity endpoint
        
        //draw an arrow at the velocity endpoiunt in the direction of the velocity
        drawArrow(
            position: velocityEndPoint,
            orientation: velocity.atan2)
        
        //draw a point at the velocity endpoint
//        drawPoint(center: velocityEndPoint)
    }
    
    let kV = 1.0
    fileprivate func calculateStanleyPath(
        _ fromPoint: ZVector2,
        _ toPoint: ZVector2,
        _ velocity: ZVector2)
    {
        //calculate errors
        let crossProduct =
            ((toPoint.getX() * velocity.getY()) - (toPoint.getY() * velocity.getX()))
            / velocity.getD()
        let dotProduct =
            ((toPoint.getX() * velocity.getX()) + (toPoint.getY() * velocity.getY()))
            / velocity.getD()

//        let lookAheadDistance = max(toPoint.getD(), velocity.getD())
//        let lookAheadDistance = velocity.getD()
//        let lookAheadDistance = dotProduct
//        let lookAheadDistance = toPoint.getY() + velocity.getY()
        var lookAheadDistance = velocity.getD() + dotProduct
//        let lookAheadDistance = (velocity.getD() + dotProduct) * sin(velocity.atan2)
//        let lookAheadDistance = toPoint.getX() + velocity.getX()
//        let lookAheadDistance = velocity.getD() * sin(headingError)

//        let steeringAngle = headingError + atan(kV * abs(crossTrackError) / lookAheadDistance)
//        let steeringAngle = headingError + atan2(kV * abs(crossProduct), lookAheadDistance)
//        let steeringAngle = headingError + atan2(kV * crossProduct, lookAheadDistance)
        let angularVelocity = addAngles(velocity.atan2, atan(kV * crossProduct / lookAheadDistance))
//        let steeringAngle = headingError + atan2(kV * crossTrackError, lookAheadDistance)

        lookAheadDistance *= 1.0 - abs(angularVelocity / Double.pi)

        //determine the new target position
        //given...
        //    L is the known length of the arc to move (lookAheadDistance)
        //    tG is the known angle of the arc to turn (steeringAngle)
        //    tC is the known subtended angle of the arc (2 * rG)
        //    rC is the unknown radius of the arc
        //    rG is the unknown linear distance we want to find from the starting position to the goal position
        //
        //solve for rG...
        //    L = tC * rC
        //    rC = L / tC
        //    rC = L / 2 * tG
        //
        //    rC = rG / 2 * sin(tG)
        //    rG = rC * 2 * sin(tG)
        //    rG = rC * 2 * sin(tG)
        //       = L * 2 * sin(tG) / 2 * tG
        //       = L * sin(tG) / tG

        //the point on the velocity line closest to the current position
        let crossTrackErrorPoint = ZVector2(
            fromPoint.getX() + (crossProduct * velocity.cos),
            fromPoint.getY() - (crossProduct * velocity.sin))
        
        //the point on the velocity line from the cross-track error point to the target endpoint
        let lookAheadPoint = ZVector2(
            crossTrackErrorPoint.getX() + (dotProduct * velocity.sin),
            crossTrackErrorPoint.getY() + (dotProduct * velocity.cos))

        //the point on the velocity line from the cross-track error point to the velocity endpoint
        let toVelocityPoint = ZVector2(
            crossTrackErrorPoint.getX() + (lookAheadDistance * velocity.sin),
            crossTrackErrorPoint.getY() + (lookAheadDistance * velocity.cos))

        let directDistanceToTarget = (lookAheadDistance * sin(angularVelocity)) / angularVelocity
        let targetPoint = ZVector2(
            fromPoint.getX() + (directDistanceToTarget * sin(angularVelocity / 2.0)),
            fromPoint.getY() + (directDistanceToTarget * cos(angularVelocity / 2.0)))

        UIColor.yellow.setStroke()
        UIColor.yellow.setFill()
        
        //draw the target point and the arc to reach it
        drawArc(relativeTargetPosition: targetPoint)
        /*
        drawLine(
            start: fromPoint,
            end: targetPoint)
         */
//        drawPoint(center: targetPoint)
        drawArrow(
            position: targetPoint,
            orientation: angularVelocity)

        UIColor.cyan.setStroke()
        UIColor.cyan.setFill()
        drawArc(lookAheadDistance, angularVelocity)

        UIColor.blue.setStroke()
        UIColor.blue.setFill()

        //draw the point on the velocity line closest to the current point
        drawPoint(center: crossTrackErrorPoint)
        drawPoint(center: lookAheadPoint)
        drawPoint(center: toVelocityPoint)
    }

}
