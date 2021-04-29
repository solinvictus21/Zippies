
import Foundation
import UIKit

let ZIPPY_WIDTH: Double = 29.4
let ZIPPY_HEIGHT: Double = 24.8
let ZIPPY_HALF_WIDTH: Double = ZIPPY_WIDTH / 2.0
let ZIPPY_HALF_HEIGHT: Double = ZIPPY_HEIGHT / 2.0
let ZIPPY_AREA_WIDTH: Double = 2000.0
let ZIPPY_AREA_HEIGHT: Double = 1000.0

let TARGET_UPDATE_TIMER_INTERVAL = 100
//let TARGET_DISTANCE: CGFloat = 300.0
let TARGET_DISTANCE_VELOCITY_TOTAL: Double = 300.0

let TARGET_POSITION_ANGULAR_INCREMENT: Double = 0.004
let TARGET_ORIENTATION_ANGULAR_INCREMENT: Double = 0.1
let TARGET_DISTANCE_VELOCITY_INCREMENT: Double = 0.05

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
    var targetUpdateTimer: Timer?
    var targetDirection: Double = 0.0
    var targetOrientation: Double = -0.1
    var targetDistanceVelocityRatio: Double = 0.6
    var targetDistanceVelocityIncrement: Double = 0.023

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
        setNeedsDisplay()
        if !spline.started {
            start()
        }
        else {
            stop()
        }
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

//        print(movement)
        /*
        let relativePositionVector = ZVector2()
        drawArcPath(
            relativePositionVector,
            relativeMovement,
            relativeEndPoint)
         */

        /*
        UIColor.green.setStroke()
        UIColor.green.setFill()
        drawPoint(center: previousPosition.position)
        drawLine(
            start: previousPosition.position,
            end: currentTargetPosition.position)
        drawArrow(
            position: currentTargetPosition.position,
            orientation: currentTargetPosition.orientation.get())

        UIColor.red.setStroke()
        UIColor.red.setFill()
        drawPoint(center: currentSplineTargetPosition)
        let endPointPosition = ZVector2(
            currentSplineTargetPosition.getX() + currentSplineTargetVelocity.getX(),
            currentSplineTargetPosition.getY() + currentSplineTargetVelocity.getY())
        drawLine(
            start: currentSplineTargetPosition,
            end: endPointPosition)
        drawPoint(center: endPointPosition)
        drawCircle(center: endPointPosition, radius: currentSplineTargetVelocity.getD())
        */

        ///*
        let targetDistance = TARGET_DISTANCE_VELOCITY_TOTAL * targetDistanceVelocityRatio
        let targetVelocity = TARGET_DISTANCE_VELOCITY_TOTAL * (1.0 - targetDistanceVelocityRatio)
        let currentPositionVector = ZVector2()
        let targetVector = ZVector2(
            targetDistance * sin(targetDirection),
            targetDistance * cos(targetDirection))
        let velocityVector = ZVector2(
            targetVelocity * sin(targetOrientation),
            targetVelocity * cos(targetOrientation))
        let targetEndVector = ZVector2(
            targetVector.getX() + velocityVector.getX(),
            targetVector.getY() + velocityVector.getY())

        //newest approach; prioritize orientation over Y movement
        //first determine how closely we will match the desired orientation by moving directly to the target point
        ///*
//        let angleAtTarget = snapAngle(2.0 * targetVector.atan2)
        let angleAtTarget = snapAngle(2.0 * targetVector.atan)
        let angleAtTargetToVelocity = subtractAngles(velocityVector.atan2, angleAtTarget)
        let cosAngle = cos(angleAtTargetToVelocity)
        let linearMoveRatio = (cosAngle + 1.0) / 2.0
//        let angularMoveRatio = 1.0 - linearMoveRatio
//        print("atv: ", RAD2DEG * velocityVector.atan2, " aat: ", RAD2DEG * angleAtTarget)
//        print("aattv: ", RAD2DEG * angleAtTargetToVelocity, " linear: ", linearMoveRatio)
        //*/
        
        /*
        let testPoint = ZVector2(
            targetVector.getX() * sin(angleAtTargetToVelocity),
            targetVector.getY() * cos(angleAtTargetToVelocity))
        drawArc(relativeTargetPosition: testPoint)
        */
        ///*
//        let scaledLinearDistance = targetVector.getY() * linearMoveRatio
//        let scaledAngularTurn = atan2(velocityVector.getX(), velocityVector.getY() - scaledLinearDistance)
        let scaledVelocityVector = ZVector2(velocityVector)
        scaledVelocityVector.multiply(linearMoveRatio)
//        scaledVelocityVector.add(velocityVector)
        let scaledEndVector = ZVector2(targetVector)
        scaledEndVector.add(scaledVelocityVector)
//        */
        /*
        drawConvergingArcPath(
            currentPositionVector,
            targetVector,
            velocityVector,
            targetEndVector)
        */
        /*
        drawArcPath(
            currentPositionVector,
            targetVector,
            targetEndVector)
        */
        /*
        drawConvergingPath(
            currentPositionVector,
            targetVector,
            velocityVector,
            targetEndVector)
//            scaledVelocityVector,
//            scaledEndVector)
         */
        /*
        drawApproachingPath(
            targetVector,
            velocityVector)
         */

        //display the path to the target
        drawDirectPath(
            currentPositionVector,
            targetVector,
            velocityVector)
//        drawArc(relativeTargetPosition: targetVector)

        /*
        UIColor.green.setStroke()
        UIColor.green.setFill()
        let midlineToTarget = ZVector2(
            targetVector.getX() / 2.0,
            targetVector.getY() / 2.0)
        drawLine(
            start: targetEndVector,
            end: midlineToTarget)
        let midlineToVelocity = ZVector2(
            targetVector.getX() + (velocityVector.getX() / 2.0),
            targetVector.getY() + (velocityVector.getY() / 2.0))
        drawLine(
            start: currentPositionVector,
            end: midlineToVelocity)
        let midlineToEndPoint = ZVector2(
            targetEndVector.getX() / 2.0,
            targetEndVector.getY() / 2.0)
        drawLine(
            start: targetVector,
            end: midlineToEndPoint)
        let midlinePoint = ZVector2(targetVector)
        midlinePoint.add(targetEndVector)
        midlinePoint.multiply(1.0 / 3.0)
        UIColor.black.setStroke()
        drawArc(relativeTargetPosition: midlinePoint)
        drawPoint(center: midlinePoint)
        //from current position to target position
//        drawCircle(center: currentPositionVector, radius: targetDistance)
        //inverse: from current position to velocity end point
//        drawCircle(center: currentPositionVector, radius: targetEndVector.getD())

        UIColor.purple.setStroke()
        UIColor.purple.setFill()
        //from target position to velocity end point
//        drawCircle(center: targetVector, radius: targetVelocity)
        //inverse: from target position to current position
//        drawCircle(center: targetVector, radius: targetDistance)

        UIColor.orange.setStroke()
        UIColor.orange.setFill()
        //from velocity end point to current position
        drawCircle(center: targetEndVector, radius: targetEndVector.getD())
        //inverse: from velocity end point to target position
        drawCircle(center: targetEndVector, radius: targetVelocity)

        UIColor.blue.setStroke()
        UIColor.blue.setFill()
        drawLine(start: currentPositionVector, end: targetEndVector)
 */

        /*
        let test0 = ZVector2(
            targetEndVector.getX() - (velocityVector.getD() * targetEndVector.sin),
            targetEndVector.getY() - (velocityVector.getD() * targetEndVector.cos))
//        let ratio = 0.5 + (cos(subtractAngles(velocityVector.atan2, targetVector.atan2) / 2.0) * 0.5)
        let ratio = 0.5
        let test1 = ZVector2(
            test0.getX() + ((targetVector.getX() - test0.getX()) * ratio),
            test0.getY() + ((targetVector.getY() - test0.getY()) * ratio))

        UIColor.blue.setStroke()
        UIColor.blue.setFill()
        drawPoint(center: test0)
        drawPoint(center: test1)
        drawArc(relativeTargetPosition: test1)
        drawLine(start: test0, end: targetVector)
         */
        
        /*
        let dotPV =
            (targetVector.getX() * velocityVector.sin) +
            (targetVector.getY() * velocityVector.cos)
        let dotVP =
            (velocityVector.getX() * targetVector.sin) +
            (velocityVector.getY() * targetVector.cos)
        let test = ZVector2(
            (targetVector.getX() + (dotVP * targetVector.sin) +
             velocityVector.getX() + (dotPV * velocityVector.sin)) / 2.0,
            (targetVector.getY() - (dotVP * targetVector.cos) +
             velocityVector.getY() - (dotPV * velocityVector.cos)) / 2.0)
         */
        /*
        let vDotT =
            (velocityVector.getX() * targetEndVector.sin) +
            (velocityVector.getY() * targetEndVector.cos)
        let vCrossT =
            (velocityVector.getX() * targetEndVector.cos) -
            (velocityVector.getY() * targetEndVector.sin)
        let tDotV =
            (targetVector.getX() * velocityVector.sin) +
            (targetVector.getY() * velocityVector.cos)
//        let crossTV =
//            (targetEndVector.getX() * velocityVector.cos) -
//            (targetEndVector.getY() * velocityVector.sin)
        let test0 = ZVector2(
//            targetEndVector.getX() - (vDotT * targetEndVector.sin),
//            targetEndVector.getY() - (vDotT * targetEndVector.cos))
            targetEndVector.getX() + (tDotV * targetEndVector.sin),
            targetEndVector.getY() + (tDotV * targetEndVector.cos))
        let test1 = ZVector2(
            test0.getX() - (vCrossT * targetEndVector.cos),
            test0.getY() + (vCrossT * targetEndVector.sin))
//            targetEndVector.getX() - (crossTV * velocityVector.cos),
//            test0.getY() + (crossTV * velocityVector.sin))
        UIColor.magenta.setStroke()
        UIColor.magenta.setFill()
        drawLine(start: currentPositionVector, end: test0)
        drawLine(start: test0, end: test1)
        drawLine(start: test1, end: currentPositionVector)
        drawArc(relativeTargetPosition: test1)
         */

        /*
        let crossPV =
            (targetVector.getX() * velocityVector.getY()) -
            (targetVector.getY() * velocityVector.getX())
//        let crossVP =
//            (velocityVector.getX() * targetVector.getY()) -
//            (velocityVector.getY() * targetVector.getX())
        let dotPV =
            (targetVector.getX() * velocityVector.getX()) +
            (targetVector.getY() * velocityVector.getY())
        let crossPVector = ZVector2(
            crossPV > 0.0 ? sqrt(abs(crossPV)) : -sqrt(abs(crossPV)),
            0.0)
//        let crossVVector = ZVector2(
//            0.0,
//            crossVP > 0.0 ? sqrt(abs(crossVP)) : -sqrt(abs(crossVP)))
        let dotVector = ZVector2(
            0.0,
            dotPV > 0.0 ? sqrt(abs(dotPV)) : -sqrt(abs(dotPV)))
        let triangulationPoint = ZVector2(
            crossPV > 0.0 ? sqrt(abs(crossPV)) : -sqrt(abs(crossPV)),
            dotPV > 0.0 ? sqrt(abs(dotPV)) : -sqrt(abs(dotPV)))

        UIColor.magenta.setStroke()
        UIColor.magenta.setFill()
//        drawLine(start: ZVector2(), end: triangulationPoint)
        drawLine(start: currentPositionVector, end: crossPVector)
        drawLine(start: crossPVector, end: triangulationPoint)
        drawLine(start: triangulationPoint, end: currentPositionVector)
//        drawLine(start: dotVector, end: currentPositionVector)
        drawArc(relativeTargetPosition: triangulationPoint)
         */

//        drawLine(start: currentPositionVector, end: crossVVector)
//        drawLine(start: crossVVector, end: dotVector)
//        drawLine(start: dotVector, end: currentPositionVector)
//        drawLine(start: triangulationPoint, end: targetVector)
//        drawLine(start: triangulationPoint, end: )
//        let cosPV = cos(subtractAngles(velocityVector.atan2, targetVector.atan2))
//        let sinPV = sin(subtractAngles(velocityVector.atan2, targetVector.atan2))
        /*
        let atanPV = atan2(crossPV, dotPV)
        let sinPV = sin(atanPV)
        let cosPV = cos(atanPV)

//        let length = sqrt(abs(crossPV) + abs(dotPV))
        let length = sqrt(abs(crossPV + dotPV))

//        let start = ZVector2()
        UIColor.magenta.setStroke()
        UIColor.magenta.setFill()
        let testTargetPosition = ZVector2(length * sinPV, length * cosPV)
        drawArc(relativeTargetPosition: testTargetPosition)
         */

        /*
        let tDotV = targetVector.dotVector(velocityVector)
        let tCrossV =
            (targetVector.getX() * velocityVector.getY()) -
            (targetVector.getY() * velocityVector.getX())
//        let tDotV = targetVector.dotVector(velocityVector.sin, velocityVector.cos)
//        let vDotT = velocityVector.dotVector(targetVector.sin, targetVector.cos)
//        let tCrossV =
//            (targetVector.getX() * velocityVector.getY()) -
//            (targetVector.getY() * velocityVector.getX())
        let defaultBiArcPath = ZBiArc(
            relativeTargetPosition: targetVector,
            d1: sqrt(abs(tDotV + tCrossV)),
//            d1: tDotV,
//            d1: targetVector.getD(),
//            d1: (tDotV + tCrossV) / 2.0,
//            d1: targetVector.getY() - velocityVector.getY(),
//            d1: targetEndVector.getY() * velocityVector.cos,
//            d1: velocityVector.getD(),
            relativeTargetVelocity: velocityVector)
//            d2: -(tDotV + tCrossV) / 2.0)
//            d2: vDotT)
//            d2: velocityVector.getD())
//            d2: -tDotV)
//            d2: tDotV * tCrossV)
        drawBiArc(biArc: defaultBiArcPath)
         */
        
        /*
        let inverseBiArcPath = ZBiArc(
            relativeTargetPosition: targetVector,
            d1: velocityVector.getD(),
            relativeTargetVelocity: velocityVector)
        drawBiArc(biArc: inverseBiArcPath)
         */

        /*
        //precalc = 2 * (1 - (t1 dot t2))
        let vDotV = targetVector.getD2()
        let t1DotT2 = velocityVector.getY()
        let t1DotT2Inv2 = 2.0 * (1.0 - t1DotT2)
        let discrim = sqrt( pow(vDotT, 2.0) + ( t1DotT2Inv2 * vDotV ) )
        
        //now find the smallest d value of the bi-arc to create the shortest bi-arc to the target
        let d = -vDotT + discrim
        let altD = -vDotT - discrim
        if abs(d) > abs(altD) {
            return altD / t1DotT2Inv2
        }
         */
        /*
        drawBiArcPath(
            targetVector,
            velocityVector,
            targetEndVector)
         */
    }
    
    fileprivate func drawDirectPath(
        _ fromPoint: ZVector2,
        _ toPoint: ZVector2,
        _ velocity: ZVector2)
    {
        UIColor.green.setStroke()
        UIColor.green.setFill()

        //draw the direct line from our current position to the target position
        drawLine(
            start: fromPoint,
            end: toPoint)
        drawPoint(center: fromPoint)
//        drawArc(relativeTargetPosition: toPoint)

        UIColor.purple.setStroke()
        UIColor.purple.setFill()
        drawArrow(
            position: toPoint,
            orientation: velocity.atan2)
        let velocityEndPoint = ZVector2(
            toPoint.getX() + velocity.getX(),
            toPoint.getY() + velocity.getY())
        drawLine(
            start: toPoint,
            end: velocityEndPoint)
        drawPoint(center: velocityEndPoint)
//        drawArc(relativeTargetPosition: velocityEndPoint)
    }
    
    fileprivate func drawApproachingPath(
//        _ k1_: Double,
//        _ k2_: Double,
//        _ min_velocity_: Double,
//        _ max_velocity_: Double,
//        _ max_accel_: Double,
//        _ max_angular_velocity_: Double,
        _ targetPoint: ZVector2,
        _ velocity: ZVector2)
    {
        let k1_ = 1.0
        let k2_ = 2.0
//        let max_velocity_ = targetPoint.getD() / k2_
        let max_velocity_ = abs((targetPoint.getD2() * targetPoint.atan) / (k2_ * targetPoint.getX()))
//        let max_velocity_ = targetPoint.getY()
        let beta_ = 0.6
        let lambda_ = 2.0

        //distance to the target position
        let r = targetPoint.getD()
        //orientation from target position to current orientation
        let delta = -targetPoint.atan2
        //orientation from goal pose to the direction of the target position
        let theta = subtractAngles(velocity.atan2, targetPoint.atan2)
//        let theta = velocity.atan2 - targetPoint.atan2

//        print("delta: ", (delta * RAD2DEG))
//        print("theta: ", (theta * RAD2DEG))

        let referenceDelta = atan(-k1_ * theta)
        let proportionalControlTerm = k2_ * subtractAngles(delta, referenceDelta)
        let feedforwardControlTerm = (1.0 + (k1_ / (1.0 + pow(k1_ * theta, 2.0)))) * sin(delta)
        //compute curvature
        let kappa = -(proportionalControlTerm + feedforwardControlTerm) / r

//        print("p: ", proportionalControlTerm)
//        print("f: ", feedforwardControlTerm)
//        print("k: ", kappa)

        //compute linear velocity
        let outputLinearVelocity = max_velocity_ / (1.0 + (beta_ * pow(fabs(kappa), lambda_)))
        //compute angular velocity
        let outputAngularVelocity = kappa * outputLinearVelocity

//        print("linear: ", outputLinearVelocity)
//        print("angular: ", (outputAngularVelocity * RAD2DEG))

        UIColor.black.setStroke()
        drawArc(outputLinearVelocity, outputAngularVelocity)
//        drawArc(-100.0, -Double.pi / 2.0)
    }

    fileprivate func drawConvergingPath(
        _ fromPoint: ZVector2,
        _ toPoint: ZVector2,
        _ velocity: ZVector2,
        _ endPoint: ZVector2)
    {
        //move linearly by halfway between toPoint y amount and endPoint y amount
//        let ratio = 0.5
//        let linearMovement = (toPoint.getY() * ratio) + (endPoint.getY() * (1.0 - ratio))
        let crossProduct = (toPoint.getX() * velocity.cos) - (toPoint.getY() * velocity.sin)
        let dotProduct = (toPoint.getX() * velocity.sin) + (toPoint.getY() * velocity.cos)
        let intersectVelocityLineY = toPoint.getY() - (toPoint.getY() * velocity.cos)
        let intersectionPoint1 = ZVector2(
            toPoint.getX() - (dotProduct * velocity.sin),
            toPoint.getY() - (dotProduct * velocity.cos))
//        let intersectVelocityLineX = toPoint.getX() + (toPoint.getX() * velocity.sin)
//        let intersectionPoint2 = ZVector2(
//            intersectVelocityLineX,
//            intersectVelocityLineY)
//        let linearMovement = max(intersectionPoint.getD(), toPoint.getD())

        //turn angularly to halfway between turn toPoint and turn to endPoint
//        let angleFromVelocityAtTarget = subtractAngles(velocity.atan2, 2.0 * toPoint.atan2)
//        let angularMovement = snapAngle((toPoint.atan2 + angleFromVelocityAtTarget) / 2.0)
//        let angularMovement = snapAngle((toPoint.atan2 * ratio) + (velocity.atan2 * (1.0 - ratio)))
//        let angularMovement = snapAngle((toPoint.atan2 + velocity.atan2) / 2.0)
//        let angularMovement = intersectionPoint.atan2

//        let targetPoint = ZVector2(linearMovement * Foundation.sin(angularMovement), linearMovement)
        
        UIColor.red.setStroke()
        drawLine(start: fromPoint, end: intersectionPoint1)
        drawLine(start: intersectionPoint1, end: toPoint)
        drawArc(relativeTargetPosition: intersectionPoint1)
//        drawLine(start: fromPoint, end: intersectionPoint2)
//        drawArc(relativeTargetPosition: intersectionPoint2)
//        drawPoint(center: intersectionPoint2)
//        drawArc(relativeTargetPosition: targetPoint)
//        drawPoint(center: targetPoint)

        //determine if the move is converging or diverging
        //angle at target / angle of velocity

        //converging
        //         left  / left
        //         right / right
        //determine if the converging move is over-converging or under-converging
        //it is under-converging when the absolute value of the angle at the target is less than the absolute value of the angle of the velocity

        //diverging
        //         left  / right
        //         right / left

    }
    
    fileprivate func calculateRelativeMonoArcPath()
    {
        //Reference: https://discourse.mcneel.com/t/how-to-make-an-arc-meet-another-curve-perpendicular/32778/9
        /*
        let innerRadius = min(endPoint.getD(), velocity.getD())
        let outerRadius = max(endPoint.getD(), velocity.getD())
        let moveY =
            (pow(outerRadius, 2.0) - pow(innerRadius, 2.0)) /
            ((2.8 * innerRadius) + (2.0 * outerRadius * endPoint.cos))
         */
//        let divisor = ((2.0 * relativeTargetVelocity.getD()) + (2.0 * relativeEndPoint.getD() * relativeEndPoint.cos))
//        let divisor = ((2.0 * relativeTargetVelocity.getD()) + (2.0 * abs(relativeEndPoint.getY())))
//        print("velocityD: ", velocity.getD(), " endPointD: ", endPoint.getD(), " endPointCos: ", endPoint.cos, " divisor: ", divisor)
//        if divisor == 0.0 {
//            relativeMovement.reset()
//            return
//        }
        
        let moveY = (relativeEndPoint.getD2() - relativeTargetVelocity.getD2()) /
            ((2.0 * relativeTargetVelocity.getD()) + (2.0 * abs(relativeEndPoint.getY())))
        let turnPoint2 = ZVector2(relativeEndPoint.getX(), relativeEndPoint.getY() - moveY)
        relativeMovement.set(
            moveY * turnPoint2.sin,
            moveY + (moveY * turnPoint2.cos))
        
//        print("moveY: ", moveY)
        /*
        if (relativeEndPoint.getD2() - relativeTargetVelocity.getD2()) * moveY < 0.0 {
            print("VIOLATION")
            relativeMovement.set(
                -moveY * turnPoint2.sin,
                moveY - (moveY * turnPoint2.cos))
        }
        else {
            relativeMovement.set(
                moveY * turnPoint2.sin,
                moveY + (moveY * turnPoint2.cos))
        }
         */
    }
    
    fileprivate func drawConvergingPath2(
        _ fromPoint: ZVector2,
        _ toPoint: ZVector2,
        _ velocity: ZVector2,
        _ endPoint: ZVector2)
    {
        let velocity_2 = ZVector2(velocity.getX() / 2.0, velocity.getY() / 2.0)
        let endPoint_2 = ZVector2(toPoint.getX() + velocity_2.getX(), toPoint.getY() + velocity_2.getY())
        /*
        let denominator = (2.0 * velocity.getD()) + (2.0 * endPoint.getY())
        let moveY = (endPoint.getD2() - velocity.getD2()) / denominator
        let convergencePoint = ZVector2(endPoint.getX(), endPoint.getY() - moveY)
         */
        let denominator = (2.0 * velocity_2.getD()) + (2.0 * endPoint_2.getY())
        let moveY = (endPoint_2.getD2() - velocity_2.getD2()) / denominator
        let convergencePoint = ZVector2(endPoint_2.getX(), endPoint_2.getY() - moveY)

        if denominator > 0.0 {
            convergencePoint.setD(moveY)
        }
        else {
            convergencePoint.setD(-moveY)
        }
        convergencePoint.add(0.0, moveY)

        let color: UIColor
        if abs(subtractAngles(2.0 * convergencePoint.atan, velocity_2.atan2)) <= (Double.pi / 2.0) {
            color = UIColor.blue
        }
        else {
            color = UIColor.red
        }
        color.setStroke()
        color.setFill()
        let turnPoint1 = ZVector2(0, moveY)
        
        //draw the basics
        drawLine(
            start: fromPoint,
            end: toPoint)
        drawLine(
            start: toPoint,
            end: endPoint)
        drawPoint(center: endPoint)

        //draw the target velocity
//        drawCircle(
//            center: endPoint2,
//            radius: velocity2.getD())
        drawCircle(
            center: endPoint_2,
            radius: velocity_2.getD())

        //draw the first arc and outline
        drawLine(
            start: fromPoint,
            end: convergencePoint)
        drawLine(
            start: fromPoint,
            end: turnPoint1)
        drawLine(
            start: turnPoint1,
            end: convergencePoint)
        drawArc(relativeTargetPosition: convergencePoint)
        
        //draw the second arc and outline
        drawLine(
            start: convergencePoint,
            end: endPoint)
        drawLine(
            start: convergencePoint,
            end: endPoint_2)

        /*
        //draw the triangle around the arc
        
        //draw the arc
        let radius = convergencePoint.getD2() / (2.0 * convergencePoint.getX())
        //draw the turn circle
        drawCircle(
            center: ZVector2(radius, 0),
            radius: abs(radius))
        */
        
        /*
        let denominator2 = -(2.0 * velocity.getD()) + (2.0 * endPoint.getY())
        let moveY2 = (endPoint.getD2() - velocity.getD2()) / denominator2
//        print("moveY: ", moveY, " moveY2: ", moveY2)
        let convergencePoint2 = ZVector2(endPoint.getX(), endPoint.getY() - moveY2)
        if denominator2 > 0.0 {
            convergencePoint2.setD(moveY2)
        }
        else {
            convergencePoint2.setD(-moveY2)
        }
        convergencePoint2.add(0.0, moveY2)
//        print("convergencePoint1: ", convergencePoint)
//        print("convergencePoint2: ", convergencePoint2)
//        let radius2 = convergencePoint2.getD2() / (2.0 * convergencePoint2.getX())
//        print("radius: ", radius, " radius2: ", radius2)
        drawPoint(center: convergencePoint2)
//        drawCircle(
//            center: ZVector2(radius2, 0),
//            radius: abs(radius2))
        */

        //draw the line from the destination point on the arc to the center of the target velocity circle
        /*
        drawLine(
            start: convergencePoint,
            end: endPoint_2)
        drawLine(
            start: endPoint_2,
            end: endPoint2)
        drawPoint(center: convergencePoint)
         */

        //draw the target velocity circle
//        drawCircle(
//            center: endPoint,
//            radius: convergentRadius)
    }
    
    fileprivate func drawConvergingArcPath(
        _ currentPosition: ZVector2,
        _ targetPosition: ZVector2,
        _ targetVelocity: ZVector2,
        _ endPoint: ZVector2)
    {
//        let dotProduct = targetPosition.dotOrientation(targetVelocity.atan2)
//        let dotProduct =
//            (targetPosition.getX() * targetVelocity.sin) +
//            (targetPosition.getY() * targetVelocity.cos)
        let dotProduct = targetPosition.dotVector(targetVelocity)
        let targetPosition2 = ZVector2(
            targetPosition.getX() + (dotProduct * targetVelocity.sin),
            targetPosition.getY() + (dotProduct * targetVelocity.cos))
        drawArcPath(currentPosition, targetPosition2, endPoint)
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
    
    /*
    fileprivate func constrain(
        _ value: CGFloat,
        _ min: CGFloat,
        _ max: CGFloat) -> CGFloat
    {
        if value < min {
            return min
        }
        else if value > max {
            return max
        }
        
        return value
    }
     */

    /*
    fileprivate func drawTurn(
        radius: CGFloat,
        theta: CGFloat)
    {
        drawArc(
            centerPoint: ZVector2(radius, 0),
            radius: -radius,
            startAngle: radius > 0 ? -PI_2 : PI_2,
            deltaAngle: theta)
    }
     */

    /*
    fileprivate func calculateRelativeBiArcKnot(
        _ relativeTargetPosition: ZVector2,
        _ relativeTargetOrientation: CGFloat) -> ZVector2
    {
        if relativeTargetOrientation == 0.0 {
            return ZVector2(
                relativeTargetPosition.getX() / 2.0,
                relativeTargetPosition.getY() / 2.0)
        }
        
        let t2x = sin(relativeTargetOrientation)
        let t2y = cos(relativeTargetOrientation)
        
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
        let vDotT = (relativeTargetPosition.getX() * t2x) +
            (relativeTargetPosition.getY() * (1.0 + t2y))

        //t1 dot t2 = (t1.x * t2.x)           + (t1.y * t2.y)
        //          = (sin(p1.o) * sin(p2.o)) + (cos(p1.o) * cos(p2.o))
        //          = t2.y
        let t1DotT2Inv2 = 2.0 * (1.0 - t2y)
        let discrim = sqrt( pow(vDotT, 2.0) + ( t1DotT2Inv2 * vDotV ) )
        let d1 = (-vDotT + discrim) / t1DotT2Inv2
        let d2 = d1

        let knot = calculateRelativeBiArcKnot(
            relativeTargetPosition: relativeTargetPosition,
            t2x: t2x,
            d1: d1,
            t2y: t2y)
        //when d1 and d2 are equal...
        //knot   = ( p1 + p2 + ( d * (t1 - t2) ) ) / 2
        //knot   = ( p2 + ( d * (t1 - t2) ) ) / 2
        */
        /*
        let knot = CGPoint(
            //knot.x = ( p2.x + ( d * (0 - t2.x) ) ) / 2
            //knot.x = ( p2.x + ( d * -t2.x ) ) / 2
            x: ( relativeTargetPosition.x + (d1 * -t2x) ) / 2.0,
            //knot.y = ( p2.y + ( d * (1 - t2.y) ) ) / 2
            y: ( relativeTargetPosition.y + (d1 * (1.0 - t2y)) ) / 2.0)
         */
//        print(knot.x, d1, vDotT, discrim, t1DotT2Inv2)

        /*
        //v dot t2 = (v.x * t2.x)      + (v.y * t2.y)
        //         = (v.x * sin(p2.o)) + (v.y * cos(p2.o))
        let vDotT2 = (relativeTargetPosition.x * t2x) + (relativeTargetPosition.y * t2y)
 //        let d1 = vDotT2 * 0.5

         //d2 = ((vDotV / 2.0) - (d * vDotT1)) / (vDotT2 - (d * (t2y - 1.0)))
        //   = ((vDotV / 2.0) - (d * v.y)) / (vDotT2 - (d * (t2y - 1.0)))
        let d2 = ((vDotV / 2.0) - (d1 * relativeTargetPosition.y)) /
            (vDotT2 - (d1 * (t2y - 1.0)))
        let d1Ratio = d1 / (d1 + d2)

        //knot   = ( (p1 + (d1 * t1) * (d2 / (d1+d2)) ) + ( (p2 - (d2 * t2) * (d1 / (d1+d2)) )
        let knot = CGPoint(
            //knot.x = ( (p1.x + (d1 * t1.x) * (d2 / (d1+d2)) ) + ( (p2.x - (d2 * t2.x) * (d1 / (d1+d2)) )
            //       = (p2.x - (d2 * t2.x)) * d1Ratio
            x: (relativeTargetPosition.x - (d2 * t2x)) * d1Ratio,
            //knot.y = ( (p1.y + (d1 * t1.y) * (d2 / (d1+d2)) ) + ( (p2.y - (d2 * t2.y) * (d1 / (d1+d2)) )
            //       = ( ( d1 * (d2 / (d1+d2)) ) + ( (p2.y - (d2 * t2.y) * (d1 / (d1+d2)) )
            //       = (d2 * d1Ratio) + ( (p2.y - (d2 * t2.y)) * d1Ratio )
            y: (d2 * d1Ratio) + ((relativeTargetPosition.y - (d2 * t2y)) * d1Ratio))
         */
        /*

        UIColor.purple.setStroke()
        let d1point = ZVector2(0.0, d1)
        let d2point = ZVector2(
            relativeTargetPosition.getX() - (d2 * t2x),
            relativeTargetPosition.getY() - (d2 * t2y))
        drawLine(
            start: ZVector2(0.0, 0.0),
            end: d1point)
        drawPoint(center: d1point)
        drawLine(
            start: relativeTargetPosition,
            end: d2point)
        drawPoint(center: d2point)
        
        return knot
    }
    */
    
    /*
    fileprivate func calculateRelativeBiArcKnot(
        relativeTargetPosition: ZVector2,
        t2x: CGFloat,
        d1: CGFloat,
        t2y: CGFloat) -> ZVector2
    {
        let vDotV = pow(relativeTargetPosition.getX(), 2.0) + pow(relativeTargetPosition.getY(), 2.0)
        let vDotT1 = relativeTargetPosition.getY()
        let vDotT2 =
            (relativeTargetPosition.getX() * t2x) +
            (relativeTargetPosition.getY() * t2y)
        let d2 = ((vDotV / 2.0) - (d1 * vDotT1)) / (vDotT2 - (d1 * (t2y - 1.0)))
        let d1Ratio = d1 / (d1 + d2)
        //knot.x = ( (p1.x + (d1 * t1.x) * (d2 / (d1+d2)) ) + ( (p2.x - (d2 * t2.x) * (d1 / (d1+d2)) )
        //       = (p2.x - (d2 * t2.x)) * d1Ratio
        //knot.y = ( (p1.y + (d1 * t1.y) * (d2 / (d1+d2)) ) + ( (p2.y - (d2 * t2.y) * (d1 / (d1+d2)) )
        //       = ( ( d1 * (d2 / (d1+d2)) ) + ( (p2.y - (d2 * t2.y) * (d1 / (d1+d2)) )
        //       = (d2 * d1Ratio) + ( (p2.y - (d2 * t2.y)) * d1Ratio )
        return ZVector2(
            (relativeTargetPosition.getX() - (d2 * t2x)) * d1Ratio,
            (d2 * d1Ratio) + ((relativeTargetPosition.getY() - (d2 * t2y)) * d1Ratio))
    }

    fileprivate func calculateRelativeBiArcKnot(
        relativeTargetPosition: ZVector2,
        t2x: CGFloat,
        t2y: CGFloat,
        d2: CGFloat) -> ZVector2
    {
        let vDotV = pow(relativeTargetPosition.getX(), 2.0) + pow(relativeTargetPosition.getY(), 2.0)
        let vDotT1 = relativeTargetPosition.getY()
        let vDotT2 =
            (relativeTargetPosition.getX() * t2x) +
            (relativeTargetPosition.getY() * t2y)
        let d1 = ((vDotV / 2.0) - (d2 * vDotT2)) / (vDotT1 - (d2 * (t2y - 1.0)))
//        print(d1, d2)
        let d1Ratio = d1 / (d1 + d2)
        //knot.x = ( (p1.x + (d1 * t1.x) * (d2 / (d1+d2)) ) + ( (p2.x - (d2 * t2.x) * (d1 / (d1+d2)) )
        //       = (p2.x - (d2 * t2.x)) * d1Ratio
        //knot.y = ( (p1.y + (d1 * t1.y) * (d2 / (d1+d2)) ) + ( (p2.y - (d2 * t2.y) * (d1 / (d1+d2)) )
        //       = ( ( d1 * (d2 / (d1+d2)) ) + ( (p2.y - (d2 * t2.y) * (d1 / (d1+d2)) )
        //       = (d2 * d1Ratio) + ( (p2.y - (d2 * t2.y)) * d1Ratio )
        return ZVector2(
            (relativeTargetPosition.getX() - (d2 * t2x)) * d1Ratio,
            (d2 * d1Ratio) + ((relativeTargetPosition.getY() - (d2 * t2y)) * d1Ratio))
    }
     */

}
