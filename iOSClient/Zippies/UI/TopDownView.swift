
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

        let targetDistance = TARGET_DISTANCE_VELOCITY_TOTAL * targetDistanceVelocityRatio
        let targetVelocity = TARGET_DISTANCE_VELOCITY_TOTAL * (1.0 - targetDistanceVelocityRatio)
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

        //display the path to the target
        drawDirectPath(
            currentPositionVector,
            targetVector,
            velocityVector)

        //display the path of our brand new algorithm
        drawScissorPath(
            currentPositionVector,
            targetVector,
            velocityVector)
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
    
    fileprivate func drawScissorPath(
        _ fromPoint: ZVector2,
        _ toPoint: ZVector2,
        _ velocity: ZVector2)
    {
        let targetLinearDistance = (toPoint.getY() / 2.0) + (velocity.getY() / 4.0)
        let velocityConvergencePoint = ZVector2(
            toPoint.getX() + (velocity.getX() / 2.0),
            toPoint.getY() + (velocity.getY() / 2.0))
        
        let velocityConvergenceVector = ZVector2(
            velocityConvergencePoint.getX(),
            velocityConvergencePoint.getY() - targetLinearDistance)
        let targetPoint = ZVector2(
            fromPoint.getX() + (targetLinearDistance * velocityConvergenceVector.sin),
            targetLinearDistance + (targetLinearDistance * velocityConvergenceVector.cos))
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
        UIColor.blue.setStroke()
        UIColor.blue.setFill();
        drawPoint(center: targetPoint)
        drawArc(relativeTargetPosition: targetPoint)
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
    
}
