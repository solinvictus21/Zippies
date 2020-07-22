
import Foundation
import UIKit

fileprivate let PI_2: CGFloat = CGFloat.pi / 2.0
fileprivate let RAD2DEG: CGFloat = 180.0 / CGFloat.pi
fileprivate let DEG2RAD: CGFloat = CGFloat.pi / 180.0

let ZIPPY_WIDTH: CGFloat = 29.4
let ZIPPY_HEIGHT: CGFloat = 24.8
let ZIPPY_HALF_WIDTH: CGFloat = ZIPPY_WIDTH / 2.0
let ZIPPY_HALF_HEIGHT: CGFloat = ZIPPY_HEIGHT / 2.0
let ZIPPY_AREA_WIDTH: CGFloat = 2000.0
let ZIPPY_AREA_HEIGHT: CGFloat = 1000.0

let TARGET_UPDATE_TIMER_INTERVAL = 100
let TARGET_DISTANCE: CGFloat = 400.0

let TARGET_POSITION_ANGULAR_INCREMENT: CGFloat = 0.004
let TARGET_ORIENTATION_ANGULAR_INCREMENT: CGFloat = 0.05
let TARGET_DISTANCE_VELOCITY_INCREMENT: CGFloat = 0.05
let RADIANS_TO_DEGREES: CGFloat = 57.295779513082321

fileprivate let ZIPPY_ARROW_RADIUS: CGFloat = 20
fileprivate let ZIPPY_LINE_WIDTH: CGFloat = 3
fileprivate let ZIPPY_THIN_LINE_WIDTH: CGFloat = 1.0
fileprivate let ZIPPY_POINT_RADIUS: CGFloat = 5

class TopDownView : UIView
{
    var targetUpdateTimer: Timer?
    var targetDirection: CGFloat = 0.0
    var targetOrientation: CGFloat = -0.1
    var targetVelocity: CGFloat = 154.7
    var distanceVelocityRatio: CGFloat = 0.5

    var zippyManager: ZippyManager?
    
    fileprivate var moveToDisplay: ZDrawable

    let EASING: Double = 1/3
    
    init(s: String, i: Int)
    {
        moveToDisplay = planTestDrawable()
        super.init(frame: CGRect(x: 0, y: 0, width: 100, height: 100))
    }
    
//    fileprivate let INCREMENTS: Int = 10
    required init?(coder aDecoder: NSCoder)
    {
        moveToDisplay = planTestDrawable()
        super.init(coder: aDecoder)
        
        /*
        for i in 0...INCREMENTS {
            let interpolatedValue = bezierEaseInOut3(
                Double(i)/Double(INCREMENTS),
                0.5)
            print(interpolatedValue)
        }
         */
    }
    
    override func touchesBegan(_ touches: Set<UITouch>, with event: UIEvent?)
    {
        super.touchesBegan(touches, with: event)
        setNeedsDisplay()
        if targetUpdateTimer == nil {
            targetUpdateTimer = Timer.scheduledTimer(withTimeInterval: 0.05, repeats: true, block: { timer in
                self.targetDirection = addAngles(self.targetDirection, TARGET_POSITION_ANGULAR_INCREMENT)
                self.targetOrientation = addAngles(self.targetOrientation, TARGET_ORIENTATION_ANGULAR_INCREMENT)
                self.setNeedsDisplay()
                })
        }
        else {
            targetUpdateTimer!.invalidate()
            targetUpdateTimer = nil
        }
    }

    fileprivate func bezierEaseInOut(
        _ t: CGFloat,
        _ a: CGFloat,
        _ b: CGFloat) -> CGFloat
    {
      let u = 1.0 - t
      let u2 = u * u
      let t2 = t * t
      let t3 = t2 * t

      let u2t3 = 3.0 * u2 * t
      let ut23 = 3.0 * u * t2
      return (u2t3 * a) + (ut23 * b) + t3
    }

    fileprivate func bezierEaseInOut2(
        _ t: Double,
        _ a: Double,
        _ b: Double) -> Double
    {
      let t2 = t * t
      let t3 = t2 * t
      let mt = 1-t
      let mt2 = mt * mt
      return (3*a*mt2*t) + (3*b*mt*t2) + t3
    }

    fileprivate func bezierEaseInOut3(
        _ t: Double,
        _ a: Double) -> Double
    {
      let t2 = t * t
      let mt = 1-t
      return (a*2*mt*t) + t2
    }
    
    override func draw(_ rect: CGRect)
    {
        guard let context = UIGraphicsGetCurrentContext() else {
            return
        }
        
        let interpolationX = self.frame.width / ZIPPY_AREA_WIDTH
        let interpolationY = self.frame.height / ZIPPY_AREA_HEIGHT
        context.translateBy(x: self.frame.width/2, y: self.frame.height/2)
        context.scaleBy(x: interpolationX, y: -interpolationY)

        let currentPoint = CGPoint(x: 0, y: 0)
        //calculate the x/y coordinates of the target position
        let target = CGPoint(
            x: TARGET_DISTANCE * sin(targetDirection),
            y: TARGET_DISTANCE * cos(targetDirection))
        let velocityPoint = CGPoint(
            x: targetVelocity * sin(targetOrientation),
            y: targetVelocity * cos(targetOrientation))

        //display the path to the target
        drawDirectPath(
            currentPoint,
            target,
            velocityPoint)
        
//        drawOriginFrameOfReference(target)
//        drawTargetFrameOfReference(target, targetOrientation)
//        drawBiArcMergePath2(
//            target,
//            targetOrientation)
        /*
        drawNewTest(
            target,
            targetOrientation)
         */
        let targetPositionVector = KVector2(
            Double(TARGET_DISTANCE * sin(targetDirection)),
            Double(TARGET_DISTANCE * cos(targetDirection)))
        let targetVelocityVector = KVector2(
            Double(targetVelocity * sin(targetOrientation)),
            Double(targetVelocity * cos(targetOrientation)))
        testDotProductTargets(
            targetPositionVector,
            targetVelocityVector)
        /*
        drawWithVelocity(
            target,
            targetOrientation,
            targetVelocity)
         */
    }
    
    fileprivate func drawDirectPath(
        _ fromPoint: CGPoint,
        _ toPoint: CGPoint,
        _ velocity: CGPoint)
    {
        UIColor.green.setStroke()
        UIColor.green.setFill()

        //draw the direct line from our current position to the target position
        drawLine(
            fromPoint,
            toPoint,
            ZIPPY_LINE_WIDTH)
        drawPoint(fromPoint)

        drawArrow(
            toPoint,
            targetOrientation,
            ZIPPY_LINE_WIDTH)

        UIColor.purple.setStroke()
        UIColor.purple.setFill()
        let velocityEndPoint = CGPoint(
            x: toPoint.x + velocity.x,
            y: toPoint.y + velocity.y)
        drawLine(toPoint, velocityEndPoint, ZIPPY_THIN_LINE_WIDTH)
        drawPoint(velocityEndPoint)
    }
    
    fileprivate func testDotProductTargets(
        _ targetPosition: KVector2,
        _ targetVelocity: KVector2)
    {
        //unit vector S = targetVelocity sin and cos
        //S0 = r1 cross S = targetPosition cross S
        let s0 =
            (targetPosition.getX() * targetVelocity.cos) -
            (targetPosition.getY() * targetVelocity.sin)
        //S0h = S0 + (h * S) = 
        let dotProduct =
            (targetPosition.getX() * targetVelocity.getX()) +
            (targetPosition.getY() * targetVelocity.getY())
        let crossProductZ =
            (targetPosition.getX() * targetVelocity.getY()) -
            (targetPosition.getY() * targetVelocity.getX())
            
        let targetPositionPoint = CGPoint(
            x: targetPosition.getX(),
            y: targetPosition.getY())
        //velocity dot position
        let velocityDotPosition = dotProduct / targetPosition.getD();
        let positionPoint = CGPoint(
            x: targetPosition.getX() + (velocityDotPosition * targetPosition.sin),
            y: targetPosition.getY() + (velocityDotPosition * targetPosition.cos))
        //position dot velocity
        let positionDotVelocity = dotProduct / targetVelocity.getD();
        let velocityPoint = CGPoint(
            x: targetPosition.getX() - (positionDotVelocity * targetVelocity.sin),
            y: targetPosition.getY() - (positionDotVelocity * targetVelocity.cos))
        let velocityEndPoint = CGPoint(
            x: targetPosition.getX() + targetVelocity.getX(),
            y: targetPosition.getY() + targetVelocity.getY())
        
        drawPoint(velocityEndPoint)
        drawLine(positionPoint, velocityEndPoint, ZIPPY_THIN_LINE_WIDTH)
        drawLine(positionPoint, targetPositionPoint, ZIPPY_THIN_LINE_WIDTH)
        drawPoint(positionPoint)
        drawLine(velocityPoint, CGPoint(), ZIPPY_THIN_LINE_WIDTH)
        drawLine(velocityPoint, targetPositionPoint, ZIPPY_THIN_LINE_WIDTH)
        drawPoint(velocityPoint)
    }
    
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
    
    fileprivate func drawWithVelocity(
        _ relativeTargetPosition: CGPoint,
        _ relativeTargetOrientation: CGFloat,
        _ velocity: CGFloat)
    {
        let dotLength = (relativeTargetPosition.x * sin(relativeTargetOrientation)) +
            (relativeTargetPosition.y * cos(relativeTargetOrientation))
        let convergencePoint = CGPoint(
            x: relativeTargetPosition.x + (dotLength * sin(relativeTargetOrientation)),
            y: relativeTargetPosition.y + (dotLength * cos(relativeTargetOrientation)))
        let currentVelocityTarget = CGPoint(
            x: relativeTargetPosition.x + (sin(relativeTargetOrientation) * velocity),
            y: relativeTargetPosition.y + (cos(relativeTargetOrientation) * velocity))
        /*
        let d2 = pow(currentVelocityTarget.x, 2.0) + pow(currentVelocityTarget.y, 2.0)
        let movementRadius = CGFloat(d2 / (2.0 * currentVelocityTarget.x))
        let movementTheta = atan2(
            relativeTargetPosition.y,
            (movementRadius - relativeTargetPosition.x))
         */
        
        UIColor.purple.setStroke()
        UIColor.purple.setFill()
        drawPoint(convergencePoint)
        drawTurn(convergencePoint)
        drawLine(
            relativeTargetPosition,
            currentVelocityTarget,
            ZIPPY_THIN_LINE_WIDTH)
        /*
        drawLine(
            CGPoint(),
            CGPoint(x: movementRadius, y: 0),
            ZIPPY_THIN_LINE_WIDTH)
        drawTurn(
            radius: movementRadius,
            theta: movementTheta);
         */
    }
    
    fileprivate func drawTurn(_ relativeTargetPosition: CGPoint)
    {
        let d2 = pow(relativeTargetPosition.x, 2.0) + pow(relativeTargetPosition.y, 2.0)
        let movementRadius = CGFloat(d2 / (2.0 * relativeTargetPosition.x))
        let movementTheta = 2.0 * atan2(
            relativeTargetPosition.x,
            relativeTargetPosition.y)
        drawTurn(radius: movementRadius, theta: movementTheta)
    }
    
    fileprivate func atanSafe(_ x: CGFloat, _ y: CGFloat) -> CGFloat
    {
        if y == 0.0 {
            if x == 0.0 {
                return 0.0
            }
            else if x < 0.0 {
                return -CGFloat.pi / 2.0
            }
            else {
                return CGFloat.pi / 2.0
            }
        }
        
        return atan(x / y)
    }
    
    fileprivate func drawNewTest(
        _ relativeTargetPosition2: CGPoint,
        _ relativeTargetOrientation: CGFloat)
    {
        /*
        let constraint = min(
            abs(relativeTargetPosition2.x),
            abs(relativeTargetPosition2.y))
        let relativeTargetPosition1 = CGPoint(
            x: constrain(
                relativeTargetPosition2.x,
                -constraint,
                constraint),
            y: constrain(
                relativeTargetPosition2.y,
                -constraint,
                constraint))
        let rG2 = pow(relativeTargetPosition1.x,2)+pow(relativeTargetPosition1.y,2)
        let idealRadius = rG2 / (2.0 * relativeTargetPosition1.x)
        drawNewTest(
            relativeTargetPosition: relativeTargetPosition2,
            relativeTargetOrientation: relativeTargetOrientation,
            movementRadius: idealRadius * 0.99999999)
         */

        UIColor.magenta.setStroke()
        UIColor.magenta.setFill()
        let targetDirection = directionToTarget(relativeTargetPosition2)
        let deltaOrientationAtTarget = subtractAngles(
            relativeTargetOrientation,
            2.0 * targetDirection)
        drawNewTest(
            relativeTargetPosition: relativeTargetPosition2,
            relativeTargetOrientation: relativeTargetOrientation,
            radiusFactor: cos(deltaOrientationAtTarget / 2.0))
//        drawNewTest(
//            relativeTargetPosition: relativeTargetPosition2,
//            relativeTargetOrientation: relativeTargetOrientation,
//            radiusFactor: 0.5 - (sin(deltaOrientationAtTarget) * 0.5))
    }
    
    fileprivate func drawNewTest(
        relativeTargetPosition: CGPoint,
        relativeTargetOrientation: CGFloat,
        radiusFactor: CGFloat)
    {
        guard relativeTargetPosition.x != 0.0 else {
            return
        }
        
        print("radiusFactor: ", radiusFactor)
        let rG2 = pow(relativeTargetPosition.x,2)+pow(relativeTargetPosition.y,2)
        let idealRadius = rG2 / (2.0 * relativeTargetPosition.x)
        let movementRadius = idealRadius * radiusFactor

        let targetDirection = directionToTarget(relativeTargetPosition)
//        let deltaOrientationAtTarget = subtractAngles(
//            relativeTargetOrientation,
//            2.0 * targetDirection)
//        let factor = cos(deltaOrientationAtTarget / 2.0)
//        let factor = 0.5 + (cos(deltaOrientationAtTarget) * 0.5)
        // factor = 0.5d + (factor * 0.25d);
//        let factor = 0.1 + (cos(deltaOrientationAtTarget / 2.0) * 0.7)

        //calculate the amount to turn directly toward the target
        let radiusToTarget = CGPoint(
            x: relativeTargetPosition.y,
            y: movementRadius - relativeTargetPosition.x)
//        let distance = sqrt(pow(radiusToTarget.x,2.0)+pow(radiusToTarget.y,2.0))
        let completeTurnToTarget = atan2(radiusToTarget.x, radiusToTarget.y)
        let movementTheta = completeTurnToTarget * radiusFactor
        /*
        let movementTheta = (completeTurnToTarget > 0.0)
            ? completeTurnToTarget - acos(movementRadius / distance)
            : completeTurnToTarget + acos(movementRadius / distance)
         */
//        print(distance, completeTurnToTarget)
        drawTurn(
            radius: movementRadius,
            theta: movementTheta)

        let centerPoint = CGPoint(x: movementRadius, y: 0)
        let intersectionPoint = CGPoint(
            x: movementRadius - (movementRadius * cos(movementTheta)),
            y: movementRadius * sin(movementTheta))
//        let idealRadius2 = sqrt(rG2) / (2.0 * sin(targetDirection))
//        print(idealRadius / distance, movementRadius / distance, movementTheta, intersectionPoint)
        print("thetaFactor:  ", movementTheta / abs(completeTurnToTarget))
        print()

        drawLine(
            intersectionPoint,
            relativeTargetPosition,
            ZIPPY_THIN_LINE_WIDTH)
        
        let halfTurnPoint = CGPoint(
            x: movementRadius - (movementRadius * cos(targetDirection)),
            y: movementRadius * sin(targetDirection))
//        drawPoint(halfTurnPoint)
        drawLine(
            centerPoint,
            halfTurnPoint,
            ZIPPY_THIN_LINE_WIDTH)
        /*
        let remainingTurn = acos(
            ((sin(targetDirection) * relativeTargetPosition.x) +
             (cos(targetDirection) * relativeTargetPosition.y)) / sqrt(rG2))
        let movementTurn2 = targetDirection + remainingTurn
        drawTurn(
            radius: movementRadius,
            theta: movementTurn2)
         */
        /*
        let dotLength =
            relativeTargetPosition.x - movementRadius
        let testDirection = atan2(relativeTargetPosition.x - movementRadius, relativeTargetPosition.y)
        print(dotLength)
        drawLine(
            centerPoint,
            CGPoint(
                x: centerPoint.x + (dotLength * sin(testDirection)),
                y: centerPoint.x + (dotLength * cos(testDirection))),
            ZIPPY_THIN_LINE_WIDTH)
        */

        drawPoint(intersectionPoint)
        drawLine(
            CGPoint(),
            intersectionPoint,
            ZIPPY_THIN_LINE_WIDTH)
        drawLine(
            intersectionPoint,
            centerPoint,
            ZIPPY_THIN_LINE_WIDTH)
        drawLine(
            centerPoint,
            relativeTargetPosition,
            ZIPPY_THIN_LINE_WIDTH)

        UIColor.blue.setStroke()
        UIColor.blue.setFill()
        drawLine(
            centerPoint,
            CGPoint(),
            ZIPPY_THIN_LINE_WIDTH)
//        print("f")
//        print("g")

        /*
        let midTurnPoint = CGPoint(
            x: movementRadius - (movementRadius * cos(movementTurn/2)),
            y: movementRadius * sin(movementTurn/2))
        drawLine(
            centerPoint,
            midTurnPoint,
            ZIPPY_THIN_LINE_WIDTH)
        drawPoint(midTurnPoint)
         */

        /*
        let midTurnPoint = CGPoint(
            x: movementRadius - (movementRadius * cos(targetDirection)),
            y: movementRadius * sin(targetDirection))
        drawLine(
            centerPoint,
            midTurnPoint,
            ZIPPY_THIN_LINE_WIDTH)
        drawPoint(midTurnPoint)
         */


        /*
        let sinO = sin(relativeTargetOrientation)
        let cosO = cos(relativeTargetOrientation)
        let dotO =
            (relativeTargetPosition.x * sinO) +
            (relativeTargetPosition.y * cosO);
        
        UIColor.blue.setStroke()
        let dotOPoint = CGPoint(
            x: relativeTargetPosition.x + (dotO * sinO),
            y: relativeTargetPosition.y + (dotO * cosO))
        drawLine(
            relativeTargetPosition,
            dotOPoint,
            ZIPPY_LINE_WIDTH)
        drawLine(
            CGPoint(),
            dotOPoint,
            ZIPPY_LINE_WIDTH)
        let dotOPoint2 = CGPoint(
            x: relativeTargetPosition.x - (dotO * sinO),
            y: relativeTargetPosition.y - (dotO * cosO))
        drawLine(
            relativeTargetPosition,
            dotOPoint2,
            ZIPPY_LINE_WIDTH)
        drawLine(
            CGPoint(),
            dotOPoint2,
            ZIPPY_LINE_WIDTH)
        */
        /*
        let movementFactor = cos(deltaTurnAtTarget)
        let perfectTargetDirection = addAngles(
            movementFactor * targetDirection,
            (1.0 - movementFactor) * deltaTurnAtTarget)
        let sinO = sin(perfectTargetDirection)
        let cosO = cos(perfectTargetDirection)
        let rG = movementFactor *
            ((relativeTargetPosition.x * sinO) +
            (relativeTargetPosition.y * cosO))
        let weightedTarget = CGPoint(
            x: rG * sinO,
            y: rG * cosO)
         */
        /*
        let perfectTargetDirection = addAngles(
            targetDirection,
            deltaTurnAtTarget)
        let rG =
            ((relativeTargetPosition.x * sinO) +
            (relativeTargetPosition.y * cosO))
        let sinO = sin(perfectTargetDirection)
        let cosO = cos(perfectTargetDirection)
        */
        /*
        let weightedTarget = CGPoint(
            x: (movementFactor * relativeTargetPosition.x) +
                ((1.0 - movementFactor) * rG * sinO),
            y: (movementFactor * relativeTargetPosition.y) +
                ((1.0 - movementFactor) * rG * cosO))
        */
        
        /*
        UIColor.red.setStroke()
        UIColor.red.setFill()
        drawArc(relativeTargetPosition: weightedTarget);
        drawPoint(weightedTarget)
         */
    }
    
    /*
    fileprivate func drawBiArcMergePath2(
        _ relativeTargetPosition: CGPoint,
        _ relativeTargetOrientation: CGFloat)
    {
        let t2x = sin(relativeTargetOrientation)
        let t2y = cos(relativeTargetOrientation)
        let vDotT2 = abs((relativeTargetPosition.x * t2x) + (relativeTargetPosition.y * t2y))
        let mergePoint = CGPoint(
            x: relativeTargetPosition.x + (vDotT2 * t2x),
            y: relativeTargetPosition.y + (vDotT2 * t2y))
        drawBiArcMergePath(
            mergePoint,
            targetOrientation)
    }
     */

    fileprivate func directionToTarget(_ target: CGPoint) -> CGFloat
    {
        if target.y == 0 {
            if target.x == 0 {
                return 0
            }

            return (target.x < 0 ? -CGFloat.pi : CGFloat.pi) / 2
        }
        
        return atan(target.x / target.y)
    }
    
    fileprivate func drawOriginFrameOfReference(
        _ relativeTargetPosition: CGPoint)
    {
        UIColor.red.setStroke()
        let origin = CGPoint()
        let upperLeft = CGPoint(x: 0, y: relativeTargetPosition.y)
        let lowerRight = CGPoint(x: relativeTargetPosition.x, y: 0)
        drawLine(
            origin,
            upperLeft,
            ZIPPY_THIN_LINE_WIDTH)
        drawLine(
            upperLeft,
            relativeTargetPosition,
            ZIPPY_THIN_LINE_WIDTH)
        drawLine(
            relativeTargetPosition,
            lowerRight,
            ZIPPY_THIN_LINE_WIDTH)
        drawLine(
            lowerRight,
            origin,
            ZIPPY_THIN_LINE_WIDTH)
    }
    
    fileprivate func drawTargetFrameOfReference(
        _ relativeTargetPosition: CGPoint,
        _ relativeTargetOrientation: CGFloat)
    {
        UIColor.red.setStroke()
        let t2x = sin(relativeTargetOrientation)
        let t2y = cos(relativeTargetOrientation)
        let vDotT2 = (relativeTargetPosition.x * t2x) + (relativeTargetPosition.y * t2y)
        let origin = CGPoint()
        let upperLeft = CGPoint(
            x: relativeTargetPosition.x - (vDotT2 * t2x),
            y: relativeTargetPosition.y - (vDotT2 * t2y))
        let lowerRight = CGPoint(
            x: vDotT2 * t2x,
            y: vDotT2 * t2y)
        drawLine(
            origin,
            upperLeft,
            ZIPPY_THIN_LINE_WIDTH)
        drawLine(
            upperLeft,
            relativeTargetPosition,
            ZIPPY_THIN_LINE_WIDTH)
        drawLine(
            relativeTargetPosition,
            lowerRight,
            ZIPPY_THIN_LINE_WIDTH)
        drawLine(
            lowerRight,
            origin,
            ZIPPY_THIN_LINE_WIDTH)
    }

    /*
    fileprivate func drawDirectArc(
        _ fromPoint: CGPoint,
        _ toPoint: CGPoint,
        _ lineWidth: CGFloat)
    {
        let deltaPoint = CGPoint(
            x: toPoint.x - fromPoint.x,
            y: toPoint.y - fromPoint.y)
        let length2 = pow(deltaPoint.x, 2.0) + pow(deltaPoint.y, 2.0)
        let radius = length2 / (2.0 * deltaPoint.x)
        drawArc(
            fromPoint,
            radius,
            2.0 * atan(deltaPoint.x / deltaPoint.y),
            lineWidth)
    }
     */

    /*
    fileprivate func drawDirectMergePath(
        _ deltaPoint: CGPoint,
        _ towardOrientation: CGFloat)
    {
        UIColor.red.setStroke()
        UIColor.red.setFill()

        let sinTheta = sin(towardOrientation)
        let cosTheta = cos(towardOrientation)
        let intersectionPointY = deltaPoint.y - (deltaPoint.x / tan(towardOrientation))
        let mergePoint = CGPoint(
            x: intersectionPointY * sinTheta,
            y: intersectionPointY + (intersectionPointY * cosTheta))
        let intersectionVector = CGPoint(
            x: mergePoint.x - deltaPoint.x,
            y: mergePoint.y - deltaPoint.y)
        drawArrow(
            mergePoint,
            towardOrientation,
            ZIPPY_LINE_WIDTH)
        drawLine(
            deltaPoint,
            mergePoint,
            ZIPPY_LINE_WIDTH)
        let intersectionHalfPoint = CGPoint(
            x: deltaPoint.x + (intersectionVector.x / 2),
            y: deltaPoint.y + (intersectionVector.y / 2))
        let deltaHalfPoint = CGPoint(
            x: deltaPoint.x / 2,
            y: deltaPoint.y / 2)
        drawLine(deltaHalfPoint, intersectionHalfPoint, ZIPPY_THIN_LINE_WIDTH)
        drawPoint(deltaHalfPoint)
        drawPoint(intersectionHalfPoint)
        drawPoint(CGPoint(
            x: (deltaHalfPoint.x + intersectionHalfPoint.x) / 2,
            y: (deltaHalfPoint.y + intersectionHalfPoint.y) / 2))

        let distance2 = pow(mergePoint.x, 2.0) + pow(mergePoint.y, 2.0)
        let radius = distance2 / (2.0 * mergePoint.x)

        drawArc(CGPoint(
            x: radius,
            y: 0),
            -radius,
            (radius > 0 ? -CGFloat.pi : CGFloat.pi) / 2,
            towardOrientation / 2.0,
            ZIPPY_THIN_LINE_WIDTH)
    }
    */
    
    /*
    fileprivate func drawBiArcMergePath(
        _ deltaPoint: CGPoint,
        _ towardOrientation: CGFloat)
    {
        UIColor.purple.setStroke()
        UIColor.purple.setFill()

        let knot = calculateRelativeBiArcKnot(
            deltaPoint,
            towardOrientation)
        let knotDirection = 2.0 * atan(knot.x / knot.y)
        drawArc(
            CGPoint(),
            0.0,
            knot,
            ZIPPY_THIN_LINE_WIDTH)
        drawArc(
            knot,
            knotDirection,
            deltaPoint,
            ZIPPY_THIN_LINE_WIDTH)
        drawPoint(knot)
        drawArrow(
            knot,
            knotDirection,
            ZIPPY_THIN_LINE_WIDTH)
    }
     */
    
    fileprivate func drawPoint(_ center: CGPoint) {
        drawPoint(center, ZIPPY_POINT_RADIUS, true)
    }
    
    fileprivate func drawPoint(_ center: CGPoint, _ radius: CGFloat, _ fill: Bool = true) {
        let drawingPath = UIBezierPath(
            ovalIn: CGRect(
                x: center.x - radius,
                y: center.y - radius,
                width: 2 * radius,
                height: 2 * radius))
        
        if fill {
            drawingPath.fill()
        }
        else {
            drawingPath.lineWidth = ZIPPY_LINE_WIDTH
            drawingPath.stroke()
        }
    }
    
    fileprivate func drawLine(
        _ start: CGPoint,
        _ end: CGPoint,
        _ lineWidth: CGFloat)
    {
        let orientationIndicator = UIBezierPath()
        orientationIndicator.move(to: start)
        orientationIndicator.addLine(to: end)
        orientationIndicator.lineWidth = lineWidth
        orientationIndicator.stroke()
    }

    fileprivate func drawArrow(
        _ position: CGPoint,
        _ orientation: CGFloat,
        _ lineWidth: CGFloat)
    {
        let radiusSinO = ZIPPY_ARROW_RADIUS * sin(orientation)
        let radiusCosO = ZIPPY_ARROW_RADIUS * cos(orientation)
        let centerPoint = CGPoint(
            x: position.x - radiusSinO,
            y: position.y - radiusCosO)
        let orientationIndicator = UIBezierPath()
        
        orientationIndicator.move(
            to: CGPoint(
                x: centerPoint.x - radiusCosO,
                y: centerPoint.y + radiusSinO))
        orientationIndicator.addLine(to: position)
        orientationIndicator.addLine(
            to: CGPoint(
                x: centerPoint.x + radiusCosO,
                y: centerPoint.y - radiusSinO))
        orientationIndicator.lineWidth = lineWidth
        orientationIndicator.stroke()
    }
    
    fileprivate func drawArc(
        direction: CGFloat,
        distance: CGFloat)
    {
        let radius = abs(distance) / (2 * sin(direction));
        drawArc(
            centerPoint: CGPoint(x: radius, y: 0),
            radius: -radius,
            startAngle: (radius > 0 ? -CGFloat.pi : CGFloat.pi) / 2,
            deltaAngle: 2 * direction,
            lineWidth: ZIPPY_THIN_LINE_WIDTH)
    }
    
    fileprivate func drawArc(
        relativeTargetPosition: CGPoint)
    {
        drawArc(
            relativeTargetPosition: relativeTargetPosition,
            lineWidth: ZIPPY_THIN_LINE_WIDTH)
    }
    
    fileprivate func drawArc(
        relativeTargetPosition: CGPoint,
        lineWidth: CGFloat)
    {
        guard relativeTargetPosition.x != 0 else {
            return
        }
        
        let rG2 = pow(relativeTargetPosition.x, 2.0) + pow(relativeTargetPosition.y, 2.0)
        let thetaG = directionToTarget(relativeTargetPosition)

//        let rG = sqrt(rG2)
//        let radius = rG / (2.0 * sin(thetaG))
        //given:
        //  rG          = distance to point
        //  sin(thetaG) = x / rG
        //  rC          = rG / (2 * sin(thetaG))
        //then:
        //  rC          = (rG * rG) / (2 * x)
        let radius = rG2 / (2.0 * relativeTargetPosition.x)
        drawArc(
            centerPoint: CGPoint(x: radius, y: 0),
            radius: -radius,
            theta: 2 * thetaG,
            lineWidth: lineWidth)
    }
    
    /*
    fileprivate func drawArc(
        _ fromPoint: CGPoint,
        _ fromOrientation: CGFloat,
        _ toPoint: CGPoint,
        _ lineWidth: CGFloat)
    {
        let deltaPoint = CGPoint(
            x: toPoint.x - fromPoint.x,
            y: toPoint.y - fromPoint.y)
        let distance2 = pow(deltaPoint.x, 2.0) + pow(deltaPoint.y, 2.0)
        if distance2 == 0.0 {
            return
        }
        
        let orientationAngle = atan2(deltaPoint.x, deltaPoint.y)
        var deltaAngle = subtractAngles(orientationAngle, fromOrientation)
        let sinDeltaAngle = sin(deltaAngle)
        if sinDeltaAngle == 0.0 {
            return
        }
        let radius = sqrt(distance2) / (2.0 * sin(deltaAngle))
        let centerPoint = CGPoint(
            x: fromPoint.x + (radius * cos(fromOrientation)),
            y: fromPoint.y + (radius * -sin(fromOrientation)))
        let startAngle = atan2(fromPoint.x - centerPoint.x, fromPoint.y - centerPoint.y)
        if deltaAngle > CGFloat.pi/2.0 {
            deltaAngle = deltaAngle - CGFloat.pi
        }
        else if deltaAngle < -CGFloat.pi/2.0 {
            deltaAngle = deltaAngle + CGFloat.pi
        }

        drawArc(
            centerPoint,
            radius,
            startAngle,
            2.0 * deltaAngle,
            lineWidth)
    }
     */

    fileprivate func drawTurn(
        radius: CGFloat,
        theta: CGFloat)
    {
        drawArc(
            centerPoint: CGPoint(x: radius, y:0),
            radius: -radius,
            startAngle: radius > 0 ? -PI_2 : PI_2,
            deltaAngle: theta,
            lineWidth: ZIPPY_THIN_LINE_WIDTH)
    }

    fileprivate func drawArc(
        centerPoint: CGPoint,
        radius: CGFloat,
        theta: CGFloat,
        lineWidth: CGFloat)
    {
        let startAngle = centerPoint.x > 0 ? -PI_2 : PI_2
//        print(
//            centerPoint.x,
//            radius,
//            RADIANS_TO_DEGREES * startAngle,
//            RADIANS_TO_DEGREES * theta)
        drawArc(
            centerPoint: centerPoint,
            radius: radius,
            startAngle: startAngle,
            deltaAngle: theta,
            lineWidth: lineWidth)
    }

    fileprivate func drawArc(
        centerPoint: CGPoint,
        radius: CGFloat,
        startAngle: CGFloat,
        deltaAngle: CGFloat,
        lineWidth: CGFloat)
    {
        let adjustedStartAngle = CGFloat.pi/2 - startAngle
        let adjustedDeltaAngle = -deltaAngle
        
        let targetMovement = UIBezierPath()
        let endAngle = adjustedStartAngle + adjustedDeltaAngle
        targetMovement.addArc(
            withCenter: centerPoint,
            radius: abs(radius),
            startAngle: adjustedStartAngle,
            endAngle: endAngle,
            clockwise: deltaAngle < 0.0)
        targetMovement.lineWidth = lineWidth
        targetMovement.stroke()
    }

    fileprivate func calculateRelativeBiArcKnot(
        _ relativeTargetPosition: CGPoint,
        _ relativeTargetOrientation: CGFloat) -> CGPoint
    {
        if relativeTargetOrientation == 0.0 {
            return CGPoint(
                x: relativeTargetPosition.x / 2.0,
                y: relativeTargetPosition.y / 2.0)
        }
        
        let t2x = sin(relativeTargetOrientation)
        let t2y = cos(relativeTargetOrientation)
        
        //v = p2 - p1
        //v dot v = (v.x * v.x) + (v.y * v.y)
        //        = distanceToTarget ^ 2
        let vDotV = pow(relativeTargetPosition.x, 2.0) + pow(relativeTargetPosition.y, 2.0)

        //v dot t1 = (v.x * t1.x)      + (v.y * t1.y)
        //         = (v.x * sin(p1.o)) + (v.y * cos(p1.o))
        //         = (v.x * 0) + (v.y * 1)
        //         = v.y
        
        //t = t1 + t2
        //v dot t  = (v.x * t.x)                     + (v.y * t.y)
        //         = (v.x * (t1.x + t2.x))           + (v.y * (t1.y + t2.y))
        //         = (v.x * (sin(p1.o) + sin(p2.o))) + (v.y * (cos(p1.o) + cos(p2.o)))
        //         = (v.x * sin(p2.o)) + (v.y * (1.0 + cos(p2.o))
        let vDotT = (relativeTargetPosition.x * t2x) +
            (relativeTargetPosition.y * (1.0 + t2y))

        //t1 dot t2 = (t1.x * t2.x)           + (t1.y * t2.y)
        //          = (sin(p1.o) * sin(p2.o)) + (cos(p1.o) * cos(p2.o))
        //          = t2.y
        let t1DotT2Inv2 = 2.0 * (1.0 - t2y)
        let discrim = sqrt( pow(vDotT, 2.0) + ( t1DotT2Inv2 * vDotV ) )
        let d1 = (-vDotT + discrim) / t1DotT2Inv2

        //when d1 and d2 are equal...
        //knot   = ( p1 + p2 + ( d * (t1 - t2) ) ) / 2
        //knot   = ( p2 + ( d * (t1 - t2) ) ) / 2
        let knot = CGPoint(
            //knot.x = ( p2.x + ( d * (0 - t2.x) ) ) / 2
            //knot.x = ( p2.x + ( d * -t2.x ) ) / 2
            x: ( relativeTargetPosition.x + (d1 * -t2x) ) / 2.0,
            //knot.y = ( p2.y + ( d * (1 - t2.y) ) ) / 2
            y: ( relativeTargetPosition.y + (d1 * (1.0 - t2y)) ) / 2.0)
        let d2 = d1
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

        UIColor.purple.setStroke()
        let d1point = CGPoint(
            x: 0.0,
            y: d1)
        let d2point = CGPoint(
            x: relativeTargetPosition.x - (d2 * t2x),
            y: relativeTargetPosition.y - (d2 * t2y))
        drawLine(
            CGPoint(x: 0.0, y:0.0),
            d1point,
            ZIPPY_THIN_LINE_WIDTH)
        drawPoint(d1point)
        drawLine(
            relativeTargetPosition,
            d2point,
            ZIPPY_THIN_LINE_WIDTH)
        drawPoint(d2point)
        
        return knot
    }

}
