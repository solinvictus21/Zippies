
import Foundation
import UIKit

let ZIPPY_WIDTH: CGFloat = 29.4
let ZIPPY_HEIGHT: CGFloat = 24.8
let ZIPPY_HALF_WIDTH: CGFloat = ZIPPY_WIDTH / 2.0
let ZIPPY_HALF_HEIGHT: CGFloat = ZIPPY_HEIGHT / 2.0
let ZIPPY_AREA_WIDTH: CGFloat = 2000.0
let ZIPPY_AREA_HEIGHT: CGFloat = 1000.0
//let ZIPPY_AREA_HALF_WIDTH = ZIPPY_AREA_WIDTH / 2.0
//let ZIPPY_AREA_HALF_HEIGHT = ZIPPY_AREA_HEIGHT / 2.0

let TARGET_UPDATE_TIMER_INTERVAL = 100
let TARGET_DISTANCE: CGFloat = 200.0
let TARGET_POSITION_ANGULAR_INCREMENT: CGFloat = 0.01
let TARGET_ORIENTATION_INDICATOR_LENGTH: CGFloat = 40.0
let TARGET_ORIENTATION_ANGULAR_INCREMENT: CGFloat = 0.1
let PI_2: CGFloat = CGFloat.pi / 2.0

fileprivate let ZIPPY_ARROW_RADIUS: CGFloat = 20
fileprivate let ZIPPY_LINE_WIDTH: CGFloat = 3
fileprivate let ZIPPY_THIN_LINE_WIDTH: CGFloat = 0.5
fileprivate let ZIPPY_POINT_RADIUS: CGFloat = 5

class TopDownView : UIView
{
    var targetUpdateTimer: Timer?
    var targetDirection: CGFloat = 0.0
    var targetOrientation: CGFloat = 0.0

    var zippyManager: ZippyManager?
    
    fileprivate var moveToDisplay: ZDrawable

    init(s: String, i: Int)
    {
        moveToDisplay = planTestDrawable()
        super.init(frame: CGRect(x: 0, y: 0, width: 100, height: 100))
    }
    
    required init?(coder aDecoder: NSCoder)
    {
        moveToDisplay = planTestDrawable()
        super.init(coder: aDecoder)
    }
    
    override func touchesBegan(_ touches: Set<UITouch>, with event: UIEvent?)
    {
        super.touchesBegan(touches, with: event)
//        moveToDisplay = planTestDrawable()
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

    override func draw(_ rect: CGRect)
    {
        guard let context = UIGraphicsGetCurrentContext() else {
            return
        }

        let interpolationX = self.frame.width / ZIPPY_AREA_WIDTH
        let interpolationY = self.frame.height / ZIPPY_AREA_HEIGHT
        context.translateBy(x: self.frame.width/2, y: self.frame.height/2)
        context.scaleBy(x: interpolationX, y: -interpolationY)
//        context.scaleBy(x: interpolationY, y: interpolationX)
//        context.rotate(by: -CGFloat.pi/2)

        //display the move
//        moveToDisplay.draw()

        /*
        drawArc(
            CGPoint(x: 0, y: 0),
            -200,
            2.010796326794897,
            -0.16533374959479058,
            ZIPPY_LINE_WIDTH)
//         */

        let currentPoint = CGPoint(x: 0, y: 0)
        //calculate the x/y coordinates of the target position
        let target = CGPoint(
            x: TARGET_DISTANCE * sin(targetDirection),
            y: TARGET_DISTANCE * cos(targetDirection))

        //setup to display the target info
        drawDirectPath(
            currentPoint,
            target)
        /*
        drawArrow(
            target,
            targetOrientation,
            ZIPPY_LINE_WIDTH)
         */

        //setup to display the planned partial movement
        drawDirectMergePath(
            currentPoint,
            target,
            targetOrientation)

        drawBiArcMergePath(
            currentPoint,
            target,
            targetOrientation)
    }
    
    fileprivate func drawDirectPath(_ fromPoint: CGPoint, _ toPoint: CGPoint) {
        UIColor.green.setStroke()
        UIColor.green.setFill()

        //draw the direct line from our current position to the target position
        drawLine(
            fromPoint,
            toPoint,
            ZIPPY_LINE_WIDTH)

        if fromPoint.x == toPoint.x {
            return
        }
        
        //draw the arc directly from our current position to the target position
        drawDirectArc(
            fromPoint,
            toPoint,
            ZIPPY_THIN_LINE_WIDTH)
    }
    
    fileprivate func drawDirectArc(
        _ fromPoint: CGPoint,
        _ toPoint: CGPoint,
        _ lineWidth: CGFloat)
    {
        let deltaPoint = CGPoint(
            x: toPoint.x - fromPoint.x,
            y: toPoint.y - fromPoint.y)
        let length = sqrt(pow(deltaPoint.x, 2.0) + pow(deltaPoint.y, 2.0))
        let radius = length / (2.0 * sin(atan2(deltaPoint.x, deltaPoint.y)))
        drawArc(
            fromPoint,
            radius,
            2.0 * atan(deltaPoint.x / deltaPoint.y),
            lineWidth)
    }

    fileprivate func drawDirectMergePath(_ fromPoint: CGPoint, _ toPoint: CGPoint, _ towardOrientation: CGFloat) {
        if fromPoint.x == toPoint.x {
            return
        }
        
        UIColor.red.setStroke()
        UIColor.red.setFill()

        let deltaPoint = CGPoint(
            x: toPoint.x - fromPoint.x,
            y: toPoint.y - fromPoint.y)

        let sinTheta = sin(towardOrientation)
        let cosTheta = cos(towardOrientation)
        let intersectionPointY = toPoint.y - (deltaPoint.x / tan(towardOrientation))
        let mergePoint = CGPoint(
            x: fromPoint.x + (intersectionPointY * sinTheta),
            y: intersectionPointY + (intersectionPointY * cosTheta))
        drawArrow(
            mergePoint,
            towardOrientation,
            ZIPPY_LINE_WIDTH)
        drawLine(
            toPoint,
            mergePoint,
            ZIPPY_LINE_WIDTH)

        let distance = sqrt(pow(mergePoint.x, 2.0) + pow(mergePoint.y, 2.0))
        let radius = distance / (2.0 * sin(atan2(mergePoint.x, mergePoint.y)))
        /*
        let arcCenter = CGPoint(x: radius, y: 0.0)
        drawLine(
            arcCenter,
            fromPoint,
            ZIPPY_THIN_LINE_WIDTH)
        drawLine(
            arcCenter,
            toPoint,
            ZIPPY_THIN_LINE_WIDTH)
        drawLine(
            arcCenter,
            mergePoint,
            ZIPPY_THIN_LINE_WIDTH)
        drawPoint(arcCenter)
         */
        drawArc(
            fromPoint,
            radius,
            towardOrientation,
            ZIPPY_THIN_LINE_WIDTH)
    }
    
    fileprivate func drawBiArcMergePath(
        _ fromPoint: CGPoint,
        _ toPoint: CGPoint,
        _ towardOrientation: CGFloat)
    {
        let deltaPoint = CGPoint(
            x: toPoint.x - fromPoint.x,
            y: toPoint.y - fromPoint.y)
        
        UIColor.purple.setStroke()
        UIColor.purple.setFill()

//        let direction = atan(deltaPoint.x / deltaPoint.y)
        let knot = calculateRelativeBiArcKnot(
            deltaPoint,
            towardOrientation)
        let knotDirection = 2.0 * atan(knot.x / knot.y)
        /*
        drawDirectArc(
            fromPoint,
            knot,
            ZIPPY_THIN_LINE_WIDTH)
        */
        drawArc(
            fromPoint,
            0.0,
            knot,
            ZIPPY_THIN_LINE_WIDTH)
        drawArc(
            knot,
            knotDirection,
            toPoint,
            ZIPPY_THIN_LINE_WIDTH)
        drawArrow(
            knot,
            knotDirection,
            ZIPPY_THIN_LINE_WIDTH)
        /*
        drawArrow(
            toPoint,
            towardOrientation,
            ZIPPY_THIN_LINE_WIDTH)
         */
    }
    
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
    
    fileprivate func drawLine(_ start: CGPoint, _ end: CGPoint, _ lineWidth: CGFloat) {
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
        _ fromPoint: CGPoint,
        _ fromOrientation: CGFloat,
        _ toPoint: CGPoint,
        _ lineWidth: CGFloat)
    {
        let deltaPoint = CGPoint(
            x: toPoint.x - fromPoint.x,
            y: toPoint.y - fromPoint.y)
        let distance = sqrt(pow(deltaPoint.x, 2.0) + pow(deltaPoint.y, 2.0))
        if distance == 0.0 {
            return
        }
        
        let orientationAngle = atan2(deltaPoint.x, deltaPoint.y)
        var deltaAngle = subtractAngles(orientationAngle, fromOrientation)
        if deltaAngle == 0.0 {
            return
        }
        let radius = distance / (2.0 * sin(deltaAngle))
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

        /*
        drawLine(
            centerPoint,
            fromPoint,
            lineWidth)
        drawLine(
            centerPoint,
            toPoint,
            lineWidth)
        drawPoint(centerPoint)
         */
        drawArc(
            centerPoint,
            radius,
            startAngle,
            2.0 * deltaAngle,
            lineWidth)
    }

    fileprivate func drawArc(
        _ centerPoint: CGPoint,
        _ radius: CGFloat,
        _ theta: CGFloat,
        _ lineWidth: CGFloat)
    {
        drawArc(
            CGPoint(
                x: centerPoint.x + radius,
                y: centerPoint.y),
            radius,
            (radius > 0.0 ? -CGFloat.pi : CGFloat.pi) / 2.0,
            theta,
            lineWidth)
    }

    fileprivate func drawArc(
        _ centerPoint: CGPoint,
        _ radius: CGFloat,
        _ startAngle: CGFloat,
        _ deltaAngle: CGFloat,
        _ lineWidth: CGFloat)
    {
        let adjustedStartAngle = CGFloat.pi/2 - startAngle
//        let adjustedStartAngle = startAngle - CGFloat.pi/2
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

    /*
    fileprivate func constrain(_ value: CGFloat, _ minimum: CGFloat, _ maximum: CGFloat) -> CGFloat {
        return max(min(value, maximum), minimum)
    }
     */
    
    /*
    fileprivate func drawArc(_ fromPoint: CGPoint, _ toPoint: CGPoint) {
        if toPoint.y == fromPoint.y {
            return
        }
        
        let deltaY = toPoint.y - fromPoint.y
        let deltaX = toPoint.x - fromPoint.x
        let length = sqrt(pow(deltaX, 2.0) + pow(deltaY, 2.0))
        let rC = length / (2.0 * sin(atan2(deltaX, deltaY)))
//        print("arc:", rC, (2.0 * thetaG) * 180 / CGFloat.pi)
        drawArc(
            fromPoint,
            rC,
            2.0 * atan(toPoint.x / toPoint.y))
    }
 */
    
    /*
    fileprivate func drawArc(_ fromPoint: CGPoint, _ toPoint: CGPoint, _ theta: CGFloat) {
        if toPoint.x == fromPoint.x {
            drawLine(fromPoint, toPoint)
            return
        }
        
        let deltaY = toPoint.y - fromPoint.y
        let deltaX = toPoint.x - fromPoint.x
        let length = sqrt(pow(deltaX, 2.0) + pow(deltaY, 2.0))
        let rC = length / (2.0 * sin(atan2(deltaX, deltaY)))
        drawArc(
            fromPoint,
            rC,
            theta)
    }
     */
    
    /*
    fileprivate func drawArc(_ fromPoint: CGPoint, _ radius: CGFloat, _ theta: CGFloat) {
        let startAngle = ((radius > 0.0 ? -CGFloat.pi : CGFloat.pi) / 2.0) - PI_2
        let endAngle = startAngle - theta
        let targetMovement = UIBezierPath()
        targetMovement.addArc(
            withCenter: CGPoint(x: fromPoint.x + radius, y: fromPoint.y),
            radius: abs(radius),
            startAngle: startAngle,
            endAngle: endAngle,
            clockwise: theta < 0.0)
        targetMovement.lineWidth = ZIPPY_LINE_WIDTH
        targetMovement.stroke()
    }
     */
    
    /*
    fileprivate func drawArc(_ fromPoint: CGPoint, _ length: CGFloat, _ thetaG: CGFloat) {
        let deltaX = length * sin(thetaG)
        let deltaY = length * cos(thetaG)
        let rC = length / (2.0 * sin(atan2(deltaX, deltaY)))
//        print("arc:", rC, (2.0 * thetaG) * 180 / CGFloat.pi)
        drawArc(
            CGPoint(x: fromPoint.x + rC, y: fromPoint.y),
            -rC,
            (rC > 0 ? -CGFloat.pi : CGFloat.pi) / 2.0,
            2.0 * thetaG)
    }
    */

    fileprivate func calculateRelativeBiArcKnot(_ relativeTargetPosition: CGPoint,
                                                _ relativeTargetOrientation: CGFloat) -> CGPoint
    {
        if relativeTargetOrientation == 0.0 {
            return CGPoint(
                x: relativeTargetPosition.x / 2.0,
                y: relativeTargetPosition.y / 2.0)
        }
        
        let t2x = sin(relativeTargetOrientation)
        let t2y = cos(relativeTargetOrientation)
        
        //v dot v = (v.x * v.x) + (v.y * v.y)
        //        = distanceToTarget ^ 2
        //v dot t = (v.x * t.x)                               + (v.y * t.y)
        //        = ((p2.x - p1.x) * (t1.x + t2.x))           + ((p2.y - p1.y) * (t1.y + t2.y))
        //        = ((p2.x - p1.x) * (sin(p1.o) + sin(p2.o))) + ((p2.y - p1.y) * (cos(p1.o) + cos(p2.o)))
        //t1 dot t2 = (t1.x * t2.x)           + (t1.y * t2.y)
        //          = (sin(p1.o) * sin(p2.o)) + (cos(p1.o) * cos(p2.o))
        //          = t2.y
        
        //use these calculations to find the appropriate d value for the bi-arc
        let vDotT = (relativeTargetPosition.x * t2x) +
            (relativeTargetPosition.y * (1.0 + t2y))
        let vDotV = pow(relativeTargetPosition.x, 2.0) + pow(relativeTargetPosition.y, 2.0)
        
        /*
        let d = calculateBiArcD(
            d2,
            vDotT,
            t2y)
         */
        let t1DotT2Inv2 = 2.0 * (1.0 - t2y)
        let discrim = sqrt( pow(vDotT, 2.0) + ( t1DotT2Inv2 * vDotV ) )
        let d: CGFloat
        if relativeTargetPosition.y > 0.0 {
//        if abs(subtractAngles(relativeTargetOrientation, 2.0 * atan(relativeTargetPosition.x / relativeTargetPosition.y))) < CGFloat.pi {
            d = (-vDotT + discrim) / t1DotT2Inv2
        }
        else {
            d = (-vDotT - discrim) / t1DotT2Inv2
        }
        
        //now we can use the d value to calculation the position of the "knot", which is the intersection
        //of the two arcs which make up the bi-arc
        return CGPoint(
            x: ( relativeTargetPosition.x + (d * (-t2x)) ) / 2.0,
            y: ( relativeTargetPosition.y + (d * (1.0 - t2y)) ) / 2.0)
    }

    fileprivate func calculateBiArcD(_ vDotV: CGFloat,
                                     _ vDotT: CGFloat,
                                     _ t1DotT2: CGFloat) -> CGFloat
    {
        //precalc = 2 * (1 - (t1 dot t2))
        let t1DotT2Inv2 = 2.0 * (1.0 - t1DotT2)
        let discrim = sqrt( pow(vDotT, 2.0) + ( t1DotT2Inv2 * vDotV ) )
        
        //now find the smallest d value of the bi-arc to create the shortest bi-arc to the target
        let d = -vDotT + discrim
//        let altD = -vDotT - discrim
//        if abs(d) > abs(altD) {
//            return altD / t1DotT2Inv2
//        }
        return d / t1DotT2Inv2
    }


}
