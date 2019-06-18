
import Foundation
import UIKit

let ZIPPY_WIDTH: CGFloat = 29.4
let ZIPPY_HEIGHT: CGFloat = 24.8
let ZIPPY_HALF_WIDTH: CGFloat = ZIPPY_WIDTH / 2.0
let ZIPPY_HALF_HEIGHT: CGFloat = ZIPPY_HEIGHT / 2.0
let ZIPPY_AREA_WIDTH: CGFloat = 2000.0
let ZIPPY_AREA_HEIGHT: CGFloat = 1000.0
let ZIPPY_ORIENTATION_LENGTH: CGFloat = 50
let ZIPPY_POINT_INDICATOR_RADIUS: CGFloat = 10
//let ZIPPY_AREA_HALF_WIDTH = ZIPPY_AREA_WIDTH / 2.0
//let ZIPPY_AREA_HALF_HEIGHT = ZIPPY_AREA_HEIGHT / 2.0

class TopDownView : UIView
{
    
    var zippyManager: ZippyManager?
    
    fileprivate var moveToDisplay: BiArc
    
    init(s: String, i: Int)
    {
        moveToDisplay = initBiArc()
        super.init(frame: CGRect(x: 0, y: 0, width: 100, height: 100))
    }
    
    required init?(coder aDecoder: NSCoder)
    {
        moveToDisplay = initBiArc()
        super.init(coder: aDecoder)
    }

    override func touchesBegan(_ touches: Set<UITouch>, with event: UIEvent?)
    {
        super.touchesBegan(touches, with: event)
        moveToDisplay = initBiArc()
        setNeedsDisplay()
    }

    override func draw(_ rect: CGRect)
    {
        //calculate the appropriate transform to turn Zippy coordinates into display coordinates
        let interpolationX = self.frame.width / ZIPPY_AREA_WIDTH
        let interpolationY = self.frame.height / ZIPPY_AREA_HEIGHT
        let transform = CGAffineTransform(translationX: self.frame.width / 2.0, y: self.frame.height / 2.0)
            .scaledBy(x: interpolationX, y: -interpolationY)
//        let transform = CGAffineTransform(rotationAngle: CGFloat.pi)
//            .translatedBy(x: -self.frame.width / 2.0, y: -self.frame.height / 2.0)
//            .scaledBy(x: -interpolationX, y: interpolationY)

        /*
        UIColor.blue.setFill()
        for (_, zippy) in zippyManager!.zippies
        {
            var icon = UIBezierPath(rect: CGRect(x: -ZIPPY_HALF_WIDTH,
                                                 y: -ZIPPY_HALF_HEIGHT,
                                                 width: ZIPPY_WIDTH,
                                                 height: ZIPPY_HEIGHT))
            icon.apply(transform)
                .rotated(by: CGFloat(zippy.orientation)))
            icon.fill()
        }
        */
        
        //display the move
        UIColor.green.setStroke()
        //draw the orientation indicator
        drawOrientation(
            from: transform,
            position: moveToDisplay.startingPosition.vector,
            orientation: moveToDisplay.startingPosition.orientation)
        drawArc(
            from: transform,
            arc: moveToDisplay.arc1!)
        /*
        drawArc(
            from: transform,
            center: moveToDisplay.arc1!.center,
            radius: moveToDisplay.arc1!.radius,
            startAngle: moveToDisplay.arc1!.startAngle,
            deltaAngle: moveToDisplay.arc1!.deltaAngle)
        */
        UIColor.red.setStroke();
        drawOrientation(
            from: transform,
            position: moveToDisplay.endingPosition.vector,
            orientation: moveToDisplay.endingPosition.orientation)
        drawArc(
            from: transform,
            arc: moveToDisplay.arc2!)
        /*
        drawArc(
            from: transform,
            center: moveToDisplay.arc2!.center,
            radius: moveToDisplay.arc2!.radius,
            startAngle: moveToDisplay.arc2!.startAngle,
            deltaAngle: moveToDisplay.arc2!.deltaAngle)
        */
    }
    
    func drawArc(from: CGAffineTransform, arc: Arc)
    {
        let numPoints = 100
        let nextPoint = KVector2()
        arc.interpolate(t: 0.0, v: nextPoint)
        let path = UIBezierPath()
        path.move(to: CGPoint(x: nextPoint.getX(), y: nextPoint.getY()))
        for n in 1...numPoints {
            let pointNum = Double(n) / Double(numPoints)
            arc.interpolate(t: pointNum, v: nextPoint)
            path.addLine(to: CGPoint(x: nextPoint.getX(), y: nextPoint.getY()))
        }
        path.apply(from)
        path.stroke()
    }
    
    func drawOrientation(from: CGAffineTransform, position: KVector2, orientation: Double)
    {
        let centerPoint = CGPoint(x: position.getX(), y: position.getY())
        let orientationIndicator = UIBezierPath()
        orientationIndicator.addArc(
            withCenter: centerPoint,
            radius: ZIPPY_POINT_INDICATOR_RADIUS,
            startAngle: 0,
            endAngle: CGFloat.pi * 2,
            clockwise: true)
        let sinO = sin(CGFloat(orientation))
        let cosO = cos(CGFloat(orientation))
        orientationIndicator.move(
            to: CGPoint(
                x: centerPoint.x + (ZIPPY_POINT_INDICATOR_RADIUS * sinO),
                y: centerPoint.y + (ZIPPY_POINT_INDICATOR_RADIUS * cosO)))
        orientationIndicator.addLine(
            to: CGPoint(
                x: centerPoint.x + (ZIPPY_ORIENTATION_LENGTH * sinO),
                y: centerPoint.y + (ZIPPY_ORIENTATION_LENGTH * cosO)))
        orientationIndicator.apply(from)
        orientationIndicator.stroke()
    }
    
    func drawArc(from: CGAffineTransform, center: KVector2, radius: Double, startAngle: Double, deltaAngle: Double)
    {
//        let correctedStartAngle = -subtractAngles(a1: startAngle, a2: Double.pi / 2)
//        let correctedStartAngle = -startAngle - (Double.pi / 2)
//        let correctedStartAngle = addAngles(a1: -startAngle, a2: Double.pi / 2)
        let start = translateAngle(startAngle)
        let end = translateAngle(addAngles(a1: startAngle, a2: deltaAngle))
        /*
        if (deltaAngle < 0) != (end >= start) {
            let tmp = start
            start = end
            end = tmp
        }
         */
        print("start:", startAngle, " end:", addAngles(a1: startAngle, a2: deltaAngle), " corrected:", start)
        let arc = UIBezierPath(
            arcCenter: CGPoint(x: CGFloat(center.getX()),
                               y: CGFloat(center.getY())),
            radius: CGFloat(radius),
            startAngle: start,
            endAngle: end,
//            clockwise: end > start)
            clockwise: deltaAngle < 0)
        arc.apply(from)
        arc.stroke()
    }
    
    func translateAngle(_ a: Double) -> CGFloat
    {
        guard a < (Double.pi / 2) else {
            return CGFloat((2.5 * Double.pi) - a)
        }
        return CGFloat((Double.pi / 2) - a)
    }
    
}

fileprivate func initBiArc() -> BiArc
{
    //        let moveToDisplay = BiArc(x: 0, y: 0, o: 0)
    //        moveToDisplay.start(KPosition(x: 200, y: 150, o: -3))
    let moveToDisplay = BiArc(x: Double.random(in: -400...400),
                          y: Double.random(in: -400...400),
                          o: Double.random(in: -Double.pi..<Double.pi))
    moveToDisplay.start(KPosition(x: Double.random(in: -400...400),
                                  y: Double.random(in: -400...400),
                                  o: Double.random(in: -Double.pi..<Double.pi)))
    return moveToDisplay
}

