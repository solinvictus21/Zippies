
import Foundation
import UIKit

fileprivate let ZIPPY_ARROW_RADIUS_DEFAULT: Double = 20
fileprivate let ZIPPY_LINE_WIDTH_DEFAULT: Double = 0.8
fileprivate let ZIPPY_LINE_WIDTH_THICK: Double = 3
fileprivate let ZIPPY_POINT_RADIUS_DEFAULT: Double = 5
//fileprivate let ZIPPY_WHEEL_RADIUS: CGFloat = 18.27840255602223

func drawPoint(
    center: ZVector2,
    radius: Double = ZIPPY_POINT_RADIUS_DEFAULT,
    lineWidth: Double = ZIPPY_LINE_WIDTH_DEFAULT)
{
    let drawingPath = UIBezierPath(
        ovalIn: CGRect(
            x: center.getX() - radius,
            y: center.getY() - radius,
            width: 2 * radius,
            height: 2 * radius))
    drawingPath.lineWidth = CGFloat(lineWidth)
    drawingPath.fill()
    drawingPath.stroke()
}

func drawLine(
    start: ZVector2,
    end: ZVector2,
    lineWidth: Double = ZIPPY_LINE_WIDTH_DEFAULT)
{
    let orientationIndicator = UIBezierPath()
    orientationIndicator.move(to:
        CGPoint(
            x: start.getX(),
            y: start.getY()))
    orientationIndicator.addLine(to:
        CGPoint(
            x: end.getX(),
            y: end.getY()))
    orientationIndicator.lineWidth = CGFloat(lineWidth)
    orientationIndicator.stroke()
}

func drawArrow(
    position: ZVector2,
    orientation: Double,
    lineWidth: Double = ZIPPY_LINE_WIDTH_DEFAULT,
    arrowRadius: Double = ZIPPY_ARROW_RADIUS_DEFAULT)
{
    let radiusSinO = arrowRadius * sin(orientation)
    let radiusCosO = arrowRadius * cos(orientation)
    let centerPoint = CGPoint(
        x: CGFloat(position.getX() - radiusSinO),
        y: CGFloat(position.getY() - radiusCosO))
    let orientationIndicator = UIBezierPath()
    
    orientationIndicator.move(
        to: CGPoint(
            x: centerPoint.x - CGFloat(radiusCosO),
            y: centerPoint.y + CGFloat(radiusSinO)))
    orientationIndicator.addLine(
        to: CGPoint(
            x: position.getX(),
            y: position.getY()))
    orientationIndicator.addLine(
        to: CGPoint(
            x: centerPoint.x + CGFloat(radiusCosO),
            y: centerPoint.y - CGFloat(radiusSinO)))
    orientationIndicator.lineWidth = CGFloat(lineWidth)
    orientationIndicator.stroke()
}

func drawArc(
    direction: Double,
    distance: Double)
{
    let radius = abs(distance) / (2 * sin(direction))
    drawArc(
        centerPoint: ZVector2(Double(radius), 0),
        radius: -radius,
        startAngle: (radius > 0 ? -Double.pi : Double.pi) / 2,
        deltaAngle: 2.0 * direction,
        lineWidth: ZIPPY_LINE_WIDTH_DEFAULT)
}

func drawArc(
    relativeTargetPosition: ZVector2,
    lineWidth: Double = ZIPPY_LINE_WIDTH_DEFAULT)
{
    guard relativeTargetPosition.getX() != 0 else {
        return
    }
    
    let rG2 = relativeTargetPosition.getD2()
    let thetaG = relativeTargetPosition.atan

//        let rG = sqrt(rG2)
//        let radius = rG / (2.0 * sin(thetaG))
    //given:
    //  rG          = distance to point
    //  sin(thetaG) = x / rG
    //  rC          = rG / (2 * sin(thetaG))
    //then:
    //  rC          = (rG * rG) / (2 * x)
    let radius = rG2 / (2.0 * relativeTargetPosition.getX())
    drawArc(
        centerPoint: ZVector2(radius, 0),
        radius: -radius,
        theta: 2.0 * thetaG,
        lineWidth: lineWidth)
}

func drawArc(
    fromPoint: ZVector2,
    fromOrientation: Double,
    toPoint: ZVector2,
    lineWidth: Double = ZIPPY_LINE_WIDTH_DEFAULT)
{
    let deltaPoint = CGPoint(
        x: toPoint.getX() - fromPoint.getX(),
        y: toPoint.getY() - fromPoint.getY())
    let distance2 = Double(pow(deltaPoint.x, 2.0) + pow(deltaPoint.y, 2.0))
    if distance2 == 0.0 {
        return
    }
    
    let orientationAngle = Double(atan2(deltaPoint.x, deltaPoint.y))
    var deltaAngle = subtractAngles(orientationAngle, fromOrientation)
    let sinDeltaAngle = sin(deltaAngle)
    if sinDeltaAngle == 0.0 {
        return
    }
    let radius = sqrt(distance2) / (2.0 * sin(deltaAngle))
    let centerPoint = ZVector2(
        fromPoint.getX() + (radius * cos(fromOrientation)),
        fromPoint.getY() + (radius * -sin(fromOrientation)))
    let startAngle = atan2(fromPoint.getX() - centerPoint.getX(), fromPoint.getY() - centerPoint.getY())
    if deltaAngle > Double.pi/2.0 {
        deltaAngle = deltaAngle - Double.pi
    }
    else if deltaAngle < -Double.pi/2.0 {
        deltaAngle = deltaAngle + Double.pi
    }

    drawArc(
        centerPoint: centerPoint,
        radius: radius,
        startAngle: startAngle,
        deltaAngle: 2.0 * deltaAngle,
        lineWidth: lineWidth)
}

func drawArc(
    _ linearVelocity: Double,
    _ angularVelocity: Double)
{
    let radius = linearVelocity / angularVelocity
    drawArc(
        centerPoint: ZVector2(radius, 0.0),
        radius: -radius,
        theta: angularVelocity)
}

func drawArc(
    centerPoint: ZVector2,
    radius: Double,
    theta: Double,
    lineWidth: Double = ZIPPY_LINE_WIDTH_DEFAULT)
{
    let startAngle = centerPoint.getX() > 0 ? -PI_2 : PI_2
    drawArc(
        centerPoint: centerPoint,
        radius: radius,
        startAngle: startAngle,
        deltaAngle: theta,
        lineWidth: lineWidth)
}

func drawArc(
    centerPoint: ZVector2,
    radius: Double,
    startAngle: Double,
    deltaAngle: Double,
    lineWidth: Double = ZIPPY_LINE_WIDTH_DEFAULT)
{
    let adjustedStartAngle = Double.pi/2 - startAngle
    let adjustedDeltaAngle = -deltaAngle
    let endAngle = adjustedStartAngle + adjustedDeltaAngle

    let targetMovement = UIBezierPath()
    targetMovement.addArc(
        withCenter: CGPoint(
            x: centerPoint.getX(),
            y: centerPoint.getY()),
        radius: CGFloat(abs(radius)),
        startAngle: CGFloat(adjustedStartAngle),
        endAngle: CGFloat(endAngle),
        clockwise: deltaAngle < 0.0)
    targetMovement.lineWidth = CGFloat(lineWidth)
//    targetMovement.fill()
    targetMovement.stroke()
}

func drawBiArc(
    biArc: ZBiArc)
{
    let d1EndVector = ZVector2(
        biArc.startPosition.getX() + (biArc.d1 * sin(biArc.startOrientation)),
        biArc.startPosition.getY() + (biArc.d1 * cos(biArc.startOrientation)))
    let d2EndVector = ZVector2(
        biArc.targetPosition.getX() - (biArc.d2 * biArc.targetVelocity.sin),
        biArc.targetPosition.getY() - (biArc.d2 * biArc.targetVelocity.cos))

    UIColor.green.setStroke()
    UIColor.green.setFill()
    drawArc(
        fromPoint: biArc.startPosition,
        fromOrientation: biArc.startOrientation,
        toPoint: biArc.knotPosition)
    drawLine(
        start: biArc.startPosition,
        end: d1EndVector)
//    drawLine(
//        start: d1EndVector,
//        end: biArc.knotPosition)
    drawPoint(center: d1EndVector)
    UIColor.orange.setStroke()
    UIColor.orange.setFill()
    drawPoint(center: biArc.knotPosition)
    drawArrow(
        position: biArc.knotPosition,
        orientation: biArc.knotOrientation)
    UIColor.red.setStroke()
    UIColor.red.setFill()
    drawArc(
        fromPoint: biArc.knotPosition,
        fromOrientation: biArc.knotOrientation,
        toPoint: biArc.targetPosition)
    drawLine(
        start: biArc.targetPosition,
        end: d2EndVector)
//    drawLine(
//        start: biArc.knotPosition,
//        end: d2EndVector)
    drawPoint(center: d2EndVector)
}

func drawCircle(
    center: ZVector2,
    radius: Double,
    lineWidth: Double = ZIPPY_LINE_WIDTH_DEFAULT)
{
    let circlePath = UIBezierPath(
        arcCenter: CGPoint(x: center.getX(), y: center.getY()),
        radius: CGFloat(radius),
        startAngle: 0,
        endAngle: CGFloat(_2PI),
        clockwise: true)
    circlePath.lineWidth = CGFloat(lineWidth)
    circlePath.stroke()
}
