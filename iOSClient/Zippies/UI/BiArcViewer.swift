
import Foundation
import UIKit

fileprivate let ZIPPY_RANGE_X: CGFloat = 600
fileprivate let ZIPPY_RANGE_Y: CGFloat = 400
fileprivate let ZIPPY_ARROW_RADIUS: CGFloat = 10

class BiArcViewer: ZDrawable
{
    
    fileprivate var _start: ZMatrix2
    fileprivate var _end: ZMatrix2

    fileprivate var vDotV: CGFloat = 0
    fileprivate var vDotT1: CGFloat = 0
    fileprivate var vDotT2: CGFloat = 0
    fileprivate var t1DotT2: CGFloat = 0
    fileprivate var _d1: CGFloat = 0
    fileprivate var _d2: CGFloat = 0
    
    fileprivate var path: ZPath? = nil
    
    init()
    {
        _start = ZMatrix2(
            CGFloat.random(in: -ZIPPY_RANGE_X...ZIPPY_RANGE_X),
            CGFloat.random(in: -ZIPPY_RANGE_Y...ZIPPY_RANGE_Y),
            CGFloat.random(in: -CGFloat.pi..<CGFloat.pi))
        _end = generateRandomTarget(_start)
        recalculate()
    }
    
    func setStartPosition(_ x: CGFloat, _ y: CGFloat)
    {
        _start.position.set(x, y)
        recalculate()
    }
    
    func setStartOrientation(_ o: CGFloat)
    {
        _start.orientation.rotation = o
        recalculate()
    }
    
    func setEndPosition(_ x: CGFloat, _ y: CGFloat)
    {
        _end.position.set(x, y)
        recalculate()
    }
    
    func setEndOrientation(_ o: CGFloat)
    {
        _end.orientation.rotation = o
        recalculate()
    }
    
    func draw()
    {
        UIColor.green.setStroke()
        drawArrow(_start)
        UIColor.red.setStroke()
        drawArrow(_start)
    }
    
    fileprivate func drawArrow(_ p: ZMatrix2)
    {
        let radiusSinO = ZIPPY_ARROW_RADIUS * p.orientation.sin
        let centerPointX = p.position.getX() - radiusSinO
        let radiusCosO = ZIPPY_ARROW_RADIUS * p.orientation.cos
        let centerPointY = p.position.getY() - radiusCosO

        let orientationIndicator = UIBezierPath()
        orientationIndicator.move(
            to: CGPoint(
                x: centerPointX - radiusCosO,
                y: centerPointY + radiusSinO))
        orientationIndicator.addLine(
            to: CGPoint(
                x: p.position.getX(),
                y: p.position.getY()))
        orientationIndicator.addLine(
            to: CGPoint(
                x: centerPointX + radiusCosO,
                y: centerPointY - radiusSinO))
        orientationIndicator.stroke()
    }

    fileprivate func recalculate()
    {
        let deltaX = _end.position.getX() - _start.position.getX()
        let deltaY = _end.position.getY() - _start.position.getY()
        vDotV = (deltaX * deltaX) + (deltaY * deltaY)

        let t1x = sin(_start.orientation.rotation)
        let t1y = cos(_start.orientation.rotation)
        vDotT1 = (deltaX * t1x) + (deltaY * t1y)

        let t2x = sin(_end.orientation.rotation)
        let t2y = cos(_end.orientation.rotation)
        vDotT2 = (deltaX * t2x) + (deltaY * t2y)

        t1DotT2 = (t1x * t2x) + (t1y * t2y)
        
        calculateD()
    }
    
    fileprivate func calculateD()
    {
        if t1DotT2 == 0.0 {
            let deltaAngle = subtractAngles(_end.orientation.rotation, _start.orientation.rotation)
            _d1 = cos(deltaAngle)
        }
        else {
            //precalc = 2 * (1 - (t1 dot t2))
            let t1DotT2Inv2 = 2.0 * (1.0 - t1DotT2)
            let discrim = sqrt(pow(vDotT1, 2.0) + (t1DotT2Inv2 * vDotV))
            
            //now find the smallest d value of the bi-arc to create the shortest bi-arc to the target
            let d = -vDotT1 + discrim
            let altD = -vDotT1 - discrim
            if abs(d) > abs(altD) {
                _d1 = altD / t1DotT2Inv2
            }
            else {
                _d1 = d / t1DotT2Inv2
            }
            _d2 = _d1
        }
    }

    fileprivate func recalculateD1()
    {
        _d1 = ((vDotV/2.0) - (_d2*vDotT2)) / (vDotT1 - ((_d1*t1DotT2) - 1.0))
    }
    
    fileprivate func recalculateD2()
    {
        _d2 = ((vDotV/2.0) - (_d1*vDotT1)) / (vDotT2 - ((_d1*t1DotT2) - 1.0))
    }
    
    func planPath(_ fromPosition: ZMatrix2, _ toPosition: ZMatrix2) -> ZPath?
    {
        /*
        let knot: KMatrix2
        if fromPosition.orientation.rotation == toPosition.orientation.rotation {
            knot = KMatrix2(
                (toPosition.position.getX() + fromPosition.position.getX()) / 2.0,
                (toPosition.position.getY() + fromPosition.position.getY()) / 2.0,
                addAngles(toPosition.orientation.rotation / 2.0, fromPosition.orientation.rotation) / 2.0)
        }
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
        */
        return nil
    }


}
