
import Foundation
import UIKit

let TURN_RADIUS: CGFloat = 30.0

protocol ZDrawable
{
    
    func draw()
    
}

class ZCompoundDrawable: ZDrawable
{

    fileprivate let drawables: [ZDrawable]
    
    init(_ drawables: ZDrawable...)
    {
        self.drawables = drawables
    }
    
    func draw()
    {
        for nextDrawable in drawables {
            nextDrawable.draw()
        }
    }
    
}

class ZDrawableLine: ZDrawable
{
    fileprivate let color: UIColor
    fileprivate let start: CGPoint
    fileprivate let end: CGPoint
    
    init(
        _ color: UIColor,
        _ startX: CGFloat,
        _ startY: CGFloat,
        _ endX: CGFloat,
        _ endY: CGFloat)
    {
        self.color = color
        self.start = CGPoint(x: startX, y: startY)
        self.end = CGPoint(x: endX, y: endY)
    }
    
    convenience init(
        _ color: UIColor,
        _ start: ZVector2,
        _ endX: CGFloat,
        _ endY: CGFloat)
    {
        self.init(color, start.getX(), start.getY(), endX, endY)
    }
    
    convenience init(
        _ color: UIColor,
        _ startX: CGFloat,
        _ startY: CGFloat,
        _ end: ZVector2)
    {
        self.init(color, startX, startY, end.getX(), end.getY())
    }
    
    convenience init(
        _ color: UIColor,
        _ start: ZVector2,
        _ end: ZVector2)
    {
        self.init(color, start.getX(), start.getY(), end.getX(), end.getY())
    }
    
    func draw()
    {
        color.setStroke()
        let orientationIndicator = UIBezierPath()
        orientationIndicator.move(to: start)
        orientationIndicator.addLine(to: end)
        orientationIndicator.stroke()
    }
    
}

class ZDrawableArrow: ZDrawable
{
    fileprivate let color: UIColor
    fileprivate let position: CGPoint
    fileprivate let orientation: CGFloat
    fileprivate let size: CGFloat
    
    init(_ color: UIColor, _ x: CGFloat, _ y: CGFloat, _ o: CGFloat, _ s: CGFloat)
    {
        self.color = color
        self.position = CGPoint(x: x, y: y)
        self.orientation = CGFloat(o)
        self.size = CGFloat(s)
    }
    
    init(_ color: UIColor, _ p: ZMatrix2, _ s: CGFloat)
    {
        self.color = color
        self.position = CGPoint(x: p.position.getX(), y: p.position.getY())
        self.orientation = CGFloat(p.orientation.rotation)
        self.size = CGFloat(s)
    }
    
    func draw()
    {
        color.setStroke()
        let radiusSinO = size * sin(self.orientation)
        let radiusCosO = size * cos(self.orientation)
        let centerPoint = CGPoint(
            x: self.position.x - radiusSinO,
            y: self.position.y - radiusCosO)
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
        orientationIndicator.stroke()
    }
    
}

class ZDrawableCircle: ZDrawable
{
    
    fileprivate let color: UIColor
    fileprivate let center: CGPoint
    fileprivate let radius: CGFloat
    fileprivate let fill: Bool
    
    init(_ color: UIColor, _ x: CGFloat, _ y: CGFloat, _ r: CGFloat, _ fill: Bool)
    {
        self.color = color
        self.center = CGPoint(x: x, y: y)
        self.radius = CGFloat(r)
        self.fill = fill
    }
    
    convenience init(_ color: UIColor, _ x: CGFloat, _ y: CGFloat, _ r: CGFloat)
    {
        self.init(color, x, y, r, false)
    }
    
    convenience init(_ color: UIColor, _ center: ZVector2, _ radius: CGFloat)
    {
        self.init(color, center.getX(), center.getY(), radius, false)
    }
    
    convenience init(_ color: UIColor, _ center: ZVector2, _ radius: CGFloat, _ fill: Bool)
    {
        self.init(color, center.getX(), center.getY(), radius, fill)
    }
    
    func draw()
    {
        let drawingPath = UIBezierPath(
            ovalIn: CGRect(
                x: center.x - radius,
                y: center.y - radius,
                width: 2 * radius,
                height: 2 * radius))
            /*
            UIBezierPath(
            arcCenter: center,
            radius: radius,
            startAngle: 0,
            endAngle: 0,
            clockwise: true)
            */
        
        if fill {
            color.setFill()
            drawingPath.fill()
        }
        else {
            color.setStroke()
            drawingPath.stroke()
        }
    }
    
}

class ZDrawablePath: ZDrawable
{
    
    fileprivate let color: UIColor
    fileprivate let paths: [ZPath]
    
    init(_ color: UIColor, _ paths: ZPath...)
    {
        self.color = color
        self.paths = paths
    }
    
    init(_ color: UIColor, _ paths: [ZPath])
    {
        self.color = color
        self.paths = paths
    }
    
    func draw()
    {
        color.setStroke()

        //draw the path
        let nextPoint = ZMatrix2()
        let drawingPath = UIBezierPath()
        for nextPath in paths {
            nextPath.interpolate(0, nextPoint)
            drawingPath.move(to: CGPoint(x: nextPoint.position.getX(), y: nextPoint.position.getY()))
            for n in 1...SEGMENTS_PER_PATH {
                let pointNum = CGFloat(n) / CGFloat(SEGMENTS_PER_PATH)
                nextPath.interpolate(pointNum, nextPoint)
                drawingPath.addLine(to: CGPoint(x: nextPoint.position.getX(), y: nextPoint.position.getY()))
            }
        }
        drawingPath.stroke()
    }
    
}
