
import Foundation
import UIKit

protocol ZDrawable
{
    
    func draw(_ transform: CGAffineTransform)
    
}

class ZCompoundDrawable: ZDrawable
{

    fileprivate let drawables: [ZDrawable]
    
    init(_ drawables: ZDrawable...)
    {
        self.drawables = drawables
    }
    
    func draw(_ transform: CGAffineTransform)
    {
        for nextDrawable in drawables {
            nextDrawable.draw(transform)
        }
    }
    
}

class ZDrawableArrow: ZDrawable
{
    fileprivate let color: UIColor
    fileprivate let position: CGPoint
    fileprivate let orientation: CGFloat
    fileprivate let size: CGFloat
    
    init(_ color: UIColor, _ x: Double, _ y: Double, _ o: Double, _ s: Double)
    {
        self.color = color
        self.position = CGPoint(x: x, y: y)
        self.orientation = CGFloat(o)
        self.size = CGFloat(s)
    }
    
    func draw(_ transform: CGAffineTransform)
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
        orientationIndicator.apply(transform)
        orientationIndicator.stroke()
    }
    
}

class ZDrawableCircle: ZDrawable
{
    
    fileprivate let color: UIColor
    fileprivate let center: CGPoint
    fileprivate let radius: CGFloat
    fileprivate let fill: Bool
    
    init(_ color: UIColor, _ x: Double, _ y: Double, _ r: Double)
    {
        self.color = color
        self.center = CGPoint(x: x, y: y)
        self.radius = CGFloat(r)
        self.fill = false
    }
    
    init(_ color: UIColor, _ x: Double, _ y: Double, _ r: Double, _ fill: Bool)
    {
        self.color = color
        self.center = CGPoint(x: x, y: y)
        self.radius = CGFloat(r)
        self.fill = fill
    }
    
    func draw(_ transform: CGAffineTransform)
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
        drawingPath.apply(transform)
        
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
    
    func draw(_ transform: CGAffineTransform)
    {
        color.setStroke()

        //draw the path
        let nextPoint = KPosition()
        let drawingPath = UIBezierPath()
        for nextPath in paths {
            nextPath.interpolate(0, nextPoint)
            drawingPath.move(to: CGPoint(x: nextPoint.vector.getX(), y: nextPoint.vector.getY()))
            for n in 1...SEGMENTS_PER_PATH {
                let pointNum = Double(n) / Double(SEGMENTS_PER_PATH)
                nextPath.interpolate(pointNum, nextPoint)
                drawingPath.addLine(to: CGPoint(x: nextPoint.vector.getX(), y: nextPoint.vector.getY()))
            }
        }
        
        drawingPath.apply(transform)
        drawingPath.stroke()
    }
    
}
