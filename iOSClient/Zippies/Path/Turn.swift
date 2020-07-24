
import Foundation
import UIKit

class Turn: ZPath
{
    
    let start: ZMatrix2
//    let startO: CGFloat
    let deltaO: CGFloat
    
    init(_ start: ZMatrix2, _ deltaO: CGFloat)
    {
        self.start = start
        self.deltaO = deltaO
    }
    
    func updatesPosition() -> Bool { return false }
    func getLength() -> CGFloat { return WHEEL_OFFSET_X * abs(deltaO) }

    func getDrawable(_ color: UIColor) -> ZDrawable
    {
        return ZDrawableTurn2(color, start, deltaO)
    }
    
    func interpolate(_ t: CGFloat, _ p: ZMatrix2)
    {
        p.orientation.rotation = addAngles(start.orientation.get(), deltaO * t)
    }

}

class ZDrawableTurn: ZDrawable
{
    
    fileprivate let color: UIColor
    fileprivate let startPosition: ZVector2
    fileprivate let turn: Turn
    
    init(_ color: UIColor, _ startX: CGFloat, _ startY: CGFloat, _ turn: Turn)
    {
        self.color = color
        self.startPosition = ZVector2(startX, startY)
        self.turn = turn
    }
    
    func draw()
    {
        color.setStroke()
        let drawingPath = UIBezierPath(
            ovalIn: CGRect(
                x: startPosition.getX() - TURN_RADIUS,
                y: startPosition.getY() - TURN_RADIUS,
                width: 2 * TURN_RADIUS,
                height: 2 * TURN_RADIUS))
        drawingPath.stroke()
    }
    
}

class ZDrawableTurn2: ZDrawable
{
    
    fileprivate let color: UIColor
    fileprivate let start: ZMatrix2
    fileprivate let deltaO: CGFloat
    
    init(_ color: UIColor, _ start: ZMatrix2, _ deltaO: CGFloat)
    {
        self.color = color
        self.start = start
        self.deltaO = deltaO
    }
    
    func draw()
    {
        color.setStroke()
        let drawingPath = UIBezierPath(
            ovalIn: CGRect(
                x: start.position.getX() - TURN_RADIUS,
                y: start.position.getY() - TURN_RADIUS,
                width: 2 * TURN_RADIUS,
                height: 2 * TURN_RADIUS))
        drawingPath.stroke()
    }
    
}

