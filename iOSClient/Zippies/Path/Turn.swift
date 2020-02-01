
import Foundation
import UIKit

class Turn: ZPath
{
    
    let start: KMatrix2
//    let startO: Double
    let deltaO: Double
    
    init(_ start: KMatrix2, _ deltaO: Double)
    {
        self.start = start
        self.deltaO = deltaO
    }
    
    func updatesPosition() -> Bool { return false }
    func getLength() -> Double { return WHEEL_OFFSET_X * abs(deltaO) }

    func getDrawable(_ color: UIColor) -> ZDrawable
    {
        return ZDrawableTurn2(color, start, deltaO)
    }
    
    func interpolate(_ t: Double, _ p: KMatrix2)
    {
        p.orientation.rotation = addAngles(start.orientation.get(), deltaO * t)
    }

}

class ZDrawableTurn: ZDrawable
{
    
    fileprivate let color: UIColor
    fileprivate let startPosition: KVector2
    fileprivate let turn: Turn
    
    init(_ color: UIColor, _ startX: Double, _ startY: Double, _ turn: Turn)
    {
        self.color = color
        self.startPosition = KVector2(startX, startY)
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
    fileprivate let start: KMatrix2
    fileprivate let deltaO: Double
    
    init(_ color: UIColor, _ start: KMatrix2, _ deltaO: Double)
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

