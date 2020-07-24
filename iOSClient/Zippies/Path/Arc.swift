
import Foundation
import UIKit

class Arc: ZPath
{
    
    let center: ZMatrix2
    let radius: CGFloat
    let startOrientation: CGFloat
    let deltaAngle: CGFloat
    let arcLength: CGFloat
    let reverseMotion: Bool
    
    init(
        _ startPosition: ZMatrix2,
        _ arcLength: CGFloat,
        _ subtendedAngle: CGFloat)
    {
        self.startOrientation = startPosition.orientation.rotation
        self.arcLength = abs(arcLength)
        self.deltaAngle = subtendedAngle

        self.radius = arcLength / subtendedAngle
        self.center = ZMatrix2(radius, 0.0, -CGFloat.pi / 2.0)
        self.center.concat(startPosition)
        self.reverseMotion = subtendedAngle < 0.0
    }
    
    init(
        _ startPosition: ZMatrix2,
        _ relativeTarget: ZMatrix2)
    {
        self.startOrientation = startPosition.orientation.get()
        self.radius = relativeTarget.position.getD() / (2.0 * sin(relativeTarget.position.atan2))
        self.deltaAngle = 2.0 * relativeTarget.position.atan
        self.arcLength = radius * deltaAngle
        self.center = ZMatrix2(radius, 0.0, -CGFloat.pi / 2.0)
        self.center.concat(startPosition)
        self.reverseMotion = (self.radius * self.deltaAngle) < 0.0
    }
    
    func updatesPosition() -> Bool { return true }
    func getLength() -> CGFloat { return (abs(radius) + WHEEL_OFFSET_X) * abs(deltaAngle) }
    func getDrawable(_ color: UIColor) -> ZDrawable {
        return ZDrawablePath(color, self)
    }

    func interpolate(_ t: CGFloat, _ p: ZMatrix2)
    {
        let currentAngle = deltaAngle * t
        let angleOnArc = addAngles(center.orientation.rotation, currentAngle)
        p.position.set(center.position.getX() + (radius * sin(angleOnArc)),
                       center.position.getY() + (radius * cos(angleOnArc)))
        p.orientation.rotation = addAngles(startOrientation, currentAngle)
    }
    
}
