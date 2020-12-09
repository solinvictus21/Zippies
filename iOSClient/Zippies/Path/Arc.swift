
import Foundation

class Arc: ZPath
{
    
    let center: ZMatrix2
    let radius: Double
    let startOrientation: Double
    let deltaAngle: Double
    let arcLength: Double
    let reverseMotion: Bool
    
    init(
        _ startPosition: ZMatrix2,
        _ arcLength: Double,
        _ subtendedAngle: Double)
    {
        self.startOrientation = startPosition.orientation.rotation
        self.arcLength = abs(arcLength)
        self.deltaAngle = subtendedAngle

        self.radius = arcLength / subtendedAngle
        self.center = ZMatrix2(radius, 0.0, -Double.pi / 2.0)
        self.center.concatTo(startPosition)
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
        self.center = ZMatrix2(radius, 0.0, -Double.pi / 2.0)
        self.center.concatTo(startPosition)
        self.reverseMotion = (self.radius * self.deltaAngle) < 0.0
    }
    
    func updatesPosition() -> Bool { return true }
    func getLength() -> Double { return (abs(radius) + WHEEL_OFFSET_X) * abs(deltaAngle) }
    /*
    func getDrawable(_ color: UIColor) -> ZDrawable {
        return ZDrawablePath(color, self)
    }
     */

    func interpolate(_ t: Double, _ p: ZMatrix2)
    {
        let currentAngle = deltaAngle * t
        let angleOnArc = addAngles(center.orientation.rotation, currentAngle)
        p.position.set(center.position.getX() + (radius * sin(angleOnArc)),
                       center.position.getY() + (radius * cos(angleOnArc)))
        p.orientation.rotation = addAngles(startOrientation, currentAngle)
    }
    
}
