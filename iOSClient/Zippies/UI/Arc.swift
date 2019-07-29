
import Foundation
import UIKit

class Arc: ZPath
{
    
    let center: KMatrix2
    let radius: Double
    let startOrientation: Double
    let deltaAngle: Double
    let arcLength: Double
    let reverseDirection: Bool
    
    /*
//    init(_ centerX: Double, _ centerY: Double, _ radius: Double,
//         _ startO: Double, _ startAngle: Double, _ deltaAngle: Double)
    init(_ startX: Double, _ startY: Double, _ startO: Double,
         _ endX: Double, _ endY: Double, _ reverseDirection: Bool)
    {
        let deltaX = endX - startX
        let deltaY = endY - startY
        let tangentX = sin(startO)
        let tangentY = cos(startO)
        let deltaDotDelta = (deltaX * deltaX) + (deltaY * deltaY)
        let n2DotDelta = ((2.0 * -tangentY) * deltaX) + ((2.0 * tangentX) * deltaY)
        let s = deltaDotDelta / n2DotDelta
        let centerX = startX + (s * -tangentY)
        let centerY = startY + (s * tangentX)
        self.center = KMatrix2(centerX,
                               centerY,
                               atan2(startX - centerX, startY - centerY))
        self.radius = abs(s)
        self.startOrientation = startO
        let endAngle = atan2((startX+deltaX) - centerX, (startY+deltaY) - centerY);
        self.deltaAngle = subtractAngles(endAngle, center.orientation.rotation);
        self.arcLength = abs(radius * abs(deltaAngle))
        self.reverseDirection = reverseDirection
//        print("Arc   startO", (startOrientation / Double.pi) * 180.0)
//        print("Arc     endO", ((startOrientation + deltaAngle) / Double.pi) * 180.0)
//        print("Arc reversed", reverseDirection)
        /*
        self.center = KVector2(centerX, centerY)
        self.radius = radius
        self.startOrientation = startO
        self.deltaAngle = deltaAngle
//        self.startAngle = addAngles(startOrientation, Double.pi/2)
        self.startAngle = startAngle;
        self.arcLength = abs(radius * deltaAngle)
        */
    }
    
    init(_ radius: Double, _ subtendedAngle: Double)
    {
        self.center = KMatrix2(radius, 0.0, -Double.pi / 2.0)
        self.radius = radius
        //        print("r:", radius)
        self.startOrientation = 0.0
        self.deltaAngle = subtendedAngle
        //        print("a:", (deltaAngle * 180.0) / Double.pi)
        self.arcLength = abs(radius * subtendedAngle)
        self.reverseDirection = subtendedAngle < 0.0
    }
     */
    
//    init(_ startPosition: KMatrix2, _ radius: Double, _ subtendedAngle: Double)
    init(_ startPosition: KMatrix2, _ arcLength: Double, _ subtendedAngle: Double)
    {
        self.arcLength = abs(arcLength)
        self.radius = arcLength / subtendedAngle
        self.center = KMatrix2(radius, 0.0, -Double.pi / 2.0)
        self.center.concat(startPosition)
        self.startOrientation = 0.0
        self.deltaAngle = subtendedAngle
        self.reverseDirection = subtendedAngle < 0.0
    }
    
    func interpolate(_ t: Double, _ p: KMatrix2)
    {
        let currentAngle = deltaAngle * t
        let angleOnArc = addAngles(center.orientation.rotation, currentAngle)
        p.position.set(center.position.getX() + (radius * sin(angleOnArc)),
                       center.position.getY() + (radius * cos(angleOnArc)))
        p.orientation.rotation = addAngles(startOrientation, currentAngle)
    }
    
}
