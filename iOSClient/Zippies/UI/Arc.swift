
import Foundation
import UIKit

class Arc: ZPath
{
    
    let startOrientation: Double
    let center: KVector2
    let radius: Double
    let startAngle: Double
    let deltaAngle: Double
    let arcLength: Double
    
    init(_ startX: Double, _ startY: Double, _ startO: Double,
         _ endX: Double, _ endY: Double)
    {
        self.startOrientation = startO
        let deltaX = endX - startX
        let deltaY = endY - startY
        let deltaDotDelta = (deltaX * deltaX) + (deltaY * deltaY)
        let tangentX = sin(startO)
        let tangentY = cos(startO)
        let n2DotDelta = ((2.0 * -tangentY) * deltaX) + ((2.0 * tangentX) * deltaY)

        //the radius; a negative value indicates a turn forward to the right or backward to the left
        let s = deltaDotDelta / n2DotDelta

        //c = the center point
        center = KVector2(startX + (s * -tangentY), startY + (s * tangentX))
        startAngle = atan2(startX - center.getX(), startY - center.getY())
        let endAngle = atan2(endX - center.getX(), endY - center.getY());
        deltaAngle = subtractAngles(endAngle, startAngle)
        radius = abs(s)
        arcLength = abs(radius * deltaAngle)
    }
    
    func interpolate(_ t: Double, _ p: KPosition)
    {
        let currentAngle = deltaAngle * t
        let angleOnArc = addAngles(startAngle, currentAngle)
        p.vector.set(center.getX() + (radius * sin(angleOnArc)),
              center.getY() + (radius * cos(angleOnArc)))
        p.orientation = addAngles(startOrientation, currentAngle)
    }
    
}
