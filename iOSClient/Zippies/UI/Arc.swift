
import Foundation

class Arc: NSObject
{
    
    var center = KVector2()
    var radius: Double
    var startAngle: Double
    var deltaAngle: Double
    
    init(startX: Double, startY: Double,
         endX: Double, endY: Double,
         tangentX: Double, tangentY: Double)
    {
        //pmp = the vector from the starting point to the ending point
        let deltaX = endX - startX
        let deltaY = endY - startY
        let deltaDotDelta = (deltaX * deltaX) + (deltaY * deltaY)
        let n2DotDelta = ((2.0 * -tangentY) * deltaX) + ((2.0 * tangentX) * deltaY)
        
        //the radius; a negative value indicates a turn forward to the right or backward to the left
        radius = deltaDotDelta / n2DotDelta

        //c = the center point
        center.set(x: startX + (radius * -tangentY), y: startY + (radius * tangentX))
        startAngle = atan2(startX - center.getX(), startY - center.getY())
        let endAngle = atan2(endX - center.getX(), endY - center.getY());
        deltaAngle = subtractAngles(a1: endAngle, a2: startAngle)
//        print("r:", radius, "sa:", startAngle * 180.0 / Double.pi,
//              "ea:", endAngle * 180.0 / Double.pi, "da:", deltaAngle * 180.0 / Double.pi)
        radius = abs(radius)
    }
    
    func interpolate(t: Double, v: KVector2)
    {
        let currentAngle = deltaAngle * t
        let angleOnArc = addAngles(a1: startAngle, a2: currentAngle)
        v.set(x: center.getX() + (radius * sin(angleOnArc)),
              y: center.getY() + (radius * cos(angleOnArc)))
    }
        
}
