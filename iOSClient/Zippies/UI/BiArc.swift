
import Foundation
import UIKit

class BiArc: ZPath
{

    fileprivate let arc1: Arc
    fileprivate let arc2: Arc
    fileprivate let totalArcLength: Double
    
    init(_ arc1: Arc, _ arc2: Arc)
    {
        self.arc1 = arc1
        self.arc2 = arc2
        self.totalArcLength = arc1.arcLength + arc2.arcLength
    }

    func interpolate(_ t: Double, _ p: KMatrix2)
    {
        let distance = t * totalArcLength;
        if (distance < arc1.arcLength) {
            //moving through first arc
            arc1.interpolate(distance / arc1.arcLength, p);
        }
        else {
            //moving through second arc
            arc2.interpolate((distance - arc1.arcLength) / arc2.arcLength, p);
        }
    }
    
    /*
    override func draw(_ transform: CGAffineTransform)
    {
        UIColor.green.setStroke()
        arc1.draw(transform)
        UIColor.red.setStroke()
        arc2.draw(transform)
    }
     */

}
