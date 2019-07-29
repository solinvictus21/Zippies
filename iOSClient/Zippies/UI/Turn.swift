
import Foundation
import UIKit

class Turn: ZPath
{
    
    let startO: Double
    let deltaO: Double
    
    init(_ o: Double, _ deltaO: Double)
    {
        self.startO = o
        self.deltaO = deltaO
    }
    
    func interpolate(_ t: Double, _ p: KMatrix2)
    {
        p.orientation.rotation = addAngles(startO, deltaO * t);
    }

}
