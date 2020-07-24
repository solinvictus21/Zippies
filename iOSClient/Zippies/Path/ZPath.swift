
import Foundation
import UIKit

let SEGMENTS_PER_PATH = 100

protocol ZPath
{
    
    func updatesPosition() -> Bool
    func getLength() -> CGFloat
    func getDrawable(_ color: UIColor) -> ZDrawable
    
    func interpolate(_ t: CGFloat, _ p: ZMatrix2)

}
