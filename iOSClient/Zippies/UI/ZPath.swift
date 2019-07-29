
import Foundation
import UIKit

let SEGMENTS_PER_PATH = 100

protocol ZPath
{
    
    func interpolate(_ t: Double, _ p: KMatrix2)

}
