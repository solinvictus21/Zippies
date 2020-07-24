
import Foundation
import UIKit

class ZPlucker3: NSObject
{
    
    let rotation: ZVector3
    let translation: ZVector3

    override init()
    {
        rotation = ZVector3()
        translation = ZVector3()
    }
    
    init(
        _ rx: CGFloat, _ ry: CGFloat, _ rz: CGFloat,
        _ tx: CGFloat, _ ty: CGFloat, _ tz: CGFloat)
    {
        rotation = ZVector3(rx, ry, rz)
        translation = ZVector3(tx, ty, tz)
    }
    
}
