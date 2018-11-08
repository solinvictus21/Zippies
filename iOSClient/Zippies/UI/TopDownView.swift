
import Foundation
import UIKit

let ZIPPY_WIDTH : CGFloat = 29.4
let ZIPPY_HEIGHT : CGFloat = 24.8
let ZIPPY_HALF_WIDTH = ZIPPY_WIDTH / 2.0
let ZIPPY_HALF_HEIGHT = ZIPPY_HEIGHT / 2.0
let ZIPPY_AREA_WIDTH : CGFloat = 2000.0
let ZIPPY_AREA_HEIGHT : CGFloat = 1000.0
//let ZIPPY_AREA_HALF_WIDTH = ZIPPY_AREA_WIDTH / 2.0
//let ZIPPY_AREA_HALF_HEIGHT = ZIPPY_AREA_HEIGHT / 2.0

class TopDownView : UIView
{
    
    var zippyManager: ZippyManager?
    
    override func draw(_ rect: CGRect)
    {
//        print("Redrawing")
        let viewHalfWidth = self.frame.width / 2.0
        let viewHalfHeight = self.frame.height / 2.0
        let interpolationX = self.frame.width / ZIPPY_AREA_WIDTH
        let interpolationY = self.frame.height / ZIPPY_AREA_HEIGHT
        UIColor.blue.setFill()
        let icon = UIBezierPath(rect: CGRect(x: -ZIPPY_HALF_WIDTH * interpolationX,
                                             y: -ZIPPY_HALF_HEIGHT * interpolationY,
                                             width: ZIPPY_WIDTH * interpolationX,
                                             height: ZIPPY_HEIGHT * interpolationY))
        for (_, zippy) in zippyManager!.zippies
        {
            icon.apply(CGAffineTransform(translationX: viewHalfWidth - (CGFloat(zippy.x) * interpolationX),
                                         y: viewHalfHeight + (CGFloat(zippy.y) * interpolationY))
                .rotated(by: CGFloat(zippy.orientation)))
            icon.fill()
        }
    }
    
}
