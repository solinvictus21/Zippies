
import Foundation
import UIKit

let ZIPPY_WIDTH: CGFloat = 29.4
let ZIPPY_HEIGHT: CGFloat = 24.8
let ZIPPY_HALF_WIDTH: CGFloat = ZIPPY_WIDTH / 2.0
let ZIPPY_HALF_HEIGHT: CGFloat = ZIPPY_HEIGHT / 2.0
let ZIPPY_AREA_WIDTH: CGFloat = 2000.0
let ZIPPY_AREA_HEIGHT: CGFloat = 1000.0
//let ZIPPY_AREA_HALF_WIDTH = ZIPPY_AREA_WIDTH / 2.0
//let ZIPPY_AREA_HALF_HEIGHT = ZIPPY_AREA_HEIGHT / 2.0

class TopDownView : UIView
{
    
    var zippyManager: ZippyManager?
    
    fileprivate var moveToDisplay: ZDrawable

    init(s: String, i: Int)
    {
        moveToDisplay = createTestPath()
        super.init(frame: CGRect(x: 0, y: 0, width: 100, height: 100))
    }
    
    required init?(coder aDecoder: NSCoder)
    {
        moveToDisplay = createTestPath()
        super.init(coder: aDecoder)
    }

    override func touchesBegan(_ touches: Set<UITouch>, with event: UIEvent?)
    {
        super.touchesBegan(touches, with: event)
        moveToDisplay = createTestPath()
        setNeedsDisplay()
    }

    override func draw(_ rect: CGRect)
    {
        //calculate the appropriate transform to turn Zippy coordinates into display coordinates
        let interpolationX = self.frame.width / ZIPPY_AREA_WIDTH
        let interpolationY = self.frame.height / ZIPPY_AREA_HEIGHT
        let transform = CGAffineTransform(translationX: self.frame.width/2, y: self.frame.height/2)
            .scaledBy(x: interpolationX, y: -interpolationY)

        //display the move
        moveToDisplay.draw(transform)
    }
    
    /*
    func translateAngle(_ a: Double) -> CGFloat
    {
        guard a < (Double.pi / 2) else {
            return CGFloat((2.5 * Double.pi) - a)
        }
        return CGFloat((Double.pi / 2) - a)
    }
     */
    
}
