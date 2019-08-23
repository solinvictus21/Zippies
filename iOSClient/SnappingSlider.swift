
import QuartzCore
import UIKit

protocol SnappingSliderDelegate: class
{
    func snappingSliderDidChangeValue(_ slider:SnappingSlider)
}

@IBDesignable
class SnappingSlider: UIView
{
    
    @IBInspectable
    var cornerRadius: CGFloat = 3.0 {
        didSet {
            setNeedsLayout()
        }
    }
    
    @IBInspectable
    var sliderColor: UIColor = UIColor(red:0.42, green:0.76, blue:0.74, alpha:1) {
        didSet {
            setNeedsLayout()
        }
    }
    
    final public weak var delegate: SnappingSliderDelegate?
    
    final fileprivate let sliderView = UIView(frame: CGRect.zero)
    
    fileprivate var zeroY: CGFloat = 0.0
    fileprivate var centerY: CGFloat = 0.0
    final fileprivate var touchesBeganPoint = CGPoint.zero
    
    final fileprivate let sliderPanGestureRecognizer = UIPanGestureRecognizer()
    
    override public init(frame:CGRect) {
        super.init(frame: frame)
        
        setup()
        setNeedsLayout()
    }
    
    required public init?(coder aDecoder: NSCoder) {
        super.init(coder: aDecoder)
        
        setup()
        setNeedsLayout()
    }
    
    fileprivate func setup() {
        self.backgroundColor = UIColor(red:0.36, green:0.65, blue:0.65, alpha:1)
        
        self.addSubview(sliderView)
        
        sliderPanGestureRecognizer.addTarget(self, action: #selector(type(of: self).handleGesture(_:)))
        sliderView.addGestureRecognizer(sliderPanGestureRecognizer)
        
        clipsToBounds = true
    }
    
    override open func layoutSubviews() {
        super.layoutSubviews()
        
        let height = bounds.size.height * 0.25
        centerY = bounds.size.height/2.0
        zeroY = centerY - (height/2.0)
        sliderView.frame = CGRect(x: 4.0, y: zeroY, width: bounds.size.width - 8.0, height: height)
        sliderView.backgroundColor = sliderColor
        
        layer.cornerRadius = cornerRadius
    }
    
    public func value() -> CGFloat {
        let sliderValue = (self.sliderView.frame.minY - zeroY)/centerY
        if sliderValue < -1.0 {
            return -1.0
        }
        else if sliderValue > 1.0 {
            return 1.0
        }
        return sliderValue
    }
    
    @objc
    final func handleGesture(_ sender: UIGestureRecognizer) {
        guard sender == sliderPanGestureRecognizer else {
            return
        }
        
        switch sender.state {
            
        case .began:
            touchesBeganPoint = sliderPanGestureRecognizer.translation(in: sliderView)
            
        case .changed:
            let translationInView = sliderPanGestureRecognizer.translation(in: sliderView)
            let translatedCenter: CGFloat = (bounds.size.width * 0.5) + (touchesBeganPoint.y + translationInView.y)
            sliderView.center = CGPoint(x: sliderView.center.x, y: translatedCenter)
            
            delegate?.snappingSliderDidChangeValue(self)
            
        case .ended:
            fallthrough
            
        case .failed:
            fallthrough
            
        case .cancelled:
            //user released the view
            sliderView.frame = CGRect(x: 4.0, y: zeroY, width: bounds.size.width - 8.0, height: bounds.size.height * 0.25)
            delegate?.snappingSliderDidChangeValue(self)
            
        case .possible:
            //swift requires at least one statement per case
            _ = 0
            
        }
    }
    
}

