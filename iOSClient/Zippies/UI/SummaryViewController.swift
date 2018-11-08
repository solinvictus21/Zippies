
import Foundation
import UIKit
import CoreBluetooth

class SummaryViewController: UIViewController
{
    
    var zippyManager: ZippyManager?
    
    @IBOutlet weak var zippyView : TopDownView!

    override func viewDidLoad() {
        super.viewDidLoad()
        
        let appDelegate = UIApplication.shared.delegate as! AppDelegate
        self.zippyManager = appDelegate.zippyManager
        self.zippyManager?.delegate = self
        zippyView.zippyManager = self.zippyManager
    }
    
}

extension SummaryViewController: ZippyManagerDelegate
{
    
    func zippyDiscovered(_ zippy: Zippy2)
    {
//        print("Requesting redraw")
        zippyView.setNeedsDisplay();
    }
    
    func zippyConnected(_ zippy: Zippy2)
    {
    }
    
    func compassUpdated(_ zippy: Zippy2, x: Int32, y: Int32, z: Int32)
    {
    }
    
    func zippyDisconnected(_ zippy: Zippy2)
    {
    }
    
}

