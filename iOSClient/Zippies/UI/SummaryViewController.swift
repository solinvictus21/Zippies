
import Foundation
import UIKit
import CoreBluetooth

class SummaryViewController: UIViewController
{
    
    var zippyManager: ZippyManager?
    
    @IBOutlet weak var zippyView : TopDownView!

    override func viewDidLoad() {
        super.viewDidLoad()
        
        /*
        let appDelegate = UIApplication.shared.delegate as! AppDelegate
        self.zippyManager = appDelegate.zippyManager
        self.zippyManager?.delegate = self
        zippyView.zippyManager = self.zippyManager
         */
    }
    
}

extension SummaryViewController: ZippyManagerDelegate
{
    
    func zippyConnected(_ zippy: Zippy)
    {
    }
    
    func zippyDisconnected(_ zippy: Zippy)
    {
//        print("Requesting redraw")
        zippyView.setNeedsDisplay()
    }
    
}

