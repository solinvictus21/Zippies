
import Foundation
import UIKit
import CoreBluetooth

let ModeChangeDrive = "kModeChangeDrive";
let ModeChangeDebug = "kModeChangeDebug";

class ZippyNavigationController: UINavigationController
{
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        let appDelegate = UIApplication.shared.delegate as! AppDelegate
//        appDelegate.zippyManager.startDiscovery()
        appDelegate.zippyManager.startObserving()
    }
    
}


