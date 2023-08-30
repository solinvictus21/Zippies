
import Foundation
import UIKit

class DebugViewController: UIViewController
{
    
    @IBOutlet weak var modeSwitch: UISegmentedControl!

    @IBOutlet weak var statusLabel: UILabel!

    @IBOutlet weak var lighthouseXLabel: UILabel!
    @IBOutlet weak var lighthouseYLabel: UILabel!
    @IBOutlet weak var lighthouseZLabel: UILabel!
    
    @IBOutlet weak var lighthouseThetaXLabel: UILabel!
    @IBOutlet weak var lighthouseThetaYLabel: UILabel!
    @IBOutlet weak var lighthouseThetaZLabel: UILabel!
    
    @IBOutlet weak var diodeLeftXSyncFlashLabel: UILabel!
    @IBOutlet weak var diodeLeftXSweepTickLabel: UILabel!
    @IBOutlet weak var diodeLeftXPositionLabel: UILabel!
    
    @IBOutlet weak var diodeLeftYSyncFlashLabel: UILabel!
    @IBOutlet weak var diodeLeftYSweepTickLabel: UILabel!
    @IBOutlet weak var diodeLeftYPositionLabel: UILabel!
    
    @IBOutlet weak var diodeRightXSyncFlashLabel: UILabel!
    @IBOutlet weak var diodeRightXSweepTickLabel: UILabel!
    @IBOutlet weak var diodeRightXPositionLabel: UILabel!
    
    @IBOutlet weak var diodeRightYSyncFlashLabel: UILabel!
    @IBOutlet weak var diodeRightYSweepTickLabel: UILabel!
    @IBOutlet weak var diodeRightYPositionLabel: UILabel!
    
    @IBOutlet weak var diodeSeparationLabel: UILabel!
    @IBOutlet weak var xPositionLabel: UILabel!
    @IBOutlet weak var yPositionLabel: UILabel!
    @IBOutlet weak var linearVelocityLabel: UILabel!
    @IBOutlet weak var orientationLabel: UILabel!
    @IBOutlet weak var rotationalVelocityLabel: UILabel!

    var zippyManager: ZippyManager?
    var zippy: Zippy2?
    
    var sensor0X: Float32 = 0.0
    var sensor0Y: Float32 = 0.0
    var sensor1X: Float32 = 0.0
    var sensor1Y: Float32 = 0.0
    
    override func viewDidLoad() {
        /*
        super.viewDidLoad()
        
        let appDelegate = UIApplication.shared.delegate as! AppDelegate
        self.zippyManager = appDelegate.zippyManager
        self.zippyManager?.delegate = self
        self.zippy = zippyManager?.anyZippy()
        self.zippy?.delegate = self
        
        statusLabel.textColor = zippy != nil ? UIColor.green : UIColor.red
         */
    }
    
    @IBAction func modeChanged(_ sender: Any) {
        /*
        if self.zippy != nil {
          zippy!.setManualModeEnabled(modeSwitch.selectedSegmentIndex == 0)
        }
        */
    }

}

extension DebugViewController: ZippyManagerDelegate
{
    
    func zippyDiscovered(_ zippy: Zippy2)
    {
    }
    
    func zippyConnected(_ zippy: Zippy2)
    {
        /*
        if self.zippy == nil {
            self.zippy = zippy
            self.zippy?.delegate = self
            self.statusLabel.textColor = UIColor.green
        }
        */
    }
    
    func zippyDisconnected(_ zippy: Zippy2)
    {
        /*
        if self.zippy == zippy {
            self.zippy = nil
            self.zippy?.delegate = nil
            self.statusLabel.textColor = UIColor.red
            self.zippyManager?.startDiscovery()
        }
        */
    }
    
}

extension DebugViewController: ZippyDelegate
{
    
    func sensorLeftDataUpdated(xSyncTicks: UInt16, xSweepTicks: UInt32, xPosition: Float32,
                               ySyncTicks: UInt16, ySweepTicks: UInt32, yPosition: Float32)
    {
        diodeLeftXSyncFlashLabel.text = String(xSyncTicks)
        diodeLeftXSweepTickLabel.text = String(xSweepTicks)
        diodeLeftXPositionLabel.text = String(format: "%0.1f", xPosition)
        diodeLeftYSyncFlashLabel.text = String(ySyncTicks)
        diodeLeftYSweepTickLabel.text = String(ySweepTicks)
        diodeLeftYPositionLabel.text = String(format: "%0.1f", yPosition)
        sensor0X = xPosition
        sensor0Y = yPosition
    }
    
    func sensorRightDataUpdated(xSyncTicks: UInt16, xSweepTicks: UInt32, xPosition: Float32,
                                ySyncTicks: UInt16, ySweepTicks: UInt32, yPosition: Float32)
    {
        diodeRightXSyncFlashLabel.text = String(xSyncTicks)
        diodeRightXSweepTickLabel.text = String(xSweepTicks)
        diodeRightXPositionLabel.text = String(format: "%0.1f", xPosition)
        diodeRightYSyncFlashLabel.text = String(ySyncTicks)
        diodeRightYSweepTickLabel.text = String(ySweepTicks)
        diodeRightYPositionLabel.text = String(format: "%0.1f", yPosition)
        sensor1X = xPosition
        sensor1Y = yPosition
    }
    
    func computedDataUpdated(xPosition: Float32, yPosition: Float32, linearVelocity: Float32,
                             orientation: Float32, rotationalVelocity: Float32)
    {
        diodeSeparationLabel.text = String(format: "%0.2f", sqrt(pow(sensor1X - sensor0X, 2.0) + pow(sensor1Y - sensor0Y, 2.0)))
        xPositionLabel.text = String(format: "%0.1f", xPosition)
        yPositionLabel.text = String(format: "%0.1f", yPosition)
        linearVelocityLabel.text = String(format: "%0.1f", linearVelocity)
        orientationLabel.text = String(format: "%0.3f", orientation)
        rotationalVelocityLabel.text = String(format: "%0.3f", rotationalVelocity)
    }
    
}
