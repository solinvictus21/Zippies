
import Foundation
import UIKit
import CoreBluetooth
import SwiftUI

//a "max turn" when moving forward or backward would be with one motor at 0.0 and the other at 1.0/-1.0
//we scale that so that the slower moving motor can never be different from the faster motor to keep
//the turning from being "touchy" and favor driving straight. this does not apply when spinning
let MOTORS_MIN_TURNING_PERCENTAGE : Float32  = 0.99

class UserDriveViewController: UIViewController {
    
    @IBOutlet weak var sliderLeft: SnappingSlider!
    @IBOutlet weak var sliderRight: SnappingSlider!
    @IBOutlet weak var statusLabel: UILabel!
    @IBOutlet weak var xPositionLabel: UILabel!
    @IBOutlet weak var yPositionLabel: UILabel!
    @IBOutlet weak var orientationLabel: UILabel!
    @IBOutlet weak var linearVelocityLabel: UILabel!
    @IBOutlet weak var angularVelocityLabel: UILabel!
    @IBOutlet weak var zippyMapView: ZippyMapView!

    let zippyManager = ZippyManager()
    var zippy: Zippy?
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        sliderLeft.delegate = self
        sliderRight.delegate = self
        
        self.zippyManager.delegate = self
        self.zippyManager.startDiscovery()

        statusLabel.textColor = zippy != nil ? UIColor.green : UIColor.red
    }

    func sendUpdate() {
        guard self.zippy != nil else {
            //no Zippy to control
            return
        }
        
        let motorLeft = Float32(-sliderLeft.value())
        let motorRight = Float32(-sliderRight.value())

//        print("Setting motors: \(motorLeft), \(motorRight)")
        //turn less at higher speeds
//        let turnFactor = 1.0 - pow(1.0 - abs((motorLeft + motorRight) / 2.0), 2.0)
        let turnFactor = 1.0 - pow(1.0 - ((abs(motorLeft) + abs(motorRight)) / 2.0), 2.0)
        zippy?.setMotors(left: motorLeft * turnFactor, right: motorRight * turnFactor)
    }
    
}

extension UserDriveViewController: ZippyManagerDelegate {
    
    func zippyConnected(_ zippy: Zippy) {
        guard self.zippy == nil else {
            return
        }

        zippy.delegate = self
        self.zippy = zippy
        self.statusLabel.textColor = UIColor.green
        self.zippyManager.stopDiscovery()
//        print("Zippy connected.")
    }
    
    func zippyDisconnected(_ zippy: Zippy) {
        guard self.zippy == zippy else {
            return
        }

//        print("Zippy disconnected.")
        zippy.delegate = nil
        self.zippy = nil
        self.statusLabel.textColor = UIColor.red
        self.zippyManager.startDiscovery()
        
        //clear the map view
//        self.zippyMapView.model.reset()
    }
    
}

extension UserDriveViewController: ZippyDelegate {
    
    func positionUpdated(_ zippy: Zippy) {
        xPositionLabel.text = String(format: "%.1f", zippy.pose.position.getX())
        yPositionLabel.text = String(format: "%.1f", zippy.pose.position.getY())
        orientationLabel.text = String(format: "%.1f", zippy.pose.orientation)
        linearVelocityLabel.text = String(format: "%.1f", zippy.linearVelocity)
        angularVelocityLabel.text = String(format: "%.1f", zippy.angularVelocity)
        
        // update our map view, keeping track of when the Zippy moves in and out of range of the Lighthouse and show a visual map of
        // the borders of the tracking range as well as the trail of the most recent movement of our Zippy
        zippyMapView.addPoint(zippy.pose, isLighthouseVisible: zippy.lighthouseVisible)
    }
    
}

extension UserDriveViewController: SnappingSliderDelegate {
    
    func snappingSliderDidChangeValue(_ slider:SnappingSlider) {
        sendUpdate()
    }
    
}
