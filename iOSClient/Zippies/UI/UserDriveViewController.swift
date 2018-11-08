
import Foundation
import UIKit
import CoreBluetooth

//a "max turn" when moving forward or backward would be with one motor at 0.0 and the other at 1.0/-1.0
//we scale that so that the slower moving motor can never be different from the faster motor to keep
//the turning from being "touchy" and favor driving straight; this does not apply when spinning
let MOTORS_MIN_TURNING_PERCENTAGE : Float32  = 0.99

class UserDriveViewController: UIViewController
{
    
    @IBOutlet weak var sliderLeft: SnappingSlider!
    @IBOutlet weak var sliderRight: SnappingSlider!
    @IBOutlet weak var statusLabel: UILabel!
    
    var zippyManager: ZippyManager?
    var zippy: Zippy2?
    
    var relativeMotorControl: Bool = false
    fileprivate var valueChangingTimer: Timer? = nil

    override func viewDidLoad() {
        super.viewDidLoad()

        sliderLeft.delegate = self
        sliderRight.delegate = self

        let appDelegate = UIApplication.shared.delegate as! AppDelegate
        self.zippyManager = appDelegate.zippyManager
        self.zippyManager?.delegate = self
//        self.zippy = zippyManager?.anyZippy()

//        statusLabel.textColor = zippy != nil ? UIColor.green : UIColor.red
    }

    @objc
    func sendValues() {
        let motorLeft : Float32 = Float32(-sliderLeft.value())
        let motorRight : Float32 = Float32(-sliderRight.value())
        send(motorLeft: motorLeft, motorRight: motorRight)
    }
    
    func send(motorLeft: Float32, motorRight: Float32) {
        /*
        guard self.zippy != nil else {
            return
        }
        
        guard motorLeft != 0.0 || motorRight != 0.0 else {
            zippy?.stop()
            return
        }
        
//        print("Sending motor values: ", motorLeft, " / ", ml, ", ", motorRight, " / ", mr)
        var ml = motorLeft
        var mr = motorRight
        if relativeMotorControl {
            indirectMotorControl(motorLeft: &ml, motorRight: &mr)
        }
        
        zippy?.setMotors(left: ml, right: mr)
        */
    }
    
    func indirectMotorControl(motorLeft: inout Float32, motorRight: inout Float32) {
        /*
        var slowerMotor, fasterMotor: Float32
        let absLeft = abs(motorLeft)
        let absRight = abs(motorRight)
        if absLeft > absRight {
            slowerMotor = motorRight
            fasterMotor = motorLeft
        }
        else {
            slowerMotor = motorLeft
            fasterMotor = motorRight
        }
        
        let turningRatio = slowerMotor / fasterMotor
        if turningRatio > 0.0 {
            //both values are in the same direction, so the mode is "moving forward or backward while turning"
            //in this mode, whichever value is the max, the other is considered the "turning ratio" and scaled
            //between the minimum turning percentage and the other motor value to keep the turning from being
            //too touchy and favor moving in a straight direction forward or backward
            slowerMotor = (fasterMotor * MOTORS_MIN_TURNING_PERCENTAGE) +
                (turningRatio * fasterMotor * (1.0-MOTORS_MIN_TURNING_PERCENTAGE))
        }
        else {
            //the values are in different directions; while the smaller value goes from zero to the negative
            //of the higher value, scale from the value of the turning ratio toward the negative of the higher
            //value; so when the magnitude of both values is the same, the robot should be perfectly spinning
            let difference = fasterMotor - slowerMotor
            slowerMotor = (fasterMotor * MOTORS_MIN_TURNING_PERCENTAGE) +
                (turningRatio * difference)
        }
        
        if absLeft > absRight {
            motorLeft = fasterMotor
            motorRight = slowerMotor
        }
        else {
            motorLeft = slowerMotor
            motorRight = fasterMotor
        }
        */
    }
    
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
    }
    
}

extension UserDriveViewController: ZippyManagerDelegate
{
    
    func zippyDiscovered(_ zippy: Zippy2)
    {
        
    }
    
    func zippyConnected(_ zippy: Zippy2)
    {
        self.zippy = zippy
        self.statusLabel.textColor = UIColor.green
    }
    
    func compassUpdated(_ zippy: Zippy2, x: Int32, y: Int32, z: Int32)
    {
        
    }
    
    func zippyDisconnected(_ zippy: Zippy2)
    {
        if self.zippy == zippy
        {
            self.zippy = nil
            self.statusLabel.textColor = UIColor.red
        }
    }
    
}

extension UserDriveViewController: SnappingSliderDelegate
{
    
    func snappingSliderDidChangeValue(_ slider:SnappingSlider) {
        /*
        let motorLeft : Float32 = Float32(-sliderLeft.value())
        let motorRight : Float32 = Float32(-sliderRight.value())
        
        if motorLeft == 0.0 && motorRight == 0.0 {
            //come to a complete stop and shut off the message timer
            if valueChangingTimer != nil {
                valueChangingTimer!.invalidate()
                valueChangingTimer = nil
            }
            zippy?.stop()
        }
        else if valueChangingTimer == nil {
            //send first value immediately and start the timer
            send(motorLeft: motorLeft, motorRight: motorRight)
            valueChangingTimer = Timer.scheduledTimer(timeInterval: 0.2, target: self, selector: #selector(self.sendValues), userInfo: nil, repeats: true)
        }
        */
    }
    
}
