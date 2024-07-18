
import Foundation
import CoreBluetooth

//services & characteristics UUIDs
let ZippyBroadcastUUID = CBUUID(string: "B00B")
let ZippySensorRightRxUUID = CBUUID(string: "F652B269-4405-42D7-8CD8-0630E778E0D0")
let ZippySensorLeftRxUUID = CBUUID(string: "53372C2D-2BD5-44C9-990F-A329E5E1F4B5")
let ZippyComputedDataRxUUID = CBUUID(string: "7965B674-B7AE-4E02-B334-872ABBE5999D")

protocol ZippyDelegate: class
{
    func positionUpdated(_ zippy: Zippy)
}

class Zippy: NSObject
{
    
    var delegate: ZippyDelegate?

    var zippyPeripheral: CBPeripheral
    var motorControlService: CBService
    var motorControlCharacteristic: CBCharacteristic
    var positionCharacteristic: CBCharacteristic
    
    var lighthouseVisible: Bool = false
    var pose = ZMatrix2()
    var linearVelocity: Float32 = 0.0
    var angularVelocity: Float32 = 0.0

    init(_ peripheral: CBPeripheral,
         motorControlService: CBService,
         motorControlCharacteristic: CBCharacteristic,
         positionCharacteristic: CBCharacteristic)
    {
        self.zippyPeripheral = peripheral
        self.motorControlService = motorControlService
        self.motorControlCharacteristic = motorControlCharacteristic
        self.positionCharacteristic = positionCharacteristic
        super.init()

        self.zippyPeripheral.delegate = self
        self.zippyPeripheral.setNotifyValue(true, for: positionCharacteristic)
    }
    
    func setMotors(left: Float32, right: Float32) {
//        print(String(format: "Setting motors: %.1f, %.1f", left, right))
//        print("Setting motors: \(left), \(right)")
        var data = Data(capacity: 8)
        withUnsafePointer(to: left) { pointer in
            data.append(UnsafeBufferPointer(start: pointer, count: 1))
        }
        withUnsafePointer(to: right) { pointer in
            data.append(UnsafeBufferPointer(start: pointer, count: 1))
        }

        self.zippyPeripheral.writeValue(data, for: self.motorControlCharacteristic, type: CBCharacteristicWriteType.withoutResponse)
    }
    
    fileprivate func extractPositionData(_ positionData: [UInt8]) {
        lighthouseVisible = positionData[0] == 0 ? false : true
        pose.position.setX(Double(extractInt16(positionData, position: 1)) / 10.0)
        pose.position.setY(Double(extractInt16(positionData, position: 3)) / 10.0)
        pose.orientation.rotation = Double(extractInt16(positionData, position: 5)) / 10.0
        linearVelocity = Float32(extractInt16(positionData, position: 7)) / 10.0
        angularVelocity = Float32(extractInt16(positionData, position: 9)) / 10.0
        
        delegate?.positionUpdated(self)
    }
    
    fileprivate func extractUInt16(_ data: [UInt8],
                       position: Int) -> UInt16
    {
        return (UInt16(data[position+1]) << 8) | UInt16(data[position])
    }
    
    fileprivate func extractInt16(_ data: [UInt8],
                       position: Int) -> Int16
    {
        return (Int16(data[position+1]) << 8) | Int16(data[position])
    }
    
    fileprivate func extractUInt32(_ data: [UInt8],
                       position: Int) -> UInt32
    {
        return (UInt32(data[position+3]) << 24) | (UInt32(data[position+2]) << 16) |
                (UInt32(data[position+1]) << 8) | UInt32(data[position])
    }
    
    fileprivate func extractInt32(_ data: [UInt8],
                       position: Int) -> Int32
    {
        return (Int32(data[position+3]) << 24) | (Int32(data[position+2]) << 16) |
                (Int32(data[position+1]) << 8) | Int32(data[position])
    }
    
    fileprivate func extractFloat16(_ data: [UInt8],
                       position: Int) -> Float16
    {
        return Float16(bitPattern: extractUInt16(data, position: position))
    }
    
    fileprivate func extractFloat32(_ data: [UInt8],
                        position: Int) -> Float32
    {
        return Float32(bitPattern: extractUInt32(data, position: position))
    }
    
    deinit {
        self.zippyPeripheral.delegate = nil
    }
    
}

extension Zippy: CBPeripheralDelegate
{

    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: (any Error)?) {
        switch (characteristic) {
        case positionCharacteristic:
            self.extractPositionData([UInt8](characteristic.value!))
            break
        default:
            break
        }
    }

    func peripheral(_ peripheral: CBPeripheral, didUpdateNotificationStateFor characteristic: CBCharacteristic, error: Error?) {
        if error != nil {
            print("Received error for peripheral '", peripheral.name!, "' on characteristic '", characteristic, "': ", error!)
        }
    }

}

