
import Foundation
import CoreBluetooth

//services & characteristics UUIDs
let ZippyBroadcastUUID = CBUUID(string: "B00B")
let ZippyServiceUUID = CBUUID(string: "5BF1CEC2-EFC2-44D2-81AE-73FCFD5F7A13")
let ZippyTxUUID = CBUUID(string: "C9589239-D751-47F7-9C5B-38355F0E9811")
let ZippySensorRightRxUUID = CBUUID(string: "F652B269-4405-42D7-8CD8-0630E778E0D0")
let ZippySensorLeftRxUUID = CBUUID(string: "53372C2D-2BD5-44C9-990F-A329E5E1F4B5")
let ZippyComputedDataRxUUID = CBUUID(string: "7965B674-B7AE-4E02-B334-872ABBE5999D")

let MESSAGE_SEND_MOTORS_STOP: UInt8     = 0x00
let MESSAGE_SEND_MOTORS_SET: UInt8      = 0x15
let MESSAGE_RECEIVE_DEBUG: UInt8        = 0x16
let MESSAGE_AUTODRIVE_MODE: UInt8       = 0x20
let MESSAGE_MANUAL_MODE: UInt8          = 0x21

protocol ZippyDelegate: class
{
    func sensorLeftDataUpdated(xSyncTicks: UInt16, xSweepTicks: UInt32, xPosition: Float32,
                               ySyncTicks: UInt16, ySweepTicks: UInt32, yPosition: Float32)
    
    func sensorRightDataUpdated(xSyncTicks: UInt16, xSweepTicks: UInt32, xPosition: Float32,
                                ySyncTicks: UInt16, ySweepTicks: UInt32, yPosition: Float32)
    
    func computedDataUpdated(xPosition: Float32, yPosition: Float32, linearVelocity: Float32,
                             orientation: Float32, rotationalVelocity: Float32)
}

struct SensorData {
    var xSyncTicks: UInt16 = 0
    var xSweepTicks: UInt32 = 0
    var xPosition: Float32 = 0.0
    var ySyncTicks: UInt16 = 0
    var ySweepTicks: UInt32 = 0
    var yPosition: Float32 = 0.0
}

class Zippy: BluetoothPeripheral
{
    
    var delegate: ZippyDelegate?

    var sendCharacteristic: CBCharacteristic
    var sensorLeftReceiveCharacteristic: CBCharacteristic
    var sensorRightReceiveCharacteristic: CBCharacteristic
    var computedDataReceiveCharacteristic: CBCharacteristic
    
    var sensorLeftData = SensorData()
    var sensorRightData = SensorData()
    var xPosition: Float32 = 0.0
    var yPosition: Float32 = 0.0
    var linearVelocity: Float32 = 0.0
    var orientation: Float32 = 0.0
    var rotationalVelocity: Float32 = 0.0

    override init(_ peripheral: CBPeripheral,
                  primaryService: CBService,
                  characteristics: [CBUUID: CBCharacteristic])
    {
        self.sendCharacteristic = characteristics[ZippyTxUUID]!
        self.sensorLeftReceiveCharacteristic = characteristics[ZippySensorLeftRxUUID]!
        self.sensorRightReceiveCharacteristic = characteristics[ZippySensorRightRxUUID]!
        self.computedDataReceiveCharacteristic = characteristics[ZippyComputedDataRxUUID]!
        
        super.init(peripheral,
                   primaryService: primaryService,
                   characteristics: characteristics)

        self.peripheral.delegate = self
        self.peripheral.setNotifyValue(true, for: sensorLeftReceiveCharacteristic)
        self.peripheral.setNotifyValue(true, for: sensorRightReceiveCharacteristic)
        self.peripheral.setNotifyValue(true, for: computedDataReceiveCharacteristic)
    }
    
    func setManualModeEnabled(_ e: Bool) {
        let data = NSMutableData(capacity: 1)
        var header: UInt8 = e ? MESSAGE_MANUAL_MODE : MESSAGE_AUTODRIVE_MODE
        data!.append(&header, length: 1);
        writeValue(data! as Data)
    }
    
    func setMotors(left: Float32, right: Float32) {
        let data = NSMutableData(capacity: 9)
        var header : UInt8 = MESSAGE_SEND_MOTORS_SET
        data!.append(&header, length: 1)
        var f = left
        data!.append(&f, length: 4)
        f = right
        data!.append(&f, length: 4)
        writeValue(data! as Data)
    }
    
    func stop() {
        let data = Data.init(bytes: [MESSAGE_SEND_MOTORS_STOP])
        writeValue(data)
    }
    
    fileprivate func writeValue(_ data: Data) {
        self.peripheral.writeValue(data, for: self.sendCharacteristic, type: CBCharacteristicWriteType.withoutResponse)
    }
    
    fileprivate func extractSensorData(values: [UInt8], sensorData: inout SensorData) {
        sensorData.xSyncTicks = UInt16(bitPattern: Int16(values[1]) << 8 | Int16(values[0]))
        var bitPattern32 = Int32(values[5]) << 24
        bitPattern32 |= Int32(values[4]) << 16
        bitPattern32 |= Int32(values[3]) << 8
        bitPattern32 |= Int32(values[2])
        sensorData.xSweepTicks = UInt32(bitPattern: bitPattern32)
        var bitPatternU32 = UInt32(values[9]) << 24
        bitPatternU32 |= UInt32(values[8]) << 16
        bitPatternU32 |= UInt32(values[7]) << 8
        bitPatternU32 |= UInt32(values[6])
        sensorData.xPosition = Float32(bitPattern: bitPatternU32)
        sensorData.ySyncTicks = UInt16(bitPattern: Int16(values[11]) << 8 | Int16(values[10]))
        bitPattern32 = Int32(values[15]) << 24
        bitPattern32 |= Int32(values[14]) << 16
        bitPattern32 |= Int32(values[13]) << 8
        bitPattern32 |= Int32(values[12])
        sensorData.ySweepTicks = UInt32(bitPattern: bitPattern32)
        bitPatternU32 = UInt32(values[19]) << 24
        bitPatternU32 |= UInt32(values[18]) << 16
        bitPatternU32 |= UInt32(values[17]) << 8
        bitPatternU32 |= UInt32(values[16])
        sensorData.yPosition = Float32(bitPattern: bitPatternU32)
    }
    
    deinit {
        self.peripheral.delegate = nil
    }
    
}

extension Zippy: CBPeripheralDelegate
{

    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        switch (characteristic) {
        case sensorLeftReceiveCharacteristic:
            let values = [UInt8](characteristic.value!)
            extractSensorData(values: values, sensorData: &sensorLeftData)
            DispatchQueue.main.async(execute: {
                self.delegate?.sensorLeftDataUpdated(xSyncTicks: self.sensorLeftData.xSyncTicks,
                                                     xSweepTicks: self.sensorLeftData.xSweepTicks,
                                                     xPosition: self.sensorLeftData.xPosition,
                                                     ySyncTicks: self.sensorLeftData.ySyncTicks,
                                                     ySweepTicks: self.sensorLeftData.ySweepTicks,
                                                     yPosition: self.sensorLeftData.yPosition)
            })
            break
        case sensorRightReceiveCharacteristic:
            let values = [UInt8](characteristic.value!)
            extractSensorData(values: values, sensorData: &sensorRightData)
            DispatchQueue.main.async(execute: {
                self.delegate?.sensorRightDataUpdated(xSyncTicks: self.sensorRightData.xSyncTicks,
                                                      xSweepTicks: self.sensorRightData.xSweepTicks,
                                                      xPosition: self.sensorRightData.xPosition,
                                                      ySyncTicks: self.sensorRightData.ySyncTicks,
                                                      ySweepTicks: self.sensorRightData.ySweepTicks,
                                                      yPosition: self.sensorRightData.yPosition)
            })
            break
        case computedDataReceiveCharacteristic:
            let values = [UInt8](characteristic.value!)
            var bitPatternU32 = UInt32(values[3]) << 24
            bitPatternU32 |= UInt32(values[2]) << 16
            bitPatternU32 |= UInt32(values[1]) << 8
            bitPatternU32 |= UInt32(values[0])
            self.xPosition = Float32(bitPattern: bitPatternU32)
            bitPatternU32 = UInt32(values[7]) << 24
            bitPatternU32 |= UInt32(values[6]) << 16
            bitPatternU32 |= UInt32(values[5]) << 8
            bitPatternU32 |= UInt32(values[4])
            self.yPosition = Float32(bitPattern: bitPatternU32)
            bitPatternU32 = UInt32(values[11]) << 24
            bitPatternU32 |= UInt32(values[10]) << 16
            bitPatternU32 |= UInt32(values[9]) << 8
            bitPatternU32 |= UInt32(values[8])
            self.linearVelocity = Float32(bitPattern: bitPatternU32)
            bitPatternU32 = UInt32(values[15]) << 24
            bitPatternU32 |= UInt32(values[14]) << 16
            bitPatternU32 |= UInt32(values[13]) << 8
            bitPatternU32 |= UInt32(values[12])
            self.orientation = Float32(bitPattern: bitPatternU32)
            bitPatternU32 = UInt32(values[19]) << 24
            bitPatternU32 |= UInt32(values[18]) << 16
            bitPatternU32 |= UInt32(values[17]) << 8
            bitPatternU32 |= UInt32(values[16])
            self.rotationalVelocity = Float32(bitPattern: bitPatternU32)
            DispatchQueue.main.async(execute: {
                self.delegate?.computedDataUpdated(xPosition: self.xPosition,
                                                   yPosition: self.yPosition,
                                                   linearVelocity: self.linearVelocity,
                                                   orientation: self.orientation,
                                                   rotationalVelocity: self.rotationalVelocity)
            })
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

