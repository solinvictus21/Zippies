
import Foundation
import CoreBluetooth

let ZippyTxUUID = CBUUID(string: "C9589239-D751-47F7-9C5B-38355F0E9811")
let ZippySensor0RxUUID = CBUUID(string: "F652B269-4405-42D7-8CD8-0630E778E0D0")
let ZippySensor1RxUUID = CBUUID(string: "53372C2D-2BD5-44C9-990F-A329E5E1F4B5")
let ZippyComputedDataRxUUID = CBUUID(string: "7965B674-B7AE-4E02-B334-872ABBE5999D")

let MESSAGE_SEND_MOTORS_STOP: UInt8     = 0x00
let MESSAGE_SEND_MOTORS_SET: UInt8      = 0x15
let MESSAGE_RECEIVE_DEBUG: UInt8        = 0x16

protocol ZippyDelegate: class
{
    func sensor0DataUpdated(xSyncTicks: UInt16, xSweepTicks: UInt32, xPosition: Float32,
                            ySyncTicks: UInt16, ySweepTicks: UInt32, yPosition: Float32)
    
    func sensor1DataUpdated(xSyncTicks: UInt16, xSweepTicks: UInt32, xPosition: Float32,
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

class Zippy: NSObject
{
    
    var delegate: ZippyDelegate?

    let peripheral: CBPeripheral
    var sendCharacteristic: CBCharacteristic?
    var sensor0ReceiveCharacteristic: CBCharacteristic?
    var sensor1ReceiveCharacteristic: CBCharacteristic?
    var computedDataReceiveCharacteristic: CBCharacteristic?
    
    var sensor0Data = SensorData()
    var sensor1Data = SensorData()
    var xPosition: Float32 = 0.0
    var yPosition: Float32 = 0.0
    var linearVelocity: Float32 = 0.0
    var orientation: Float32 = 0.0
    var rotationalVelocity: Float32 = 0.0

    init(peripheral: CBPeripheral) {
        self.peripheral = peripheral

        super.init()

        //discover services on this peripheral
        self.peripheral.delegate = self
        self.peripheral.discoverServices([ZippyServiceUUID])
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
        if self.sendCharacteristic != nil {
            self.peripheral.writeValue(data, for: self.sendCharacteristic!, type: CBCharacteristicWriteType.withoutResponse)
        }
//        self.peripheral.setNotifyValue(true, for: self.sendCharacteristic)
    }
    
    deinit {
        self.peripheral.delegate = nil
    }
    
}

extension Zippy: CBPeripheralDelegate
{
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        if (error != nil) {
            print("Received unexpected bluetooth error during service discovery.")
            return
        }
        
        if peripheral.services == nil || peripheral.services!.count == 0 {
            //no services
            return
        }
        
        let uuidsForBTService: [CBUUID] = [ZippyTxUUID, ZippySensor0RxUUID, ZippySensor1RxUUID, ZippyComputedDataRxUUID]
        for service in peripheral.services! {
            if service.uuid == ZippyServiceUUID {
                //step #4; discover characteristics
                peripheral.discoverCharacteristics(uuidsForBTService, for: service)
            }
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        guard error == nil else {
            print("Received unexpected bluetooth error during characteristic discovery.")
            return
        }
        
        guard service.characteristics != nil else {
            return
        }
        
        for characteristic in service.characteristics! {
            if characteristic.uuid == ZippyTxUUID {
                sendCharacteristic = characteristic
            }
            else if characteristic.uuid == ZippySensor0RxUUID {
                sensor0ReceiveCharacteristic = characteristic
            }
            else if characteristic.uuid == ZippySensor1RxUUID {
                sensor1ReceiveCharacteristic = characteristic
            }
            else if characteristic.uuid == ZippyComputedDataRxUUID {
                computedDataReceiveCharacteristic = characteristic
            }
        }
        
        //step #5; connection is complete; notify our delegate
        if sendCharacteristic != nil && sensor0ReceiveCharacteristic != nil &&
            sensor1ReceiveCharacteristic != nil && computedDataReceiveCharacteristic != nil {
//            print("Connected to Zippy.")
            self.peripheral.setNotifyValue(true, for: sensor0ReceiveCharacteristic!)
            self.peripheral.setNotifyValue(true, for: sensor1ReceiveCharacteristic!)
            self.peripheral.setNotifyValue(true, for: computedDataReceiveCharacteristic!)
        }
        else {
            print("Failed to find all desired characteristics.")
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didUpdateNotificationStateFor characteristic: CBCharacteristic, error: Error?) {
        if error != nil {
            print("Received error for peripheral '", peripheral.name!, "' on characteristic '", characteristic, "': ", error!)
        }
    }
    
    func extractSensorData(values: [UInt8], sensorData: inout SensorData) {
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
    
    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        switch (characteristic) {
        case sensor0ReceiveCharacteristic!:
            let values = [UInt8](characteristic.value!)
            extractSensorData(values: values, sensorData: &sensor0Data)
            DispatchQueue.main.async(execute: {
                self.delegate?.sensor0DataUpdated(xSyncTicks: self.sensor0Data.xSyncTicks,
                                                  xSweepTicks: self.sensor0Data.xSweepTicks,
                                                  xPosition: self.sensor0Data.xPosition,
                                                  ySyncTicks: self.sensor0Data.ySyncTicks,
                                                  ySweepTicks: self.sensor0Data.ySweepTicks,
                                                  yPosition: self.sensor0Data.yPosition)
            })
            break
        case sensor1ReceiveCharacteristic!:
            let values = [UInt8](characteristic.value!)
            extractSensorData(values: values, sensorData: &sensor1Data)
            DispatchQueue.main.async(execute: {
                self.delegate?.sensor1DataUpdated(xSyncTicks: self.sensor1Data.xSyncTicks,
                                                  xSweepTicks: self.sensor1Data.xSweepTicks,
                                                  xPosition: self.sensor1Data.xPosition,
                                                  ySyncTicks: self.sensor1Data.ySyncTicks,
                                                  ySweepTicks: self.sensor1Data.ySweepTicks,
                                                  yPosition: self.sensor1Data.yPosition)
            })
            break
        case computedDataReceiveCharacteristic!:
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
    
}

