
import Foundation
import CoreBluetooth

//base station ID is 1956086213, hardcoded for now because the only way I currently know how to get it is from the infrared pulses
let LighthouseAdvertisedUUID = CBUUID(string: "CB00")
//firmware upgrade service
//let LighthouseServiceUUID = CBUUID(string: "9E5D1E47-5C13-43A0-8635-82AD38A1386F")
//firmware upgrade control point characteristic
//let LighthouseReadWriteUUID = CBUUID(string: "E3DD50BF-F7A7-4E99-838E-570A086C666B")
//firmware upgrade data characteristic
//let LighthouseReadWriteUUID = CBUUID(string: "92E86C7A-D961-4091-B74F-2409E72EFE36")
let LighthouseServiceUUID = CBUUID(string: "CB00")
let LighthouseReadWriteUUID = CBUUID(string: "CB01")
let LighthouseBaseStationID: UInt32 = 0x749781C5

protocol LighthouseDelegate: class
{
    
}

class Lighthouse: BluetoothPeripheral
{
    
    var delegate: ZippyDelegate?
    
    var sendCharacteristic: CBCharacteristic

    override init(_ peripheral: CBPeripheral,
                  primaryService: CBService,
                  characteristics: [CBUUID: CBCharacteristic])
    {
        self.sendCharacteristic = characteristics[LighthouseReadWriteUUID]!

        super.init(peripheral,
                   primaryService: primaryService,
                   characteristics: characteristics)
        
        self.peripheral.delegate = self
        self.peripheral.setNotifyValue(true, for: sendCharacteristic)
        
        sendLighthouseTimeout()
    }
    
    func sendLighthouseTimeout() {
        //set Lighthouse timeout to 300 seconds
        self.peripheral.writeValue(Data(bytes: [
            //unkown data
            0x12, 0x02,
            //big-endian, 16-bit integer
            0x01, 0x2C,
            //the base station device identifier
            0x74, 0x97, 0x81, 0xC5,
            //padding to make up a total of 20 bytes
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
                                   for: self.sendCharacteristic,
                                   type: CBCharacteristicWriteType.withResponse)
    }
    
    func wakeLighthouse() {
        //wake up the Lighthouse
        self.peripheral.writeValue(Data(bytes: [
            //unkown data
            0x00, 0x02,
            //big-endian, 16-bit integer
            0x01, 0x2C,
            //the base station device identifier
            0x74, 0x97, 0x81, 0xC5,
            //padding to make up a total of 20 bytes
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
                                   for: self.sendCharacteristic,
                                   type: CBCharacteristicWriteType.withResponse)
    }
    
    func requestFirmwareVersion() {
        
    }

    deinit {
        
    }
}

extension Lighthouse: CBPeripheralDelegate
{
    
    func peripheral(_ peripheral: CBPeripheral,
                    didWriteValueFor characteristic: CBCharacteristic,
                    error: Error?)
    {
        guard error == nil else {
            print("ERROR Bluetooth: ", error!.localizedDescription)
            return;
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral,
                    didUpdateValueFor characteristic: CBCharacteristic,
                    error: Error?)
    {
        guard error == nil else {
            print("ERROR Bluetooth: ", error!.localizedDescription)
            return;
        }
        
        switch (characteristic) {
        case sendCharacteristic:
            break
        default:
            break
        }
    }
    
}
