
import Foundation
import CoreBluetooth

class BluetoothPeripheral: NSObject
{
    
    let peripheral: CBPeripheral
    let primaryService: CBService
    let characteristics: [CBUUID: CBCharacteristic]
    
    init(_ peripheral: CBPeripheral,
         primaryService: CBService,
         characteristics: [CBUUID: CBCharacteristic])
    {
        self.peripheral = peripheral
        self.primaryService = primaryService
        self.characteristics = characteristics
        
        super.init()
    }
    
}
