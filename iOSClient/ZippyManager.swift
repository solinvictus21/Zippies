
import Foundation
import CoreBluetooth

//services & characteristics UUIDs
let ZippyServiceUUID = CBUUID(string: "5BF1CEC2-EFC2-44D2-81AE-73FCFD5F7A13")

protocol ZippyManagerDelegate: class
{
    func zippyConnected(_ zippy: Zippy)
    func zippyDisconnected(_ zippy: Zippy)
}

class ZippyManager: NSObject
{
    
    var delegate: ZippyManagerDelegate?
    
    fileprivate var centralManager: CBCentralManager?
    fileprivate var retainedPeripherals = Set<CBPeripheral>()
    fileprivate var zippiesByPeriphal = [CBPeripheral: Zippy]()

    override init() {
        super.init()
    }
    
    func startDiscovery() {
        if centralManager == nil {
            centralManager = CBCentralManager(delegate: self, queue: nil)
        }
        
        //start the bluetooth discovery process
        centralManager!.scanForPeripherals(withServices: [ZippyServiceUUID], options: nil)
    }
    
    func clearDevices() {
        //TODO: iterate the dictionary of zippies and remove them
        /*
         DispatchQueue.main.async(execute: {
         self.delegate?.zippyDisconnected(disconnectedZippy!)
         })
         */
    }
    
    func anyZippy() -> Zippy? {
        return zippiesByPeriphal.first?.value
    }
    
    func stopDiscovery() {
        centralManager?.stopScan()
        centralManager?.delegate = nil
        centralManager = nil
    }
    
    deinit {
        stopDiscovery()
    }
    
}

extension ZippyManager: CBCentralManagerDelegate
{
    
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        switch (central.state) {
        case .poweredOn:
            //step #1; Bluetooth is now on; scan for Zippies
            print("Bluetooth powered on.")
            centralManager!.scanForPeripherals(withServices: [ZippyServiceUUID], options: nil)
            break
        case .resetting:
            print("Bluetooth resetting.")
            self.clearDevices()
            break
        case .poweredOff:
            print("Bluetooth powered off.")
            self.clearDevices()
            break
        case .unauthorized:
            print("Bluetooth unauthorized.")
            break
        case .unsupported:
            print("Bluetooth unsupported.")
            break
        case .unknown:
            break
        }
    }
    
    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String : Any], rssi RSSI: NSNumber) {
        //retain the peripheral first
        retainedPeripherals.insert(peripheral)
        
        //stop scanning for new peripherals
        central.stopScan()
        
        //step #2; connect to this peripheral
        central.connect(peripheral, options: nil)
    }
    
    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        let zippy = Zippy(peripheral: peripheral)
        self.zippiesByPeriphal[peripheral] = zippy
        DispatchQueue.main.async(execute: {
            self.delegate?.zippyConnected(zippy)
        })
    }
    
    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        //a peripheral disconnected
        let zippy = zippiesByPeriphal.removeValue(forKey: peripheral)
        retainedPeripherals.remove(peripheral)
        if zippy != nil {
            DispatchQueue.main.async(execute: {
                self.delegate?.zippyDisconnected(zippy!)
            })
        }
    }
    
}
