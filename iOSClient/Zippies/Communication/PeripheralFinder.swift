
import Foundation
import CoreBluetooth

protocol PeripheralFinderDelegate: class
{
    func peripheral(_ peripheral: CBPeripheral,
                    didDiscover advertisementData: [String : Any])
    
    func peripheral(_ peripheral: CBPeripheral,
                    didConnect service: CBService,
                    characteristics: [CBUUID: CBCharacteristic])
    
    func peripheral(didDisconnect peripheral: CBPeripheral)
}

class PeripheralFinder: NSObject
{
    var delegate: PeripheralFinderDelegate?
    
    fileprivate var centralManager: CBCentralManager?
    
    fileprivate var advertisedServiceUUID: CBUUID?
    fileprivate var primaryServiceUUID: CBUUID?
    fileprivate var characteristicUUIDs: [CBUUID]?
    
    fileprivate var discoveredPeripherals = Set<CBPeripheral>()
    fileprivate var servicesByPeripheral = [CBPeripheral: CBService]()
    
    override init() {
        super.init()
    }
    
    func startObserving(_ primaryServiceUUID: CBUUID)
    {
        stopDiscovery()
        
        self.advertisedServiceUUID = primaryServiceUUID
        self.primaryServiceUUID = primaryServiceUUID
        self.characteristicUUIDs = nil

        if centralManager == nil {
            self.centralManager = CBCentralManager(delegate: self, queue: nil)
        }
        
        if self.centralManager!.state == CBManagerState.poweredOn {
            if self.centralManager!.isScanning {
                self.centralManager?.stopScan()
            }
            
            //start scanning
            self.centralManager!.scanForPeripherals(withServices: [self.advertisedServiceUUID!], options: nil)
        }
    }
    
    func startDiscovery(_ primaryServiceUUID: CBUUID,
                        characteristicUUIDs: [CBUUID])
    {
        startDiscovery(primaryServiceUUID,
                       primaryServiceUUID: primaryServiceUUID,
                       characteristicUUIDs: characteristicUUIDs)
    }
    
    func startDiscovery(_ advertisedServiceUUID: CBUUID,
                        primaryServiceUUID: CBUUID,
                        characteristicUUIDs: [CBUUID])
    {
        stopDiscovery()
        
        self.advertisedServiceUUID = advertisedServiceUUID
        self.primaryServiceUUID = primaryServiceUUID
        self.characteristicUUIDs = characteristicUUIDs
        
        if centralManager == nil {
            self.centralManager = CBCentralManager(delegate: self, queue: nil)
        }

        if self.centralManager!.state == CBManagerState.poweredOn {
            if self.centralManager!.isScanning {
                self.centralManager?.stopScan()
            }
            
            //start scanning
            self.centralManager!.scanForPeripherals(withServices: [self.advertisedServiceUUID!], options: nil)
        }
    }
    
    func stopDiscovery() {
        guard self.centralManager != nil && self.centralManager!.isScanning else {
            return
        }
        
        //stop scanning for new peripherals
        self.centralManager!.stopScan()
        self.primaryServiceUUID = nil
        self.characteristicUUIDs = nil
        self.discoveredPeripherals.removeAll()
        self.servicesByPeripheral.removeAll()
    }

    deinit {
        stopDiscovery()
        centralManager?.delegate = nil
        centralManager = nil
    }
    
}

extension PeripheralFinder: CBCentralManagerDelegate
{
    
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        switch (central.state) {
        case .poweredOn:
            //step #1. Bluetooth is now on. scan for Zippies
            print("Bluetooth powered on.")
//            self.centralManager!.scanForPeripherals(withServices: [advertisedServiceUUID!], options: nil)
            self.centralManager!.scanForPeripherals(withServices: [advertisedServiceUUID!],
                                                    options: [CBCentralManagerScanOptionAllowDuplicatesKey:true])
            break
        case .resetting:
            print("Bluetooth resetting.")
            //we'll have to restart the scan when it powers back on
            self.discoveredPeripherals.removeAll()
            break
        case .poweredOff:
            print("Bluetooth powered off.")
            self.discoveredPeripherals.removeAll()
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
    
    func centralManager(_ central: CBCentralManager,
                        didDiscover peripheral: CBPeripheral,
                        advertisementData: [String : Any],
                        rssi RSSI: NSNumber)
    {
        self.delegate?.peripheral(peripheral, didDiscover: advertisementData)

        //retain the peripheral first
//        discoveredPeripherals.insert(peripheral)
        
        //step #2. connect to this peripheral
//        central.connect(peripheral, options: nil)
    }
    
    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        //a periphal connected. find the primary service
        //TODO: short-circuit straight to characteristic discovery if the primary service already exists on the peripheral
        peripheral.delegate = self
        peripheral.discoverServices([self.primaryServiceUUID!])
    }
    
    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        //a peripheral disconnected
        if self.servicesByPeripheral.removeValue(forKey: peripheral) == nil {
            //it's a peripheral we already handed to the user as "connected", so notify of a disconnect
            self.discoveredPeripherals.remove(peripheral)
            DispatchQueue.main.async(execute: {
                self.delegate?.peripheral(didDisconnect: peripheral)
            })
        }
    }
    
}

extension PeripheralFinder: CBPeripheralDelegate
{
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        guard error == nil else {
            print("Received unexpected bluetooth error during service discovery.")
            self.discoveredPeripherals.remove(peripheral)
            return
        }
        
        guard peripheral.services != nil && peripheral.services!.count != 0 else {
            //no services
            self.discoveredPeripherals.remove(peripheral)
            return
        }
        
        for service in peripheral.services! {
            if service.uuid == primaryServiceUUID {
                //step #4. discover characteristics
                servicesByPeripheral[peripheral] = service
                peripheral.discoverCharacteristics(self.characteristicUUIDs, for: service)
            }
        }
        
        //remove the peripheral from the list of discovered peripherals if the primary service was not found
        if !self.servicesByPeripheral.keys.contains(peripheral) {
            self.discoveredPeripherals.remove(peripheral)
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        //no matter what we do after this point, there is no need for us to continue retaining this data, since
        //it has all been collected to be passed along to the delegate
        self.servicesByPeripheral.removeValue(forKey: peripheral)
        self.discoveredPeripherals.remove(peripheral)
        
        guard error == nil else {
            print("Received unexpected bluetooth error during characteristic discovery.")
            return
        }
        
        guard service.characteristics != nil else {
            return
        }
        
        var characteristicsByUUID = [CBUUID: CBCharacteristic]()
        for characteristic in service.characteristics! {
            if self.characteristicUUIDs!.contains(characteristic.uuid) {
                characteristicsByUUID[characteristic.uuid] = characteristic
                if characteristicsByUUID.count == self.characteristicUUIDs!.count {
                    break
                }
            }
        }
        
        //step #5. connection is complete. notify our delegate
        DispatchQueue.main.async(execute: {
            self.delegate?.peripheral(peripheral, didConnect: service, characteristics: characteristicsByUUID)
        })
    }
    
//    func peripheral(_ peripheral: CBPeripheral, didDiscoverDescriptorsFor characteristic: CBCharacteristic, error: Error?) {
//        print("Received descriptors")
//    }

    func peripheral(_ peripheral: CBPeripheral, didUpdateNotificationStateFor characteristic: CBCharacteristic, error: Error?) {
        if error != nil {
            print("Received error for peripheral '", peripheral.name!, "' on characteristic '", characteristic, "': ", error!)
        }
    }
    
}
