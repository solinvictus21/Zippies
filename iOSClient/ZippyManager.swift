
import Foundation
import CoreBluetooth

//services & characteristics UUIDs

protocol ZippyManagerDelegate: class
{
    func zippyConnected(_ zippy: Zippy)
    func zippyDisconnected(_ zippy: Zippy)
}

class ZippyManager: NSObject
{
    
    var delegate: ZippyManagerDelegate?
    
    fileprivate var peripheralFinder = PeripheralFinder()
//    fileprivate var centralManager: CBCentralManager?
//    fileprivate var retainedPeripherals = Set<CBPeripheral>()
//    fileprivate var zippiesByPeripheral = [CBPeripheral: Zippy]()
    fileprivate var zippies = [CBPeripheral: Zippy]()
    fileprivate var lighthouse: Lighthouse?

    override init() {
        super.init()
        
        self.peripheralFinder.delegate = self
    }
    
    func startDiscovery() {
        discoverZippies()
    }
    
    fileprivate func discoverZippies() {
        self.peripheralFinder.startDiscovery(ZippyServiceUUID,
                                             characteristicUUIDs: [ZippyTxUUID,
                                                                   ZippySensorLeftRxUUID,
                                                                   ZippySensorRightRxUUID,
                                                                   ZippyComputedDataRxUUID])
        /*
        if centralManager == nil {
            centralManager = CBCentralManager(delegate: self, queue: nil)
        }
        
        //start the bluetooth discovery process
//        centralManager!.scanForPeripherals(withServices: [ZippyServiceUUID], options: nil)
         */
    }
    
    fileprivate func discoverLighthouse() {
        self.peripheralFinder.startDiscovery(LighthouseAdvertisedUUID,
                                             primaryServiceUUID: LighthouseServiceUUID,
                                             characteristicUUIDs: [LighthouseReadWriteUUID])
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
        return zippies.first?.value
//        return zippiesByPeripheral.first?.value
    }
    
    func stopDiscovery() {
        self.peripheralFinder.stopDiscovery()
        /*
        centralManager?.stopScan()
        centralManager?.delegate = nil
        centralManager = nil
         */
    }
    
    deinit {
        stopDiscovery()
    }
    
}

extension ZippyManager: PeripheralFinderDelegate
{
    
    func peripheral(_ peripheral: CBPeripheral,
                    didConnect service: CBService,
                    characteristics: [CBUUID: CBCharacteristic])
    {
        if service.uuid == ZippyServiceUUID {
            //found the zippy
            let zippy = Zippy(peripheral,
                              primaryService: service,
                              characteristics: characteristics)
            zippies[peripheral] = zippy
            self.delegate?.zippyConnected(zippy)

            //now discover the lighthouse if we still need it (this could just be a Zippy reconnect)
            if lighthouse == nil {
                discoverLighthouse()
            }
            else {
                stopDiscovery()
            }
        }
        else {
            //found the lighthouse
            self.lighthouse = Lighthouse(peripheral,
                                         primaryService: service,
                                         characteristics: characteristics)
            
            if zippies.count == 0 {
                //if we lost our zippies, rediscover them
                discoverZippies()
            }
            else {
                //stop further discovery
                self.peripheralFinder.stopDiscovery()
            }
        }
    }
    
    func peripheral(didDisconnect peripheral: CBPeripheral) {
        if peripheral == lighthouse?.peripheral {
            //lighthouse disconnected; rediscover it
            self.lighthouse = nil
            self.peripheralFinder.startDiscovery(LighthouseServiceUUID,
                                                 characteristicUUIDs: [LighthouseReadWriteUUID])
        }
        else if let zippy = zippies.removeValue(forKey: peripheral) {
            self.delegate?.zippyDisconnected(zippy)
            if zippies.count == 0 {
                startDiscovery()
            }
        }
    }
    
    /*
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        switch (central.state) {
        case .poweredOn:
            //step #1; Bluetooth is now on; scan for Zippies
            print("Bluetooth powered on.")
            centralManager!.scanForPeripherals(withServices: [LighthouseServiceUUID, ZippyServiceUUID], options: nil)
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
        self.zippiesByPeripheral[peripheral] = zippy
        DispatchQueue.main.async(execute: {
            self.delegate?.zippyConnected(zippy)
        })
    }
    
    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        //a peripheral disconnected
        let zippy = zippiesByPeripheral.removeValue(forKey: peripheral)
        retainedPeripherals.remove(peripheral)
        if zippy != nil {
            DispatchQueue.main.async(execute: {
                self.delegate?.zippyDisconnected(zippy!)
            })
        }
    }
     */
    
}
