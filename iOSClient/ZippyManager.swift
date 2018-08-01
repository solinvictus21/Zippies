
import Foundation
import CoreBluetooth

protocol ZippyManagerDelegate: class
{
    func zippyConnected(_ zippy: Zippy)
    func zippyDisconnected(_ zippy: Zippy)
}

class ZippyManager: NSObject
{
    
    var delegate: ZippyManagerDelegate?
    
    fileprivate var peripheralFinder = PeripheralFinder()
    fileprivate var zippies = [CBPeripheral: Zippy]()
    fileprivate var lighthouse: Lighthouse?

    override init() {
        super.init()
        
        self.peripheralFinder.delegate = self
    }
    
    func startObserving() {
        self.peripheralFinder.startObserving(ZippyBroadcastUUID)
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
    }
    
    fileprivate func discoverLighthouse() {
        self.peripheralFinder.stopDiscovery()
        /*
        self.peripheralFinder.startDiscovery(LighthouseAdvertisedUUID,
                                             primaryServiceUUID: LighthouseServiceUUID,
                                             characteristicUUIDs: [LighthouseReadWriteUUID])
        */
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
    }
    
    func stopDiscovery() {
        self.peripheralFinder.stopDiscovery()
    }
    
    deinit {
        stopDiscovery()
        peripheralFinder.delegate = nil
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
            discoverLighthouse()
        }
        else if let zippy = zippies.removeValue(forKey: peripheral) {
            self.delegate?.zippyDisconnected(zippy)
            if zippies.count == 0 {
                discoverZippies()
            }
        }
    }
    
}
