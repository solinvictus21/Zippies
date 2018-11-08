
import Foundation
import CoreBluetooth

protocol ZippyManagerDelegate: class
{
    func zippyDiscovered(_ zippy: Zippy2)
    
    func zippyConnected(_ zippy: Zippy2)
    func zippyDisconnected(_ zippy: Zippy2)
}

class ZippyManager: NSObject
{
    
    var delegate: ZippyManagerDelegate?
    
    fileprivate var peripheralFinder = PeripheralFinder()
    fileprivate var _zippies = [UUID : Zippy2]()
    
    var zippies : [UUID : Zippy2]!
    {
        get
        {
            return _zippies
        }
    }
    
    /*
    fileprivate var zippies = [CBPeripheral: Zippy2]()
    fileprivate var lighthouse: Lighthouse?
    */

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
    
    /*
    func anyZippy() -> Zippy? {
        return zippies.first?.value
    }
    */
    
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
                    didDiscover advertisementData: [String : Any])
    {
        let id = peripheral.identifier;
        let values = [UInt8](advertisementData[CBAdvertisementDataManufacturerDataKey] as! Data)
        let zippyX = extractFloat32(values, position: 0)
        let zippyY = extractFloat32(values, position: 4)
        let zippyO = extractFloat32(values, position: 8)
//        print("Got update: X=", zippyX, ", Y=", zippyY)

        DispatchQueue.main.async(execute: {
            if let zippy = self._zippies[id]
            {
                zippy.setPosition(x: zippyX, y: zippyY, o: zippyO)
                self.delegate?.zippyDiscovered(zippy)
            }
            else
            {
                let zippy = Zippy2(id, x: zippyX, y: zippyY, o: zippyO)
                self._zippies[id] = zippy
                self.delegate?.zippyDiscovered(zippy)
            }
        })

        /*
        guard let serviceUUIDs = advertisementData[CBAdvertisementDataServiceUUIDsKey] as? [CBUUID] else
        {
            return
        }
        
//        print(peripheral.identifier.uuidString);
//        print(advertisementData.count)
        let values = [UInt8](advertisementData[CBAdvertisementDataManufacturerDataKey] as! Data)
        print(serviceUUIDs.first!.uuidString, ", X=", extractFloat32(values, position: 0), ", Y=", extractFloat32(values, position: 4))
        */
    }
    
    func peripheral(_ peripheral: CBPeripheral,
                    didConnect service: CBService,
                    characteristics: [CBUUID: CBCharacteristic])
    {
        /*
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
        */
    }
    
    func peripheral(didDisconnect peripheral: CBPeripheral) {
        /*
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
        */
    }
    
    func extractFloat32(_ data: [UInt8],
                        position: Int) -> Double
    {
        var bitPatternU32 = UInt32(data[position+3]) << 24
        bitPatternU32 |= UInt32(data[position+2]) << 16
        bitPatternU32 |= UInt32(data[position+1]) << 8
        bitPatternU32 |= UInt32(data[position])
        return Double(Float32(bitPattern: bitPatternU32))
    }
    
}
