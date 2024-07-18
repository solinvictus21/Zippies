
import Foundation
import CoreBluetooth
import UIKit

let ZippyMotorControlServiceUUID = CBUUID(string: "5BF1CEC2-EFC2-44D2-81AE-73FCFD5F7A13")
let ZippyMotorControlCharacteristicUUID = CBUUID(string: "C9589239-D751-47F7-9C5B-38355F0E9811")
let ZippyPositionCharacteristicUUID = CBUUID(string: "7E94E051-995E-4ECB-92C6-12EE64729C4D")

protocol ZippyManagerDelegate: class
{
    func zippyConnected(_ zippy: Zippy)
    func zippyDisconnected(_ zippy: Zippy)
}

class ZippyManager: NSObject
{
    
    var delegate: ZippyManagerDelegate?
    
    fileprivate var centralManager: CBCentralManager?
    fileprivate var discoveredPeripherals = Set<CBPeripheral>()
    fileprivate var zippies = [CBPeripheral : Zippy]()
    
    /**
     * Starts the process of discovering the available Zippies, represented as Bluetooth peripherals.
     */
    func startDiscovery() {
        stopDiscovery()
        
        if centralManager == nil {
            self.centralManager = CBCentralManager(delegate: self, queue: DispatchQueue.main)
        }

        //TODO: handle the situation where the user has Bluetooth powered off
        if self.centralManager!.state == CBManagerState.poweredOn {
            //start scanning
            print("Starting discovery.")
            self.centralManager!.scanForPeripherals(withServices: [ZippyMotorControlServiceUUID],
                                                    options: [CBCentralManagerScanOptionSolicitedServiceUUIDsKey : [ZippyMotorControlServiceUUID]])
        }
    }
    
    func stopDiscovery() {
        if self.centralManager == nil || !self.centralManager!.isScanning {
            return
        }
        
        //stop scanning for new peripherals
        self.centralManager!.stopScan()
    }
    
    fileprivate func zippyDiscoveryComplete(_ peripheral: CBPeripheral,
                                            motorControlService: CBService,
                                            motorControlCharacteristic: CBCharacteristic,
                                            positionCharacteristic: CBCharacteristic)
    {
        print("Completed Zippy discovery.")
        let zippy = Zippy(peripheral,
                          motorControlService: motorControlService,
                          motorControlCharacteristic: motorControlCharacteristic,
                          positionCharacteristic: positionCharacteristic)
        zippies[peripheral] = zippy
        delegate?.zippyConnected(zippy)
    }
    
    fileprivate func reset() {
        //iterate the dictionary of zippies and remove them
        for (nextPeripheral, nextZippy) in zippies {
            nextPeripheral.delegate = nil
            nextZippy.delegate = nil
            delegate?.zippyDisconnected(nextZippy)
        }
        zippies.removeAll()
        discoveredPeripherals.removeAll()
    }
    
    deinit {
        stopDiscovery()
        centralManager?.delegate = nil
        centralManager = nil
    }
    
}

extension ZippyManager: CBCentralManagerDelegate
{
    
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        switch (central.state) {
        case .poweredOn:
            //step #1. Bluetooth is now on. scan for Zippies
//            print("Bluetooth powered on. Starting discovery")
//            self.centralManager!.scanForPeripherals(withServices: [advertisedServiceUUID!], options: nil)
            self.startDiscovery()
            break
        case .resetting:
//            print("Bluetooth resetting.")
            //we'll have to restart the scan when it powers back on
            self.reset()
            break
        case .poweredOff:
//            print("Bluetooth powered off.")
            self.reset()
            break
        case .unauthorized:
//            print("Bluetooth unauthorized.")
            break
        case .unsupported:
//            print("Bluetooth unsupported.")
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
//        print("Peripheral discovered.")
        discoveredPeripherals.insert(peripheral)
        //now try to connect to it
        peripheral.delegate = self
        central.connect(peripheral, options: nil)
    }

    //by the time we reach the point of connecting, all services and characteristics have been discovered and the Zippy object
    //has already been created to represent this peripheral
    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        attemptServiceDiscovery(peripheral)
    }

    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        //a peripheral disconnected
        print("Zippy got disconnected")
        peripheral.delegate = nil
        discoveredPeripherals.remove(peripheral)

        guard let removedZippy = zippies.removeValue(forKey: peripheral) else {
            //peripheral disconnected before all services and characteristics were discovered
            return
        }

        //notify the delegate that the Zippy has disconnected; requires mapping from peripheral to Zippy; remove mapping after
        removedZippy.delegate = nil
        self.delegate?.zippyDisconnected(removedZippy)
    }
    
}

/**
 * The CBPeripheralDelegate is only used by the ZippyManager during the setup phase while discovering the necessary services and characteristics. Once those
 * have been discovered, the Zippy instance is created and delegate responsibility is handled by the Zippy o
 */
extension ZippyManager: CBPeripheralDelegate
{
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        guard error == nil else {
            discoveredPeripherals.remove(peripheral)
            print("Received unexpected bluetooth error during service discovery.")
            return
        }
        
        guard let services = peripheral.services else {
            discoveredPeripherals.remove(peripheral)
            print("Failed to discover characteristics.")
            return
        }
        
        completeServiceDiscovery(peripheral,
                                 services: services)
    }
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        //no matter what we do after this point, there is no need for us to continue retaining this data, since
        //it has all been collected to be passed along to the delegate
        guard error == nil else {
            discoveredPeripherals.remove(peripheral)
            print("Received unexpected bluetooth error during characteristic discovery.")
            return
        }
        
        guard let characteristics = service.characteristics else {
            discoveredPeripherals.remove(peripheral)
            print("Failed to discover characteristics.")
            return
        }
        
        completeCharacteristicDiscovery(peripheral,
                                        motorControlService: service,
                                        characteristics: characteristics)
    }
    
    fileprivate func attemptServiceDiscovery(_ peripheral: CBPeripheral) {
        guard let services = peripheral.services else {
            print("Starting service discovery.")
            peripheral.discoverServices([ZippyMotorControlServiceUUID])
            return
        }

        //service have already been discovered
        print("Found services without discovery.")
        completeServiceDiscovery(peripheral,
                                 services: services)
    }
    
    fileprivate func completeServiceDiscovery(_ peripheral: CBPeripheral,
                                              services: [CBService])
    {
        guard let motorControlService = findMotorService(peripheral.services!) else {
            discoveredPeripherals.remove(peripheral)
            print("Failed to discover all services.")
            return
        }

        //done with services; now find the characteristics
        attemptCharacteristicDiscovery(peripheral,
                                       motorControlService: motorControlService)
    }
    
    fileprivate func attemptCharacteristicDiscovery(_ peripheral: CBPeripheral,
                                                    motorControlService: CBService)
    {
        guard let characteristics = motorControlService.characteristics else {
            //characteristics have not yet been discovered
            print("Starting characteristic discovery.")
            peripheral.discoverCharacteristics([ZippyMotorControlCharacteristicUUID, ZippyPositionCharacteristicUUID], for: motorControlService)
            return
        }
        
        print("Found characterics without discovery.")
        completeCharacteristicDiscovery(peripheral,
                                        motorControlService: motorControlService,
                                        characteristics: characteristics)
    }

    fileprivate func completeCharacteristicDiscovery(_ peripheral: CBPeripheral,
                                                     motorControlService: CBService,
                                                     characteristics: [CBCharacteristic])
    {
        guard let (motorControlCharacteristic, positionCharacteristic) = findCharacteristics(characteristics) else {
            discoveredPeripherals.remove(peripheral)
            print("Failed to discover all characteristics.")
            return
        }
        
        //found all the services and characteristics
        zippyDiscoveryComplete(peripheral,
                               motorControlService: motorControlService,
                               motorControlCharacteristic: motorControlCharacteristic,
                               positionCharacteristic: positionCharacteristic)
    }
    
    fileprivate func findMotorService(_ services: [CBService]) -> CBService? {
        for service in services {
            if service.uuid == ZippyMotorControlServiceUUID {
                return service
            }
        }
        return nil
    }
    
    fileprivate func findCharacteristics(_ characteristics: [CBCharacteristic]) -> (CBCharacteristic, CBCharacteristic)? {
        var motorControlCharacteristic: CBCharacteristic? = nil
        var positionCharacteristic: CBCharacteristic? = nil
        for characteristic in characteristics {
            if characteristic.uuid == ZippyMotorControlCharacteristicUUID {
                if positionCharacteristic != nil {
                    return (characteristic, positionCharacteristic!)
                }
                
                //keep looking for the others
                motorControlCharacteristic = characteristic
            }
            else if characteristic.uuid == ZippyPositionCharacteristicUUID {
                if motorControlCharacteristic != nil {
                    return (motorControlCharacteristic!, characteristic)
                }
                
                //keep looking for the others
                positionCharacteristic = characteristic
            }
        }
        //only succeed if both characteristics are found
        return nil
    }
    
}
