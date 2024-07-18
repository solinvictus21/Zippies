
import UIKit
import SwiftUI

let MAX_TRAIL_LENGTH = 100

/**
 * This view is intended to show a "map" of the visible range of the Lighthouse from the Zippy.
 */
@IBDesignable
class ZippyMapView: UIView {

    // An indicator of whether the Lighthouse is currently visible from the Zippy
    private var isLighthouseVisible = false
    
    // A queue of points indicating the trail of the Zippy. The last element in the queue should be the current location.
    private let trail = BasicQueue<ZMatrix2>()
    
    // A set of points indicating the locations where the Zippy lost Lighthouse visibility
    private var visibilityLostPoints = Set<ZMatrix2>()
    
    //a set of points indicating where the Zippy gained Lighthouse visibility
    private var visibilityEstablishedPoints = Set<ZMatrix2>()
    
    func reset() {
        self.isLighthouseVisible = false
        //clear the Zippy trail
        trail.removeAll()
        visibilityLostPoints.removeAll()
        visibilityEstablishedPoints.removeAll()
    }

    func addPoint(_ point: ZMatrix2, isLighthouseVisible: Bool) {
        if self.isLighthouseVisible != isLighthouseVisible {
            if isLighthouseVisible {
                //clear the previous Zippy trail
                trail.removeAll()
                visibilityEstablishedPoints.insert(point)
            }
            else {
                visibilityLostPoints.insert(point)
            }
        }
        self.isLighthouseVisible = isLighthouseVisible

        //add point to trail
        //TODO: only append the next point if it is outside of some threshold of time, distance or orientation from the last point on the trail
        //TODO: only maintain up to some maximum number of points
        if trail.size() > MAX_TRAIL_LENGTH {
            trail.removeFirst()
        }
        
        self.trail.append(point)

        setNeedsDisplay()
    }
    
    override func draw(_ rect: CGRect) {
        guard let context = UIGraphicsGetCurrentContext() else {
            return
        }

        guard let currentPose = trail.peekTail() else {
            return
        }
        
        let scale = self.frame.width / CGFloat(ZIPPY_AREA_WIDTH)
        context.translateBy(x: self.frame.width/2, y: self.frame.height/2)
        context.scaleBy(x: scale, y: -scale)

        UIColor.blue.setFill()
        UIColor.black.setStroke()
        drawPoint(center: currentPose.position, radius: 30.0, lineWidth: 8)
        
        UIColor.red.setFill()
        UIColor.red.setStroke()
        for pose in visibilityLostPoints {
            drawPoint(center: pose.position, radius: 10.0, lineWidth: 2)
        }
    }
    
}

struct ZippyPoint: Hashable {
    
    let x: Float32
    let y: Float32
    let o: Float32
    
    init(x: Float32, y: Float32, o: Float32) {
        self.x = x
        self.y = y
        self.o = o
    }

    func hash(into hasher: inout Hasher) {
        hasher.combine(x)
        hasher.combine(y)
        hasher.combine(o)
    }
    
}

class BasicQueue<Element> {
    
    private var head: BasicQueueEntry<Element>? = nil
    private var tail: BasicQueueEntry<Element>? = nil
    private var _size = 0
    
    func peekHead() -> Element? {
        return head?.value
    }

    func peekTail() -> Element? {
        return tail?.value
    }

    @discardableResult
    func removeFirst() -> Element? {
        if head == nil {
            return nil
        }
        
        let element = head!.value
        head = head!.next
        if head == nil {
            tail = nil
        }
        _size -= 1
        
        return element
    }
    
    func append(_ value: Element) {
        let newEntry = BasicQueueEntry<Element>(value)
        if head == nil {
            //first insertion
            head = newEntry
            tail = head
            _size = 1
            return
        }
        
        //normal insertion
        tail!.next = newEntry
        tail = newEntry
        _size += 1
    }
    
    func removeAll() {
        head = nil
        tail = nil
        _size = 0
    }
    
    func size() -> Int {
        return _size
    }
    
    func isEmpty() -> Bool {
        return head == nil
    }
    
    //TODO: create a method of iteration
    
}

fileprivate class BasicQueueEntry<Element> {
    
    let value: Element
    var next: BasicQueueEntry<Element>? = nil
    
    init(_ value: Element) {
        self.value = value
    }
    
}
