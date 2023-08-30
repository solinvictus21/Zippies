
import Foundation

class CompositePath: ZPath
{

    fileprivate let paths: [ZPath]
    fileprivate let pathCount: Int32
    fileprivate let totalLength: Double

    init(_ p1: ZPath, _ p2: ZPath) {
        paths = [p1, p2]
        pathCount = 2
        totalLength = p1.getLength() + p2.getLength()
    }
    
    func updatesPosition() -> Bool { return true }
    func getLength() -> Double { return totalLength }
    /*
    func getDrawable(_ color: UIColor) -> ZDrawable {
        return ZDrawablePath(color, paths)
    }
    */

    func interpolate(
        _ normalizedTime: Double,
        _ targetPosition: ZMatrix2)
    {
        var distance = normalizedTime * totalLength
        var currentPathIndex = 0
        while currentPathIndex < pathCount-1 && distance >= paths[currentPathIndex].getLength() {
            distance -= paths[currentPathIndex].getLength()
            currentPathIndex += 1
        }

        //moving through first arc
        paths[currentPathIndex].interpolate(
            distance / paths[currentPathIndex].getLength(),
            targetPosition)
    }
    
}
