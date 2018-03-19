# Parts Required For Body Assembly

Nearly all parts required to build a Zippy are included in the 3D models in this repository, with a few minor additions noted below.

![Body Parts](https://solinvictus21.github.io/images/BodyParts.png)

## 3D-Printed Items

The parts are all modeled using [SketchUp](https://www.sketchup.com/), inspected for 3D printing solidity with the [Solid Inspector² extension](https://extensions.sketchup.com/en/content/solid-inspector%C2%B2), and exported to STL files using [this excellent STL exporter extension](https://extensions.sketchup.com/en/content/sketchup-stl).

I currently slice these models with [Ultimaker Cura](https://ultimaker.com/en/products/ultimaker-cura-software) and print them with ABS on a [Monoprice Select Mini V2](https://www.monoprice.com/product?p_id=21711) 3D printer, but other slicers, materials, and 3D printers should work just fine. I print all parts with a raft to prevent compression of the bottom layers of each part. You will likely need to adjust 3D print settings during slicing for some of the parts to ensure they print properly as noted for each part.

### Body Bottom (1x)

The container for the motors. It is separated from the rest of the body to ease the assembly process. Notable features include (1) a "pillow" on the bottom to prevent corners of the model from snagging on irregularities on the surface as it moves around, (2) annular snap fits on the top to attach it to the rest of the body, and (3) holes for the screws to firmly attach the motor.

I recommend printing this part with supports so that the bottom pillow forms cleanly, and after removing the raft and supports, finish the bottom with a bit of acetone to ensure a smooth surface.

### Body (1x)

The main/center portion of the body and is the housing for the various circuit boards. Notable features include (1) four insets along the bottom to connect to the annular snap fits on the body bottom, (2) brackets inside near the front to keep the TinyScreen in place, (3) holes on the left side to allow access to the TinyScreen power switch and reset button, and (4) insets on the inside left and right to connect to the cantilever snap fits on the body top.

This part likely does not require printing with supports. The overhangs for the insets on the bottom are small enough that they should bridge perfectly fine during printing without supports.

### Body Top (1x)

As the name implies, this is the top that attaches to the body to fully enclose the Zippy. Notable features include (1) two cantilever snap fits on either side to connect the body top to the body, (2) lugs on the back corners to ensure proper alignment with the body when attached, (3) two holes for the infrared sensors from the Lighthouse circuit, and (4) a hole for the USB port to allow reprogramming of the Zippy without requiring the top to be removed.

### Wheel Motor Gears (2x)

The gears which attach to the axles on the two motors. Note that the gear is actually modeled as a proper "involute gear" and that the center hole is D-shaped to ensure tight attachment to the motor axles.

### Wheels (2x)

The name is self-explanatory. The important thing to note about this part is that the axle of the wheel and the outer rim of the wheel must be printed together even though they are technically modeled as two separate parts. That's because the axle has two endcaps to hold the rim in place which are larger than the hole in the rim for the central rod of the axle, a rather ingenious idea that I stole from another wheel model I found while randomly browsing around Thingiverse. Once the part is printed and the raft is fully removed, the axle should be able to turn within the rim, allowing the wheel to rotate once attached to the body. I even recommend applying a small amount of a light oil-based lubricant to this part after it is printed and cleaned up, to ensure that the outer rim turns freely around the axle.

## Accessories

In addition to the 3D printed parts, you'll also require the following accessories to complete assembly.

- 4x [M1.6 x 3mm screws](https://www.amazon.com/gp/product/B071DXG8D4/ref=oh_aui_detailpage_o09_s00?ie=UTF8&psc=1) used to attach the motors firmly to the bottom of the body.
- 2x [5mm x 1.5mm O-Rings](https://www.amazon.com/gp/product/B0180EQC22/ref=oh_aui_search_detailpage?ie=UTF8&psc=1) used as the "tires" that attach to the 3D-printed wheels.

