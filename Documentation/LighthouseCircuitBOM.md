# Lighthouse Sensor Circuit

This circuit is designed to provide sensors that detect IR signals from the HTC Vive Lighthouse in order to provide localized positioning for the Zippy. The board layout is published on [Osh Park](https://oshpark.com/shared_projects/vZVPRIY6) for easy ordering. All you have to do is solder the components onto it. The BOM and links to Digikey components are provided below.

1. 2x [BP104-FASZ](https://www.digikey.com/product-detail/en/osram-opto-semiconductors-inc/BP-104-FAS-Z/475-1344-1-ND/1227850) These are the IR sensors, the eyes of the circuit
2. 1x [TLV2464ID](https://www.digikey.com/product-detail/en/texas-instruments/TLV2464ID/296-10608-5-ND/380874) The heart of the circuit: a fast and reliable quad op-amp from TI.
3. First phase: voltage divider.
  - 2x [27k resistors](https://www.digikey.com/product-detail/en/panasonic-electronic-components/ERJ-3EKF2702V/P27.0KHCT-ND/1746753)
  - 2x [100nF capacictors](https://www.digikey.com/product-detail/en/samsung-electro-mechanics/CL10B104JB8NNNC/1276-1033-1-ND/3889119)
4. Second phase: transimpedance amplification
  - 2x [2.2pF capacitcors](https://www.digikey.com/product-detail/en/samsung-electro-mechanics/CL10C2R2BB8NNNC/1276-1084-1-ND/3889170)
  - 4x [100k resistors](https://www.digikey.com/product-detail/en/panasonic-electronic-components/ERJ-PA3J104V/P100KBZCT-ND/5036238) Also used in fourth-phase inverting amplification.
5. Third phase: high-pass filter.
  - 2x [4.7nf capacitors](https://www.digikey.com/product-detail/en/samsung-electro-mechanics/CL10B472JB8NNNC/1276-2061-1-ND/3890147)
  - 2x [18k resistors](https://www.digikey.com/product-detail/en/panasonic-electronic-components/ERJ-3EKF1802V/P18.0KHCT-ND/1746738)
6. Fourth phase: inverting amplification
  - 2x [1k resistors](https://www.digikey.com/product-detail/en/panasonic-electronic-components/ERJ-PB3D1001V/P20283CT-ND/6214538)

The passives can be soldered onto the board in any orientation, but pay particular attention to the orientation of the op-amp and diodes. Schematic diagram follows below if you're not an Eagle user.

![Lighthouse Sensor Schematic](https://solinvictus21.github.io/images/LighthouseSensorSchematic.png)

This is essentially a mash-up and a doubling of the circuit I originally started with from [Alex Shtuchkin's project](https://github.com/ashtuchkin/vive-diy-position-sensor) and [another circuit](https://trmm.net/Lighthouse) tweeted about by Alan Yates along with [some lessons I've learned about op-amps](http://www.radio-electronics.com/info/circuits/opamp_non_inverting/op_amp_non-inverting.php) along the way and a LOT of experimentation. At this point, the circuit seems to be fairly reliable across the range of distances that I experiment with.

For the curious, it has been suggested that, if you only have one Lighthouse, you must have three sensors to determine orientation along a 2D plane. Not true, actually. To do it with only two sensors on a robot that does not move in the up/down direction (assuming your sensors are equally-spaced on either side of the robot like this one), then you can just cross the vector between the two sensors with the down vector (0,0,-1) to get the orientation.
