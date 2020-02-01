
#include <SPI.h>

#include "zippies/hardware/SensorFusor.h"
#include "zippies/config/BodyConfig.h"

// #define DEBUG_LIGHTHOUSE_SENSOR_SYNC              1

#define LIGHTHOUSE_LOCKED_SIGNAL_PAUSE         2000
// #define LIGHTHOUSE_UNLOCKED_SIGNAL_TIMEOUT      200
// #define SENSOR_OFFSET_X  10.8d
// #define SENSOR_OFFSET_Y   4.2d
//height of the lighthouse from the diode plane
//mounted on surface of entertainment center
// #define LIGHTHOUSE_HEIGHT            908.0d

SensorFusor* currentSensorFusor = NULL;
LighthouseSensorInput leftSensorInput;
LighthouseSensorInput rightSensorInput;

SensorFusor::SensorFusor()
{
  sensors[0].debugNumber = 0;
  sensors[0].sensorInput = &leftSensorInput;
#if SENSOR_COUNT == 2
  sensors[1].debugNumber = 1;
  sensors[1].sensorInput = &rightSensorInput;
#endif
}

void SensorFusor::start()
{
  if (currentSensorFusor != NULL)
    currentSensorFusor->stop();
  currentSensorFusor = this;

  for (int i = 0; i < SENSOR_COUNT; i++)
    sensors[i].restart();
  // sensors[0].restart();
  // sensors[1].restart();

  // previousPositionTimeStamp = 0;
  positionTimeStamp = 0;
  positionLockedTimeStamp = 0;

  if (!receivedLighthouseData) {
    preambleBitCount = 0;
    // preambleFound = false;
    ootxParser.restart();
    currentSignalState = LighthouseSignalState::ReceivingLighthouseData;
  }
  else
    currentSignalState = LighthouseSignalState::AcquiringSyncSignal;

  //configure the timing clock we'll use for counting cycles between IR pules
  setupClock();

  connectPortPinsToInterrupts();

  //setup our external interrupt controller
  setupEIC();

  connectInterruptsToTimer();

  setupTimer();
}

//#define NVM_SW_CALIB_DFLL48M_COARSE_VAL 58
void SensorFusor::setupClock()
{
  /*
  // DFLL default is open loop mode:
  SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_MUL (48000);   // Set to multiply USB SOF frequency (when USB attached)
  uint32_t coarse =( *((uint32_t *)(NVMCTRL_OTP4) + (NVM_SW_CALIB_DFLL48M_COARSE_VAL / 32)) >> (NVM_SW_CALIB_DFLL48M_COARSE_VAL % 32) )
                   & ((1 << 6) - 1);
  if (coarse == 0x3f) {
    coarse = 0x1f;
  }
  // TODO(tannewt): Load this value from memory we've written previously. There
  // isn't a value from the Atmel factory.
  uint32_t fine = 0x1ff;
  SYSCTRL->DFLLVAL.reg =
    SYSCTRL_DFLLVAL_COARSE(coarse) |
    SYSCTRL_DFLLVAL_FINE(fine);
  */

  SYSCTRL->DFLLCTRL.reg =
   // SYSCTRL_DFLLCTRL_WAITLOCK |                     //output clock when DFLL is locked
   // SYSCTRL_DFLLCTRL_BPLCKC |                       //bypass coarse lock
   // SYSCTRL_DFLLCTRL_QLDIS |                        //disable quick lock
    SYSCTRL_DFLLCTRL_CCDIS |                        //disable chill cycle
   // SYSCTRL_DFLLCTRL_STABLE |                       //stable frequency mode; testing indicates this poorly skews detection results
    SYSCTRL_DFLLCTRL_MODE |                         //closed-loop mode
    SYSCTRL_DFLLCTRL_ENABLE;
  while (!SYSCTRL->PCLKSR.bit.DFLLRDY);

  //setup the divisor for the GCLK0 clock source generator
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(0) |                                  //do not divide the input clock (48MHz / 1)
                    GCLK_GENDIV_ID(3);                                    //for GCLK3

//  SYSCTRL->OSC32K.bit.ENABLE = 1;
//  SYSCTRL->OSC32K.reg = SYSCTRL_OSC32K_STARTUP(0x6u);
  //configure GCLK0 and enable it
  REG_GCLK_GENCTRL =
//    GCLK_GENCTRL_IDC |                                   //50/50 duty cycles; optimization when dividing input clock by an odd number
    GCLK_GENCTRL_GENEN |                                 //enable the clock generator
    GCLK_GENCTRL_SRC_DFLL48M |                           //set the clock source to 48MHz
//    GCLK_GENCTRL_SRC_OSC32K |                            //set the clock source to high-accuracy 32KHz clock
//    GCLK_GENCTRL_SRC_XOSC |                              //set the clock source to 32MHz
//    GCLK_GENCTRL_SRC_DPLL32K |                          //set the clock source to 48MHz
//    (0x08 << 8) |
    GCLK_GENCTRL_ID(3);                                  //for GCLK3
  while (GCLK->STATUS.bit.SYNCBUSY);

  //setup the clock output to go to the EIC
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |                                 //enable the clock
                     GCLK_CLKCTRL_GEN_GCLK3 |                             //to send GCLK3
                     GCLK_CLKCTRL_ID_EIC;                                 //to the EIC peripheral

  //setup the clock output to go to EVSYS channel 0
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |                                 //enable the clock
                     GCLK_CLKCTRL_GEN_GCLK3 |                             //to send GCLK3
                     GCLK_CLKCTRL_ID_EVSYS_0;                             //to EVSYS channel 0

  //setup the clock output to go to EVSYS channel 1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |                                 //enable the clock
                     GCLK_CLKCTRL_GEN_GCLK3 |                             //to send GCLK3
                     GCLK_CLKCTRL_ID_EVSYS_1;                             //to EVSYS channel 1

  //setup the clock output to go to EVSYS channel 0
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |                                 //enable the clock
                     GCLK_CLKCTRL_GEN_GCLK3 |                             //to send GCLK3
                     GCLK_CLKCTRL_ID_EVSYS_2;                             //to EVSYS channel 2

  //setup the clock output to go to EVSYS channel 1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |                                 //enable the clock
                     GCLK_CLKCTRL_GEN_GCLK3 |                             //to send GCLK3
                     GCLK_CLKCTRL_ID_EVSYS_3;                             //to EVSYS channel 3

  //setup the clock output to go to the TCC
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |                                 //enable the clock
                     GCLK_CLKCTRL_GEN_GCLK3 |                             //to send GCLK3
                     GCLK_CLKCTRL_ID_TCC0_TCC1;                           //to TCC0 and TCC1

  //wait for synchronization
  while (GCLK->STATUS.bit.SYNCBUSY);
}

void SensorFusor::connectPortPinsToInterrupts()
{
  //enable the PORT subsystem
  PM->APBBMASK.bit.PORT_ = 1;

  //set port A (group 0), pin 21 (PA21, Tinyduino proto board pin IO7) as an input
  PORT->Group[0].DIRCLR.reg = PORT_PA21;      //set port to input direction
  PORT->Group[0].OUTCLR.reg = PORT_PA21;      //pull-down when pull is enabled
  PORT->Group[0].CTRL.reg |= PORT_PA21;       //enable input sampling

  //configure PA21
  PORT->Group[0].PINCFG[21].reg =
    PORT_PINCFG_PULLEN |         //enable pull-down
    PORT_PINCFG_INEN |           //enable input buffering
    PORT_PINCFG_PMUXEN;          //enable pin muxing

  //mux PA21 over to EXTINT5
  PORT->Group[0].PMUX[10].reg = PORT_PMUX_PMUXO(PORT_PMUX_PMUXE_A_Val);

  //set port A (group 0), pin 9 (PA09, Tinyduino proto board pin IO3) as an input
  PORT->Group[0].DIRCLR.reg = PORT_PA09;      //set port to input direction
  PORT->Group[0].OUTCLR.reg = PORT_PA09;      //pull-down when pull is enabled
  PORT->Group[0].CTRL.reg |= PORT_PA09;       //enable input sampling

  //configure PA09
  PORT->Group[0].PINCFG[9].reg =
    PORT_PINCFG_PULLEN |         //enable pull resistor
    PORT_PINCFG_INEN |           //enable input buffering
    PORT_PINCFG_PMUXEN;          //enable pin muxing

  //mux PA09 over to EXTINT9
  PORT->Group[0].PMUX[4].reg = PORT_PMUX_PMUXO(PORT_PMUX_PMUXE_A_Val);
}

void SensorFusor::setupEIC()
{
  //turn on power to the external interrupt controller (EIC)
  PM->APBAMASK.bit.EIC_ = 1;

  //disable the EIC while we configure it
  EIC->CTRL.bit.ENABLE = 0;
  while (EIC->STATUS.bit.SYNCBUSY);

  //right diode interrupt config
//  EIC->CONFIG[1].bit.FILTEN1 = 1;
  //detect both rising and falling edges
  EIC->CONFIG[1].bit.SENSE1 = EIC_CONFIG_SENSE1_HIGH_Val;
  //generate interrupts on interrupt #9 when edges are detected
  EIC->EVCTRL.bit.EXTINTEO9 = 1;

  //left diode interrupt config
//  EIC->CONFIG[0].bit.FILTEN5 = 1;
  //detect both rising and falling edges
  EIC->CONFIG[0].bit.SENSE5 = EIC_CONFIG_SENSE1_HIGH_Val;
  //generate interrupts on interrupt #5 when edges are detected
  EIC->EVCTRL.bit.EXTINTEO5 = 1;

  //enable the EIC
  EIC->CTRL.bit.ENABLE = 1;

  //wait for synchronization
  while (EIC->STATUS.bit.SYNCBUSY);
}

void SensorFusor::connectInterruptsToTimer()
{
  //enable the event subsystem
  PM->APBCMASK.bit.EVSYS_ = 1;

  //input config for right diode
  REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EDGSEL(1) |                           //detect rising edge
//                      EVSYS_CHANNEL_PATH_SYNCHRONOUS |                    //synchronously
                      EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_9) |    //from external interrupt 9
                      EVSYS_CHANNEL_CHANNEL(0);                           //to EVSYS channel 0

  //output config for right diode
  REG_EVSYS_USER = EVSYS_USER_CHANNEL(1) |                                //attach output from channel 0 (n+1)
                   EVSYS_USER_USER(EVSYS_ID_USER_TCC0_MC_0);              //to user (recipient) TCC0, MC0
  while (!EVSYS->CHSTATUS.bit.USRRDY0);

  //input config for right diode
  REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EDGSEL(2) |                           //detect falling edge
//                      EVSYS_CHANNEL_PATH_SYNCHRONOUS |                    //synchronously
                      EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_9) |    //from external interrupt 9
                      EVSYS_CHANNEL_CHANNEL(1);                           //to EVSYS channel 1

  //output config for right diode
  REG_EVSYS_USER = EVSYS_USER_CHANNEL(2) |                                //attach output from channel 1 (n+1)
                   EVSYS_USER_USER(EVSYS_ID_USER_TCC0_MC_1);              //to user (recipient) TCC0, MC1
  while (!EVSYS->CHSTATUS.bit.USRRDY1);

  //input config for left diode
  REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EDGSEL(1) |                           //detect rising edge
//                      EVSYS_CHANNEL_PATH_SYNCHRONOUS |                    //synchronously
                      EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_5) |    //from external interrupt 5
                      EVSYS_CHANNEL_CHANNEL(2);                           //to EVSYS channel 2

  //output config for left diode
  REG_EVSYS_USER = EVSYS_USER_CHANNEL(3) |                                //attach output from channel 2 (n+1)
                   EVSYS_USER_USER(EVSYS_ID_USER_TCC1_MC_0);              //to user (recipient) TCC1, MC0
  while (!EVSYS->CHSTATUS.bit.USRRDY2);

  //input config for left diode
  REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EDGSEL(2) |                           //detect falling edge
//                      EVSYS_CHANNEL_PATH_SYNCHRONOUS |                    //synchronously
                      EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_5) |    //from external interrupt 5
                      EVSYS_CHANNEL_CHANNEL(3);                           //to EVSYS channel 3

  //output config for left diode
  REG_EVSYS_USER = EVSYS_USER_CHANNEL(4) |                                //attach output from channel 3 (n+1)
                   EVSYS_USER_USER(EVSYS_ID_USER_TCC1_MC_1);              //to user (recipient) TCC1, MC1

  while (!EVSYS->CHSTATUS.bit.USRRDY3);
}

void SensorFusor::setupTimer()
{
  //enable the TCC0 subsystem
  PM->APBCMASK.bit.TCC0_ = 1;

  //disable TCC0 while we configure it
  REG_TCC0_CTRLA &= ~TCC_CTRLA_ENABLE;

  //configure TCC0
  REG_TCC0_CTRLA =
    TCC_CTRLA_CPTEN0 |              //place MC0 into capture (not compare) mode
    TCC_CTRLA_CPTEN1 |              //place MC1 into capture (not compare) mode
//    TCC_CTRLA_RESOLUTION_DITH4 |
//    TCC_CTRLA_ALOCK |
//    TCC_CTRLA_PRESCSYNC_GCLK |
    TCC_CTRLA_PRESCALER_DIV1;       //set timer prescaler to 1 (48MHz)
//    TCC_CTRLA_CPTEN3 |              //place MC3 into capture (not compare) mode
//    TCC_CTRLA_CPTEN2 |              //place MC2 into capture (not compare) mode

  //set the event control register
  REG_TCC0_EVCTRL =
    TCC_EVCTRL_MCEI0 |              //when MC0 events occur, capture COUNT to CC0
    TCC_EVCTRL_MCEI1;               //when MC1 events occur, capture COUNT to CC1
//    TCC_EVCTRL_MCEI3 |             //when MC3 events occur, capture COUNT to CC3
//    TCC_EVCTRL_MCEI2 |             //when MC2 events occur, capture COUNT to CC2
//    TCC_EVCTRL_TCEI1 |             //enable the event 1 input
//    TCC_EVCTRL_TCEI0 |             //enable the event 0 input
//    TCC_EVCTRL_TCINV1 |             //enable the event 1 inverted input
//    TCC_EVCTRL_TCINV0 |             //enable the event 0 inverted input
//    TCC_EVCTRL_CNTEO |
//    TCC_EVCTRL_TRGEO |
//    TCC_EVCTRL_OVFEO |
//    TCC_EVCTRL_CNTSEL_BOUNDARY |
//    TCC_EVCTRL_EVACT1_RETRIGGER |  //retrigger CC1 on event 1 (each time an edge is detected)
//    TCC_EVCTRL_EVACT0_RETRIGGER;   //retrigger CC0 on event 0 (each time an edge is detected)

  //setup our desired interrupts
  REG_TCC0_INTENSET =
    // TCC_INTENSET_MC0 |              //enable interrupts when a capture occurs on MC0
    TCC_INTENSET_MC1;               //enable interrupts when a capture occurs on MC1
//    TCC_INTENSET_MC3 |            //enable interrupts when a capture occurs on MC3
//    TCC_INTENSET_MC2 |            //enable interrupts when a capture occurs on MC2
//    TCC_INTENSET_CNT |            //enable interrupts for every tick of the counter
//    TCC_INTENSET_OVF |            //enable interrupts on overflow
//    TCC_INTENSET_TRG;             //enable interrupts on retrigger

  //connect the interrupt handler for TCC0
  NVIC_SetPriority(TCC0_IRQn, 1);
  NVIC_EnableIRQ(TCC0_IRQn);

  //enable TCC0
  REG_TCC0_CTRLA |= TCC_CTRLA_ENABLE;

  //wait for TCC0 synchronization
  while (TCC0->SYNCBUSY.bit.ENABLE);

  //enable the TCC1 subsystem
  PM->APBCMASK.bit.TCC1_ = 1;

  //disable TCC1 while we configure it
  REG_TCC1_CTRLA &= ~TCC_CTRLA_ENABLE;

  //configure TCC1
  REG_TCC1_CTRLA =
    TCC_CTRLA_CPTEN0 |              //place MC0 into capture (not compare) mode
    TCC_CTRLA_CPTEN1 |              //place MC1 into capture (not compare) mode
    TCC_CTRLA_PRESCALER_DIV1;       //set timer prescaler to 1 (48MHz)

  REG_TCC1_EVCTRL =
    TCC_EVCTRL_MCEI0 |              //when MC0 events occur, capture COUNT to CC0
    TCC_EVCTRL_MCEI1;               //when MC1 events occur, capture COUNT to CC1

  REG_TCC1_INTENSET =
    // TCC_INTENSET_MC0 |              //enable interrupts when a capture occurs on TCC1/MC0
    TCC_INTENSET_MC1;               //enable interrupts when a capture occurs on TCC1/MC1

  //connect the interrupt handler for TCC1
  NVIC_SetPriority(TCC1_IRQn, 1);
  NVIC_EnableIRQ(TCC1_IRQn);

  //enable TCC1
  REG_TCC1_CTRLA |= TCC_CTRLA_ENABLE;

  //wait for synchronization
  while (TCC1->SYNCBUSY.bit.ENABLE);
}

void TCC0_Handler()
{
  // SerialUSB.println("Got sensor 0 hit.");
  if (TCC0->INTFLAG.bit.MC0) {
    //capture CC0; required regardless of whether we actually use the value in order to reset the interrupt flag
    unsigned int cc0 = REG_TCC0_CC0;

    //make sure the buffer is not full
    if (rightSensorInput.hitTickWritePtr != rightSensorInput.hitTickReadPtr) {
      *rightSensorInput.hitTickWritePtr = cc0;

      //updating this must be atomic, so check if we're at the end first
      if (rightSensorInput.hitTickWritePtr == rightSensorInput.hitTickEndPtr)
        rightSensorInput.hitTickWritePtr = rightSensorInput.hitTickBuffer;
      else
        rightSensorInput.hitTickWritePtr++;
    }
  }

  if (TCC0->INTFLAG.bit.MC1) {
    //capture CC1; required regardless of whether we actually use the value in order to reset the interrupt flag
    unsigned int cc1 = REG_TCC0_CC1;

    //make sure the buffer is not full
    if (rightSensorInput.hitTickWritePtr != rightSensorInput.hitTickReadPtr) {
      *rightSensorInput.hitTickWritePtr = cc1;

      //updating this must be atomic, so check if we're at the end first
      if (rightSensorInput.hitTickWritePtr == rightSensorInput.hitTickEndPtr)
        rightSensorInput.hitTickWritePtr = rightSensorInput.hitTickBuffer;
      else
        rightSensorInput.hitTickWritePtr++;
    }
  }
}

void TCC1_Handler()
{
  // SerialUSB.println("Got sensor 1 hit.");
  if (TCC1->INTFLAG.bit.MC0) {
    //capture CC0; required regardless of whether we actually use the value in order to reset the interrupt flag
    unsigned int cc0 = REG_TCC1_CC0;

    //make sure the buffer is not full
    if (leftSensorInput.hitTickWritePtr != leftSensorInput.hitTickReadPtr) {
      *leftSensorInput.hitTickWritePtr = cc0;

      //updating this must be atomic, so check if we're at the end first
      if (leftSensorInput.hitTickWritePtr == leftSensorInput.hitTickEndPtr)
        leftSensorInput.hitTickWritePtr = leftSensorInput.hitTickBuffer;
      else
        leftSensorInput.hitTickWritePtr++;
    }
  }

  if (TCC1->INTFLAG.bit.MC1) {
    //capture CC1; required regardless of whether we actually use the value in order to reset the interrupt flag
    unsigned int cc1 = REG_TCC1_CC1;

    //make sure the buffer is not full
    if (leftSensorInput.hitTickWritePtr != leftSensorInput.hitTickReadPtr) {
      *leftSensorInput.hitTickWritePtr = cc1;

      //updating this must be atomic, so check if we're at the end first
      if (leftSensorInput.hitTickWritePtr == leftSensorInput.hitTickEndPtr)
        leftSensorInput.hitTickWritePtr = leftSensorInput.hitTickBuffer;
      else
        leftSensorInput.hitTickWritePtr++;
    }
  }
}

bool SensorFusor::loop(unsigned long currentTime)
{
  //process all the sensors
  int sensorsReceivingSyncSignal = 0;
  int sensorsWithUpdate = 0;
  int sensorsReceivingSweepHit = 0;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensors[i].loop(currentTime);
    if (sensors[i].completedCycleTimeStamp) {
      //sensor is still locked onto the Lighthouse sync signals
      sensorsReceivingSyncSignal++;
      if (sensors[i].completedCycleTimeStamp > cycleProcessedTime) {
        //sensor has received a new sweep cycle (combination of X and Z axis sweeps) to be processed
        sensorsWithUpdate++;
        if (sensors[i].completedHitCycles[0].sweepHitStartTicks && sensors[i].completedHitCycles[1].sweepHitStartTicks) {
          //sensor received sweep hits on both the X and Z axes during the most recent sweep cycle; this must be confirmed
          //because it's entirely possible for the sensor to receive sync pulses, which are infrared flashes from flood
          //LEDs, without ever actually seeing the laser sweep across the sensor during the cycle
          sensorsReceivingSweepHit++;
        }
      }
    }
  }

  if (sensorsReceivingSyncSignal != SENSOR_COUNT) {
    //one or more sensors missed the sync pulses for the X and/or Z axes; wait for a sync pulse signal lock again
    // SerialUSB.println("WARNING: Some sensors not receiving signal.");
    currentSignalState = LighthouseSignalState::AcquiringSyncSignal;
    return false;
  }

  if (sensorsWithUpdate != SENSOR_COUNT) {
    //one or more sensors did not yet receive an updated sweep cycle; this is not an error condition; the Lighthouse may
    //just not have completed their sweep cycles by the time we completed this processing loop
    return currentSignalState == LighthouseSignalState::SignalLocked;
  }

  //all sensors have at least received sync pulses for both the X and Z axes; try to fuse the sync pulses together
  //by averaging the pulse widths together
  if (!fuseSyncPulses()) {
    // SerialUSB.println("WARNING: Sensors receiving differing sync pulse numbers.");
    currentSignalState = LighthouseSignalState::AcquiringSyncSignal;
    return false;
  }
  cycleProcessedTime = currentTime;

  //count the number of zero bits received from the sync pulses; used to sync with the preamble upon request via
  //a call to syncWithPreable()
  if (preambleBitCount < 17) {
    processPreambleBit(fusedSyncPulses[0]);
    if (preambleBitCount < 17)
      processPreambleBit(fusedSyncPulses[1]);
  }

  /*
  if (preambleBitCount == 17) {
    SerialUSB.print("Found preamble at: ");
    SerialUSB.println(currentTime);
    preambleBitCount = 0;
  }
 */

  //by the time we reach this point, we know that all sensors have an update on the same sync signal
  //check for the need to transition to a new state
  switch (currentSignalState) {
    case LighthouseSignalState::AcquiringSyncSignal:
      if (!receivedLighthouseData) {
        // SerialUSB.println("Starting receipt of lighthouse data.");
        ootxParser.restart();
        currentSignalState = LighthouseSignalState::ReceivingLighthouseData;
      }
      else {
        //transition to acquiring position
        // SerialUSB.println("Starting acquisition of position lock.");
        positionLockedTimeStamp = 0;
        currentSignalState = LighthouseSignalState::AcquiringSensorHits;
      }
      break;

    case LighthouseSignalState::ReceivingLighthouseData:
      if (ootxParser.receivedBaseStationInfoBlock()) {
        calculateLighthouseData();

        //transition to waiting for sensor hits
        // SerialUSB.println("Starting acquisition of position lock.");
        positionLockedTimeStamp = 0;
        currentSignalState = LighthouseSignalState::AcquiringSensorHits;
      }
      break;

    case LighthouseSignalState::AcquiringSensorHits:
      if (sensorsReceivingSweepHit == SENSOR_COUNT) {
          //we received sweep hits on all sensors for both the X and Z axes during the most recent sweep cycle
          //transition to the initial pause after receiving the first set of sweep hits to allow the signal to
          //"settle" such as, for instance, when the user is in the process of setting the Zippy on the ground
          positionLockedTimeStamp = currentTime;
          currentSignalState = LighthouseSignalState::AwaitingPositionLock;
      }
      break;

    case LighthouseSignalState::AwaitingPositionLock:
      if (sensorsReceivingSweepHit != SENSOR_COUNT) {
        //we're no long receiving sweep hits for both axes on all sensors
        positionLockedTimeStamp = 0;
        currentSignalState = LighthouseSignalState::AcquiringSensorHits;
      }
      else if (currentTime - positionLockedTimeStamp >= LIGHTHOUSE_LOCKED_SIGNAL_PAUSE)
        currentSignalState = LighthouseSignalState::SignalLocked;
      break;

    case LighthouseSignalState::SignalLocked:
      if (sensorsReceivingSweepHit != SENSOR_COUNT) {
        //we're no long receiving sweep hits for both axes on all sensors
        positionLockedTimeStamp = 0;
        currentSignalState = LighthouseSignalState::AcquiringSensorHits;
      }
      break;
  }

  switch (currentSignalState) {
    case LighthouseSignalState::ReceivingLighthouseData:
      // processLighthouseData(currentTime);
      ootxParser.processOOTXBit(fusedSyncPulses[0]);
      ootxParser.processOOTXBit(fusedSyncPulses[1]);
      break;

    case LighthouseSignalState::AwaitingPositionLock:
    case LighthouseSignalState::SignalLocked:
      recalculatePosition(currentTime);
      positionTimeStamp = currentTime;
      break;
  }

  return currentSignalState == LighthouseSignalState::SignalLocked;
}

bool SensorFusor::fuseSyncPulses()
{
  fusedSyncPulses[0] = sensors[0].completedHitCycles[0].syncTicks;
  fusedSyncPulses[1] = sensors[0].completedHitCycles[1].syncTicks;
  unsigned long syncPulseNumberX = SYNC_PULSE_NUMBER(fusedSyncPulses[0]);
  unsigned long syncPulseNumberY = SYNC_PULSE_NUMBER(fusedSyncPulses[1]);
  for (int i = 1; i < SENSOR_COUNT; i++) {
    if (SYNC_PULSE_NUMBER(sensors[i].completedHitCycles[0].syncTicks) != syncPulseNumberX ||
        SYNC_PULSE_NUMBER(sensors[i].completedHitCycles[1].syncTicks) != syncPulseNumberY)
    {
#ifdef DEBUG_LIGHTHOUSE_SENSOR_SYNC
      SerialUSB.println("Lighthouse sensor syncs did not match.");
#endif
      //sensors did not see the same sync pairs; that's a problem; restart the OOTX parsing process
      return false;
    }

    fusedSyncPulses[0] += sensors[i].completedHitCycles[0].syncTicks;
    fusedSyncPulses[1] += sensors[i].completedHitCycles[1].syncTicks;
  }

  fusedSyncPulses[0] /= SENSOR_COUNT;
  fusedSyncPulses[1] /= SENSOR_COUNT;
  /*
  SerialUSB.print("Sync Pulses:  X=");
  SerialUSB.print(fusedSyncPulses[0]);
  SerialUSB.print("  Y=");
  SerialUSB.println(fusedSyncPulses[1]);
  */
  return true;
}

void SensorFusor::recalculatePosition(unsigned long currentTime)
{
  //recalculate the new position
  for (int i = 0; i < SENSOR_COUNT; i++) {
    calculateSensorPosition(
      fusedSyncPulses[0] + sensors[i].completedHitCycles[0].sweepHitStartTicks + (sensors[0].completedHitCycles[0].sweepHitEndTicks / 2),
      fusedSyncPulses[1] + sensors[i].completedHitCycles[1].sweepHitStartTicks + (sensors[0].completedHitCycles[1].sweepHitEndTicks / 2),
      &sensorPositions[i]);
  }

  calculatePosition();
}

void SensorFusor::processPreambleBit(unsigned long syncTickCount)
{
  if (SYNC_PULSE_BIT(syncTickCount))
    preambleBitCount = 0;
  else
    preambleBitCount++;
}

void SensorFusor::calculateSensorPosition(unsigned long xTicks, unsigned long zTicks, KVector2* out)
{
  //Step 1: Calculate the vector from the lighthouse in its reference frame to the diode by normalizing the angle on each axis
  //from the lighthouse to +/- M_PI_2
  double observedAngleX = ((((double)xTicks) / ((double)SWEEP_DURATION_TICK_COUNT)) - 0.5d) * M_PI;
  double observedAngleY = ((((double)zTicks) / ((double)SWEEP_DURATION_TICK_COUNT)) - 0.5d) * M_PI;

  //correct for the factory calibration data; algorithm pulled from notes and code from the open source libsurvive project
  //    https://github.com/cnlohr/libsurvive/wiki/BSD-Calibration-Values
  observedAngleX += xRotor.phase;
  observedAngleY += zRotor.phase;
  double correctionX = sin(xRotor.tilt * observedAngleY);
  double correctionY = sin(zRotor.tilt * observedAngleX);
  observedAngleX += correctionX;
  observedAngleY += correctionY;
  correctionX = xRotor.curve * observedAngleY;
  correctionY = zRotor.curve * observedAngleX;
  observedAngleX += correctionX;
  observedAngleY += correctionY;
  observedAngleX += cos(xRotor.gibbousPhase + observedAngleX) * xRotor.gibbousMagnitude;
  observedAngleY += cos(zRotor.gibbousPhase + observedAngleY) * zRotor.gibbousMagnitude;

  //calculate the normal for the plane of the intersection from the right-to-left sweep, which is a vertical plane whose normal
  //is a vector with no Y component
  double verticalNormalX = cos(observedAngleX);
  double verticalNormalZ = -sin(observedAngleX);

  //calculate the normal for the plane of the intersection from the bottom-to-top sweep, which is a horizontal plane whose normal
  //is a vector with no X component
  // observedAngleZ = -observedAngleZ;
  // double horizontalNormalY = sin(observedAngleY);
  // double horizontalNormalZ = cos(observedAngleY);
  double horizontalNormalY = cos(observedAngleY);
  double horizontalNormalZ = -sin(observedAngleY);

  //now cross the plane of the vertical sweep with the plane of the horizontal sweep and normalize it; this will give us a
  //unit vector which represents the intersection of these two planes in the local coordinate system of the lighthouse
  //this calculation reduces to the following
  /*
  //horizontal X vertical
  // x = yz - zy = yz - z0 = yz
  // y = zx - xz = zx - 0z = zx
  // z = xy - yx = 00 - yx = -yx
  // (yz, zx, -yx)
  KVector3 directionFromLighthouse(
      horizontalNormalY * verticalNormalZ,
      horizontalNormalZ * verticalNormalX,
      -(horizontalNormalY * verticalNormalX),
      1.0d);
  // */
  // /*
  //vertical X horizontal
  // x = yz - zy = 0z - zy = -zy
  // y = zx - xz = z0 - xz = -xz
  // z = xy - yx = xy - 00 = xy
  // (-zy, -xz, xy)
  KVector3 directionFromLighthouse(
      -(verticalNormalZ * horizontalNormalY),
      -(verticalNormalX * horizontalNormalZ),
      verticalNormalX * horizontalNormalY,
      1.0d);
  // */

  //now convert the vector from the lighthouse in its local coordinate system to our global coordinate system
  directionFromLighthouse.unrotate(&lighthouseOrientation);

  //now intersect with the plane of the diodes on the robot; since our diode plane is defined by the normal 0,1,0 and we have a
  //vector which identifies the position of the lighthouse from 0,0,0, the entire formula for our diode-plane intersection reduces
  //to the following
  /*
  double t = -LIGHTHOUSE_CENTER_BODY_TOP_OFFSET_Z / directionFromLighthouse.getZ();
  out->set(
      directionFromLighthouse.getX() * t,
      directionFromLighthouse.getY() * t);
  */
  double t = -LIGHTHOUSE_CENTER_BODY_TOP_OFFSET_Z / directionFromLighthouse.getY();
  out->set(
      -directionFromLighthouse.getX() * t,
      directionFromLighthouse.getZ() * t);
}

void SensorFusor::calculateLighthouseData()
{
  //capture the factory calibration data for the x rotor
  xRotor.phase = ootxParser.getXRotorPhase();
  xRotor.tilt = ootxParser.getXRotorTilt();
  xRotor.curve = ootxParser.getXRotorCurve();
  xRotor.gibbousPhase = ootxParser.getXRotorGibbousPhase();
  xRotor.gibbousMagnitude = ootxParser.getXRotorGibbousMagnitude();

  //capture the factory calibration data for the z rotor
  zRotor.phase = ootxParser.getZRotorPhase();
  zRotor.tilt = ootxParser.getZRotorTilt();
  zRotor.curve = ootxParser.getZRotorCurve();
  zRotor.gibbousPhase = ootxParser.getZRotorGibbousPhase();
  zRotor.gibbousMagnitude = ootxParser.getZRotorGibbousMagnitude();

  //The accelerometer reading from the lighthouse provides a vector that represents the lighthouse orientation vector
  //in a coordinate system where...
  //  X axis = -right to +left
  //  Y axis = -down to +up
  //  Z axis = -bock to +front
  // KVector3 rotationUnitVector(-ootxParser.getAccelDirX(), ootxParser.getAccelDirZ(), ootxParser.getAccelDirY(), 1.0d);
  KVector3 rotationUnitVector(ootxParser.getAccelDirX(), ootxParser.getAccelDirY(), ootxParser.getAccelDirZ(), 1.0d);

  //now calculate the angle of rotation from the "up" normal (0,1,0) to the rotation unit vector
  //this calculation ultimately reduces to the inverse cosine of the Y axis of the rotation unit vector
  //normal dot vector
  // d = xx + yy + zz / |n| * |v| = 0x + yy + 0z = 1y = y
  double angleOfRotation = acos(rotationUnitVector.getY());

  //now cross the "up" vector of the lighthouse with the "up" normal (0,1,0) to obtain the axis of rotation for our quaternion
  // rotationUnitVector.set(rotationUnitVector.getY(), -rotationUnitVector.getX(), 0.0d, 1.0d);
  //normal to vector
  // x = yz - zy = 1z - 0y = z
  // y = zx - xz = 0x - 0z = 0
  // z = xy - yx = 0x - 1x = -x
  // (z, 0, -x)
  // rotationUnitVector.set(rotationUnitVector.getZ(), 0.0d, -rotationUnitVector.getX(), 1.0d);
  //vector to normal
  // x = yz - zy = y0 - z1 = -z
  // y = zx - xz = z0 - x0 = 0
  // z = xy - yx = x1 - y0 = x
  // (-z, 0, x)
  rotationUnitVector.set(-rotationUnitVector.getZ(), 0.0d, rotationUnitVector.getX(), 1.0d);

  //now that we have both the axis and angle of rotation, we can calculate our quaternion
  lighthouseOrientation.set(rotationUnitVector.getX(), rotationUnitVector.getY(), rotationUnitVector.getZ(), angleOfRotation);

#ifdef LIGHTHOUSE_ORIENTATION_Z
  lighthouseOrientation.rotateZ(LIGHTHOUSE_ORIENTATION_Z);
#endif

  receivedLighthouseData = true;
}

void SensorFusor::calculatePosition()
{
  /*
  //capture the previous position to allow us to later calculate the velocity
  previousPosition.set(&position);
  previousPositionTimeStamp = positionTimeStamp;
  */

  //the orientation is calculated by crossing the vector between the sensors with the vector (0, 0, 1), which
  //simplifies to (y, -x); then take the atan2 of that resulting vector to obtain the orientation
  position.orientation.set(atan2(sensorPositions[0].getY() - sensorPositions[1].getY(),
          -(sensorPositions[0].getX() - sensorPositions[1].getX())));
  //the center position of the robot is the average position between the sensors
  position.position.set(
      (BODY_CENTER_SENSOR_CENTER_OFFSET_Y * position.orientation.sin()) + ((sensorPositions[0].getX() + sensorPositions[1].getX()) / 2.0d),
      (BODY_CENTER_SENSOR_CENTER_OFFSET_Y * position.orientation.cos()) + ((sensorPositions[0].getY() + sensorPositions[1].getY()) / 2.0d));

  /*
  //now calculate the change in position and orientation
  positionDelta.set(&position);
  positionDelta.unconcat(&previousPosition);

  //the robot can only move along a straight line or circular path; this means that the velocity vector can only be along the line
  //represented by half of the change in orientation; project the velocity vector along this line to reduce velocity error
  //note that this is something of a "hack" for now until we can get a real Kalman filter in place, but it does the job for now
  positionDelta.position.projectAlong(positionDelta.orientation.get() / 2.0d);
  */

/*
#ifdef VELOCITY_UNITS_PER_SEC
  double velocityFactor = 1000.0d / ((double)positionTimeStamp - previousPositionTimeStamp);
  positionDelta.position.multiply(velocityFactor);
  positionDelta.orientation.set(velocityFactor * positionDelta.orientation.get());
#endif
*/
}

/*
void SensorFusor::estimatePosition(unsigned long currentTime)
{
  //calculate the change f-rom the last known position to the position prior to that; this change occurred over the time delta between
  //those two positions, but we need to scale that over the time delta between our last known position time stamp to the new time stamp
  double scale = currentTime - positionTimeStamp;
  double divide = positionTimeStamp - previousPositionTimeStamp;

  //calculate the change in orientation based on the previous change in orientation scaled to the new change in time
  double deltaOrientation = (subtractAngles(position.orientation.get(), previousPosition.orientation.get()) * scale) / divide;

  //calculate the previous change in position
  KVector2 deltaPosition(position.position.getX() - previousPosition.position.getX(),
      position.position.getY() - previousPosition.position.getY());
  //rotate it by the change in orientation
  deltaPosition.rotate(deltaOrientation);
  //scale it to the new change in time
  deltaPosition.setD((deltaPosition.getD() * scale) / divide);

  //capture the previous position and velocity
  previousPosition.position.set(&position.position);
  previousPosition.orientation.set(position.orientation.get());
  previousPositionTimeStamp = positionTimeStamp;

  //calculate the new position
  // SerialUSB.println("Esitmating position.");
  position.position.addVector(&deltaPosition);
  position.orientation.set(addAngles(position.orientation.get(), deltaOrientation));
  //when estimating position updates, it is assumed that velocity does not change in magnitude, only in direction
  positionDelta.position.rotate(deltaOrientation);
  positionDelta.position.setD((positionDelta.position.getD() * scale) / divide);
  positionTimeStamp = currentTime;
}
*/

void SensorFusor::stop()
{
  REG_TCC0_CTRLA &= ~TCC_CTRLA_ENABLE;
  REG_TCC1_CTRLA &= ~TCC_CTRLA_ENABLE;
}
