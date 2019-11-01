
#include <SPI.h>
#include "SensorFusor.h"

#define LIGHTHOUSE_LOCKED_SIGNAL_PAUSE         2000
// #define LIGHTHOUSE_UNLOCKED_SIGNAL_TIMEOUT      200
#define SENSOR_OFFSET_X  10.8d
#define SENSOR_OFFSET_Y   4.2d
//height of the lighthouse from the diode plane
//mounted on surface of entertainment center
#define LIGHTHOUSE_HEIGHT            908.0d

SensorFusor* currentLighthouse = NULL;
LighthouseSensorInput leftSensorInput;
LighthouseSensorInput rightSensorInput;

SensorFusor::SensorFusor()
  // : sensors[0](&leftSensorInput, 0),
    // sensors[1](&rightSensorInput, 1)
{
  sensors[0].debugNumber = 0;
  sensors[0].sensorInput = &leftSensorInput;
  sensors[1].debugNumber = 0;
  sensors[1].sensorInput = &rightSensorInput;
}

void SensorFusor::start()
{
  if (currentLighthouse != NULL)
    currentLighthouse->stop();
  currentLighthouse = this;

  sensors[0].restart();
  sensors[1].restart();

  previousPositionTimeStamp = 0;
  positionTimeStamp = 0;
  positionLockedTimeStamp = 0;

  if (!receivedLighthouseData) {
    preambleBitCount = 0;
    preambleFound = false;
    ootxParser.restart();
    currentSignalState = LighthouseSignalState::ReceivingLighthouseData;
  }
  else
    currentSignalState = LighthouseSignalState::AcquiringPositionLock;

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
  PORT->Group[0].DIRCLR.reg = PORT_PA21;

  //configure PA21
  PORT->Group[0].PINCFG[21].reg =
    PORT_PINCFG_PULLEN |         //enable pull-down
    PORT_PINCFG_INEN |           //enable input buffering
    PORT_PINCFG_PMUXEN;          //enable pin muxing

  //mux PA21 over to EXTINT5
  PORT->Group[0].PMUX[10].reg = PORT_PMUX_PMUXO(PORT_PMUX_PMUXE_A_Val);

  //set port A (group 0), pin 9 (PA09, Tinyduino proto board pin IO3) as an input
  PORT->Group[0].DIRCLR.reg = PORT_PA09;

  //configure PA09
  PORT->Group[0].PINCFG[9].reg =
    PORT_PINCFG_PULLEN |         //enable pull-down
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
  while (!EVSYS->CHSTATUS.bit.USRRDY0);

  //input config for left diode
  REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EDGSEL(1) |                           //detect rising edge
//                      EVSYS_CHANNEL_PATH_SYNCHRONOUS |                    //synchronously
                      EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_5) |    //from external interrupt 5
                      EVSYS_CHANNEL_CHANNEL(2);                           //to EVSYS channel 2

  //output config for left diode
  REG_EVSYS_USER = EVSYS_USER_CHANNEL(3) |                                //attach output from channel 2 (n+1)
                   EVSYS_USER_USER(EVSYS_ID_USER_TCC1_MC_0);              //to user (recipient) TCC1, MC0

  //input config for left diode
  REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EDGSEL(2) |                           //detect falling edge
//                      EVSYS_CHANNEL_PATH_SYNCHRONOUS |                    //synchronously
                      EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_5) |    //from external interrupt 5
                      EVSYS_CHANNEL_CHANNEL(3);                           //to EVSYS channel 3

  //output config for left diode
  REG_EVSYS_USER = EVSYS_USER_CHANNEL(4) |                                //attach output from channel 3 (n+1)
                   EVSYS_USER_USER(EVSYS_ID_USER_TCC1_MC_1);              //to user (recipient) TCC1, MC1

  while (!EVSYS->CHSTATUS.bit.USRRDY0);
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
//    TCC_INTENSET_MC0 |              //enable interrupts when a capture occurs on MC0
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
//    TCC_INTENSET_MC0 |              //enable interrupts when a capture occurs on TCC1/MC0
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
  if (TCC0->INTFLAG.bit.MC0) {
    //capture CC0; required regardless of whether we actually use the value in order to reset the interrupt flag
    unsigned int cc0 = REG_TCC0_CC0;
//  SerialUSB.print("Start: ");
//  SerialUSB.println(cc0);

#ifdef DEBUG_SIGNAL_EDGES
    //keep track of the falling edge counts
    rightSensorInput.risingEdgeCount++;
#endif

//    /*
    //make sure the buffer is not full
    if (rightSensorInput.hitTickWritePtr != rightSensorInput.hitTickReadPtr) {
      *rightSensorInput.hitTickWritePtr = cc0;

      //updating this must be atomic, so check if we're at the end first
      if (rightSensorInput.hitTickWritePtr == rightSensorInput.hitTickEndPtr)
        rightSensorInput.hitTickWritePtr = rightSensorInput.hitTickBuffer;
      else
        rightSensorInput.hitTickWritePtr++;
    }
//    */
  }

  if (TCC0->INTFLAG.bit.MC1) {
    //capture counter; required regardless of whether we actually use the value in order to reset the interrupt flag
    unsigned int cc0 = REG_TCC0_CC1;
//  SerialUSB.print("End: ");
//  SerialUSB.println(cc0);

#ifdef DEBUG_SIGNAL_EDGES
    //keep track of the falling edge counts
    rightSensorInput.fallingEdgeCount++;
#endif

// SerialUSB.println("Got right sensor pulse.");
//    /*
// SerialUSB.println("Got right sensor edge.");
    //make sure the buffer is not full
    if (rightSensorInput.hitTickWritePtr != rightSensorInput.hitTickReadPtr) {
      *rightSensorInput.hitTickWritePtr = cc0;

      //updating this must be atomic, so check if we're at the end first
      if (rightSensorInput.hitTickWritePtr == rightSensorInput.hitTickEndPtr)
        rightSensorInput.hitTickWritePtr = rightSensorInput.hitTickBuffer;
      else
        rightSensorInput.hitTickWritePtr++;
    }
//    */
  }
}

void TCC1_Handler()
{
  if (TCC1->INTFLAG.bit.MC0) {
    //capture CC0; required regardless of whether we actually use the value in order to reset the interrupt flag
    unsigned int cc0 = REG_TCC1_CC0;

#ifdef DEBUG_SIGNAL_EDGES
    //keep track of the falling edge counts
    leftSensorInput.risingEdgeCount++;
#endif

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
    unsigned int cc0 = REG_TCC1_CC1;

#ifdef DEBUG_SIGNAL_EDGES
    //keep track of the falling edge counts
    leftSensorInput.fallingEdgeCount++;
#endif

// SerialUSB.println("Got left sensor pulse.");
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
}

bool SensorFusor::loop(unsigned long currentTime)
{
  sensors[0].loop(currentTime);
  sensors[1].loop(currentTime);

  switch (currentSignalState) {
    case LighthouseSignalState::ReceivingLighthouseData:
      processLighthouseData(currentTime);
      break;

    case LighthouseSignalState::AcquiringPositionLock:
      acquirePosition(currentTime);
      break;

    case LighthouseSignalState::SignalLocked:
      acquirePosition(currentTime);
      break;
  }

  return currentSignalState == LighthouseSignalState::SignalLocked;
}

void SensorFusor::processLighthouseData(unsigned long currentTime)
{
  if (!sensors[0].completedCycleTimeStamp || !sensors[1].completedCycleTimeStamp) {
    //signal lock was Lost
    ootxParser.restart();
    return;
  }

  if (sensors[0].completedCycleTimeStamp <= cycleProcessedTime || sensors[1].completedCycleTimeStamp <= cycleProcessedTime)
    return;
  cycleProcessedTime = currentTime;

  if (SYNC_PULSE_NUMBER(sensors[0].completedHitCycles[0].syncTicks) != SYNC_PULSE_NUMBER(sensors[1].completedHitCycles[0].syncTicks) ||
      SYNC_PULSE_NUMBER(sensors[0].completedHitCycles[1].syncTicks) != SYNC_PULSE_NUMBER(sensors[1].completedHitCycles[1].syncTicks))
  {
    //sensors did not see the same sync pairs; that's a problem; restart the OOTX parsing process
    ootxParser.restart();
    return;
  }

  ootxParser.processOOTXBit((sensors[0].completedHitCycles[0].syncTicks + sensors[1].completedHitCycles[0].syncTicks) / 2);
  ootxParser.processOOTXBit((sensors[0].completedHitCycles[1].syncTicks + sensors[1].completedHitCycles[1].syncTicks) / 2);
  if (ootxParser.receivedBaseStationInfoBlock()) {
    preambleFound = true;
    calculateLighthouseData();
    resetPositionLock();
  }
}

void SensorFusor::resetPositionLock()
{
  previousPositionTimeStamp = 0;
  positionTimeStamp = 0;
  positionLockedTimeStamp = 0;
  if (!preambleFound)
    preambleBitCount = 0;
  currentSignalState = LighthouseSignalState::AcquiringPositionLock;
}

void SensorFusor::acquirePosition(unsigned long currentTime)
{
  if (!sensors[0].completedCycleTimeStamp || !sensors[1].completedCycleTimeStamp) {
    //signal lock was Lost
    // SerialUSB.println("Lost cycle signal lock.");
    resetPositionLock();
    return;
  }

  if (sensors[0].completedCycleTimeStamp <= cycleProcessedTime || sensors[1].completedCycleTimeStamp <= cycleProcessedTime)
    return;
  cycleProcessedTime = currentTime;

  if (SYNC_PULSE_NUMBER(sensors[0].completedHitCycles[0].syncTicks) != SYNC_PULSE_NUMBER(sensors[1].completedHitCycles[0].syncTicks) ||
      SYNC_PULSE_NUMBER(sensors[0].completedHitCycles[1].syncTicks) != SYNC_PULSE_NUMBER(sensors[1].completedHitCycles[1].syncTicks))
  {
    //sensors did not see the same sync pairs; that's a problem; restart the OOTX parsing process
    // SerialUSB.println("Detected mismatched sync cycles.");
    resetPositionLock();
    return;
  }

  if (!sensors[0].completedHitCycles[0].sweepHitStartTicks || !sensors[1].completedHitCycles[0].sweepHitStartTicks ||
      !sensors[0].completedHitCycles[1].sweepHitStartTicks || !sensors[1].completedHitCycles[1].sweepHitStartTicks)
  {
    // if (currentSignalState == LighthouseSignalState::SignalLocked)
      // SerialUSB.println("No hit detected on one or more sensor axes.");
    resetPositionLock();
    return;
  }

  unsigned long averageXSyncTicks = (sensors[0].completedHitCycles[0].syncTicks + sensors[1].completedHitCycles[0].syncTicks) / 2;
  unsigned long averageZSyncTicks = (sensors[0].completedHitCycles[1].syncTicks + sensors[1].completedHitCycles[1].syncTicks) / 2;
  if (!preambleFound) {
    processPreambleBit(averageXSyncTicks);
    processPreambleBit(averageZSyncTicks);
  }

  //recalculate the new position
  calculateSensorPosition(
    averageXSyncTicks + sensors[0].completedHitCycles[0].sweepHitStartTicks + (sensors[0].completedHitCycles[0].sweepHitEndTicks / 2),
    averageZSyncTicks + sensors[0].completedHitCycles[1].sweepHitStartTicks + (sensors[0].completedHitCycles[1].sweepHitEndTicks / 2),
    &sensorPositions[0]);
  calculateSensorPosition(
    averageXSyncTicks + sensors[1].completedHitCycles[0].sweepHitStartTicks + (sensors[1].completedHitCycles[0].sweepHitEndTicks / 2),
    averageZSyncTicks + sensors[1].completedHitCycles[1].sweepHitStartTicks + (sensors[1].completedHitCycles[1].sweepHitEndTicks / 2),
    &sensorPositions[1]);
  calculatePosition();
  // position.printDebug();
  positionTimeStamp = currentTime;
  if (!positionLockedTimeStamp) {
    if (previousPositionTimeStamp)
      positionLockedTimeStamp = currentTime;
  }
  else if (currentSignalState != LighthouseSignalState::SignalLocked &&
      currentTime - positionLockedTimeStamp >= LIGHTHOUSE_LOCKED_SIGNAL_PAUSE)
  {
    // SerialUSB.println("Sensors locked. Starting routine.");
    currentSignalState = LighthouseSignalState::SignalLocked;
  }
}

void SensorFusor::processPreambleBit(unsigned long syncTickCount)
{
  if (SYNC_PULSE_BIT(syncTickCount)) {
    preambleBitCount = 0;
    return;
  }

  preambleBitCount++;
  if (preambleBitCount >= 17)
    preambleFound = true;
}

void SensorFusor::calculateSensorPosition(unsigned long xTicks, unsigned long zTicks, KVector2* out)
{
  //Step 1: Calculate the vector from the lighthouse in its reference frame to the diode by normalizing the angle on each axis
  //from the lighthouse to +/- M_PI_2
  double observedAngleX = ((((double)xTicks) / ((double)SWEEP_DURATION_TICK_COUNT)) - 0.5d) * M_PI;
  double observedAngleZ = ((((double)zTicks) / ((double)SWEEP_DURATION_TICK_COUNT)) - 0.5d) * M_PI;

  //correct for the factory calibration data; algorithm pulled from notes and code from the open source libsurvive project
  //    https://github.com/cnlohr/libsurvive/wiki/BSD-Calibration-Values
  observedAngleX += xRotor.phase;
  observedAngleZ += zRotor.phase;
  double correctionX = sin(xRotor.tilt * observedAngleZ);
  double correctionZ = sin(zRotor.tilt * observedAngleX);
  observedAngleX += correctionX;
  observedAngleZ += correctionZ;
  correctionX = xRotor.curve * observedAngleZ;
  correctionZ = zRotor.curve * observedAngleX;
  observedAngleX += correctionX;
  observedAngleZ += correctionZ;
  observedAngleX += cos(xRotor.gibbousPhase + observedAngleX) * xRotor.gibbousMagnitude;
  observedAngleZ += cos(zRotor.gibbousPhase + observedAngleZ) * zRotor.gibbousMagnitude;

  //calculate the normal for the plane of the intersection from the right-to-left sweep, which is a vertical plane whose normal
  //is a vector with no z component
  //note that we must flip the x axis while converting to our global coordinate system because our tick counts get greater
  //from right-to-left from the perspective of the lighthouse; this is contrary to some animations online which illustrate
  //the horizontal beam sweeping from left-to-right from the perspective of the lighthouse
  double verticalNormalX = cos(observedAngleX);
  double verticalNormalY = sin(observedAngleX);

  //calculate the normal for the plane of the intersection from the bottom-to-top sweep, which is a horizontal plane whose normal
  //is a vector with no x component
  double horizontalNormalY = -sin(observedAngleZ);
  double horizontalNormalZ = cos(observedAngleZ);

  //now cross the plane of the horizontal sweep with the plane of the vertical sweep and normalize it; this will give us a
  //unit vector which represents the intersection of these two planes in the local coordinate system of the lighthouse
  //this calculation reduces to the following
  KVector3 directionFromLighthouse(
      -(horizontalNormalZ * verticalNormalY),
      horizontalNormalZ * verticalNormalX,
      -(horizontalNormalY * verticalNormalX),
      1.0d);

  //now convert the vector from the lighthouse in its local coordinate system to our global coordinate system
  directionFromLighthouse.unrotate(&lighthouseOrientation);

  //now intersect with the plane of the diodes on the robot; since our diode plane is defined by the normal 0,0,1, and we have a
  //vector which identifies the position of the lighthouse from 0,0,0, the entire formula for our diode-plane intersection reduces
  //to the following
  double t = -LIGHTHOUSE_HEIGHT / directionFromLighthouse.getZ();
  out->set(
      // directionFromLighthouse.getX() * t,
      // directionFromLighthouse.getY() * t);
      (directionFromLighthouse.getX() * t) - LIGHTHOUSE_CENTER_OFFSET_X,
      (directionFromLighthouse.getY() * t) - LIGHTHOUSE_CENTER_OFFSET_Y);
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

  //The accelerometer reading from the lighthouse provides a vector that represents the lighthouse "up" direction in a coordinate system
  //from the perspective of the lighthouse such that x and z axes are parallel to the ground, positive x is to the lighthouse "right",
  //positive z is "forward, and positive y is "up".
  // KVector3 rotationUnitVector(-ootxParser.getAccelDirX(), ootxParser.getAccelDirZ(), ootxParser.getAccelDirY(), 1.0d);
  KVector3 rotationUnitVector(-ootxParser.getAccelDirX(), ootxParser.getAccelDirZ(), ootxParser.getAccelDirY(), 1.0d);

  //now calculate the angle of rotation from the "up" normal in our global coordinate system (0,0,1) to the rotation unit vector
  //this calculation ultimately reduces to the inverse cosine of the z axis of the rotation unit vector
  double angleOfRotation = acos(rotationUnitVector.getZ());

  //now cross the "up" vector of the lighthouse with the "up" normal of the global coordinate system to obtain the axis of rotation for
  //our quaternion; this calculation ultimately reduces to the y axis from the rotation unit vector becoming the x axis and the x axis
  //becoming the negative y axis; then obtain the unit vector of the result
  rotationUnitVector.set(rotationUnitVector.getY(), -rotationUnitVector.getX(), 0.0d, 1.0d);

  //now that we have both the axis and angle of rotation, we can calculate our quaternion
  lighthouseOrientation.set(rotationUnitVector.getX(), rotationUnitVector.getY(), rotationUnitVector.getZ(), angleOfRotation);

#ifdef LIGHTHOUSE_ORIENTATION_Z
  lighthouseOrientation.rotateZ(LIGHTHOUSE_ORIENTATION_Z);
#endif

  receivedLighthouseData = true;
}

void SensorFusor::calculatePosition()
{
  //capture the previous position to allow us to later calculate the velocity
  previousPosition.position.set(&position.position);
  previousPosition.orientation.set(position.orientation.get());
  // previousPositionDelta.position.set(&positionDelta.position);
  // previousPositionDelta.orientation.get() = positionDelta.orientation;
  previousPositionTimeStamp = positionTimeStamp;

  //the orientation is calculated by crossing the vector between the sensors with the vector (0, 0, 1), which
  //simplifies to (y, -x); then take the atan2 of that resulting vector to obtain the orientation
  position.orientation.set(atan2(sensorPositions[0].getY() - sensorPositions[1].getY(),
          -(sensorPositions[0].getX() - sensorPositions[1].getX())));
  //the center position of the robot is the average position between the sensors
  position.position.set(
      (SENSOR_OFFSET_Y * position.orientation.sin()) + ((sensorPositions[0].getX() + sensorPositions[1].getX()) / 2.0d),
      (SENSOR_OFFSET_Y * position.orientation.cos()) + ((sensorPositions[0].getY() + sensorPositions[1].getY()) / 2.0d));

  //now calculate the change in position and orientation
  positionDelta.set(&position);
  positionDelta.unconcat(&previousPosition);

  //the robot can only move along a straight line or circular path; this means that the velocity vector can only be along the line
  //represented by half of the change in orientation; project the velocity vector along this line to reduce velocity error
  positionDelta.position.projectAlong(positionDelta.orientation.get() / 2.0d);

#ifdef VELOCITY_UNITS_PER_SEC
  double velocityFactor = 1000.0d / ((double)positionTimeStamp - previousPositionTimeStamp);
  positionDelta.position.multiply(velocityFactor);
  positionDelta.orientation.set(velocityFactor * positionDelta.orientation.get());
#endif
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
