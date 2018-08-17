
#include <SPI.h>
#include "../ZippyConfig.h"
#include "Lighthouse.h"

Lighthouse* currentLighthouse = NULL;
LighthouseSensorInput rightSensorInput;
LighthouseSensorInput leftSensorInput;

Lighthouse::Lighthouse()
  : leftSensor(&leftSensorInput, 0),
    rightSensor(&rightSensorInput, 1)
{
}

void Lighthouse::start()
{
  if (currentLighthouse != NULL)
    currentLighthouse->stop();
  currentLighthouse = this;

  //configure the timing clock we'll use for counting cycles between IR pules
  setupClock();

  connectPortPinsToInterrupts();

  //setup our external interrupt controller
  setupEIC();

  connectInterruptsToTimer();

  setupTimer();
}

//#define NVM_SW_CALIB_DFLL48M_COARSE_VAL 58
void Lighthouse::setupClock()
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
   // SYSCTRL_DFLLCTRL_CCDIS |                        //disable chill cycle
   // SYSCTRL_DFLLCTRL_STABLE |                       //stable frequency mode
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

void Lighthouse::connectPortPinsToInterrupts()
{
  //enable the PORT subsystem
  PM->APBBMASK.bit.PORT_ = 1;

#ifdef PLATFORM_EXEN_MINI
  //set port A (group 0), pin 11 (PA11, Exen Mini pin 1) as an input
  PORT->Group[0].DIRCLR.reg = PORT_PA11;

  //configure PA11
  PORT->Group[0].PINCFG[11].reg =
    PORT_PINCFG_PULLEN |         //enable pull-down
    PORT_PINCFG_INEN |           //enable input buffering
    PORT_PINCFG_PMUXEN;          //enable pin muxing

  //mux PA21 over to EXTINT5
  PORT->Group[0].PMUX[5].reg = PORT_PMUX_PMUXO(PORT_PMUX_PMUXO_A_Val);

  //set port A (group 0), pin 10 (PA10, Exen Mini pin 0) as an input
  PORT->Group[0].DIRCLR.reg = PORT_PA10;

  //configure PA09
  PORT->Group[0].PINCFG[10].reg =
    PORT_PINCFG_PULLEN |         //enable pull-down
    PORT_PINCFG_INEN |           //enable input buffering
    PORT_PINCFG_PMUXEN;          //enable pin muxing

  //mux PA09 over to EXTINT9
  PORT->Group[0].PMUX[5].reg = PORT_PMUX_PMUXE(PORT_PMUX_PMUXE_A_Val);
#else
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
#endif
}

void Lighthouse::setupEIC()
{
  //turn on power to the external interrupt controller (EIC)
  PM->APBAMASK.bit.EIC_ = 1;

  //disable the EIC while we configure it
  EIC->CTRL.bit.ENABLE = 0;
  while (EIC->STATUS.bit.SYNCBUSY);

#ifdef PLATFORM_EXEN_MINI
  //right diode interrupt config
  //detect both rising and falling edges
  EIC->CONFIG[1].bit.SENSE3 = EIC_CONFIG_SENSE3_HIGH_Val;
  //generate interrupts on interrupt #9 when edges are detected
  EIC->EVCTRL.bit.EXTINTEO11 = 1;

  //left diode interrupt config
  //detect both rising and falling edges
  EIC->CONFIG[1].bit.SENSE2 = EIC_CONFIG_SENSE2_HIGH_Val;
  //generate interrupts on interrupt #5 when edges are detected
  EIC->EVCTRL.bit.EXTINTEO10 = 1;
#else
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
#endif

  //enable the EIC
  EIC->CTRL.bit.ENABLE = 1;

  //wait for synchronization
  while (EIC->STATUS.bit.SYNCBUSY);
}

void Lighthouse::connectInterruptsToTimer()
{
  //enable the event subsystem
  PM->APBCMASK.bit.EVSYS_ = 1;

#ifdef PLATFORM_EXEN_MINI
  //input config for right diode
  REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EDGSEL(1) |                           //detect rising edge
//                      EVSYS_CHANNEL_PATH_SYNCHRONOUS |                    //synchronously
                      EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_11) |    //from external interrupt 9
                      EVSYS_CHANNEL_CHANNEL(0);                           //to EVSYS channel 0

  //output config for right diode
  REG_EVSYS_USER = EVSYS_USER_CHANNEL(1) |                                //attach output from channel 0 (n+1)
                   EVSYS_USER_USER(EVSYS_ID_USER_TCC0_MC_0);              //to user (recipient) TCC0, MC0
  while (!EVSYS->CHSTATUS.bit.USRRDY0);

  //input config for right diode
  REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EDGSEL(2) |                           //detect falling edge
//                      EVSYS_CHANNEL_PATH_SYNCHRONOUS |                    //synchronously
                      EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_11) |    //from external interrupt 9
                      EVSYS_CHANNEL_CHANNEL(1);                           //to EVSYS channel 1

  //output config for right diode
  REG_EVSYS_USER = EVSYS_USER_CHANNEL(2) |                                //attach output from channel 1 (n+1)
                   EVSYS_USER_USER(EVSYS_ID_USER_TCC0_MC_1);              //to user (recipient) TCC0, MC1
  while (!EVSYS->CHSTATUS.bit.USRRDY0);

  //input config for left diode
  REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EDGSEL(1) |                           //detect rising edge
//                      EVSYS_CHANNEL_PATH_SYNCHRONOUS |                    //synchronously
                      EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_10) |    //from external interrupt 5
                      EVSYS_CHANNEL_CHANNEL(2);                           //to EVSYS channel 2

  //output config for left diode
  REG_EVSYS_USER = EVSYS_USER_CHANNEL(3) |                                //attach output from channel 2 (n+1)
                   EVSYS_USER_USER(EVSYS_ID_USER_TCC1_MC_0);              //to user (recipient) TCC1, MC0

  //input config for left diode
  REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EDGSEL(2) |                           //detect falling edge
//                      EVSYS_CHANNEL_PATH_SYNCHRONOUS |                    //synchronously
                      EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_10) |    //from external interrupt 5
                      EVSYS_CHANNEL_CHANNEL(3);                           //to EVSYS channel 3

  //output config for left diode
  REG_EVSYS_USER = EVSYS_USER_CHANNEL(4) |                                //attach output from channel 3 (n+1)
                   EVSYS_USER_USER(EVSYS_ID_USER_TCC1_MC_1);              //to user (recipient) TCC1, MC1

  while (!EVSYS->CHSTATUS.bit.USRRDY0);
#else
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
#endif
}

void Lighthouse::setupTimer()
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
  NVIC_SetPriority(TCC0_IRQn, 0);
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
  NVIC_SetPriority(TCC1_IRQn, 0);
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

void Lighthouse::loop()
{
  rightSensor.loop();
  leftSensor.loop();
}

/**
 * Recalculates the position of each sensor based on the most recently received sync and sweep pulses for each combined
 * with the information about the orientation of the Lighthouse (received from the base station info block) and the
 * known distance of the Lighthouse from the plane of the sensors.
 */
void Lighthouse::recalculate()
{
  unsigned long currentTime = millis();

  //update the position of the sensors
  leftSensor.recalculate(currentTime);
  rightSensor.recalculate(currentTime);

  //calculate the center position and orientation of the robot
  unsigned long combinedPositionTimeStamp = max(leftSensor.positionTimeStamp, rightSensor.positionTimeStamp);
  if (positionTimeStamp != combinedPositionTimeStamp) {
    //calculate the center position
    previousPositionVector.set(&positionVector);
    previousPositionTimeStamp = positionTimeStamp;

    //the center position of the robot is the average position between the sensors
    positionVector.set((leftSensor.positionVector.getX() + rightSensor.positionVector.getX()) / 2.0d,
        (leftSensor.positionVector.getY() + rightSensor.positionVector.getY()) / 2.0d);
    positionTimeStamp = combinedPositionTimeStamp;

    //calculate the orientation
    previousOrientationVector.set(&orientationVector);
    previousOrientationTimeStamp = orientationTimeStamp;

    //calculate the new orientation unit vector, which is just the down direction (0,0,-1) crossed with the vector
    //between the sensors; this calculation simplifies to the following
    orientationVector.set(leftSensor.positionVector.getY() - rightSensor.positionVector.getY(),
        -(leftSensor.positionVector.getX() - rightSensor.positionVector.getX()), 1.0d);
    orientationTimeStamp = combinedPositionTimeStamp;
  }

  //now we can use the change in orientation to accurately calculate the velocities of each sensor
  unsigned long combinedVelocityTimeStamp = max(leftSensor.velocityTimeStamp, rightSensor.velocityTimeStamp);
  if (combinedVelocityTimeStamp != velocityTimeStamp) {
    velocityVector.set((leftSensor.getVelocity()->getX() + rightSensor.getVelocity()->getX()) / 2.0d,
        (leftSensor.getVelocity()->getY() + rightSensor.getVelocity()->getY()) / 2.0d);
    velocityTimeStamp = combinedVelocityTimeStamp;
  }
}

void Lighthouse::stop()
{
  REG_TCC0_CTRLA &= ~TCC_CTRLA_ENABLE;
  REG_TCC1_CTRLA &= ~TCC_CTRLA_ENABLE;
}
