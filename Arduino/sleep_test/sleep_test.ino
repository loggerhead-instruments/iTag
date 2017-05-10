volatile bool TC_flag_CC0 = false;
volatile bool TC_flag_CC1 = false;


void setup() 
{
  delay(5000);
  SerialUSB.println("Startup");

  configureTC3(1024, 3072);

  SerialUSB.println("Configuration Complete");
}


void loop() 
{
  // If there was a CC0 match
  if (TC_flag_CC0)
  {
    TC_flag_CC0 = false;
    SerialUSB.print("Matched on CC0, ms: ");
    SerialUSB.println(millis());
    flashLED();
  }

  // If there was a CC1 match
  if (TC_flag_CC1)
  {
    TC_flag_CC1 = false;
    SerialUSB.print("Matched on CC1, ms: ");
    SerialUSB.println(millis());
    flashLED();
  }

  //systemSleep();
}


void flashLED()
{
  pinMode(PIN_LED_TXL, OUTPUT);
  digitalWrite(PIN_LED_TXL, LOW);
  delay(100);
  digitalWrite(PIN_LED_TXL, HIGH);
}


void systemSleep()
{
  // Set the sleep mode
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

  USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE; //disable USB
  __WFI(); // enter sleep mode and wait for interrupt

  // ... SLEEPING ...

  USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE; //enable USB

////  if (USB->DEVICE.FSMSTATUS.bit.FSMSTATE == USB_FSMSTATUS_FSMSTATE_SUSPEND_Val) {
//    // Disable USB
//    USBDevice.detach();
//    
//    // SAMD sleep
//    __WFI();
//
//    // Enable USB and wait for resume if attached
//    USBDevice.attach();
//    USB->DEVICE.CTRLB.bit.UPRSM = 0x01u;
//    while (USB->DEVICE.CTRLB.bit.UPRSM);
//  }
}


void configureTC3(uint16_t match0, uint16_t match1)
{
  // The GCLK clock provider to use
  // GCLK0, GCLK1 & GCLK3 are used already
  uint8_t GCLK_SRC = 4;

  // Configure the XOSC32K to run in standby
  SYSCTRL->XOSC32K.reg |= SYSCTRL_XOSC32K_RUNSTDBY;
 
  // Setup clock provider GCLK_SRC with a 32 source divider
  // GCLK_GENDIV_ID(X) specifies which GCLK we are configuring
  // GCLK_GENDIV_DIV(Y) specifies the clock prescalar / divider
  // If GENCTRL.DIVSEL is set (see further below) the divider 
  // is 2^(Y+1). If GENCTRL.DIVSEL is 0, the divider is simply Y
  // This register has to be written in a single operation
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(GCLK_SRC) | 
                     GCLK_GENDIV_DIV(4);

  // Configure the GCLK module
  // GCLK_GENCTRL_GENEN, enable the specific GCLK module
  // GCLK_GENCTRL_SRC_XOSC32K, set the source to the XOSC32K
  // GCLK_GENCTRL_ID(X), specifies which GCLK we are configuring
  // GCLK_GENCTRL_DIVSEL, specify which prescalar mode we are using
  // GCLK_RUNSTDBY, keep the GCLK running when in standby mode
  // Output from this module is 1khz (32khz / 32)
  // This register has to be written in a single operation.
  GCLK->GENCTRL.reg = GCLK_GENCTRL_GENEN | 
                      GCLK_GENCTRL_SRC_XOSC32K | 
                      GCLK_GENCTRL_ID(GCLK_SRC) | 
                      GCLK_GENCTRL_RUNSTDBY |
                      GCLK_GENCTRL_DIVSEL;
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

  // Turn the power to the TC3 module on
  PM->APBCMASK.reg |= PM_APBCMASK_TC3;

  // Set TC3 (shared with TCC2) GCLK source to GCLK_SRC
  // GCLK_CLKCTRL_CLKEN, enable the generic clock
  // GCLK_CLKCTRL_GEN(X), specifices the GCLK generator source
  // GCLK_CLKCTRL_ID(X), specifies which generic clock we are configuring
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | 
                      GCLK_CLKCTRL_GEN(GCLK_SRC) | 
                      GCLK_CLKCTRL_ID(GCM_TCC2_TC3);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

  // Disable TC3 this is required (if enabled already)
  // before setting certain registers
  TC3->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);

  // Set the mode to 16 bit and set it to run in standby
  // TC_CTRLA_MODE_COUNT16, specify 16bit mode
  // TC_CTRLA_RUNSTDBY, keep the module running when in standby
  // TC_CTRLA_PRESCALER_DIV1, set the prescalar to 1
  // Prescalar options include: DIV1, DIV2, DIV4, DIV8, 
  // DIV16, DIV64, DIV256, DIV1024
  TC3->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 |
                           TC_CTRLA_RUNSTDBY |
                           TC_CTRLA_PRESCALER_DIV1 ;
  while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);

  // Set the compare channel 0 value
  TC3->COUNT16.CC[0].reg = match0;
  while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);

  // Set the compare channel 1 value
  TC3->COUNT16.CC[1].reg = match1;
  while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
 
  // Enable interrupt on both match compare channels
  // TC_INTENSET_MC0, enable an interrupt on match channel 0
  // TC_INTENSET_MC1, enable an interrupt on match channel 0
  TC3->COUNT16.INTENSET.reg = TC_INTENSET_MC0 | TC_INTENSET_MC1;
  while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);

  // Enable TC3
  TC3->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);

  // Enable the TC3 interrupt vector
  // Set the priority to max
  NVIC_EnableIRQ(TC3_IRQn);
  NVIC_SetPriority(TC3_IRQn, 0x00); 
}


// TC3 ISR
void TC3_Handler()
{
  if (TC3->COUNT16.INTFLAG.reg & TC_INTFLAG_MC0) {
    // Set compare match flag for CC0
    TC_flag_CC0 = true;

    // Reset MC0 interrupt flag
    TC3->COUNT16.INTFLAG.reg |= TC_INTFLAG_MC0;
  }

  if (TC3->COUNT16.INTFLAG.reg & TC_INTFLAG_MC1) {
    // Set compare match flag for CC1
    TC_flag_CC1 = true;

    // Reset MC1 interrupt flag
    TC3->COUNT16.INTFLAG.reg |= TC_INTFLAG_MC1;

    // Reset the counter to repeat
    // Fails if CC1 < CC0
    TC3->COUNT16.COUNT.reg = 0;
  }
}
