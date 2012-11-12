void initUI(void);
void uiEvent(void);
void btnEvent(void);
void ledEvent(void);
void setBlink(uint32 pattern);

#define UI_RATE 31250    // in microseconds; should give 32Hz toggles

volatile uint32 blinkPattern = 0;
uint32 blinkBuffer = 0, blinkCount = 0;
int btnState = 0, btnStatePrev = 0;

// We'll use timer 2
HardwareTimer timer(2);

void initUI() {
  pinMode(BOARD_LED_PIN, OUTPUT);
  pinMode(BOARD_BUTTON_PIN, INPUT);
  //attachInterrupt(0, buttonEvent, CHANGE);
  
  setBlink(0x00010001);

  // Pause the timer while we're configuring it
  timer.pause();

  // Set up period
  timer.setPeriod(UI_RATE); // in microseconds

  // Set up an interrupt on channel 1
  timer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  timer.setCompare(TIMER_CH1, 1);  // Interrupt 1 count after each update
  timer.attachCompare1Interrupt(uiEvent);

  // Refresh the timer's count, prescale, and overflow
  timer.refresh();

  // Start the timer counting
  timer.resume();
}

void uiEvent(void) {
  btnState = digitalRead(BOARD_BUTTON_PIN);
  if (btnState != btnStatePrev) {
    btnStatePrev = btnState;
    btnEvent();
  }
  ledEvent();
}

void btnEvent()
{
  if (btnState)
    toggleLED();
}

void ledEvent()
{
  digitalWrite(BOARD_LED_PIN, (blinkBuffer >>= 1) & 1);
  if (++blinkCount % 32 == 0) {
    blinkBuffer = blinkPattern;
  }
}

void setBlink(uint32 pattern)
{
  blinkBuffer = blinkPattern = pattern;
  blinkCount = 0;
}
