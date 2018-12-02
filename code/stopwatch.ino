#include <LiquidCrystal.h>

#define DEBUG

#define BUZZER_TIME     200   // ms

typedef struct timer {
  unsigned char minutes = 0;
  unsigned char seconds = 0;
  unsigned char tenths = 0;
  unsigned char hundreds = 0;
  unsigned char thousands = 0;
} Timer;

const uint8_t BUZZER_PIN = 10;
const uint8_t START_BUTTON_PIN = 3;
const uint8_t STOP_BUTTON_PIN = 2;
const uint8_t REGISTER_SELECT_PIN = 9; 
const uint8_t ENABLE_PIN = 8;
const uint8_t DATA_BUS_LINE_1_PIN = 7;
const uint8_t DATA_BUS_LINE_2_PIN = 6;
const uint8_t DATA_BUS_LINE_3_PIN = 5;
const uint8_t DATA_BUS_LINE_4_PIN = 4;

LiquidCrystal lcd(REGISTER_SELECT_PIN, 
                  ENABLE_PIN, 
                  DATA_BUS_LINE_1_PIN, 
                  DATA_BUS_LINE_2_PIN, 
                  DATA_BUS_LINE_3_PIN, 
                  DATA_BUS_LINE_4_PIN);

uint16_t buzzerTimer = 0;
Timer timer[5];

bool timerRunning = false;
bool printTime = false;
bool updateFlag = false;
bool firstUpdate = false;
bool startBuzzer = false;

//void serialToBluetooth(void);
#ifdef DEBUG
void checkSerial();
void updateSerialDisplay(uint8_t index);
#endif
void startButtonInterrupt();
void stopButtonInterrupt();
void startTimer();
void stopTimer();
void updateLcdDisplay(uint8_t index);

void setup() 
{    
  pinMode(BUZZER_PIN, OUTPUT);
  
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(START_BUTTON_PIN), startButtonInterrupt, FALLING);

#ifdef DEBUG
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);  
#endif

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  cli(); // Stop interrupts

  // Set timer1 interrupt at 1Hz
  TCCR1A = 0; // Set entire TCCR1A register to 0
  TCCR1B = 0; // Same for TCCR1B
  TCNT1  = 0; // Initialize counter value to 0
  // Set compare match register for 1hz increments
  OCR1A = 249;// = (16*10^6) / (1*64) - 1 (must be <65536)
  // Turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 bit for 64 prescaler
  TCCR1B |= (1 << CS11) | (1 << CS10);  
  // Enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei(); // Allow interrupts
}

void loop() 
{ 
  uint8_t index;
 
#ifdef DEBUG  
  checkSerial();
#endif

  if (printTime)
  {   
#ifdef DEBUG
    updateSerialDisplay(0);
#endif    
    delay(100);

    for (index = 1; index < 5; index++)
    {
      updateLcdDisplay(index);
    }
   
    printTime = false;
  }

  if (firstUpdate)
  {
    for (index = 4; index > 1; index--)
    {
      timer[index] = timer[index - 1];
    }    
    
    for (index = 2; index < 5; index++)
    {
      updateLcdDisplay(index);
    }    
        
    firstUpdate = false;
  }

  if (timerRunning)
  {
    if (updateFlag)
    {
      updateLcdDisplay(0);
      updateFlag = false;
    }
  }

  if (startBuzzer)
  {
    buzzerTimer = BUZZER_TIME;
    digitalWrite(BUZZER_PIN, HIGH);
    startBuzzer = false;
  }
  
  if (buzzerTimer == 0)
  {
      digitalWrite(BUZZER_PIN, LOW);  
  }
}

#ifdef DEBUG
void checkSerial()
{
  int incomingByte = 0;   // for incoming serial data
  
  if (Serial.available() > 0) 
  {
    // read the incoming byte:
    incomingByte = Serial.read();

    if (incomingByte == 's')
    {
      startTimer();      
    }
    else if (incomingByte == 'e')
    {
      stopTimer(); 
    }
  }
}
#endif

void startButtonInterrupt() 
{
  startTimer();
}

void stopButtonInterrupt() 
{
  stopTimer(); 
}

ISR(TIMER1_COMPA_vect)
{
  // timer1 interrupt 1kHz    
  if (timerRunning)
  {
    timer[0].thousands++;
    if (timer[0].thousands > 9)
    {
      timer[0].thousands = 0;
      timer[0].hundreds++;
      if (timer[0].hundreds > 9)
      {
        timer[0].hundreds = 0;
        timer[0].tenths++;
        
        updateFlag = true;
        if (timer[0].tenths > 9)
        {
          timer[0].tenths = 0;
          timer[0].seconds++;
          
          if (timer[0].seconds > 59)
          {
            timer[0].seconds = 0;
            timer[0].minutes++;
          }
        }
      }
    }    
  }

  if (buzzerTimer > 0)
  {
    buzzerTimer--;
  }
}

void startTimer() 
{
  if (!timerRunning)
  {
    detachInterrupt(digitalPinToInterrupt(START_BUTTON_PIN));
    attachInterrupt(digitalPinToInterrupt(STOP_BUTTON_PIN), stopButtonInterrupt, FALLING);

    EIFR = 0xFF;

    timer[0].minutes = 0;
    timer[0].seconds = 0;
    timer[0].tenths = 0;
    timer[0].hundreds = 0;

    timerRunning = true;
    firstUpdate = true;
    startBuzzer = true;
  }
}

void stopTimer() 
{
  if (timerRunning)
  {
    detachInterrupt(digitalPinToInterrupt(STOP_BUTTON_PIN));
    attachInterrupt(digitalPinToInterrupt(START_BUTTON_PIN), startButtonInterrupt, FALLING);

    EIFR = 0xFF;

    timerRunning = false;
    startBuzzer = true;
    printTime = true;

    timer[1] = timer[0];

    updateFlag = true;
  }
}

void updateLcdDisplay(uint8_t index)
{
  switch (index)
  {
    case 0:
      lcd.setCursor(0, 0);
      break;
    case 1:
      lcd.setCursor(0, 0);
      break;
    case 2:
      lcd.setCursor(8, 0);    
      break;
    case 3:
      lcd.setCursor(0, 1);
      break;
    case 4:
      lcd.setCursor(8, 1);
      break;
    default:
      lcd.setCursor(0, 1);
      break;
  }  
  lcd.print(timer[index].minutes % 10);
  lcd.print(":");
  lcd.print(timer[index].seconds / 10);
  lcd.print(timer[index].seconds % 10);
  lcd.print(":");
  lcd.print(timer[index].tenths);
  lcd.print(timer[index].hundreds);
}

#ifdef DEBUG
void updateSerialDisplay(uint8_t index)
{
  Serial.print(timer[index].minutes);
  Serial.print(":");
  Serial.print(timer[index].seconds);
  Serial.print(".");
  Serial.print(timer[index].tenths);
  Serial.println(timer[index].hundreds);
}
#endif
