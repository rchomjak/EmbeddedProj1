
//Display
#include <LiquidCrystal.h>

#define BASE 10

#define MEASURE_SIZE 100
#define TIMEOUT 5
#define VOLTAGE_TOLERANCE 5

#define VOLTAGE_PORT PORTC0
#define TEMPERATURE_PORT PORTC1

#define INCREMENT_TIME_PORT PORTD2
#define DECREMENT_TIME_PORT PORTD1

#define SERIAL_MEASURMENT_SEND_PORT PORTD3

struct time_rep {
  unsigned int total_time;
  unsigned int hours;
  unsigned int minutes;
  unsigned int seconds;
};

time_rep time;

typedef enum TIME_STATE {E_NORMAL, E_HOUR, E_MINUTE, E_SECOND, E_EPSILON} TIMER_STATE;
TIMER_STATE time_state = E_NORMAL;

float temper_voltage = 0;
float temper_c = 0;

int potentiometer_voltage_curr = 0;
int potentiometer_voltage_prev = 0;

unsigned int time_diff = 0;

struct measurment {
  float temper[MEASURE_SIZE];
  float voltage[MEASURE_SIZE];
  unsigned int time[MEASURE_SIZE];
  unsigned int counter;
};

struct measurment measurment;
const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {

  Serial.begin(9600);
  
  while(!Serial);

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  cli();//stop interrupts
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();//allow interrupts
}

void loop() {
  
  

  potentiometer_voltage_prev = potentiometer_voltage_curr;
  potentiometer_voltage_curr =  (analogRead(VOLTAGE_PORT));
  
    
  measurment.voltage[measurment.counter] = (float)(potentiometer_voltage_curr * 5)/1023;
  //reading temperature from sensor (LM35)  
  temper_voltage =  (analogRead(TEMPERATURE_PORT));
  temper_voltage = (temper_voltage * 5)/1023;
  temper_c = (temper_voltage) * 100;

  measurment.temper[measurment.counter] = temper_c;
  measurment.time[measurment.counter] = time.total_time;
  
  measurment.counter = (measurment.counter + 1) % MEASURE_SIZE;

  if ((abs(potentiometer_voltage_prev - potentiometer_voltage_curr) > VOLTAGE_TOLERANCE) && time_state == E_NORMAL) {
    time_state = E_HOUR;
    time_diff = 0;
  } else if ((time_state == E_HOUR) && time_diff >= TIMEOUT) {
    time_state = E_MINUTE;
    time_diff = 0;
  } else if ((time_state == E_MINUTE) && time_diff >= TIMEOUT) {
    time_state = E_EPSILON;
    time_diff = 0;
  } else if (time_state == E_EPSILON){
    time_state = E_NORMAL;
  }

  if (time_state == E_HOUR) {
    
    if(!digitalRead(INCREMENT_TIME_PORT)) {
      time.total_time = (time.total_time + 3600); 
      time_diff = 0;
    }
    
    if(!digitalRead(DECREMENT_TIME_PORT)) {
      time.total_time = (time.total_time - 3600) < ((unsigned int)-1 - 3600)? time.total_time - 3600: 0; 
      time_diff = 0;
  }
  
    
  } else if (time_state == E_MINUTE) {
    
    if(!digitalRead(INCREMENT_TIME_PORT)) {
      time.total_time = (time.total_time + 60); 
      time_diff = 0;
    }
    
    if(!digitalRead(DECREMENT_TIME_PORT)) {
      time.total_time = (time.total_time - 60) < ((unsigned int)-1 - 60)? time.total_time - 60: 0; 
      time_diff = 0;
      
    }
    
  } 
  
  if(!digitalRead(SERIAL_MEASURMENT_SEND_PORT)) {

    Serial.print("Temperature (C)");
    Serial.print("|");
    Serial.print("Voltage (V)");
    Serial.print("|");
    Serial.print("Time (s)");
    Serial.println();

    for (size_t i = 0; i < MEASURE_SIZE; i++) {
      
      Serial.print(measurment.temper[i]);
      Serial.print("|");

      Serial.print(measurment.voltage[i]);
      Serial.print("|");
      
      Serial.print(measurment.time[i]);
      
      
      Serial.println();
      Serial.println("-------------------");
    }
  }

  delay(1000);
}


ISR(TIMER1_COMPA_vect) {
  
  time.total_time = time.total_time + 1;
  time_diff += 1;
  
  time.hours = time.total_time/3600;
  
  lcd.setCursor(0,0);
  if ((time.hours) % 24 < BASE) {
    
    lcd.print(0); 
  }

  lcd.print(time.hours%24);
  
  time.minutes = (time.total_time - 3600*(time.hours))/60;
      
  if ((time.minutes) % 60 < BASE) {
    lcd.print(0); 
  }

  lcd.print(time.minutes);
  
  time.seconds = time.total_time - 3600*(time.hours) - 60*time.minutes;
  
  if (time_state == E_SECOND) {
    time.total_time = time.total_time - time.seconds;
    time.seconds = 0;
  } 
  
  if ((time.seconds) % 60 < BASE) {
    lcd.print(0); 
  }
  

  lcd.print(time.seconds);

}



