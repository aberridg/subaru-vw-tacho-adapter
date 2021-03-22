
//#include <FreqMeasure.h>
#include <elapsedMillis.h>
#include <FrequencyTimer2.h>

//#include "SoftPWM.h"

//Initializing Fuel pin
int fuel_pin = 9;
int clutch_pin = 12;

// Tacho input pin MUST be D8! Swap it ;-)

// Frequency counter sketch, for measuring frequencies low enough to execute an interrupt for each cycle
// Connect the frequency source to the INT0 pin (digital pin 2 on an Arduino Uno)

volatile unsigned long firstPulseTime;
volatile unsigned long lastPulseTime;
volatile unsigned long numPulses;

void isr()
{
  unsigned long now = micros();
  if (numPulses == 1)
  {
    firstPulseTime = now;
  }
  else
  {
    lastPulseTime = now;
  }
  ++numPulses;
}


// Measure the frequency over the specified sample time in milliseconds, returning the frequency in Hz
float readFrequency(unsigned int sampleTime)
{
  numPulses = 0;                      // prime the system to start a new reading
  attachInterrupt(digitalPinToInterrupt(2), isr, RISING);    // enable the interrupt
  delay(sampleTime);
  detachInterrupt(0);
  Serial.print("numPulses");
  Serial.println(numPulses);
  return (numPulses < 3) ? 0 : (1000000.0 * (float)(numPulses - 2)) / (float)(lastPulseTime - firstPulseTime);
}



void setup() {
  Serial.begin(115200);
  //Declaring LED pin as output
  delay(2);
  Serial.println("Starting...");
  delay(100);
  Serial.println("Starting2...");
  pinMode(fuel_pin, OUTPUT);
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(clutch_pin, OUTPUT);
  digitalWrite(clutch_pin, HIGH);

  //FreqMeasure.begin();
  pinMode(FREQUENCYTIMER2_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  FrequencyTimer2::setPeriod(200);
  FrequencyTimer2::enable();
  FrequencyTimer2::setOnOverflow(Burp);

  //analogWrite(fuel_pin, 100);
  // Initialize
  //SoftPWMBegin();

  // Create and set pin 13 to 0 (off)
  //SoftPWMSet(fuel_pin, 0);
  //SoftPWMSetPercent(fuel_pin, 50);
}


double sum = 0;
int count = 0;
float rpm = 0;
int ticksPerSec = 0;
bool frequencyTimerEnabled = true;

elapsedMillis sinceStart = 0;
elapsedMillis sinceSpeedUpdate = 0;

// variables shared between interrupt context and main program
// context must be declared "volatile".
volatile boolean ledOn = false;
volatile int burpCount = 0;

void Burp(void) {
  burpCount++;
  if (burpCount > 100) {
    ledOn = !ledOn;
    burpCount = 0;
  }
}

void loop() {
  // slowly decrease the voltage on the fuel gauge output pin over time - aim to "empty"
  // the tank in about 4 hours
  // According to this link: https://thefactoryfiveforum.com/showthread.php?14086-Subaru-Fuel-Sender-ohm-range
  // the fuel sender reads 0.5ish Ohms when full, and 50ish Ohms when empty.
  // Subaru workshop manual for 2005 Legacy says 4 - 100 Ohms. So different vehicles probably
  // have different resistance levels.
  // That means that we have very little voltage drop when it's full and max voltage drop when empty
  // So - let's vary the duty cycle from 90% to 0% over 4 hours
  int minsSinceStart = sinceStart / 60000;
  // Four hours is 240 minutes!

  // current calculations end up setting PWM freq to 0 after 1 min
  int minsToEmpty = 240 - minsSinceStart;

  // RPM is f x 60 x 2/cylinders
  /*if (FreqMeasure.available()) {
    // average several readings together
    sum = sum + FreqMeasure.read();
    count = count + 1;
    if (count > 30) {
      float frequency = FreqMeasure.countToFrequency(sum / count);
      Serial.print(frequency);
      Serial.print(" ");
      rpm = frequency * 60 * 0.5;
      Serial.println(rpm);
      sum = 0;
      count = 0;
    }
    }*/

  float freq = readFrequency(1000);
  Serial.print("freq:");
  Serial.println(freq);
  rpm = freq * 60 * 0.5;
  
  // Divide rpm by 4 to get wheel rpm/ticks per sec. Set ticks/sec to 0 if rpm < 1000
  if (rpm > 1000) {
    ticksPerSec = rpm / 4;
  } else {
    ticksPerSec = 0;
  }

  // Update speed every 200ms
  if (sinceSpeedUpdate > 200) {
    sinceSpeedUpdate = 0;
    Serial.print("rpm: ");
    Serial.println(rpm);
    Serial.print("ticksPerSec: ");
    Serial.println(ticksPerSec);

    if (ticksPerSec == 0) {
      if (frequencyTimerEnabled) {
        FrequencyTimer2::disable();
        frequencyTimerEnabled = false;
      }
      digitalWrite(clutch_pin, LOW);
    } else {
      // Everything inside this block seems ok
      Serial.println("updating period");
      //FrequencyTimer2::disable();
      //Serial.println("timer disabled");
      delay(1);
      Serial.print("Setting period to: ");
      Serial.println(1000000 / ticksPerSec);
      FrequencyTimer2::setPeriod(1000000 / ticksPerSec);
      //Serial.println("enabling timer");
      //delay(10);
      if (!frequencyTimerEnabled) {
        FrequencyTimer2::enable();
        frequencyTimerEnabled = true;
        Serial.println("clutch high"); // not causing crash
        delay(1);
        digitalWrite(clutch_pin, HIGH);
      }
    }
    // pause frequency measurement - we can't measure frequency and analogWrite at the same time!
    // need to check that these things work!
    //Serial.println("Ending freqMeasure");
    delay(1);
    //FreqMeasure.end();
    // update fuel PWM duty cycle
    Serial.print("Updating PWM freq:");
    Serial.println((int)((minsToEmpty / 240.0) * 255));
    Serial.print("Mins to empty: ");
    Serial.println(minsToEmpty);
    analogWrite(fuel_pin, (int)((minsToEmpty / 240.0) * 220));
    delay(1);
    // resume frequency measurement
    Serial.println("Resume freqMeasure");
    //FreqMeasure.begin();
  }

  if (ledOn) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}
