
#include <FreqMeasure.h>
#include <elapsedMillis.h>
#include <FrequencyTimer2.h>

//Initializing Fuel pin
int fuel_pin = 3;


// Tacho input pin MUST be D8! Swap it ;-)



elapsedMillis sinceStart = 0;
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
  
  FreqMeasure.begin();
  pinMode(FREQUENCYTIMER2_PIN, OUTPUT);
  FrequencyTimer2::setPeriod(200);
  FrequencyTimer2::enable();
  
}


double sum = 0;
int count = 0;
float rpm = 0;
int ticksPerSec = 0;

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
  int minsToEmpty = 240 - minsSinceStart;
  //analogWrite(fuel_pin, (minsToEmpty / 240) * 255);

  // RPM is f x 60 x 2/cylinders
  
  if (FreqMeasure.available()) {
    // average several reading together
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
  }

  // Divide rpm by 4 to get wheel rpm/ticks per sec
  if (rpm > 1000) {
    ticksPerSec = rpm/4; 
    //FrequencyTimer2::setPeriod(1000 / ticksPerSec); 
  } else {
    ticksPerSec = 0;
    
  }
    
}
