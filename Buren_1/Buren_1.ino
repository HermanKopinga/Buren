/*
  PIXI by Werccollective.com
 programming: olav@werccollective.com, herman@kopinga.nl
 
 Updates on: https://github.com/HermanKopinga/Buren/
 
 Hardware:
 - AVR Atmega328p @1MHz (internal oscillator)
 - nrf24l01 wireless modules
 - tp4056 lipo charger
 - apa102 leds
 - custom PCB
 
 Based on work by:
 - https://github.com/maniacbug/RF24 (nrf24l01 library)
 - http://adafruit.com/ (DotStar library)
 - http://donalmorrissey.blogspot.nl/2010/04/sleeping-arduino-part-5-wake-up-via.html  (sleep code)
 - https://github.com/jgillick/arduino-LEDFader/blob/master/Curve.cpp Gamma correction
 */


#include <SPI.h>
#include "nRF24L01.h" // rf lib
#include "RF24.h" // rf lib
#include <Adafruit_DotStar.h>
#include "printf.h" // used for debuggin the radio.print
#include <avr/sleep.h> // For sleeping.
#include <avr/power.h> // For sleeping.
#include <avr/wdt.h> // For sleeping.

/*
 1 time (2 bytes) start time for color (fraction: 8 bits.8 bits 0-255.0-255)
 2 Frequency (byte) freq for color calcualtion / 100
 3 Broad (byte) color calculation
 4 Center (byte) range of color calculation
 5 Intensity (byte) start intensity led
 6 ColorShift (byte) Amount of color change betwe pixies / 10000
 7 FadeOut (byte) Speed to fade the led across pixies to black =0 ook stop passing on
 8 FadeSpeedIn (byte) Speed to fade in the led inside the pixie in miliseconds / 10
 9 FadeSpeedOut (byte) Speed to fade out led inside pixie miliseconds / 10
 10 LedTime (byte) Time the LED is on.
 11 Rest (byte) stop listening after sending
 12 RestSpeed (signed byte) to speed up or slow down sending
 13 Future (byte) future expansion
 14 Cricket (bit) turn on cricket
 15 BatteryLevel (bit) show us your batt level
 16 Character (4 bits) Caracter (its an nibble)
 17 FindMyPixi (bit) show were you are
 18 MonkyLives (bit) got mental!!! (test mode)
 */

#define DEBUG
#define DEBUGSLEEP
#define DEBUGSEND
#define DEBUGRECEIVE

// PIXI dust
#define PAYLOAD_SIZE 15

float time = 0;
float frequency = 0;
byte broad = 0;
byte center = 0;
byte intensity = 0;
float colorShift = 0;
byte fadeOut = 0;
byte fadeSpeedIn = 0;
byte fadeSpeedOut = 0;
byte ledTime = 0;
byte rest = 0;
byte restSpeed = 0;
byte future = 0;
byte bitField = 0;
bool cricket = 0;
bool findMyPixi = 0;
bool batteryLevel = 0;
byte character = 0;
bool monkeyLives = 0;

// Piezo delay (basis for tone frequency of cricket).
int piezoLow = 10;

// variables for color shift
byte red = 0;
byte green = 0;
byte blue = 0;

//
// Hardware configuration
//
const int buttonPin = A1; // Button for triggering sends
const int ldrPin = A5; // 
const int speakerPin = 5; // speaker pin is located underneath the atmega
#define NUMPIXELS 3 // Number of LEDs in strip
// Here's how to control the LEDs from any two pins:
#define DATAPIN    4
#define CLOCKPIN   3
Adafruit_DotStar strip = Adafruit_DotStar(NUMPIXELS, DATAPIN, CLOCKPIN,DOTSTAR_BGR);

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10 
RF24 radio(9,10);
// Topology pipe adress
const uint64_t pipe = 0xF0F0F0F0E1LL;
byte buffer[PAYLOAD_SIZE];

// Sleep counter.
volatile int justSlept = 1;

// Timing variables.
unsigned long receivedTime = 0;

// Daylight detection averaging variables
const int numLdrReadings = 15;
// Fix that pixi starts awake
// the readings from the analog input
int ldrReadings[] = {
  300,300,300,300,300,300,300,300,300,300,300,300,300,300,300};
int ldrIndex = 0;                  // the index of the current reading
int ldrTotal = 15*300; // the running total initialized to start awake.
int ldrAverage = 0;                // the average

// Local fade variables
byte fadeIntensity;
bool fadeDirection;
int fadeStepTime;
unsigned long lastFade;
#define FADINGIN 0
#define FADINGOUT 1


byte isFading = 0;

// For gamma correcting the LED brightness
// Human eyes don't preceive lightlevels as linear as a programmer might like.
const byte Gamma[256] PROGMEM = {
  0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,
  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  3,  3,  3,  3,  3,
  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  4,
  4,  4,  4,  4,  4,  4,  5,  5,  5,  5,  5,  5,  5,  5,  5,  6,
  6,  6,  6,  6,  6,  6,  6,  7,  7,  7,  7,  7,  7,  8,  8,  8,
  8,  8,  8,  9,  9,  9,  9,  9, 10, 10, 10, 10, 10, 11, 11, 11,
  11, 12, 12, 12, 12, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 16,
  16, 16, 17, 17, 18, 18, 18, 19, 19, 20, 20, 21, 21, 21, 22, 22,
  23, 23, 24, 24, 25, 25, 26, 27, 27, 28, 28, 29, 30, 30, 31, 32,
  32, 33, 34, 35, 35, 36, 37, 38, 39, 39, 40, 41, 42, 43, 44, 45,
  46, 47, 48, 49, 50, 51, 52, 53, 55, 56, 57, 58, 59, 61, 62, 63,
  65, 66, 68, 69, 71, 72, 74, 76, 77, 79, 81, 82, 84, 86, 88, 90,
  92, 94, 96, 98,100,102,105,107,109,112,114,117,119,122,124,127,
  130,133,136,139,142,145,148,151,155,158,162,165,169,172,176,180,
  184,188,192,196,201,205,210,214,219,224,229,234,239,244,250,255,
};

void setup(void){
  Serial.begin(9600);
  printf_begin();  // strart the print.h
  Serial.println("PIXI started");

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(speakerPin, OUTPUT);
  pinMode(ldrPin, INPUT);

  // Let the user know we are started by doing a led marquee.
  strip.begin(); 
  ledMarquee();

  setupSleep();

  setupRadio();

  // Marquee again to make sure we didn't crash :-S
  ledMarquee();  
}

void loop(void) {
  // Sleep on daylight OR if monkeyLives (test mode).
  // Todo: daylight gets called too fast at nighttime, some timerstuff needed.
  while(dayLight() && !monkeyLives ) {
    if (justSlept == 0) {
      radio.powerDown();  
#ifdef DEBUGSLEEP
      Serial.println("Radio off");
      delay(20); // Wait for the serial message to go through.
#endif
    }
#ifdef DEBUGSLEEP
    Serial.println("Sleep");
    delay(20); // Wait for the serial message to go through.
#endif
    /* Re-enter sleep mode. */
    enterSleep();
  }
  if (justSlept) {
#ifdef DEBUGSLEEP
    Serial.print(justSlept);
    Serial.println(" Woke up");
    delay(20); // Wait for the serial message to go through.
#endif   
    stopSleep();
    setupRadio();
    // Delay so radio settles.
    delay(100);
    justSlept = 0;
  }  

  updateFade();

  // if there is data ready
  if (radio.available() && !isFading ) {
    // Dump the payloads until we've gotten everything
    getMessage();
    // First, stop listening so we can send later.
    radio.stopListening();

    // If the battery level was requested, show it through the leds.
    if (batteryLevel) {
      batteryStatus();
    }    
    else {
      if (cricket) {
        playCricket(); // CHIRP 
      }      
      startFadeIn();
      isFading = 1;
    }

    receivedTime = millis();
  }

  if (receivedTime + fadeSpeedIn * 10 + ledTime * 10 <= millis()) {
    startFadeOut();
  }

  if (receivedTime + rest * 10 <= millis()) {
    if (intensity > fadeOut) {
      sendMessage();
      // Delay after sending the packet (wait for it to be far, far away, prevents loopback).
      delay(rest*10*3);
    }
    // Resume listening so we catch the next packets.

    // David is like Olav :-P
    radio.read(&buffer, PAYLOAD_SIZE);
    radio.read(&buffer, PAYLOAD_SIZE);
    radio.read(&buffer, PAYLOAD_SIZE);
    radio.read(&buffer, PAYLOAD_SIZE);
    radio.read(&buffer, PAYLOAD_SIZE);
    radio.read(&buffer, PAYLOAD_SIZE);
    radio.read(&buffer, PAYLOAD_SIZE);
    radio.read(&buffer, PAYLOAD_SIZE);

    radio.startListening();    
    isFading = 0;
  }

  if (!digitalRead(buttonPin)) {
    // First, stop listening so we can send later.
    radio.stopListening();

    // Set standard values, this PIXI is an origin.
    time = 10; 
    colorShift = 1;
    rest = 50;
    broad = 127;
    center = 127;
    frequency = 0.3;
    cricket = 1;
    intensity = 255;
    batteryLevel=0;
    ledTime = 50;
    fadeOut = 20; 

    sendMessage();
    // Delay after sending the packet (wait for it to be far, far away, prevents loopback).
    delay(rest*10*3);
    // Now, continue listening
    radio.startListening();
  }
}

// read batt level  
long readVcc() { 
  long result;
  // Read 1.1V reference against AVcc
  // Todo: does this work together with reading the LDR on A5 or is more setup needed?
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  //Serial.println( result, DEC );
  return result;
}

void rainbow() { 
  // Based on: http://krazydad.com/tutorials/makecolors.php
  time = time + colorShift;

  intensity = intensity - fadeOut; 

  red   = sin(frequency* time + 0 )     * broad + center;
  green = sin(frequency* time + 2*PI/3) * broad + center;
  blue  = sin(frequency* time + 4*PI/3) * broad + center;

  red   = (red* intensity)/255;
  green = (green* intensity)/255;
  blue  = (blue* intensity)/255;
  /*
 Serial.print(red);
   Serial.print("red");
   Serial.print(green);
   Serial.print("green"); 
   Serial.print(blue);
   Serial.print("blue");
   */
  if (time > 64.0){
    time=0;    
  }
}

void makeColor(byte redSend, byte greenSend, byte blueSend ) {
  for (int i = 0; i < NUMPIXELS; i++ ) {
    strip.setPixelColor(i, strip.Color(Gamma[redSend], Gamma[greenSend], Gamma[blueSend]));
  }     
  strip.show();
}

/// piezo speaker kreak
void playCricket() { 
  // tone(speakerPin, 200, 1000);
  analogWrite(speakerPin, 255);
  delay(piezoLow);
  analogWrite(speakerPin, 0);
  delay(piezoLow*random(2  ));
  analogWrite(speakerPin, 255);
  delay(piezoLow*random(2));
  analogWrite(speakerPin, 0);
  delay(piezoLow*random(10));
  analogWrite(speakerPin, 255);
  delay(piezoLow);
  analogWrite(speakerPin, 0);
}


void batteryStatus(){
  int battery = readVcc(); // sample battery status
  //  Serial.println (battery);  // print battery status
  //  Serial.println ("battery");  // print battery status
  if ( battery < 3000) {
#ifdef DEBUG
    Serial.println ("red");
#endif
    makeColor(255,0,0);
    delay(2000);
    makeColor(0,0,0);
    delay(4000);
  }
  if (battery > 3000 && battery < 3500 ) {
    delay(2000);
#ifdef DEBUG
    Serial.println ("orange");
#endif
    makeColor(255,127,0);
    delay(2000);
    makeColor(0,0,0);
    delay(4000);
  }
  if (battery > 3500) {
    delay (4000);
#ifdef DEBUG
    Serial.println ("green");
#endif
    makeColor(0,255,0);
    delay (2000);
    makeColor(0,0,0);
    delay(4000);
  }

}

void ledMarquee() {
  strip.setPixelColor(0, strip.Color(255, 0, 0));
  strip.show();
  delay(100);
  strip.setPixelColor(1, strip.Color(255, 255, 255));
  strip.show();
  delay(100);
  strip.setPixelColor(2, strip.Color(255, 255, 255));
  strip.show();
  delay(100);
  strip.setPixelColor(0, strip.Color(0, 0, 0));
  strip.setPixelColor(1, strip.Color(0, 0, 0));
  strip.setPixelColor(2, strip.Color(0, 0, 0));
  strip.show();
}

void sendMessage() {
  // Prepare buffer for transmit.
  bitField = cricket << 7 |batteryLevel << 6 | (character & 0x0F) << 2 | findMyPixi << 1 | monkeyLives;  
  buffer[0] = (byte) time;         // Only the whole number of time
  buffer[1] = ((time - (long) time)*100);   // Only the fraction of time
  buffer[2] = frequency * 100;   // Loses precision, no problem :)
  buffer[3] = broad;
  buffer[4] = center;
  buffer[5] = intensity;
  buffer[6] = colorShift * 10000;  // Loses precision, no problem :)
  buffer[7] = fadeOut;
  buffer[8] = fadeSpeedIn;
  buffer[9] = fadeSpeedOut;
  buffer[10] = ledTime;
  buffer[11] = rest;
  buffer[12] = restSpeed;
  buffer[13] = future;
  buffer[14] = bitField;

#ifdef DEBUGSEND
  Serial.println("\nNow sending ");
  for (int i = 0; i < PAYLOAD_SIZE; i++) {
    Serial.print(i);
    Serial.print(": ");
    Serial.println(buffer[i], BIN);  
  }
#endif

  // Send the packet multiple times.
  bool ok = radio.write(&buffer, sizeof(buffer));
  // If at first you don't succeed, try again :-D
  ok = radio.write(&buffer, sizeof(buffer));
  // Olav Huizer, dumb solutions to intelligent problems.
  ok = radio.write(&buffer, sizeof(buffer));
  // If at first you don't succeed, try again :-D    
  ok = radio.write(&buffer, sizeof(buffer));
  // Olav Huizer, dumb solutions to intelligent problems.
  ok = radio.write(&buffer, sizeof(buffer));    
  delay(6);    // Herman wanted a delay... *sigh*
  ok = radio.write(&buffer, sizeof(buffer));
  delay(12);   // Herman wanted a delay... *sigh*    
  ok = radio.write(&buffer, sizeof(buffer));

  // Debug info.
  if (ok) {
    Serial.println("ok...");
  }
  else {
    Serial.println("failed.");
  }

  Serial.println("Sent response.");  
  // Sending done, little #hack to stop re-sending.
  intensity = 0;
}

void getMessage() {
  bool done = false;

  // This is now too fast :(
  //  Todo: Commented out, only interested in first packet. 
  //while (!done) {
  // Fetch the payload, and see if this was the last one.
  done = radio.read(&buffer, PAYLOAD_SIZE);

  // Spew it (debugging)
#ifdef DEBUGRECEIVE
  Serial.println("Got payload");
  for (int i = 0; i < PAYLOAD_SIZE; i++) {
    Serial.print(i);
    Serial.print(": ");
    Serial.println(buffer[i], BIN);  
  }
#endif
  //}  
  // Convert the buffer to usable variables.
  time = (float)(buffer[0]) + (float) buffer [1]/100;
  frequency = (float) buffer[2] / 100; 
  broad = buffer[3];
  center = buffer[4];
  intensity = buffer[5];
  colorShift = (float) buffer[6] / 10000;
  fadeOut = buffer[7];
  fadeSpeedIn = buffer[8];
  fadeSpeedOut = buffer[9];
  ledTime = buffer[10];
  rest = buffer[11];
  restSpeed = buffer[12]; // Signed??
  future = buffer[13];
  bitField = buffer[14];
  cricket = (bitField & 0x80) >> 7;
  findMyPixi = (bitField & 0x2) >> 1;
  batteryLevel = (bitField & 0x40) >> 6;
  monkeyLives = (bitField & 0x1);
  character = (bitField & 0x3c) >> 2;  


  // To prevent re-sends messing up the logic.
  radio.stopListening();
}

void startFadeIn() {
  // calculate intensitystep 
  fadeStepTime = (int)fadeSpeedIn * 10 / intensity;
  fadeIntensity = 0;
  lastFade = millis();
  fadeDirection = FADINGIN;
  rainbow();  
}

void startFadeOut() {
  // calculate intensitystep 
  fadeStepTime = (int)fadeSpeedOut * 10 / intensity;
  fadeIntensity = intensity;
  lastFade = millis();
  fadeDirection = FADINGOUT;
}

void updateFade() {
  int timeSincelLastFade = millis() - lastFade;
  if (timeSincelLastFade < fadeStepTime) {
    return;
  }
  int fadeAmount = timeSincelLastFade / fadeStepTime;
  if (fadeDirection == FADINGIN) {
    if (fadeIntensity + fadeAmount >= intensity) {
      fadeIntensity = intensity;
      return;
    }
    fadeIntensity += fadeAmount;
  }
  else { // FADEOUT
    if (fadeIntensity - fadeAmount <= 0) {
      fadeIntensity = 0;      
      return;
    }
    fadeIntensity -= timeSincelLastFade / fadeStepTime;        
  }
  lastFade = millis();
}

void enterSleep(void)
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   /* EDIT: could also use SLEEP_MODE_PWR_DOWN for lowest power consumption. */
  sleep_enable();

  /* Now enter sleep mode. */
  sleep_mode();

  /* The program will continue from here after the WDT timeout*/
  sleep_disable(); /* First thing to do is disable sleep. */

  /* Re-enable the peripherals. */
  power_all_enable();
}

ISR(WDT_vect)
{
  justSlept++; // Semi sleep time counter in seconds.
}

void setupRadio () {
  radio.begin(); // start rf
  // Recover from sleep.
  radio.powerUp();

  // other radio setup:
  radio.setRetries(15,15); // amount of retries
  radio.setPALevel(RF24_PA_MIN); // range level
  radio.setAutoAck(0); // Don't wait nor send automatic response (dumb communication).
  radio.setDataRate(RF24_250KBPS); // set the datarate
  radio.setPayloadSize(PAYLOAD_SIZE); // amout of Bytes sending
  // Open pipes to other nodes for communication
  //
  radio.openReadingPipe(0,pipe);
  radio.openWritingPipe(pipe);  
  ledMarquee();  
  // Start listening
  radio.startListening();  

#ifdef DEBUG
  radio.printDetails();
#endif
}

bool dayLight () {
  // subtract the last reading:
  ldrTotal = ldrTotal - ldrReadings[ldrIndex];
  // read from the sensor:
  ldrReadings[ldrIndex] = analogRead(ldrPin);
  // add the reading to the total
  ldrTotal= ldrTotal + ldrReadings[ldrIndex];
  // advance to the next position in the array:
  ldrIndex = ldrIndex + 1;

  // if we're at the end of the array...
  if (ldrIndex >= numLdrReadings) {      
    // ...wrap around to the beginning: 
    ldrIndex = 0;              
  }    

  // calculate the average:
  int ldrLevel = ldrTotal / numLdrReadings;         
#ifdef DEBUGSLEEP
  Serial.println(ldrLevel);
#endif
  if (ldrLevel < 300) {
    return 1;
  }
  else {
    return 0;
  }
}

void setupSleep() {
  /* Clear the reset flag. */
  MCUSR &= ~(1<<WDRF);

  /* In order to change WDE or the prescaler, we need to
   * set WDCE (This will allow updates for 4 clock cycles).
   */
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  /* set new watchdog timeout prescaler value */
  WDTCSR = 1<<WDP3; /* 4.0 seconds */

  /* Enable the WD interrupt (note: no reset). */
  WDTCSR |= _BV(WDIE);
}

void stopSleep() {
  /* In order to change WDE or the prescaler, we need to
   * set WDCE (This will allow updates for 4 clock cycles).
   */
  WDTCSR |= (1<<WDCE);
  // Disable the WD interrupt. 
  WDTCSR &= ~(_BV(WDIE));
}




