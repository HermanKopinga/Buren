#include <SPI.h>
#include "nRF24L01.h" // rf lib
#include "RF24.h" // rf lib
#include <Adafruit_DotStar.h>
#include "printf.h"// used for debuggin the radio.print

/*
 1 time (2 bytes) start time for color (fraction: 8 bits.8 bits 0-255.0-255)
 2 Frequency (byte) freq for color calcualtion / 100
 3 Broad (byte) color calculation
 4 Center (byte) range of color calculation
 5 Intensity (byte) start intensity led
 6 ColorShift (byte) Hoeveel veranderd de kleur tussen pixies / 100
 7 FadeOut (byte) Speed to fade the led across pixies to black =0 ook stop passing on
 8 FadeSpeedIn (byte) Speed to fade in the led inside the pixie
 9 FadeSpeedOut (byte) Speed to fade out led inside pixie
 10 Rest (byte) stop listening after sending
 11 RestSpeed (signed byte) to speed up or slow down sending
 12 Future (byte) future expansion
 13 Cricket (bit) turn on cricket
 15 BatteryLevel (bit) show us your batt level
 14 Character (4 bits) Caracter (its an nibble)
 15 FindMyPixi (bit) show were you are
 16 MonkyLives (bit) got mental!!!
 */

// PIXI dust
#define PAYLOAD_SIZE 14

float time = 0;
float frequency = 0;
byte broad = 0;
byte center = 0;
byte intensity = 0;
float colorShift = 0;
byte fadeOut = 0;
byte fadeSpeedIn = 0;
byte fadeSpeedOut = 0;
byte rest = 0;
byte restSpeed = 0;
byte future = 0;
byte bitField = 0;
bool cricket = 0;
bool findMyPixi = 0;
bool batteryLevel = 0;
byte character = 0;
bool monkeyLives = 0;
byte buffer[PAYLOAD_SIZE];


// piezo hight
int low = 10;

// variables voor color shift
float redshift;
float greenshift;
float blueshift;
byte red = 0;
byte green = 0;
byte blue = 0;
// battery life

//
// Hardware configuration
//
const int buttonPin = A1; // nutton for triggering sends
const int speakerPin = 5; // speaker pin is located underneeth the atmeaga
// Set up nRF24L01 radio on SPI bus plus pins 9 & 10 
RF24 radio(9,10);
#define NUMPIXELS 3 // Number of LEDs in strip
// Here's how to control the LEDs from any two pins:
#define DATAPIN    4
#define CLOCKPIN   3
Adafruit_DotStar strip = Adafruit_DotStar(NUMPIXELS, DATAPIN, CLOCKPIN);

//
// Topology pipe adress
const uint64_t pipe = 0xF0F0F0F0E1LL;


void setup(void){

  pinMode(buttonPin, INPUT_PULLUP);
  
  Serial.begin(9600);
  printf_begin();  // strart the print.h
  pinMode(speakerPin, OUTPUT);
  // beging seq led test
  strip.begin(); 
  strip.setPixelColor(0, strip.Color(255, 255, 255));
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
  // end led test

  // Initialize all pixels to 'off'

  radio.begin(); // start rf
  // other radio setups:
  radio.setRetries(15,15); // amount of retries
  radio.setPALevel(RF24_PA_MIN); // range level
  radio.setAutoAck(0); // dont wait for responce
  radio.setDataRate(RF24_250KBPS); // set the datarate
  radio.setPayloadSize(PAYLOAD_SIZE); // amout of Bytes sending

  // Open pipes to other nodes for communication
  //
  radio.openReadingPipe(0,pipe);
  radio.openWritingPipe(pipe);  

  // Start listening
  radio.startListening();

  // Serial prints
  radio.printDetails();
  // printf("lisening");

}

void loop(void)
{
  //
  // Pong back role.  Receive each packet, dump it out, and send it back
  //

  // if there is data ready
  if ( radio.available() ) {
    // Dump the payloads until we've gotten everything
    unsigned long got_time;
    bool done = false;

    while (!done) {
      // Fetch the payload, and see if this was the last one.
      done = radio.read(&buffer, PAYLOAD_SIZE);

      // Spew it
      Serial.print("Got payload ");
      for (int i = 0; i < PAYLOAD_SIZE; i++) {
        Serial.print(i);
        Serial.print(": ");
        Serial.println(buffer[i], BIN);  
      }

      // Serial.println (readVcc(), DEC);
      readVcc();
      // Delay just a little bit to let the other unit
      // make the transition to receiver
      delay(20);
    }

    time = (float)(buffer[0]) + (float) buffer [1]/100;
    frequency = (float) buffer[2] / 100; 
    broad = buffer[3];
    center = buffer[4];
    intensity = buffer[5];
    colorShift = (float) buffer[6] / 100;
    fadeOut = buffer[7];
    fadeSpeedIn = buffer[8];
    fadeSpeedOut = buffer[9];
    rest = buffer[10];
    restSpeed = buffer[11]; // Signed??
    future = buffer[12];
    bitField = buffer[13];
    cricket = (bitField & 0x80) >> 7;
    findMyPixi = (bitField & 0x2) >> 1;
    batteryLevel = (bitField & 0x40) >> 6;
    monkeyLives = (bitField & 0x1);
    character = (bitField & 0x3c) >> 2;

    rainbow();
    makeColor();
    if (cricket) {
      playCricket();// CHIRP 
    }
    // First, stop listening so we can talk
    radio.stopListening();

    delay(rest*10);

    // Prepare buffer for transmit.
    bitField = cricket << 7 |batteryLevel << 6 | (character & 0x0F) << 2 | findMyPixi << 1 | monkeyLives;  
    buffer[0] = (byte) time;         // Only the whole number of time
    buffer[1] = ((time - (long) time)*100);   // Only the fraction of time
    buffer[2] = frequency * 100;   // Loses precision, no problem :)
    buffer[3] = broad;
    buffer[4] = center;
    buffer[5] = intensity;
    buffer[6] = colorShift * 100;  // Loses precision, no problem :)
    buffer[7] = fadeOut;
    buffer[8] = fadeSpeedIn;
    buffer[9] = fadeSpeedOut;
    buffer[10] = rest;
    buffer[11] = restSpeed;
    buffer[12] = future;
    buffer[13] = bitField;    
    Serial.println("\nNow sending ");
    for (int i = 0; i < PAYLOAD_SIZE; i++) {
      Serial.print(i);
      Serial.print(": ");
      Serial.println(buffer[i], BIN);  
    }
    bool ok = radio.write(&buffer, sizeof(buffer) );

    // Send the final one back.
    radio.write(&got_time, sizeof(unsigned long));

    Serial.print("Sent response.\n\r");
    delay(rest*10+200);
    // Now, resume listening so we catch the next packets.
    radio.startListening();
  }
  
  if (!digitalRead(buttonPin)) {
    radio.stopListening();
    time = 10; 
    colorShift = .05;
    rest = 50;
    broad = 127;
    center = 127;
    frequency = 0.3;
    cricket = 1;
    intensity = 255;
    // Prepare buffer for transmit.
    bitField = cricket << 7 |batteryLevel << 6 | (character & 0x0F) << 2 | findMyPixi << 1 | monkeyLives;  
    buffer[0] = (byte) time;         // Only the whole number of time
    buffer[1] = ((time - (long) time)*100);   // Only the fraction of time
    buffer[2] = frequency * 100;   // Loses precision, no problem :)
    buffer[3] = broad;
    buffer[4] = center;
    buffer[5] = intensity;
    buffer[6] = colorShift * 100;  // Loses precision, no problem :)
    buffer[7] = fadeOut;
    buffer[8] = fadeSpeedIn;
    buffer[9] = fadeSpeedOut;
    buffer[10] = rest;
    buffer[11] = restSpeed;
    buffer[12] = future;
    buffer[13] = bitField;    
    Serial.println("\nNow sending ");
    for (int i = 0; i < PAYLOAD_SIZE; i++) {
      Serial.print(i);
      Serial.print(": ");
      Serial.println(buffer[i], BIN);  
    }
    bool ok = radio.write(&buffer, sizeof(buffer) );
  
    if (ok) {
      Serial.println("ok...");
    }
    else {
      Serial.println("failed.");
    }

    delay(rest*10*2);
    
    // Now, continue listening
    radio.startListening();
  }
}





// ------------------------ read batt level  
long readVcc() { 
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  Serial.println( result, DEC );
  return result;
}


void rainbow() { 

  //http://krazydad.com/tutorials/makecolors.php
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
//  if (time > 64.0){
//    time=0;    
//  }
}

void makeColor() {
  for (int i = 0; i < NUMPIXELS; i++ ) {
    strip.setPixelColor(i, strip.Color(red, green, blue));
    strip.show();
  }     
}

/// piezo speaker kreak
void playCricket() { 
  // tone(speakerPin, 200, 1000);
  analogWrite(speakerPin, 255);
  delay(low);
  analogWrite(speakerPin, 0);
  delay(low*random(2  ));
  analogWrite(speakerPin, 255);
  delay(low*random(2));
  analogWrite(speakerPin, 0);
  delay(low*random(10));
  analogWrite(speakerPin, 255);
  delay(low);
  analogWrite(speakerPin, 0);
}



