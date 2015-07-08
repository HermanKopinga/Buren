#include <SPI.h>
#include "nRF24L01.h" // rf lib
#include "RF24.h" // rf lib
#include <Adafruit_DotStar.h>
#include "printf.h"// used for debuggin the radio.print



// PIXI dust
#define PAYLOAD_SIZE 9

byte red = 0;
byte green = 0;
byte blue = 0;
byte rest = 0;
byte mutation = 0;
byte lifeTime = 0;
byte colorShift = 0;
byte future = 0;
byte thingies = 0;
bool findMyPixi = 0;
bool batteryLevel = 0;
byte character = 0;
bool monkeyLives = 0;
byte buffer[PAYLOAD_SIZE];


// variables voor color shift
float frequency = .3; // color wheel
float time; // color wheel
float redshift;
float greenshift;
float blueshift;
byte redB = 255;
byte blueB= 0;
byte greenB= 0;
// bateri life

//
// Hardware configuration
//
const int speakerPin = A1;
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
  
Serial.begin(9600);
printf_begin();  // strart the print.h
pinMode(speakerPin, OUTPUT);

strip.begin();  
strip.show(); // Initialize all pixels to 'off'
  
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
  if ( radio.available() )
  {
    // Dump the payloads until we've gotten everything
    unsigned long got_time;
    bool done = false;

    while (!done)
    {
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

    red = buffer[0];
    green = buffer[1]; 
    blue = buffer[2];
    rest = buffer[3];
    mutation = buffer[4];
    lifeTime = buffer[5];
    colorShift = buffer[6];
    future = buffer[7];
    thingies = buffer[8];
    findMyPixi = (thingies & 0x2) >> 1;
    batteryLevel = (thingies & 0x40) >> 6;
    monkeyLives = (thingies & 0x1);
    character = (thingies & 0x3c) >> 2;

    
//  digitalWrite(ledPin, HIGH);
//  strip.setPixelColor(0, strip.Color(255, 255, 255));
//  strip.setPixelColor(1, strip.Color(255, 255, 255));
//  
//   strip.show();
//   

    rainbow();
    makeColor(red,green,blue);

    // First, stop listening so we can talk
    radio.stopListening();

    delay(rest*10);
    makeColor(0,0,0);
//   digitalWrite(ledPin, LOW);
//    strip.setPixelColor(0, strip.Color(0, 0, 0));
//    strip.setPixelColor(1, strip.Color(0, 0, 0));
//    strip.show();    

    // Prepare buffer for transmit.
    thingies = batteryLevel << 6 | (character & 0x0F) << 2 | findMyPixi << 1 | monkeyLives;  
    buffer[0] = red;
    buffer[1] = green; 
    buffer[2] = blue;
    buffer[3] = rest;
    buffer[4] = mutation;
    buffer[5] = lifeTime;
    buffer[6] = colorShift;
    buffer[7] = future;
    buffer[8] = thingies;
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
}

 
void makeColor(byte red,byte green, byte blue) {
  
  for (int i = 0; i < NUMPIXELS; i++ ){
    strip.setPixelColor(i, strip.Color(red, green, blue));
    strip.show();
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
  time = time++;
  red = sin(frequency* time + 0 +redB) * 127 + 128;
  green = sin(frequency* time + 2+greenB) * 127 + 128;
  blue  = sin(frequency* time + 4+blueB) * 127 + 128;
 
 Serial.print(red);
 Serial.print("red");
 Serial.print(green);
 Serial.print("green"); 
 Serial.print(blue);
 Serial.print("blue");
  //if (time > 64.0){
    //time=0;
 // }
}
 
 
