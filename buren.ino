/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

/**
 * Example for Getting Started with nRF24L01+ radios. 
 *
 * This is an example of how to use the RF24 class.  Write this sketch to two 
 * different nodes.  Put one of the nodes into 'transmit' mode by connecting 
 * with the serial monitor and sending a 'T'.  The ping node sends the current 
 * time to the pong node, which responds by sending the value back.  The ping 
 * node can then see how long the whole cycle took.
 */
//Yolo
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Adafruit_DotStar.h>


//
// Hardware configuration
//
const int ledPin = A0;
const int buttonPin = A1;
Adafruit_DotStar strip = Adafruit_DotStar(2, 3, 4);

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10 

RF24 radio(9,10);

//
// Topology
//

const uint64_t pipe = 0xF0F0F0F0E1LL;

//
// Role management
//
// Set up role.  This sketch uses the same software for all the nodes
// in this system.  Doing so greatly simplifies testing.  
//

// The various roles supported by this sketch
typedef enum { role_ping_out = 1, role_pong_back } role_e;

// The debug-friendly names of those roles
const char* role_friendly_name[] = { "invalid", "Ping out", "Pong back"};

// The role of the current running sketch
role_e role = role_ping_out;

void setup(void)
{
  pinMode(ledPin, OUTPUT);
  // Blink to welcome our new robot overloards.
  digitalWrite(ledPin, HIGH);
  
  pinMode(buttonPin, INPUT_PULLUP);
  strip.begin();  
  strip.show(); // Initialize all pixels to 'off'
     
  //
  // Print preamble
  //

  Serial.begin(9600);
  Serial.println("\n\rRF24/examples/GettingStarted/");

  digitalWrite(ledPin, LOW);
  //
  // Setup and configure rf radio
  //

  radio.begin();

  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15,15);
  radio.setPALevel(RF24_PA_MIN);
  radio.setAutoAck(0);
  // optionally, reduce the payload size.  seems to
  // improve reliability
//  radio.setPayloadSize(8);

  //
  // Open pipes to other nodes for communication
  //
  radio.openReadingPipe(0,pipe);
  radio.openWritingPipe(pipe);  

  //
  // Start listening
  //
  radio.startListening();
  digitalWrite(ledPin, HIGH);
  //
  // Dump the configuration of the rf unit for debugging
  //
  radio.printDetails();
  // Blink to welcome our new robot overloards.
  digitalWrite(ledPin, LOW);
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
    
    digitalWrite(ledPin, HIGH);
    strip.setPixelColor(0, strip.Color(255, 255, 255));
    strip.setPixelColor(1, strip.Color(255, 255, 255));
    strip.show();

    while (!done)
    {
      // Fetch the payload, and see if this was the last one.
      done = radio.read(&got_time, sizeof(unsigned long) );
      
      // Spew it
      Serial.print("Got payload ");
      Serial.println(got_time);

      // Delay just a little bit to let the other unit
      // make the transition to receiver
      delay(20);
    }

    // First, stop listening so we can talk
    radio.stopListening();
    unsigned long waittime = got_time & 0xFFFF;
    delay(waittime);
    digitalWrite(ledPin, LOW);
    strip.setPixelColor(0, strip.Color(0, 0, 0));
    strip.setPixelColor(1, strip.Color(0, 0, 0));
    strip.show();    
    waittime++;
    
    // Send the final one back.
    radio.write(&got_time, sizeof(unsigned long));
    
      Serial.print("Sent response.\n\r");
    delay(waittime+200);
    // Now, resume listening so we catch the next packets.
    radio.startListening();
  }

  if (!digitalRead(buttonPin)) {
    radio.stopListening();
    // Take the time, and send it.  This will block until complete
    unsigned long time = 300;
    unsigned long intensity = 240;
    intensity = intensity << 16;
    time = time | intensity;
    Serial.print("Now sending ");
    Serial.println(time);
    bool ok = radio.write( &time, sizeof(unsigned long) );
    
    if (ok)
        Serial.println("ok...");
    else
        Serial.println("failed.");

    delay(501);
    
    // Now, continue listening
    radio.startListening();
  }
}

