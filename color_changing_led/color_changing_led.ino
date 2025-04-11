#include <FastLED.h>

#define LED_PIN     7
#define NUM_LEDS    60

CRGB leds[NUM_LEDS];

void setup() {

  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  
}

void loop() {
  
  for(int i=0; i<60; i++)
  {
      leds[i] = CRGB(255, 255, 0); 
  }
        FastLED.show();
  // leds[0] = CRGB(255, 255, 0);
  // delay(500);  
  // leds[1] = CRGB(255, 255, 0);
  // delay(500);
  // leds[2] = CRGB(255, 255, 0);
  // delay(500);
  // leds[5] = CRGB(255, 255, 0);
  // delay(500);
  // leds[9] = CRGB(255, 255, 0);
  // delay(500);
  // leds[14] = CRGB(255, 255, 0);
  // delay(500);
  // leds[19] = CRGB(255, 255, 0);
  // FastLED.show();
  // delay(500);
}