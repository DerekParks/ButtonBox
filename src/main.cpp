#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/power.h>

#include <FastLED.h>
#include <DebounceEvent.h>

#define LED_PIN 12
#define NUM_LEDS 5
#define BRIGHTNESS 255

#define SLEEP_TIMEOUT 60000 // 1 minute in milliseconds

const uint8_t btnPin[NUM_LEDS] = {6, 5, 4, 3, 2};

CRGB leds[NUM_LEDS];
DebounceEvent *btnEvt[NUM_LEDS];

unsigned long lastActivityTime = 0;
bool isSleeping = false;

const CRGB RAINBOW_COLORS[] = {
    CRGB::Yellow,
    CRGB::Green,
    CRGB::Red,
    CRGB::Purple,
    CRGB::Orange,
};

uint8_t buttonToLed(uint8_t pin)
{
  for (int i = 0; i < 5; i++)
  {
    if (btnPin[i] == pin)
    {
      return i;
    }
  }
  return 0;
}

uint8_t ledsOn = 0;

void callback(uint8_t pin, uint8_t event, uint8_t count, uint16_t length)
{
  lastActivityTime = millis();
  const uint8_t led2Set = buttonToLed(pin);
  if (event == 2)
  {
    if (leds[led2Set] == RAINBOW_COLORS[led2Set])
    {
      leds[led2Set] = CRGB::Black;
      ledsOn--;
    }
    else
    {
      leds[led2Set] = RAINBOW_COLORS[led2Set];
      ledsOn++;
    }
  }

  Serial.print("Led: ");
  Serial.print(led2Set);
  Serial.print(" Pin : ");
  Serial.print(pin);
  Serial.print(" Event : ");
  Serial.print(event);
  Serial.print(" Count : ");
  Serial.print(count);
  Serial.print(" Length: ");
  Serial.print(length);
  Serial.println();
}

void wakeUp()
{
  // This function will be called when an interrupt triggers
  sleep_disable(); // First thing to do is disable sleep
  isSleeping = false;
}

void enterSleepMode()
{
  Serial.println("Entering sleep mode...");

  // Clear all LEDs before sleeping
  FastLED.clear();
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB::Black;
  }
  FastLED.show();
  ledsOn = 0;

  // Configure pins for low power
  for (byte i = 0; i <= A5; i++)
  {
    if (i == 2 || i == 3 || i == 4 || i == 5 || i == 6)
    {
      // Skip our button pins - configure them as input with pullup for wake
      pinMode(i, INPUT_PULLUP);
    }
    else
    {
      pinMode(i, OUTPUT);
      digitalWrite(i, LOW);
    }
  }

  // Set up interrupts for buttons (PCINT for pins 2-6)
  PCICR |= (1 << PCIE2);                                                                        // Enable PCINT for PCINT16-23 (PORTD - pins 0-7)
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19) | (1 << PCINT20) | (1 << PCINT21) | (1 << PCINT22); // Enable specific pins (2-6)

  // Disable unused modules
  ADCSRA = 0;
  power_adc_disable(); // ADC converter
  power_spi_disable(); // SPI
  power_twi_disable(); // TWI (I2C)

  // Set up sleep mode
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  // Enter sleep mode
  noInterrupts();
  sleep_enable();
  interrupts();
  isSleeping = true;
  sleep_cpu();

  // Code continues here after waking up
  Serial.println("Waking up from sleep!");

  // Re-enable only the required modules
  power_spi_enable(); // Likely needed by FastLED
  // We don't re-enable ADC or TWI since they're not needed for this application

  // Restore pin configurations
  for (int i = 0; i < 5; i++)
  {
    delete btnEvt[i]; // Delete old button instances
    btnEvt[i] = new DebounceEvent(btnPin[i], callback, BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);
  }

  // Reset activity timer
  lastActivityTime = millis();
}

// Pin change interrupt for pins 0-7 (buttons 2-6)
ISR(PCINT2_vect)
{
  wakeUp(); // Wake up when any button is pressed
}

void setup()
{
  Serial.begin(115200);
  FastLED.addLeds<WS2812B, LED_PIN, RGB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

  FastLED.clear();
  FastLED.show();

  for (int i = 0; i < 5; i++)
  {
    btnEvt[i] = new DebounceEvent(btnPin[i], callback, BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);
  }

  FastLED.setBrightness(BRIGHTNESS);

  // disable ADC
  ADCSRA = 0;

  power_adc_disable(); // ADC converter
  power_twi_disable(); // TWI (I2C)

  lastActivityTime = millis();
}

uint8_t hue = 0;

void loop()
{
  for (int i = 0; i < 5; i++)
  {
    btnEvt[i]->loop();
  }

  // Check if it's time to sleep
  if (!isSleeping && (millis() - lastActivityTime > SLEEP_TIMEOUT))
  {
    enterSleepMode();
  }

  if (ledsOn == 5)
  {

    for (int i = 0; i < 500; i++)
    {
      fill_rainbow(leds, NUM_LEDS, hue, 7);
      hue++;
      FastLED.show();
      delay(5);
    }

    ledsOn = 0;
    for (int i = 0; i < 5; i++)
    {
      leds[i] = CRGB::Black;
    }
    FastLED.clear();
  }
  FastLED.show();
}
