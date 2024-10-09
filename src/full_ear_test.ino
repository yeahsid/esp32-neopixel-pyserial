#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define ser_timeout 10 // millis for val to arrive after key

// Hardware Serial 2 pins
#define RXD2 16
#define TXD2 17

#define L_EAR_LED 15
#define R_EAR_LED 0
#define LED_COUNT 17

// Encoder pins
#define L_ENCODE_A 18
#define L_ENCODE_B 5

#define BASE_ADDRESS 0x08 // Arduino I2C address

#define DATA_SIZE 7

int command = 0;
int data[DATA_SIZE]; // Array to store the received data bytes

// Variables for encoder state
volatile int32_t encoderPosition = 0;
volatile bool aLastState = LOW;
volatile bool aCurrentState = LOW;

// Declare our NeoPixel strip object:
Adafruit_NeoPixel LED(LED_COUNT, L_EAR_LED, NEO_GRB + NEO_KHZ800);

// Struct definition for RGB
struct __attribute__((__packed__)) colorObj
{
  uint8_t red, green, blue , brightness;
};

colorObj color = {0, 0, 0 , 50};

void setup()
{
  initializeSerial();
  initializeLED();
  initializeEncoder();
  Serial.print("Program started\n");
}

void loop()
{
  static unsigned long lastUpdate = 0;
  const unsigned long updateInterval = 1000; // Update every second

  // Non-blocking delay
  if (millis() - lastUpdate >= updateInterval)
  {
    lastUpdate = millis();
    colorWipe(LED.Color(color.red, color.green, color.blue), 1);
  }

  // Check for serial data
  if (Serial2.available())
  {
    readColorFromSerial();
    setLEDColor();
  }
}

void initializeSerial()
{
  Serial.begin(9600);                            // Initialize Serial for debugging
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); // Hardware Serial of ESP32
  Serial2.setTimeout(1);
}

void initializeLED()
{
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif

  LED.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  LED.show();            // Turn OFF all pixels ASAP
  LED.setBrightness(color.brightness); // Set BRIGHTNESS to about 1/5 (max = 255)
}

void initializeEncoder()
{
  pinMode(L_ENCODE_A, INPUT_PULLUP); // Use internal pull-up resistor
  pinMode(L_ENCODE_B, INPUT_PULLUP); // Use internal pull-up resistor

  // Attach interrupts to the encoder pins
  attachInterrupt(digitalPinToInterrupt(L_ENCODE_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(L_ENCODE_B), updateEncoder, CHANGE);
}

void parseSerial()
{
  // Check if there is data available on Serial2
  if (Serial2.available())
  {
    // Read key byte
    command = Serial2.read();

    // Read up to DATA_SIZE bytes for data section
    int index = 0;
    while (Serial2.available() && index < DATA_SIZE)
    {
      data[index++] = Serial2.read();
    }

    // Print received key and data for debugging
    Serial.print("Received command: ");
    Serial.println(command);
    Serial.print("Received data: ");
    for (int i = 0; i < index; i++)
    {
      Serial.print(data[i]);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void updateEncoder()
{
  aCurrentState = digitalRead(L_ENCODE_A);
  // If the A state has changed
  if (aCurrentState != aLastState)
  {
    // Check if the B state is the same as the A state
    if (digitalRead(L_ENCODE_B) != aCurrentState)
    {
      encoderPosition++;
    }
    else
    {
      encoderPosition--;
    }
  }
  aLastState = aCurrentState;
}

void readColorFromSerial()
{
  Serial2.readBytes((char *)&color, sizeof(colorObj));
  Serial.print("Received color - R: ");
  Serial.print(color.red);
  Serial.print(", G: ");
  Serial.print(color.green);
  Serial.print(", B: ");
  Serial.println(color.blue);
}

void setLEDColor()
{
  // Set the LED color
  LED.setPixelColor(0, LED.Color(color.red, color.green, color.blue));
  LED.show();
}

void colorWipe(uint32_t color, int wait)
{
  for (int i = 0; i < LED.numPixels(); i++)
  {                              // For each pixel in strip...
    LED.setPixelColor(i, color); // Set pixel's color (in RAM)
    LED.show();                  // Update strip to match
    delay(wait);                 // Pause for a moment
  }
}