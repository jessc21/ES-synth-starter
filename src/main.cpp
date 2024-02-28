#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <cmath>
#include <STM32FreeRTOS.h>

// Constants
const uint32_t interval = 100; // Display update interval

// Pin definitions
// Row select and enable
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

// Matrix input and output
const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

// Audio analogue out
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

// Joystick analogue in
const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

// Output multiplexer bits
const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

volatile uint32_t currentStepSize;

// Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

struct
{
  std::bitset<32> inputs;
} sysState;

std::bitset<4> readCols()
{
  std::bitset<4> result;
  result[0] = digitalRead(C0_PIN);
  result[1] = digitalRead(C1_PIN);
  result[2] = digitalRead(C2_PIN);
  result[3] = digitalRead(C3_PIN);
  return result;
}

// select a given row of the switch matrix by setting the value of each row select address pin.
void setRow(uint8_t rowIdx)
{
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, rowIdx & 0x02);
  digitalWrite(RA2_PIN, rowIdx & 0x04);
  digitalWrite(REN_PIN, HIGH);
}

// Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value)
{
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, bitIdx & 0x01);
  digitalWrite(RA1_PIN, bitIdx & 0x02);
  digitalWrite(RA2_PIN, bitIdx & 0x04);
  digitalWrite(OUT_PIN, value);
  digitalWrite(REN_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(REN_PIN, LOW);
}

// Write a new function that will update the phase accumulator and
// set the analogue output voltage at each sample interval
// The function will be triggered by an interrupt 22,000 times per second.
// It will add currentStepSize to the phase accumulator to generate the output waveform.
void sampleISR()
{
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  analogWrite(OUTR_PIN, Vout + 128);
}

// Function to calculate step size based on frequency
uint32_t calculateStepSize(float frequency)
{
  return static_cast<uint32_t>((std::pow(2, 32) * frequency) / 22000);
}

// Define frequencies for each note (in Hz)
const float noteFrequencies[] = {
    261.63, // C
    277.18, // C#
    293.66, // D
    311.13, // D#
    329.63, // E
    349.23, // F
    369.99, // F#
    392.00, // G
    415.30, // G#
    440.00, // A
    466.16, // A#
    493.88  // B
};

// Calculate and store step sizes for each note
const uint32_t stepSizes[] = {
    calculateStepSize(noteFrequencies[0]),  // C
    calculateStepSize(noteFrequencies[1]),  // C#
    calculateStepSize(noteFrequencies[2]),  // D
    calculateStepSize(noteFrequencies[3]),  // D#
    calculateStepSize(noteFrequencies[4]),  // E
    calculateStepSize(noteFrequencies[5]),  // F
    calculateStepSize(noteFrequencies[6]),  // F#
    calculateStepSize(noteFrequencies[7]),  // G
    calculateStepSize(noteFrequencies[8]),  // G#
    calculateStepSize(noteFrequencies[9]),  // A
    calculateStepSize(noteFrequencies[10]), // A#
    calculateStepSize(noteFrequencies[11])  // B
};

void scanKeysTask(void *pvParameters)
{
  const TickType_t xFrequency = 50 / portTICK_PERIOD_MS; // initiation interval: 50ms
  TickType_t xLastWakeTime = xTaskGetTickCount();
  // define a local currentStepSize variable to store the current step size
  uint32_t currentStepSizeLocal;
  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    currentStepSizeLocal = 0;
    // key scanning loop and it loops over the row numbers 0 to 2.
    for (int i = 0; i < 3; i++)
    {
      setRow(i);
      delayMicroseconds(3);
      std::bitset<4> temp = readCols();
      sysState.inputs[i * 4] = temp[0];
      sysState.inputs[i * 4 + 1] = temp[1];
      sysState.inputs[i * 4 + 2] = temp[2];
      sysState.inputs[i * 4 + 3] = temp[3];
    }

    // Add code to your main loop that will check the state of each key in inputs and
    // look up the corresponding step size in the stepSizes array if the key is pressed
    //  If the key is not pressed, the step size should be set to 0
    if (sysState.inputs == 0xFFF)
    {
      currentStepSize = 0;
    }
    for (int i = 0; i < 12; i++)
    {
      if (!sysState.inputs[i])
      {
        currentStepSizeLocal = stepSizes[i];
        break; // Exit the loop once a key is pressed
      }
    }
    __atomic_store_n(&currentStepSize, currentStepSizeLocal, __ATOMIC_RELAXED);
  }
}

void displayUpdateTask(void *pvParameters)
{
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS; // initiation interval: 100ms
  TickType_t xLastWakeTime = xTaskGetTickCount();
  bool notePrinted;
  const char *notes[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    notePrinted = false;
    // Update display
    u8g2.clearBuffer();                   // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr);   // choose a suitable font
    u8g2.drawStr(2, 10, "Helllo World!"); // write something to the internal memory
    u8g2.setCursor(2, 20);
    u8g2.print(sysState.inputs.to_ulong(), HEX);
    for (int i = 0; i < 12; i++)
    {
      if (!sysState.inputs[i])
      {
        if (!notePrinted)
        {
          // Print the note if it hasn't been printed yet
          u8g2.setCursor(2, 30);
          u8g2.print(notes[i]);
          // Serial.println(notes[i]);
          notePrinted = true; // Set the flag to indicate the note has been printed
        }
        break; // Exit the loop once a key is pressed
      }
    }
    u8g2.sendBuffer(); // transfer internal memory to the display
    // digitalToggle(LED_BUILTIN);
  }
}

void setup()
{
  // put your setup code here, to run once:

  // Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  // Initialise timer interrupt
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  // Initialise display
  setOutMuxBit(DRST_BIT, LOW); // Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH); // Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH); // Enable display power supply

  // Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  // initialise and run the thread
  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
      displayUpdateTask,
      "displayUpdate",
      256,
      NULL,
      1,
      &displayUpdateHandle);

  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
      scanKeysTask,     /* Function that implements the task */
      "scanKeys",       /* Text name for the task */
      64,               /* Stack size in words, not bytes */
      NULL,             /* Parameter passed into the task */
      2,                /* Task priority */
      &scanKeysHandle); /* Pointer to store the task handle */

  // start the RTOS scheduler
  vTaskStartScheduler();
}

void loop() {}