#include <Arduino.h>
#include <hardware/pio.h>

#define BAUD_RATE 1000000 // 1 Megabaud link to Teensy

// Pin Assignments for SPH0645 on XIAO RP2040 (Validated from Hardware Map)
#define PIN_BCLK    29 // D1 (P27) - Bit Clock
#define PIN_WS      27 // D2 (P28) - Word Select
#define PIN_DATA    29 // D3 (P29) - Serial Data Out
#define PIN_UART_TX  0 // D6 (P0)  - Hardware Serial1 TX to RS-485

// Custom PIO assembly code designed specifically for the SPH0645 timing architecture
const uint16_t sph0645_pio_program_instructions[] = {
    0xe02f, //  0: set    x, 15
    0x30c1, //  1: wait   1 pin, 1          [4]  ; Wait for WS High (Right Channel)
    0x2003, //  2: wait   0 pio, 0          
    0x4001, //  3: in     pins, 1           
    0x0043, //  4: jmp    x--, 3            
    0xe02f, //  5: set    x, 15             
    0x3041, //  6: wait   0 pin, 1          [4]  ; Wait for WS Low (Left Channel)
    0x2003, //  7: wait   0 pio, 0          
    0x4001, //  8: in     pins, 1           
    0x0048, //  9: jmp    x--, 8            
};

const struct pio_program sph0645_pio_program = {
    sph0645_pio_program_instructions,
    10,
    -1
};

PIO pio = pio0;
uint sm = 0;

// High-speed block streaming structure
#define AUDIO_BLOCK_SIZE 32
struct AudioChunk {
  uint16_t header; // 0x55AA sync code
  int32_t samples[AUDIO_BLOCK_SIZE];
};

AudioChunk chunk;
int sampleIndex = 0;

// --- DIAGNOSTIC DEBUG VARIABLES ---
uint32_t totalSamplesRead = 0;
uint32_t totalChunksSent = 0;
int32_t debugPeakAmplitude = 0;
unsigned long lastDebugPrintTime = 0;
const unsigned long debugInterval = 500; // Print debug info every 500ms

void init_sph0645_pio() {
    uint offset = pio_add_program(pio, &sph0645_pio_program);
    pio_sm_config c = pio_get_default_sm_config();
    
    sm_config_set_in_pins(&c, PIN_DATA);
    sm_config_set_sideset_pins(&c, PIN_BCLK);
    
    pio_gpio_init(pio, PIN_DATA);
    pio_gpio_init(pio, PIN_BCLK);
    pio_gpio_init(pio, PIN_WS);
    
    pio_sm_set_consecutive_pindirs(pio, sm, PIN_DATA, 1, false);
    pio_sm_set_consecutive_pindirs(pio, sm, PIN_BCLK, 1, true);
    pio_sm_set_consecutive_pindirs(pio, sm, PIN_WS, 1, true);
    
    sm_config_set_clkdiv(&c, 125.0f); 
    sm_config_set_in_shift(&c, false, true, 32);
    
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

void setup() {
  // Initialize native USB port for PC Serial Monitor debugging
 Serial.begin(115200); 
  
  // Direct the hardware UART engine to the physical pins shown on your diagram
 delay(2000); 
  
  Serial1.setTX(PIN_UART_TX);
  Serial1.begin(BAUD_RATE);
  
  init_sph0645_pio();
  chunk.header = 0x55AA;
  
  // Give you a visual confirmation in the monitor that the board booted up
  delay(1000); 
  Serial.println("\n--- XIAO RP2040 AUDIO ENDPOINT ONLINE ---");
  Serial.print("Target Serial1 TX Pin: "); Serial.println(PIN_UART_TX);
  Serial.print("Target Baud Rate: "); Serial.println(BAUD_RATE);
  Serial.println("Reading SPH0645 via PIO state machine...");
  Serial.println("------------------------------------------");
}

void loop() {
  // Read a raw 32-bit container directly out of the PIO hardware FIFO queue
  if (!pio_sm_is_rx_fifo_empty(pio, sm)) {
    uint32_t rawSample = pio_sm_get(pio, sm);
    
    // Clean up sign extension for the SPH0645's true 24-bit resolution signed format
    int32_t processedSample = (int32_t)(rawSample << 8) >> 8;
    
    chunk.samples[sampleIndex] = processedSample;
    sampleIndex++;
    totalSamplesRead++;

    // Track highest absolute amplitude for the debug monitor summary
    int32_t absSample = abs(processedSample);
    if (absSample > debugPeakAmplitude) {
      debugPeakAmplitude = absSample;
    }
    
    // Once the block is completely full, dump it directly over the RS-485 line
    if (sampleIndex >= AUDIO_BLOCK_SIZE) {
      Serial1.write((uint8_t*)&chunk, sizeof(AudioChunk));
      sampleIndex = 0;
      totalChunksSent++;
    }
  }

  // --- NON-BLOCKING SERIAL DEBUG MONITOR ENGINE ---
  unsigned long currentTime = millis();
  if (currentTime - lastDebugPrintTime >= debugInterval) {
    lastDebugPrintTime = currentTime;
    
    // Print diagnostic status to USB Serial Monitor
    Serial.print("[STATUS] Samples Collected: ");
    Serial.print(totalSamplesRead);
    Serial.print(" | Chunks Blasted to Teensy: ");
    Serial.print(totalChunksSent);
    Serial.print(" | Window Peak Amplitude: ");
    Serial.println(debugPeakAmplitude);
    
    // Reset window accumulator metrics
    debugPeakAmplitude = 0;
  }
}