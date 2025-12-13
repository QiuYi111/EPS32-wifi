/**
 * @file test_wokwi.cpp
 * @brief Simple Serial Echo Demo for Wokwi Simulation
 * 
 * This demo echoes back everything you type in the serial terminal.
 * Perfect for learning Wokwi simulation and testing serial communication.
 * 
 * Usage:
 *   1. Build: pio run
 *   2. Run in Wokwi with `wokwi-cli` or VS Code Wokwi extension
 *   3. Type anything in the terminal, it will be echoed back
 */

#include <Arduino.h>

// Buffer for storing input
#define BUFFER_SIZE 256
char inputBuffer[BUFFER_SIZE];
int bufferIndex = 0;

void setup() {
    // Initialize Serial (USB CDC on ESP32-S3)
    Serial.begin(115200);
    
    // Simple delay for serial initialization in Wokwi
    delay(1000);
    
    // Welcome message
    Serial.println();
    Serial.println("====================================");
    Serial.println("   ESP32 Serial Echo Demo");
    Serial.println("   Wokwi Simulation Test");
    Serial.println("====================================");
    Serial.println();
    Serial.println("Type anything and press Enter to echo back.");
    Serial.println("Ready and waiting for input...");
    Serial.println();
    Serial.print("> ");
}

void loop() {
    // Check if data is available on Serial
    while (Serial.available() > 0) {
        char c = Serial.read();
        
        // Handle Enter key (newline)
        if (c == '\n' || c == '\r') {
            if (bufferIndex > 0) {
                // Null-terminate the string
                inputBuffer[bufferIndex] = '\0';
                
                // Echo back the input
                Serial.println();
                Serial.print("[Echo] ");
                Serial.println(inputBuffer);
                Serial.print("> ");
                
                // Reset buffer
                bufferIndex = 0;
            }
        }
        // Handle backspace
        else if (c == '\b' || c == 127) {
            if (bufferIndex > 0) {
                bufferIndex--;
                // Move cursor back, print space, move back again
                Serial.print("\b \b");
            }
        }
        // Handle normal characters
        else if (bufferIndex < BUFFER_SIZE - 1) {
            inputBuffer[bufferIndex++] = c;
            // Echo character as you type
            Serial.print(c);
        }
    }
    
    // Small delay to prevent busy-waiting
    delay(10);
}
