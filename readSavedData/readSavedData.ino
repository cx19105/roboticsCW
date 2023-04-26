#include <EEPROM.h>

// start reading from the first byte (address 0) of the EEPROM
int address = 0;
byte value;

void setup() {
  // initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}

void loop() {
  // read a byte from the current address of the EEPROM
  // Output into csv format for 3 sensors and x position
  value = EEPROM.read(address);
  Serial.print(value, DEC);
  Serial.print(",");
  address = address + 1;
  value = EEPROM.read(address);
  Serial.print(value, DEC);
  Serial.print(",");
  address = address + 1;
  value = EEPROM.read(address);
  Serial.print(value, DEC);
  Serial.print(",");
  address = address + 1;
  value = EEPROM.read(address);
  Serial.print(value, DEC);
  address = address + 1;
  
  Serial.println();

  /***
    Advance to the next address, when at the end restart at the beginning.

    Larger AVR processors have larger EEPROM sizes, E.g:
    - Arduino Duemilanove: 512 B EEPROM storage.
    - Arduino Uno:         1 kB EEPROM storage.
    - Arduino Mega:        4 kB EEPROM storage.

    Rather than hard-coding the length, you should use the pre-provided length function.
    This will make your code portable to all AVR processors.
  ***/
  if (address == EEPROM.length()) {
    delay(500000);
    address = 0;
  }

  /***
    As the EEPROM sizes are powers of two, wrapping (preventing overflow) of an
    EEPROM address is also doable by a bitwise and of the length - 1.

    ++address &= EEPROM.length() - 1;
  ***/

}
