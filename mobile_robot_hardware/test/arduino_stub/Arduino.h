#ifndef ARDUINO_STUB_H
#define ARDUINO_STUB_H

// host stand-in for the Arduino core, just enough to compile
// esp32_firmware/inc/Messages.hpp in the wire protocol tests

inline unsigned long millis() { return 0; }

#endif
