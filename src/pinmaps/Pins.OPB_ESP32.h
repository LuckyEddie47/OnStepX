// -------------------------------------------------------------------------------------------------
// Pinmap for OPB (ESP32)
#pragma once

#if defined(ESP32)

#if SERIAL_A_BAUD_DEFAULT != OFF
  #define SERIAL_A              Serial
#endif

#else
#error "Wrong processor for this configuration!"

#endif
