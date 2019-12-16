#ifndef ENDIAN
#define ENDIAN

#if (ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

// -- Little endian --

// Put integers:
void put_uint8t(uint8_t value, uint8_t* buffer, size_t pos);
void put_int16t(int16_t value, uint8_t* buffer, size_t pos); 
void put_uint16t(uint16_t value, uint8_t* buffer, size_t pos);
void put_int32t(int32_t val, uint8_t* buffer, size_t pos);

// Get integers:
uint16_t get_uint16t(uint8_t* buffer, size_t pos);
int32_t get_int32t(uint8_t* buffer, size_t pos);

#endif
