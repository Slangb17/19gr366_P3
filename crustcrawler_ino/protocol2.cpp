# include "protocol2.h"

// Defining the buffers transceiving data:
uint8_t txBuffer[30];
uint8_t rxBuffer[30];

// The amount of parameters of the package + 3 is returned:
int get_package_length(uint8_t* buffer)
{
  return get_uint16t(buffer, 5);
}

// Used for calculating the CRC-16 checksum for the package (http://emanual.robotis.com/docs/en/dxl/crc/):
// crc_accum: 0
// data_blk_ptr: buffer
// data_blk_size: size of buffer excluding CRC-16 fields:
uint16_t calculate_crc(uint16_t crc_accum, uint8_t* data_blk_ptr, size_t data_blk_size)
{
  size_t i, j;

  for (j = 0; j < data_blk_size; j++)
  {
    i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
    crc_accum = (crc_accum << 8) ^  pgm_read_word_near(crc_table + i);
  }
  return crc_accum;
}

// Add CRC to a buffer (CRC-16 is calculated and appended to the buffer):
// buffer: buffer
// data_size: size of buffer excluding CRC-16 fields:
void add_crc(uint8_t* buffer, size_t data_size)
{
  uint16_t crc;
  crc = calculate_crc(0, buffer, data_size);
  put_int16t(crc, txBuffer, data_size);
}

// Check the CRC of a received package (received CRC-16 is compared to newly computed):
// buffer: buffer
// pos: position of CRC_L
boolean check_crc(uint8_t* buffer, int16_t pos)
{
  uint16_t incommingCrc = get_uint16t(buffer, pos);
  uint16_t calculatedCrc = calculate_crc(0, buffer, pos);
  return (calculatedCrc == incommingCrc);
}

// Initializing the package with header and ID:
// id: id of actuator which should receive package
void init_package(uint8_t id)
{
  txBuffer[0] = 0xff;
  txBuffer[1] = 0xff;
  txBuffer[2] = 0xfd;
  txBuffer[3] = 0x00;
  txBuffer[4] = id;
}

// Transmit the data currently assigned to the txBuffer:
void transmit_package()
{
  int pgk_length = get_package_length(txBuffer) + 7;
  digitalWrite(2, HIGH);
  Serial1.write(txBuffer, pgk_length);
  Serial1.flush();
  digitalWrite(2, LOW);
}

// Receive package (times out if no response is received):
// timeout: milliseconds to wait for a reply.
bool receive_package(int timeout) {
  unsigned long start_time = millis();
  
  size_t bytecount = 0;
  size_t remaining_read = 1;
  while ((start_time + timeout) > millis() && remaining_read > 0) {

    if (Serial1.available()) {
      uint8_t incomming_byte = Serial1.read();
      switch (bytecount)
      {
        case 0:
        case 1: if (incomming_byte == 0xFF) {
            rxBuffer[bytecount] = incomming_byte;
            ++bytecount;
          } else {
            bytecount = 0;
          }
          break;
        case 2: if (incomming_byte == 0xFD)
          {
            rxBuffer[bytecount] = incomming_byte;
            ++bytecount;
          } else {
            bytecount = 0;
          }
          break;
        case 3:
        case 4:
        case 5: rxBuffer[bytecount] = incomming_byte;
          ++bytecount;
          break;
        case 6: rxBuffer[bytecount] = incomming_byte;
          remaining_read = get_package_length(rxBuffer);
          ++bytecount;
          break;
        default: rxBuffer[bytecount] = incomming_byte;
          ++bytecount;
          --remaining_read;
          break;
      }
    }
  }
  if (remaining_read == 0)
  {
    return check_crc(rxBuffer, bytecount - 2);
  }
  else
  {
    return false;
  }
}

// Send a READ instruction to the servo:
void send_read_instruction(uint8_t id, uint16_t from_addr, uint16_t data_length)
{
  init_package(id); // HEADER AND ID
  put_uint16t(7, txBuffer, 5); // LEN_L & LEN_H
  put_uint8t(0x02, txBuffer, 7); // INST
  put_uint16t(from_addr, txBuffer, 8); // CONTROL TABEL ADDRESS
  put_uint16t(data_length, txBuffer, 10); // BYTE SIZE OF DATA
  add_crc(txBuffer, 12); // CRC_L & CRC_H

  transmit_package();
}

// Send a 1 byte WRITE instruction to the servo:
void send_write_instruction(uint8_t id, uint16_t address, uint8_t data)
{
  init_package(id); // HEADER AND ID
  put_uint16t(6, txBuffer, 5); // LEN_L & LEN_H
  put_uint8t(0x03, txBuffer, 7); // INST
  put_uint16t(address, txBuffer, 8); // CONTROL TABEL ADDRESS
  put_uint8t(data, txBuffer, 10); // DATA
  add_crc(txBuffer, 11); // CRC_L & CRC_H

  transmit_package();
}

// Send a 2 byte WRITE instruction to the servo:
void send_write_instruction(uint8_t id, uint16_t address, uint16_t data)
{
  init_package(id); // HEADER AND ID
  put_uint16t(7, txBuffer, 5); // LEN_L & LEN_H
  put_uint8t(0x03, txBuffer, 7); // INST
  put_uint16t(address, txBuffer, 9); // CONTROL TABEL ADDRESS
  put_uint8t(data, txBuffer, 11); // DATA
  add_crc(txBuffer, 12); // CRC_L & CRC_H

  transmit_package();
}

// Send a 4 byte WRITE instruction to the servo:
void send_write_instruction(uint8_t id, uint16_t address, int32_t data)
{
  init_package(id); // HEADER AND ID
  put_uint16t(9, txBuffer, 5); // LEN_L & LEN_H
  put_uint8t(0x03, txBuffer, 7); // INST
  put_uint16t(address, txBuffer, 8); // CONTROL TABEL ADDRESS
  put_int32t(data, txBuffer, 10); // DATA
  add_crc(txBuffer, 14); // CRC_L & CRC_H

  transmit_package();
}
