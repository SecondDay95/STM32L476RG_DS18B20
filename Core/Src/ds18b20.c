#include <string.h>
#include "ds18b20.h"
#include "wire.h"

#define DS18B20_SCRATCHPAD_SIZE    9

#define DS18B20_READ_ROM           0x33
#define DS18B20_MATCH_ROM          0x55
#define DS18B20_SKIP_ROM           0xCC

#define DS18B20_CONVERT_T          0x44
#define DS18B20_READ_SCRATCHPAD    0xBE

static HAL_StatusTypeDef send_cmd(const uint8_t* rom_code, uint8_t cmd) {

  if (wire_reset() != HAL_OK)
    return HAL_ERROR;

  if(!rom_code) {

	  wire_write(DS18B20_SKIP_ROM);

  }
  else {

	  wire_write(DS18B20_MATCH_ROM);
	  for (int i = 0; i < DS18B20_ROM_CODE_SIZE; i++)
	    wire_write(rom_code[i]);

  }

  wire_write(cmd);
  return HAL_OK;

}

static HAL_StatusTypeDef ds18b20_read_scratchpad(const uint8_t* rom_code, uint8_t* scratchpad) {

  uint8_t crc;

  if (send_cmd(rom_code, DS18B20_READ_SCRATCHPAD) != HAL_OK)
    return HAL_ERROR;

  for (int i = 0; i < DS18B20_SCRATCHPAD_SIZE; i++)
    scratchpad[i] = wire_read();

  crc = wire_crc(scratchpad, DS18B20_SCRATCHPAD_SIZE - 1);
  if (scratchpad[DS18B20_SCRATCHPAD_SIZE - 1] == crc)
    return HAL_OK;
  else
    return HAL_ERROR;

}

HAL_StatusTypeDef ds18b20_init(void) {

	return wire_init();

}

HAL_StatusTypeDef ds18b20_read_address(uint8_t* rom_code) {

	uint8_t crc;

	if(wire_reset() != HAL_OK)
		return HAL_ERROR;

	wire_write(DS18B20_READ_ROM);

	for (int i = 0; i < DS18B20_ROM_CODE_SIZE; i++)
	  rom_code[i] = wire_read();

	crc = wire_crc(rom_code, DS18B20_ROM_CODE_SIZE - 1);
	if (rom_code[DS18B20_ROM_CODE_SIZE - 1] == crc)
	  return HAL_OK;
	else
	  return HAL_ERROR;

}

HAL_StatusTypeDef ds18b20_start_measure(const uint8_t* rom_code) {

  return send_cmd(rom_code, DS18B20_CONVERT_T);

}

float ds18b20_get_temp(const uint8_t* rom_code) {

  uint8_t scratchpad[DS18B20_SCRATCHPAD_SIZE];
  //int8_t temp1;
  //int8_t temp2;
  int16_t temp;

  if (ds18b20_read_scratchpad(rom_code, scratchpad) != HAL_OK)
    return 85.0f;

  memcpy(&temp, &scratchpad[0], sizeof(temp));
  //memcpy(&temp1, &scratchpad[0], sizeof(temp1));
  //memcpy(&temp2, &scratchpad[1], sizeof(temp2));
  //temp = (temp2 << 8) | temp1;

  return temp / 16.0f;

}
