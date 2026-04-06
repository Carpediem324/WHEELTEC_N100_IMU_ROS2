#ifndef WHEELTEC_N100_IMU__CRC_TABLE_HPP_
#define WHEELTEC_N100_IMU__CRC_TABLE_HPP_

#include <cstddef>
#include <cstdint>

uint8_t CRC8_Table(const uint8_t * p, std::size_t counter);
uint16_t CRC16_Table(const uint8_t * p, std::size_t counter);
uint32_t CRC32_Table(const uint8_t * p, std::size_t counter);

#endif  // WHEELTEC_N100_IMU__CRC_TABLE_HPP_
