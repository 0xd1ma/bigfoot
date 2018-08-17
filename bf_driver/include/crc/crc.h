#pragma once

#ifndef CRC16_H_
#define CRC16_H_

#include <cstdint>
#include <vector>

namespace crc
{
    std::uint32_t crc32(std::vector<std::uint8_t>::iterator begin, std::vector<std::uint8_t>::iterator end);
}

#endif
