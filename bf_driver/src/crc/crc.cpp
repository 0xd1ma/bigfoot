/***************************************************************************************************

 В этом файле находятся функции для подсчета CRC-16  нетабличным методом

***************************************************************************************************/

#include "crc/crc.h"

/***************************************************************************************************
****************************************************************************************************
                                          ФУНКЦИИ
****************************************************************************************************
***************************************************************************************************/
namespace crc
{
std::uint32_t crc32(std::vector<std::uint8_t>::iterator begin, std::vector<std::uint8_t>::iterator end)
{
    unsigned long crc_table[256];
    unsigned long crc;

    for (int i = 0; i < 256; i++)
    {
        crc = i;
        for (int j = 0; j < 8; j++)
            crc = crc & 1 ? (crc >> 1) ^ 0xEDB88320UL : crc >> 1;
        crc_table[i] = crc;
    };

    crc = 0xFFFFFFFFUL;

    while (begin != end)
        crc = crc_table[(crc ^ *begin++) & 0xFF] ^ (crc >> 8);

    return crc ^ 0xFFFFFFFFUL;
}

}

