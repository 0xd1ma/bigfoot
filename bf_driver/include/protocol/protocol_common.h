/*******************************************************************************
 Общий файл для библиотеки
*******************************************************************************/
#pragma once

#ifndef SLP_COMMON_H_
#define SLP_COMMON_H_

#include <cstdint>
#include <vector>

/*******************************************************************************
                            Глобальные дефайны
*******************************************************************************/
const std::uint32_t PROTOCOL_TWIST_MSG_SIZE = 23;
/*******************************************************************************
                            Глобальные типы данных
*******************************************************************************/
/*******************************************************************************
                            Глобальные inline функции
*******************************************************************************/
namespace protocol
{

    // преобразование массива чаров в инт32 (little-endian)
    inline std::uint32_t charArrToInt32(const std::vector< std::uint8_t >::iterator buf)
    {
        std::uint32_t tmp = 0;

        tmp |= buf[0] << 0;
        tmp |= buf[1] << 8;
        tmp |= buf[2] << 16;
        tmp |= buf[3] << 24;

        return tmp;
    }

    // преобразование массива чаров в инт16 (little-endian)
    inline std::uint16_t charArrToInt16(const std::vector< std::uint8_t >::iterator buf)
    {
        std::uint16_t tmp = 0;

        tmp |= buf[0] << 0;
        tmp |= buf[1] << 8;

        return tmp;
    }

    // преобразование инта32 в массив чаров (little-endian)
    inline void int32ToCharArr(const std::vector< std::uint8_t >::iterator buf,
            std::uint32_t integer)
    {
        buf[0] = (integer >> 0)  & 0xFF;
        buf[1] = (integer >> 8)  & 0xFF;
        buf[2] = (integer >> 16) & 0xFF;
        buf[3] = (integer >> 24) & 0xFF;
    }

    // преобразование инта16 в массив чаров (little-endian)
    inline void int16ToCharArr(const std::vector< std::uint8_t >::iterator buf,
            std::uint16_t halfInt)
    {
        buf[0] = (halfInt >> 0)  & 0xFF;
        buf[1] = (halfInt >> 8)  & 0xFF;
    }

    union u2f
    {
        float f;
        std::uint32_t u;
    };

    inline float convertVectorBytesToFloat(const std::vector< std::uint8_t >::iterator buf)
    {
        u2f tmp;

        tmp.u = charArrToInt32(buf);

        return tmp.f;
    }

}

#endif
