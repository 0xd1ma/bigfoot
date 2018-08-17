/*******************************************************************************
 *
 * В этом файле будет полное описание формата сообщений для STARLINE_PROTOCOL
 *
*******************************************************************************/
#pragma once

#ifndef SLP_MSG_FORMAT_H_
#define SLP_MSG_FORMAT_H_

#include <cstdint>
#include <vector>
#include <cassert>

#include "crc/crc.h"

#include "protocol/protocol_common.h"

#include <iostream>

/*
================================================================================
                                      СМЕЩЕНИЯ И РАЗМЕРЫ ПОЛЕЙ
================================================================================
*/
namespace protocol
{
    /*******************************************************************************
    *
    * ЗАГОЛОВОК
    *
    *******************************************************************************/
    // размеры
    const std::uint32_t PROTOCOL_SOF_SIZE = 1;
    const std::uint32_t PROTOCOL_MSG_SIZE_SIZE = 1;
    const std::uint32_t PROTOCOL_TYPE_SIZE = 1;
    // смещения заголовка
    const std::uint32_t PROTOCOL_SOF_OFFSET = 0;
    const std::uint32_t PROTOCOL_SIZE_OFFSET = (PROTOCOL_SOF_OFFSET + PROTOCOL_SOF_SIZE);
    const std::uint32_t PROTOCOL_TYPE_OFFSET = (PROTOCOL_SIZE_OFFSET + PROTOCOL_MSG_SIZE_SIZE);

/*******************************************************************************
 *
 * ДАННЫЕ
 *
*******************************************************************************/
    // размеры
    const std::uint32_t PROTOCOL_VX_SIZE = 4;
    const std::uint32_t PROTOCOL_VY_SIZE = 4;
    const std::uint32_t PROTOCOL_W_SIZE = 4;
    const std::uint32_t PROTOCOL_CNTRL_SIZE = 4;

    // смещение
    const std::uint32_t PROTOCOL_VX_OFFSET = (PROTOCOL_TYPE_OFFSET + PROTOCOL_TYPE_SIZE);
    const std::uint32_t PROTOCOL_VY_OFFSET = (PROTOCOL_VX_OFFSET + PROTOCOL_VX_SIZE);
    const std::uint32_t PROTOCOL_W_OFFSET = (PROTOCOL_VY_OFFSET + PROTOCOL_VY_SIZE);
    const std::uint32_t PROTOCOL_CNTRL_OFFSET = (PROTOCOL_W_OFFSET + PROTOCOL_W_SIZE);

/*******************************************************************************
 *
 * КОНТРОЛЬНАЯ СУММА НА ДАННЫЕ
 *
*******************************************************************************/
    const std::uint32_t PROTOCOL_MSG_CRC_SIZE = 4;
    const std::uint32_t PROTOCOL_MSG_CRC_OFFSET = PROTOCOL_TWIST_MSG_SIZE - PROTOCOL_MSG_CRC_SIZE;

}
/*******************************************************************************
 *
 * Ф. для приема
 *
*******************************************************************************/
namespace protocol
{

    namespace rx_messages
    {

        const std::uint8_t correctStartByte = 0xBB;

        inline std::uint16_t getDataSize(std::vector< std::uint8_t > &msg)
        {
            return msg.at(PROTOCOL_SIZE_OFFSET);
        }

        inline std::uint32_t getCrc(std::vector< std::uint8_t > &msg)
        {
            return charArrToInt32(msg.begin() + PROTOCOL_MSG_CRC_OFFSET);
        }

        inline bool isCrcValid(std::vector< std::uint8_t > &msg)
        {
            std::vector< std::uint8_t >::iterator begin;
            std::vector< std::uint8_t >::iterator end;

            begin = msg.begin();
            end = msg.begin() + PROTOCOL_MSG_CRC_OFFSET;

            std::uint32_t trueCrc = crc::crc32(begin, end);

            std::uint32_t receivedCrc = getCrc(msg);

            return trueCrc == receivedCrc;
        }

        inline std::uint8_t getMsgType(std::vector< std::uint8_t > &msg)
        {
            return msg.at(PROTOCOL_TYPE_SIZE);
        }
    }

}

/*******************************************************************************
 *
 * Ф. для передачи
 *
*******************************************************************************/
namespace protocol
{

    namespace tx_messages
    {
        const std::uint8_t correctStartByte = 0xAA;

        inline void wrapMsg(std::vector< std::uint8_t > &msg)
        {
            std::vector< std::uint8_t > tmp (PROTOCOL_SOF_SIZE + PROTOCOL_MSG_SIZE_SIZE + PROTOCOL_TYPE_SIZE, 0);

            tmp.at(PROTOCOL_SOF_OFFSET) = correctStartByte;
            tmp.at(PROTOCOL_SIZE_OFFSET)  = 23;
            tmp.at(PROTOCOL_TYPE_OFFSET)  = 0;

            // add header to msg
            msg.insert(msg.begin(), tmp.begin(), tmp.end());

            uint32_t dataCrc;
            dataCrc = 0;

            dataCrc = crc::crc32(msg.begin(), msg.end());

            tmp.resize(PROTOCOL_MSG_CRC_SIZE);

            int32ToCharArr(tmp.begin(), dataCrc);

            msg.insert(msg.end(), tmp.begin(), tmp.end());
        }

    }

}

#endif

