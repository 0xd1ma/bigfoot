#pragma once

#ifndef MSG_SENDER_SESSION_H_
#define MSG_SENDER_SESSION_H_

#include <cstdint>
#include "vehicle/vehicle_common.h"
#include "protocol/protocol_common.h"
#include <serial/serial.h>
#include "bf_driver/msg_data_packer.h"

namespace bf_driver
{

    class MsgSenderSession
    {

    public:

        MsgSenderSession(vehicle::idDataMap &txMap);
        ~MsgSenderSession();

        void setPort(serial::Serial &port);

        bool worker(void);

    private:

        // проходим по всему словарю, просматриваем флаг обновления данных
        bool isAnyMapDataUpdated(void);

        // получаем обновленную пару (ID, data)
        vehicle::idDataPair getUpdatedPair(void);

        serial::Serial *m_port;

        std::vector< std::uint8_t > m_slpMsg;

        enum class State
        {
            AVAILABLE,
            INMAP,
            PACKER,
            WRAPPER,
            SEND,
            DONE
        };

        State m_curState;

        MsgDataPacker m_packer;

        vehicle::idDataMap &m_txDataMap;
        vehicle::idDataPair m_dataPair;

    };

}

#endif
