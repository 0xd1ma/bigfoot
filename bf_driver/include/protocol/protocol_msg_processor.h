#pragma once

#ifndef SLP_MSG_PROCESSOR_H_
#define SLP_MSG_PROCESSOR_H_

#include "protocol/protocol_common.h"
#include "protocol/protocol_msg_format.h"
#include "vehicle/vehicle_common.h"

namespace protocol
{

    class MsgProcessor
    {

    public:

    	MsgProcessor(vehicle::idDataMap &rxMap);
    	~MsgProcessor();

        void process(std::vector< std::uint8_t > &msg);

        /***********************************************************************
         *
         * обработчики входящих сообщений
         *
        ***********************************************************************/
        // сюда добавляются ф. обработки входящих сообщений
        // ---> ADD HERE NEW FUNCTION
        boost::any twistTelemetryConverting(std::vector< std::uint8_t > msg);
        boost::any imuTelemetryConverting(std::vector< std::uint8_t > msg);

    private:

        inline std::uint8_t getPayloadCmd(std::vector< std::uint8_t > &msg)
        {
            return msg.at(PROTOCOL_TYPE_OFFSET);
        }

        inline std::vector< std::uint8_t > getData(std::vector<std::uint8_t> &msg)
        {
            std::vector< std::uint8_t > tmp;

            std::vector< std::uint8_t >::iterator begin;
            std::vector< std::uint8_t >::iterator end;

            begin = msg.begin() + PROTOCOL_VX_OFFSET;
            end = msg.begin() + PROTOCOL_MSG_CRC_OFFSET;

            tmp.insert( tmp.begin(), begin, end );

            return tmp;
        }

        vehicle::idDataMap &m_rxDataMap;

        vehicle::idCallbackMap m_callbackMap;
    };

}

#endif
