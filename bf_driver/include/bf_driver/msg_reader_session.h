#pragma once

#ifndef MSG_READER_SESSION_H_
#define MSG_READER_SESSION_H_

#include <cstdint>
#include <serial/serial.h>

#include "vehicle/vehicle_common.h"
#include "protocol/protocol_common.h"

#include "protocol/protocol_msg_composer.h"
#include "protocol/protocol_msg_processor.h"

namespace bf_driver
{

    class MsgReaderSession
    {

    public:

        MsgReaderSession( vehicle::idDataMap &rxMap );
        ~MsgReaderSession();

        void setPort( serial::Serial &port );
        bool isReaderCompleted(void);
        void msgComposerCompleted(void);

        bool worker(void);

    private:

        serial::Serial *m_port;

        std::vector< std::uint8_t > m_slpMsg;

        enum class State
        {
            AVAILABLE,
            INBOX,
            COMPOSER,
            PROCESSOR,
            DONE
        };

        State m_curState;

        bool m_readerIsCompleted;
        bool m_composerIsCompleted;

        protocol::MsgComposer m_composer;
        protocol::MsgProcessor m_processor;

    };

}

#endif
