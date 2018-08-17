#pragma once

#ifndef SLP_MSG_COMPOSER_H_
#define SLP_MSG_COMPOSER_H_

#include <functional>
#include <vector>
#include "protocol/protocol_msg_format.h"
#include "protocol/protocol_common.h"

namespace protocol
{
    using MsgComposed_cb = std::function< void(void) >;

    class MsgComposer
    {

    public:

        MsgComposer(MsgComposed_cb fint);
        ~MsgComposer();

        void compose(uint8_t byte, std::vector< std::uint8_t > &msg);
        bool isHeaderPassed(std::vector< std::uint8_t > &msg);

        void setCompliteCallback(MsgComposed_cb fint);

    private:

        void resetOnHeaderError(std::vector< std::uint8_t > &msg);
        void resetOnCrcError(std::vector< std::uint8_t > &msg);
        void resetAfterComplete(void);
        void composerComplete(void);

        enum class State {HEADER, DATA};
        State m_curState;

        MsgComposed_cb m_msgComposed;
    };

}

#endif
