#include "protocol/protocol_msg_composer.h"
#include <cassert>

#include <iostream>

namespace protocol
{

	MsgComposer::MsgComposer(MsgComposed_cb fint) : m_msgComposed{fint},
	                                                m_curState{State::HEADER}
	{
		;
	}

	MsgComposer::~MsgComposer()
	{
	    ;
	}

	void MsgComposer::setCompliteCallback(MsgComposed_cb fint)
	{
	    m_msgComposed = fint;
	}

    void MsgComposer::compose(uint8_t byte, std::vector< std::uint8_t > &msg)
    {
        msg.push_back(byte);

        switch (m_curState)
        {
            case State::HEADER:
            {
                if (!isHeaderPassed(msg))
                {
                    resetOnHeaderError(msg);
                    return;
                }

                if (msg.size() == 3)
                {
                    m_curState = State::DATA;
                }

                break;
            }
            case State::DATA:
            {
                if (msg.size() < PROTOCOL_TWIST_MSG_SIZE)
                {
                    return;
                }

//                std::cout << "CRC is " << std::boolalpha << rx_messages::isCrcValid(msg) << std::endl;

                //проверяем контрольную сумму
                if (!rx_messages::isCrcValid(msg))
                {
                    resetOnCrcError(msg);
                    return;
                }

                composerComplete();

                break;
            }
        }
    }

    bool MsgComposer::isHeaderPassed(std::vector<std::uint8_t> &msg)
    {
        // костыль для совпадения с оффсетами
        switch (msg.size() - 1)
        {
            case PROTOCOL_SOF_OFFSET:
            {
                return rx_messages::correctStartByte == msg.at(PROTOCOL_SOF_OFFSET);
            }

            case PROTOCOL_SIZE_OFFSET:
            {
                return true;
            }

            case PROTOCOL_TYPE_OFFSET:
            {
                return true;
            }

            default:
            {
                assert(0);
                break;
            }
        }

        return false;
    }

    void MsgComposer::resetOnHeaderError(std::vector<std::uint8_t> &msg)
    {
        if (!msg.empty())
        {
            resetOnCrcError(msg);
        }
    }

    void MsgComposer::resetOnCrcError(std::vector<std::uint8_t> &msg)
    {
        m_curState = State::HEADER;

        std::uint8_t last;
        last = 0;

        // возможно завершающий байт является стартовым
        last = *(msg.end() - 1);
        msg.clear();
        // проверим на старт
        msg.push_back(last);

        if (!isHeaderPassed(msg))
        {
            msg.clear();
        }
    }

    void MsgComposer::resetAfterComplete(void)
    {
        m_curState = State::HEADER;
    }

    void MsgComposer::composerComplete(void)
    {
        if (nullptr != m_msgComposed)
        {
            m_msgComposed();
        }
        else
        {
            assert(0);
        }

        resetAfterComplete();
    }

}
