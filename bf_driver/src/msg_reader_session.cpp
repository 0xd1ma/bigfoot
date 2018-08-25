#include "bf_driver/msg_reader_session.h"
#include <cassert>
#include <cstdint>
#include <ros/ros.h>

namespace bf_driver
{

    MsgReaderSession::MsgReaderSession(vehicle::idDataMap &rxMap) :
    		m_slpMsg{},
            m_port{},
            m_curState{State::AVAILABLE},
			m_composer{std::bind(&MsgReaderSession::msgComposerCompleted, this)},
			m_processor{rxMap},
			m_readerIsCompleted{false},
			m_composerIsCompleted{false}
    {
        ;
    }

    MsgReaderSession::~MsgReaderSession()
    {
        ;
    }

    void MsgReaderSession::setPort(serial::Serial &port)
    {
        m_port = &port;
    }

    bool MsgReaderSession::isReaderCompleted(void)
    {
        return m_readerIsCompleted;
    }

    void MsgReaderSession::msgComposerCompleted(void)
    {
        m_composerIsCompleted = true;
    }

    bool MsgReaderSession::worker()
    {
        switch (m_curState)
        {
            case State::AVAILABLE:
            {
                if (m_port->isOpen())
                {
                    m_readerIsCompleted = false;

                    m_curState = State::INBOX;
                }
                else
                {
                    ROS_ERROR((std::string{"Connection fail"}).c_str());
                    ros::requestShutdown();
                }

                break;
            }
            case State::INBOX:
            {
                // байты в наличие. надо парсить.
                if (m_port->available())
                {
                    m_curState = State::COMPOSER;
                }
                else
                {
                    m_curState = State::DONE;
                }

                break;
            }
            case State::COMPOSER:
            {
                std::size_t inSize;

                inSize = m_port->available();

                std::vector<std::uint8_t> bytesStream;

                m_port->read(bytesStream, inSize);

//                for (auto x : bytesStream)
//                {
//                    std::cout << std::hex << static_cast<int>(x) << " ";
//                }
//                std::cout << std::endl;

//                m_curState = State::DONE;

                for (auto x : bytesStream)
                {
                    m_composer.compose(x, m_slpMsg);
                }

                if (m_composerIsCompleted)
                {
                    m_composerIsCompleted = false;

//                    ROS_INFO("Composer is completed");

                    m_curState = State::PROCESSOR;
                }
                else
                {
                    m_curState = State::INBOX;
                }

                break;
            }
            case State::PROCESSOR:
            {
                m_processor.process(m_slpMsg);

                m_curState = State::DONE;

                break;
            }
            case State::DONE:
            {
                m_readerIsCompleted = true;

                // очищаем для принятия новых данных
                m_slpMsg.clear();

                m_curState = State::AVAILABLE;

                return true;
            }
        }

        return false;
    }

}

//                ROS_INFO("To composer: ");
//                for (auto x : bytesStream)
//                {
//                    std::cout << std::hex << static_cast<int>(x) << " ";
//                }
//                std::cout << std::endl;