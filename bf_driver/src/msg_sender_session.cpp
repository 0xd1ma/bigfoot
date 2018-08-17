#include "bf_driver/msg_sender_session.h"
#include "protocol/protocol_msg_format.h"
#include "cassert"
#include <ros/ros.h>

namespace bf_driver
{

    MsgSenderSession::MsgSenderSession(vehicle::idDataMap &txDataMap) :
            m_slpMsg{},
            m_port{},
            m_curState{State::AVAILABLE},
            m_txDataMap{txDataMap},
            m_dataPair{},
            m_packer{txDataMap}
    {

    }

    MsgSenderSession::~MsgSenderSession()
    {
        ;
    }

    bool MsgSenderSession::worker()
    {
        switch (m_curState)
        {
            case State::AVAILABLE:
            {

                if (m_port->isOpen())
                {
                    m_curState = State::INMAP;
                }
                else
                {
                    ROS_ERROR((std::string{"Connection fail"}).c_str());
                    ros::requestShutdown();
                }

                break;

            }
            case State::INMAP:
            {

                if (isAnyMapDataUpdated())
                {
                    m_dataPair = getUpdatedPair();
                    // подготавливаем vector для заполнения данными
                    m_slpMsg.clear();

                    m_curState = State::PACKER;
                }
                else
                {
                    m_curState = State::DONE;
                }

                break;

            }
            case State::PACKER:
            {
                m_slpMsg = m_packer.pack(m_dataPair);

                m_curState = State::WRAPPER;

                break;
            }
            case State::WRAPPER:
            {
                protocol::tx_messages::wrapMsg(m_slpMsg);

//                for(auto x: m_slpMsg)
//                {
//                    std::cout << std::hex << static_cast<int>(x) << " ";
//                }
//
//                std::cout << std::endl;

                m_curState = State::SEND;

                break;
            }
            case State::SEND:
            {
                m_port->write(m_slpMsg);

                m_curState = State::DONE;

                break;
            }
            case State::DONE:
            {

                m_curState = State::AVAILABLE;

                return true;

            }
        }

        return false;

    }

    void MsgSenderSession::setPort(serial::Serial &port)
    {
        m_port = &port;
    }

    bool MsgSenderSession::isAnyMapDataUpdated(void)
    {

        for (auto &x : m_txDataMap)
        {
            if ( x.second.dataStatus )
            {
                return true;
            }
        }

        return false;

    }

    vehicle::idDataPair MsgSenderSession::getUpdatedPair(void)
    {

        for (auto &x : m_txDataMap)
        {
            if ( x.second.dataStatus )
            {
                // при считывании данных сбрасываем статус
                x.second.dataStatus = false;

                return x;
            }
        }

        ROS_ERROR("vehicle::canIdAndDataPair MsgSenderSession::getUpdatedPair(void) no data updated");
        assert(0);

        // warning off
        vehicle::idDataPair tmp;
        return tmp;

    }

}

////////////////////////////////////////////////////////////////////////////////
