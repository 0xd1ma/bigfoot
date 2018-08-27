#include "protocol/protocol_msg_processor.h"
#include <cassert>

#include <iostream>

namespace protocol
{

	MsgProcessor::MsgProcessor(vehicle::idDataMap &rxMap) :
	        m_callbackMap{},
	        m_rxDataMap{rxMap}
	{
        /***********************************************************************
         *
         * инициализируем словарь парами CAN_ID - CALLBACK
         *
        ***********************************************************************/
        // ---> ADD HERE NEW

	    m_callbackMap.insert(vehicle::idCallbackPair(vehicle::twist_telemetry_id,
	            vehicle::callback(std::bind(&MsgProcessor::twistTelemetryConverting,
	                    this, std::placeholders::_1))));

        m_callbackMap.insert(vehicle::idCallbackPair(vehicle::imu_telemetry_id,
                vehicle::callback(std::bind(&MsgProcessor::imuTelemetryConverting,
                        this, std::placeholders::_1))));

	}

	MsgProcessor::~MsgProcessor()
	{
		;
	}

    void MsgProcessor::process(std::vector< std::uint8_t > &msg)
    {
        std::uint8_t payloadCmd = getPayloadCmd(msg);

        std::vector< std::uint8_t > rcData;
        rcData = getData(msg);

        vehicle::storage tmp;

//        // проверяем вхождение can id в множество необходимых для работы
//        if (vehicle::isIdInAllowedSet(ID))
//        {
            // пробуем дернуть обработчик
            try
            {
                tmp.data = m_callbackMap[vehicle::twist_telemetry_id](rcData);
            }
            catch(const std::bad_function_call &e)
            {
                // дергаем ассерт тк дальнейшее выполнение чревато ошибками
                assert(0);
            }

            // если НЕОБХОДИМО опубликовать сообщение в ROS
            // первоначально необходимо проверить
            tmp.dataStatus = vehicle::isIdInfoAllowedToPublish(vehicle::twist_telemetry_id);


            // проверим существование пары в данных
            if (m_rxDataMap.count(vehicle::twist_telemetry_id))
            {
                // присутствует
                m_rxDataMap[vehicle::twist_telemetry_id] = tmp;
            }
            else
            {
                // отсутствует
                m_rxDataMap.insert(vehicle::idDataPair(vehicle::twist_telemetry_id, tmp));
            }

//        }
//        else
//        {
//        }
    }

    /***************************************************************************
     *
     * обработчики входящих сообщений
     *
    ***************************************************************************/
    /***************************************************************************
     * считывание данных положения рулевого колеса
    ***************************************************************************/
    boost::any MsgProcessor::twistTelemetryConverting(std::vector< std::uint8_t > msg)
    {
        vehicle::encoder_telemetry encoder;

        encoder.forward_left = static_cast<std::int32_t>(charArrToInt32(msg.begin()));
//        std::cout << encoder.forward_left << std::endl;

        encoder.forward_right = static_cast<std::int32_t>(charArrToInt32(msg.begin() + 4));
//        std::cout << encoder.forward_right << std::endl;

        encoder.back_right = static_cast<std::int32_t>(charArrToInt32(msg.begin() + 8));
//        std::cout << encoder.back_right << std::endl;

        encoder.back_left = static_cast<std::int32_t>(charArrToInt32(msg.begin() + 12));
//        std::cout << encoder.back_left << std::endl;

//        std::cout << "telemetry is received" << std::endl;
        encoder.voltage_mv = static_cast<std::uint32_t>(charArrToInt32(msg.begin() + 16));

        return encoder;
    }

    boost::any MsgProcessor::imuTelemetryConverting(std::vector< std::uint8_t > msg)
    {
        vehicle::imu_telemetry imu;

        std::cout << "IMU telemetry is received" << std::endl;

        return imu;
    }

}

