#pragma once

#ifndef MSG_DATA_PACKER_HPP_
#define MSG_DATA_PACKER_HPP_

#include <cstdint>
#include <vector>
#include "vehicle/vehicle_common.h"

namespace bf_driver
{
    using packCallback = std::function< std::vector< std::uint8_t >(void) >;

    using idPackerMap = std::map< std::uint32_t, packCallback >;
    using idPackerPair = std::pair< std::uint32_t, packCallback >;

    class MsgDataPacker
    {

    public:

        MsgDataPacker(vehicle::idDataMap &txMap);
        ~MsgDataPacker();

        std::vector< std::uint8_t > pack(vehicle::idDataPair pair);

    private:
        // внешний словарь хранения данных
        // приходится затаскивать его сюда, тк необходимо обновлять
        // счетчики внутри сообщений
        vehicle::idDataMap &m_txDataMap;

        idPackerMap m_packersCalls;
        idPackerPair m_packerPair;

        // сюда добавляем коллбеки для преобразования данных из словаря
        // в массив данных пересылки с использованием служебных полей авто
        // ---> ADD HERE NEW CALLBACKS
        std::vector< std::uint8_t > twistControlPack(void);

    };

}

#endif
