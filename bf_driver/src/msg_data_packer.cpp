#include "bf_driver/msg_data_packer.h"
#include "protocol/protocol_common.h"
#include <cmath>
#include <algorithm>

namespace bf_driver
{

    MsgDataPacker::MsgDataPacker(vehicle::idDataMap &txMap) :
            m_txDataMap{txMap},
            m_packersCalls{},
            m_packerPair{}
    {
        /***********************************************************************
         *
         * инициализируем словарь парами ID - PACK_CALLBACK
         *
        ***********************************************************************/
        m_packersCalls.insert(idPackerPair( vehicle::twist_control_id,
                packCallback(std::bind(&MsgDataPacker::twistControlPack, this))));
    }

    MsgDataPacker::~MsgDataPacker()
    {
        ;
    }

    std::vector< std::uint8_t > MsgDataPacker::pack(vehicle::idDataPair pair)
    {
        // просто дергаем по id функцию из словаря
        return m_packersCalls[pair.first]();
    }

    /***************************************************************************
     *
     * методы преобразования данных в зависимости от ID
     *
    ***************************************************************************/
    /***************************************************************************
     * формирование данных управление рулем
    ***************************************************************************/
    std::vector< std::uint8_t > MsgDataPacker::twistControlPack(void)
    {
        vehicle::storage store;
        store = m_txDataMap[vehicle::twist_control_id];

        vehicle::twist_control dt;
        dt = boost::any_cast< vehicle::twist_control >(store.data);

        std::vector< std::uint8_t > msg(16, 0);

        std::uint8_t const * vx = reinterpret_cast<std::uint8_t const *>(&dt.vx);

        msg.at(0) = vx[0];
        msg.at(1) = vx[1];
        msg.at(2) = vx[2];
        msg.at(3) = vx[3];

        std::uint8_t const * vy = reinterpret_cast<std::uint8_t const *>(&dt.vy);

        msg.at(4) = vy[0];
        msg.at(5) = vy[1];
        msg.at(6) = vy[2];
        msg.at(7) = vy[3];

        std::uint8_t const * w = reinterpret_cast<std::uint8_t const *>(&dt.w);

        msg.at(8) = w[0];
        msg.at(9) = w[1];
        msg.at(10) = w[2];
        msg.at(11) = w[3];

        std::uint8_t const * cntrl = reinterpret_cast<std::uint8_t const *>(&dt.cntrl);

        msg.at(12) = cntrl[0];
        msg.at(13) = cntrl[1];
        msg.at(14) = cntrl[2];
        msg.at(15) = cntrl[3];

        return msg;

    }

}
