#pragma once

#ifndef VEHICLE_COMMON_H_
#define VEHICLE_COMMON_H_

#include <cstdint>
#include <set>
#include <boost/any.hpp>
#include <map>
#include <functional>
#include <vector>


namespace vehicle
{
    /***************************************************************************
     *
     * ID сообщений CAN
     *
    ***************************************************************************/
    // установка
    const std::uint32_t twist_control_id{0x00};
    // считывание скоростей
    const std::uint32_t twist_telemetry_id{0x00};
    // считывание инерциальных данных
    const std::uint32_t imu_telemetry_id{0x01};
    // считывание данных энкодеров
    const std::uint32_t encoder_telemetry_id{0x02};

    /***************************************************************************
     *
     * для проверки вхождения в множество разрешенных ID-ов
     *
    ***************************************************************************/
    const std::set< std::uint32_t > allowedId =
            {
                twist_control_id,
                twist_telemetry_id,
                imu_telemetry_id,
                encoder_telemetry_id
            };

    const std::set< std::uint32_t > allowedToPublishId =
            {
                twist_telemetry_id
//                imu_telemetry_id
            };

    inline bool isIdInAllowedSet(std::uint32_t id)
    {
        return allowedId.find(id) != allowedId.end();
    }

    inline bool isIdInfoAllowedToPublish(std::uint32_t id)
    {
        return allowedToPublishId.find(id) != allowedToPublishId.end();
    }


    /***************************************************************************
     *
     * структура для хранения полученых данных с флагом обновления данных
     *
    ***************************************************************************/
    struct storage
    {
        bool dataStatus;
        boost::any data;
    };

    /***************************************************************************
     *
     * проверка на изменение данных
     *
    ***************************************************************************/
    inline bool isDataChanged(storage str)
    {
        return str.dataStatus;
    }

    /***************************************************************************
     *
     * структуры хранения принятой информации от автомобиля
     *
    ***************************************************************************/
    struct twist_control
    {
        std::float_t vx;
        std::float_t vy;
        std::float_t w;

        std::uint32_t cntrl;
    };

    struct twist_telemetry
    {
        std::float_t vx;
        std::float_t vy;
        std::float_t w;

        std::uint32_t cntrl;
    };

    struct imu_telemetry
    {
        std::float_t orientation_x;
        std::float_t orientation_y;
        std::float_t orientation_z;
        std::float_t orientation_w;

        std::float_t angular_velocity_x;
        std::float_t angular_velocity_y;
        std::float_t angular_velocity_z;

        std::float_t linear_acceleration_x;
        std::float_t linear_acceleration_y;
        std::float_t linear_acceleration_z;
    };

    struct encoder_telemetry
    {
        std::int32_t forward_left;
        std::int32_t forward_right;
        std::int32_t back_right;
        std::int32_t back_left;

        std::uint32_t voltage_mv;
    };

    /***************************************************************************
     *
     * два типа хранилищ:
     * - canID <--> callback на функцию обработки данных в структуру
     * - canID <--> структура хранения + флаг обновления данных
     *
    ***************************************************************************/
    using callback = std::function< boost::any(std::vector< std::uint8_t >) >;

    using idCallbackMap = std::map< std::uint32_t, callback >;
    using idCallbackPair = std::pair< std::uint32_t, callback >;

    using idDataMap = std::map< std::uint32_t, storage >;
    using idDataPair = std::pair< std::uint32_t, storage >;

}

#endif
