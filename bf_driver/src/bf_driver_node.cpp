#include <ros/ros.h>
#include <string>
#include <cstdint>
#include <serial/serial.h>

#include "ros_operation/ros_subscriber.h"
#include "ros_operation/ros_publisher.h"

#include "bf_driver/msg_reader_session.h"
#include "bf_driver/msg_sender_session.h"


int main(int argc, char** argv)
{
    /***************************************************************************

     * инициализируем взаимодействие с ROS
     *
    ***************************************************************************/
    ros::init(argc, argv, "bf_driver");
    ros::NodeHandle nodeHandle("");

    std::string portName;

    std::double_t loopRate;
    std::uint32_t portBaudrate;

//    if (!nodeHandle.getParam("serial_port_name", portName))
//    {
        portName = "/dev/ttyUSB0";
//        ROS_WARN("Default serial port name");
//    }

//    if (!nodeHandle.getParam("serial_port_baudrate", portBaudrate))
//    {
        portBaudrate = 57600;
//        ROS_WARN("Default serial port baudrate");
//    }

//    if (!nodeHandle.getParam("loop_rate", loopRate))
//    {
        loopRate = 100.0;
//        ROS_WARN("Default loop rate");
//    }

    // частота работы программы
    ros::Rate rate(loopRate);
    /***************************************************************************
     *
     * инициализируем работу с портом передачи данных
     *
    ***************************************************************************/
    serial::Serial port;

    port.setPort(portName);
    port.setBaudrate(portBaudrate);
    serial::Timeout to = serial::Timeout(50,50,0,50,0);
    port.setTimeout(to);

    try
    {
        port.open();
    }
    catch (const serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to connect to port: " << portName);
        ros::shutdown();
    }

    // если пришли сюда он точно будет открыт
    if (port.isOpen())
    {
        ROS_INFO_STREAM("Successfully connected to serial port: " << portName);
    }

    /***************************************************************************
     *
     * создаем и инициализируем алгоритмические данные
     *
    ***************************************************************************/
    // вроде как должны быть в шине уникальные айдишники
    // следовательно возможно использовать общие словари
    // для приема и прередачи
    // !но пока разделим
    vehicle::idDataMap rxDataMap;
    vehicle::idDataMap txDataMap;

    bf_driver::MsgReaderSession reader{rxDataMap};
	reader.setPort(port);

	bf_driver::MsgSenderSession sender{txDataMap};
	sender.setPort(port);

    /***************************************************************************
     *
     * запускаем считывание информации из РОС в словарь
     *
    ***************************************************************************/
	bf_driver::RosSubscriber subscribers{nodeHandle, txDataMap};

    /***************************************************************************
     *
     * инициализируем публикацию информации в РОС
     *
    ***************************************************************************/
	bf_driver::RosPublisher publishers{nodeHandle, rxDataMap};


    while (ros::ok())
    {
        // - разбираем поток байтов от SIGMA
        // - фильтруем
        // - сохраняем в словарь для RosPublisher-a
        while (!reader.worker())
        {
            ;
        }

        // - просматриваем словарь
        // - публикуем обновленные данные в ROS
        publishers.work();

        // - просматриваем словарь от RosSubscriber
        // - отсылаем данные обновленные данные в SIGMA
        while (!sender.worker())
        {
            ;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
