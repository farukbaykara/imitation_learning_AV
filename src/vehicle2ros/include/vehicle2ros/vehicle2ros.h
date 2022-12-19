#ifndef VEHICLE2ROS_INCLUDED
#define VEHICLE2ROS_INCLUDED

#include <ros/ros.h>

#include <iostream>
#include <cmath>
#include <vector>
#include <array>
#include <math.h>

#include <lgsvl_msgs/CanBusData.h>

#include <boost/asio.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/function.hpp>

namespace car
{
    class Car
    {
    private:
        #pragma pack(1)

        union DATA
        {

            struct VCU_data
            {
                uint8_t autonomous_mode;

            } vcu_data;
        };

        struct frame
        {

            uint8_t header1;
            uint8_t header2;
            DATA veriler;
        }frame_data;

        struct receivedData
        {

            uint32_t speed;
            uint32_t steer_pct;
            uint8_t vehicle_active;

        } received_data;

        ros::Publisher vehicle_data_pub;

        lgsvl_msgs::CanBusData vehicle_data;

        std::string param_port_name;

        std::string param_pub_topic;

        int param_port_baud_rate;

        std::vector<uint8_t> receive_buffer_;

        ros::NodeHandle nh;

    public:
        Car(ros::NodeHandle &t_node_handle);

        void serial_receive_callback(const char *data, unsigned int len);

        void unpack_data(const char *data, unsigned int len);

        void get_parameter();
    };

}
#endif