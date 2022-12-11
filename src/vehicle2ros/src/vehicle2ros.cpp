

#include "vehicle2ros.h"
#include "AsyncSerial.h"

using namespace std;

namespace car
{
    Car::Car(ros::NodeHandle &t_node_handle) : nh(t_node_handle)
    {

        get_parameter();

        vehicle_data_pub = nh.advertise<lgsvl_msgs::CanBusData>(param_pub_topic, 10);

        CallbackAsyncSerial serial(param_port_name, param_port_baud_rate);

        serial.setCallback(bind(&Car::serial_receive_callback, this, _1, _2));

        ros::spin();
    }

    void Car::get_parameter()
    {

        nh.getParam("port_name", param_port_name);
        nh.getParam("pub_topic", param_pub_topic);
        nh.getParam("port_baud_rate", param_port_baud_rate);
    }
    void Car::serial_receive_callback(const char *data, unsigned int len)
    {

        unpack_data(data, len);

        vehicle_data_pub.publish(vehicle_data);
    }
    void Car::unpack_data(const char *data, unsigned int len)
    {

        receive_buffer_.insert(receive_buffer_.end(), data, data + len);

        for (size_t i = 0; i < sizeof(frame); i++)
        {

            const auto frame_ptr = reinterpret_cast<frame *>(receive_buffer_.data() + i);

            if (frame_ptr->header1 == 171 && frame_ptr->header2 == 172)
            {

                // CRC Kontrolü burada yapılacak
                // Pack ID : x15550000

                received_data.vehicle_active = frame_ptr->veriler.vcu_data.autonomous_mode;
                received_data.speed = frame_ptr->veriler.vcu_data.throttle;
                received_data.steer_pct = frame_ptr->veriler.vcu_data.steering;

                receive_buffer_.erase(receive_buffer_.begin(),
                                      receive_buffer_.begin() +
                                          static_cast<long>(i) +
                                          sizeof(DATA));
            }
        }
    }

}