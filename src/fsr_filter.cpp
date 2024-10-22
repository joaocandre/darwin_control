#include <map>
#include <vector>
#include <string>
#include <iostream>
#include <memory>]
#include <math>
#include <ros/ros.h>
#include <ros_mplibrary_msgs/FilteredFSR.h>
#include <ext/math.hpp>

#include


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define MSG_QUEUE 10
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
using FSRSensor = ros::Sensor< gazebo_msgs::ContactsState >;
using std::vector;
using std::string;
using std::shared_ptr;


// Implements a ROS node that pre-process FSR readings
// This node creates new topics for each FSR sensor (original name + "_filtered") that publishes minimal uniform unidimensional output
// Temporary workaround for the data mismatch being published on the FSR sensor topics (excessive number of collisions, occasionally empty vector)
// TODO(joao): Implemente a message_filter (https://wiki.ros.org/message_filters#Example_.28C.2B-.2B-.29) on HumanoidSensorInterface or
//  multiSensor classes that create subscribers to topics ansd connect to the sensor callbacks (NOTE: different callback signature?)
// NOTE: only working for the darwin and nao robots atm, as sensor topics are hardcoded (bad design) in order to keep this implementation
//  completely separate from the walker controller

static const std::vector< std::string > darwin_fsr_topics = {
    "FSR/left_fsr_FL",  "FSR/left_fsr_FR",  "FSR/left_fsr_BL",  "FSR/left_fsr_BR",
    "FSR/right_fsr_FL", "FSR/right_fsr_FR", "FSR/right_fsr_BL", "FSR/right_fsr_BR"
};

static const std::vector< std::string > nao_fsr_topics = {
    "FSR/LFoot/FrontLeft", "FSR/LFoot/FrontRight", "FSR/LFoot/RearLeft", "FSR/LFoot/RearRight",
    "FSR/RFoot/FrontLeft", "FSR/RFoot/FrontRight", "FSR/RFoot/RearLeft", "FSR/RFoot/RearRight"
};

static const std::map< std::string, const std::vector< std::string >* > fsr_topics = {
    {"darwin", &darwin_fsr_topics },
    {"nao",    &nao_fsr_topics    }
};




//------------------------------------------------------------------------------
/// @brief      Calculate the Euclidean norm of a 3D vector
///
/// @param[in]  x     Value along the 'x' axis
/// @param[in]  y     Value along the 'y' axis
/// @param[in]  z     Value along the 'z' axis
///
double norm(double x, double y, double z) {
    return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}



namespace ros {

template < typename T >
T convert(const gazebo_msgs::ContactsState& fsr_reading, bool average = true);


//------------------------------------------------------------------------------
/// @brief      Converts data from a *gazebo_msgs::ContactsState* instance into a *geometry_msgs::PointStamped* message.
///             Averages the values for all collisions in given *fsr_reading*
///
/// @note       Useful for simplicity - fsr systems are in most cases only required to provide a single reading per sensor.
///
/// @param[in]  fsr_reading  Input reading to convert.
///
/// @return     Stamped message with average of collision wrenches in x, y and z axis.
///
template < >
geometry_msgs::PointStamped convert< geometry_msgs::PointStamped >(const gazebo_msgs::ContactsState& fsr_reading, bool average = true) {
    geometry_msgs::PointStamped filtered;
    filtered.data = 0.0;

    size_t n_collisions = fsr_reading.states.size();

    for (size_t idx = 0; idx < n_collisions; idx++) {
        // @note it may be needed to use the absolute value of the wrench
        filtered.point.x += fsr_reading.states[idx].total_wrench.force.x;
        filtered.point.y += fsr_reading.states[idx].total_wrench.force.y;
        filtered.point.z += fsr_reading.states[idx].total_wrench.force.z;
    }

    if (average) {
        // compute average
        filtered.point.x /= n_collisions;
        filtered.point.y /= n_collisions;
        filtered.point.z /= n_collisions;
    }

    return filtered;
}


//------------------------------------------------------------------------------
/// @brief      Converts data from a *gazebo_msgs::ContactsState* instance into a *std_msgs* message.
///             Averages the values for all collisions in given *fsr_reading*
///
/// @note       Useful for simplicity - fsr systems are in most cases only required to provide a single reading per sensor.
///
/// @param[in]  fsr_reading  Input reading to convert.
///
/// @return     Stamped message with average of the norm of collision wrenches.
///
std_msgs::Float64 convert< std_msgs::Float64 >(const gazebo_msgs::ContactsState& fsr_reading, bool average = true) {
    std_msgs::Float64 filtered;
    filtered.data = 0.0;

    size_t n_collisions = fsr_reading.states.size();

    for (size_t idx = 0; idx < n_collisions; idx++) {
        filtered.data += norm(fsr_reading.states[idx].total_wrench.force.x,
                              fsr_reading.states[idx].total_wrench.force.y,
                              fsr_reading.states[idx].total_wrench.force.z);
    }

    if (average) {
        filtered.data /= n_collisions;
    }

    return filtered;
}

}  // namespace ros



template < typename MSG >
static void filterFSR(ros::NodeHandle* node, const std::string& source, const std::string& target) {
    // check if data is being received on source topic? or just subscribe with a callback (requires a class)
    // using waitForMessages is synchronous, class w/ member callback is better!


    // advertise target topic
    ros::Publisher pub = node.advertise< MSG >(target);


    while (ros::ok()) {
        // wait for single message
        auto msg = ros::topic::waitForMessage< gazebo_msgs::ContactsState >(source, *node, ros::Duration(0.5) /*timeout*/);
        if (msg.get()) {
            pub.publish(convert< MSG >(msg));
        } else {
            ROS_WARN("Messages not being published on source topic!");
        }
    }


    // ros::spin();
}

template< MSG > // better to use polymorphism vs meta programming?
class FSRFilter {
 public:
    using FSRType = gazebo_msgs::ContactsState;
    using FilteredMessage = MSG;

    explicit FSRFilter(const std::string& source, const std::string& target);

    virtual ~FSRFilter() = default;

 protected:
    void callback(const gazebo_msgs::ContactsState::Ptr& fsr_reading);
    ros::Subscriber _subscriber;
    ros::Publisher _publisher;
};



FSRFilter::FSRFilter(const std::string& source, const std::string& target, const std::string& ns) :
    _node(ns) {

        _subscriber = node.subscribe(source, &FSRFilter::callback, this);

        _publisher = 

}


// 

int main(int argc, char **argv) {
    // assert(argc >= 1);
    // std::string name(argv[1]);

    for (size_t idx = 1; idx < argc; idx++){
        /* code */
        // std::thread(argv[idx]);
        filters.emplace_back(FSRFilter< geometry_msgs::PointStamped >(argv[idx], argv[idx] + "/filtered"));
    }

    // ROS initializiaton (NOTE: node name is bypassed by launch file)
    // A private node handle is used to keep FSR topic naming consistent
    ros::init(argc, argv, "fsr_filter");
    ros::NodeHandle node_handle(robot_name);

    // instantiate Sensor & Publisher objects
    // multiSensor class is only useful for different types of sensors, as it stores pointers to baseSensor and requires recast to derived sensor types
    // if all sensors share the same message type, a vector is prefered (simpler interface)
    // TODO(joao): fix ros::Sensor copy and move costructors
    vector< shared_ptr<FSRSensor> > fsr_sensors;
    vector< ros::Publisher >        fsr_publishers;
    for (auto& sensor_topic : *fsr_topics.at(robot_name)) {
        std::cout << "[FSR_FILTER] Looking for FSR [" << robot_name << "/" << sensor_topic << "]";
        // waits for 0.5s for a message to be published
        auto check_msg = ros::topic::waitForMessage<gazebo_msgs::ContactsState>(sensor_topic, node_handle, ros::Duration(0.5));
        if (check_msg.get() != 0) {
            // only append to vectors if message was received (returned pointer is not empty -> topic does exist)
            // NOTE: Careful when instantiating Sensor objects, it may go wrong!
            fsr_sensors.emplace_back(new FSRSensor(&node_handle, "/" + robot_name + "/" + sensor_topic));
            fsr_publishers.emplace_back(node_handle.advertise<ros_mplibrary_msgs::FilteredFSR>(sensor_topic + "_filtered", MSG_QUEUE));
            std::cout << "... OK!" << std::endl;
            continue;
        }
        std::cout << " ... FAIL!"<< std::endl;
    }

    // publishing rate 20Hz -> should match the plugins update rate!
    // TODO(joao): fetch update rate from parameter server (urdf is already uploaded as a parameter)
    ros::Rate filter_rate(20);

    std::cout << "[FSR_FILTER] Ready..." << std::endl;

    // main loop
    while (ros::ok()) {
        // perform
        filterFSRValues(&fsr_sensors, &fsr_publishers);

        // spin and loop
        ros::spinOnce();
        filter_rate.sleep();
    }

    return 0;
}





// void filterFSRValues(std::vector< std::shared_ptr< FSRSensor > >* _sensors, std::vector< ros::Publisher >* _publishers) {
//     // it is assumed both arguments have the same dimension
//     for (size_t idx = 0; idx < _sensors->size(); idx++) {
//         // initialize data
//         ros_mplibrary_msgs::FilteredFSR filtered_value;
//         filtered_value.data = 0.0;
//         // read from sensor topics
//         auto n_states = _sensors->at(idx)->value().states.size();
//         if (n_states > 0) {
//             // if colllisions were detected
//             // returned value is averaged over all detected collisions (should be summed)
//             // norm is computed over 3-axial force values
//             float norm     = 0.0;
//             float norm_avg = 0.0;
//             float norm_sum = 0.0;
//             float norm_max = 0.0;
//             size_t ground_cols = 0;
//             for (size_t col = 0; col < n_states; col++) {
//                 norm = math::norm(_sensors->at(idx)->value().states[col].total_wrench.force.x,
//                                   _sensors->at(idx)->value().states[col].total_wrench.force.y,
//                                   _sensors->at(idx)->value().states[col].total_wrench.force.z);
//                 norm_sum += norm;
//                 norm_avg += (norm / n_states);
//                 if (norm > norm_max) norm_max = norm;
//                 // if (_sensors->at(idx)->value().states[col].collision2_name == "ground_plane::link::collision") ground_cols++;
//             }
//             // filtered_value.data = static_cast<float>(norm_sum / n_states);
//             // if (norm > filtered_value.data) filtered_value.data = norm;
//             // std::cout << "[FSR_FILTER][" << idx << "] Number of collisions: " << n_states << " (" << ground_cols << ") Avg F: " << norm_avg << ", Max F: " << norm_max << ", Sum F: " << norm_sum << std::endl;
//             filtered_value.data = norm_avg;
//         }
//         // publish to new topic
//         filtered_value.header = _sensors->at(idx)->value().header;
//         _publishers->at(idx).publish(filtered_value);
//     }
//     // std::cout << "]" << std::endl;
// }



// void contactsStatePrint(const gazebo_msgs::ContactState& _msg) {
//     // https://docs.ros.org/melodic/api/gazebo_msgs/html/msg/ContactState.html
//     std::cout << _msg.info << std::endl;
//     std::cout << "[" << _msg.collision1_name << "]\n\thas collided with \n[" << _msg.collision2_name << "]" << std::endl;
//     std::cout << "[Total Wrench]:"<< std::endl;
//     std::cout << "[Force]:  " << _msg.total_wrench.force.x  << ", " << _msg.total_wrench.force.y  << ", " << _msg.total_wrench.force.z  << std::endl;
//     std::cout << "[Torque]: " << _msg.total_wrench.torque.x << ", " << _msg.total_wrench.torque.y << ", " << _msg.total_wrench.torque.z << std::endl;
// }



// int main(int argc, char **argv) {
//     assert(argc >= 3);
//     string robot_name(argv[1]);

//     // ROS initializiaton (NOTE: node name is bypassed by launch file)
//     // A private node handle is used to keep FSR topic naming consistent
//     ros::init(argc, argv, "fsr_filter");
//     ros::NodeHandle node_handle(robot_name);

//     // instantiate Sensor & Publisher objects
//     // multiSensor class is only useful for different types of sensors, as it stores pointers to baseSensor and requires recast to derived sensor types
//     // if all sensors share the same message type, a vector is prefered (simpler interface)
//     // TODO(joao): fix ros::Sensor copy and move costructors
//     vector< shared_ptr<FSRSensor> > fsr_sensors;
//     vector< ros::Publisher >        fsr_publishers;
//     for (auto& sensor_topic : *fsr_topics.at(robot_name)) {
//         std::cout << "[FSR_FILTER] Looking for FSR [" << robot_name << "/" << sensor_topic << "]";
//         // waits for 0.5s for a message to be published
//         auto check_msg = ros::topic::waitForMessage<gazebo_msgs::ContactsState>(sensor_topic, node_handle, ros::Duration(0.5));
//         if (check_msg.get() != 0) {
//             // only append to vectors if message was received (returned pointer is not empty -> topic does exist)
//             // NOTE: Careful when instantiating Sensor objects, it may go wrong!
//             fsr_sensors.emplace_back(new FSRSensor(&node_handle, "/" + robot_name + "/" + sensor_topic));
//             fsr_publishers.emplace_back(node_handle.advertise<ros_mplibrary_msgs::FilteredFSR>(sensor_topic + "_filtered", MSG_QUEUE));
//             std::cout << "... OK!" << std::endl;
//             continue;
//         }
//         std::cout << " ... FAIL!"<< std::endl;
//     }

//     // publishing rate 20Hz -> should match the plugins update rate!
//     // TODO(joao): fetch update rate from parameter server (urdf is already uploaded as a parameter)
//     ros::Rate filter_rate(20);

//     std::cout << "[FSR_FILTER] Ready..." << std::endl;

//     // main loop
//     while (ros::ok()) {
//         // perform
//         filterFSRValues(&fsr_sensors, &fsr_publishers);

//         // spin and loop
//         ros::spinOnce();
//         filter_rate.sleep();
//     }

//     return 0;
// }
