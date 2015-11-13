#include <iw_tools/events_generator.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "events_generator");

    ros::NodeHandle n, np("~");
    iw_tools::EventsGenerator node(n, np);
    ros::spin();
}

