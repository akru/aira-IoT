#include <small_atc/SmallATC.h>
#include <small_atc/PlannerOMPL.h>
#include <small_atc/DynamicOctoMap.h>
#include <ros/ros.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "small_atc");

    dron_common_msgs::SatFix origin;
    SmallATC atc(argv[1], {3, {100, 100, 100}, origin});

    return atc.exec();
}
