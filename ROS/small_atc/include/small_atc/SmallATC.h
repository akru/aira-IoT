#ifndef __SMALL_ATC_H__
#define __SMALL_ATC_H__

#include "Planner.h" 
#include "DynamicOctoMap.h" 
#include "HasCollision.h" 
#include "ObstacleProvider.h" 

#include <ros/ros.h>
#include <dron_common_msgs/LocalRouteRequest.h>

using namespace ros;
using namespace dron_common_msgs;

class SmallATC : public ObstacleProvider
{
public:
    SmallATC(const std::string &filename, const MapMetaData &map_meta);
    ~SmallATC() {
        delete atc_planner;
    }

    ObstacleVector getObstacles() const;

    /**
     * Obstacle registration method,
     * returns obstacle id
     **/
    int addObstacle(const DynamicOctoMap &obstacle);

    /**
     * Remove obstacle with id from collider
     **/
    bool removeObstacle(int id);

    /**
     * Exec the Small Air Traffic Controller
     **/
    int exec();

protected:
    void requestHandler(const LocalRouteRequest::ConstPtr &msg);

private:
    Planner      *atc_planner;
    NodeHandle    node_handle;

    Publisher     route_response;
    Subscriber    route_request;

    // Map with all obstacles
    std::map<int,DynamicOctoMap> collider;
    // Map with all obstacle topics
    std::map<int,Publisher>  obstacle_topics;
    // The last used id
    int last_id;
};

#endif
