#include <small_atc/SmallATC.h>
#include <small_atc/PlannerOMPL.h>
#include <small_atc/DynamicOctoMap.h>
#include <dron_common_msgs/LocalRouteResponse.h>
#include <octomap_msgs/Octomap.h>

using namespace ros;
using namespace dron_common_msgs;

SmallATC::SmallATC(const std::string &filename, const MapMetaData &map_meta)
    : atc_planner(new PlannerOMPL(this, map_meta))
    , last_id(-1) {
    // Making the route request/response handlers
    route_response = node_handle.advertise<LocalRouteResponse>("/route/response_local", 1);
    route_request  = node_handle.subscribe<LocalRouteRequest>("/route/request_local", 1, 
            &SmallATC::requestHandler, this); 
    // Adding the first obstacle - the topographic map
    DynamicOctoMap map(filename);
    addObstacle(map);
}

ObstacleVector SmallATC::getObstacles() const {
    ObstacleVector obstacles;
    for (auto k = collider.begin(); k != collider.end(); ++k)
        obstacles.push_back(&k->second);
    return obstacles;
}

void SmallATC::requestHandler(const LocalRouteRequest::ConstPtr &msg) { 
    std::cout << "Planning..." << std::endl;
    LocalRouteResponse response;
    for (int i = 0; i < msg->checkpoints.size() - 1; i++) {
        auto plan = atc_planner->plan(msg->checkpoints[i],
                                      msg->checkpoints[i+1]);
        response.route.insert(response.route.end(), plan.begin(), plan.end());
    }
    response.valid = response.route.size() > 0;
    std::string valid_str = response.valid ? "valid" : "invalid"; 
    std::cout << "Plan: " << valid_str << std::endl;
    if (response.valid) {
        // Register route
        DynamicOctoMap obstacle_map(atc_planner->getMapMetaData().resolution);
        obstacle_map.drawRoute(response.route, 25);
        addObstacle(obstacle_map);
        std::cout << "Plan registered" << std::endl;
    }
    // Publish response
    route_response.publish(response);
    std::cout << "Plan published" << std::endl;
}

int SmallATC::exec() {
    ros::Rate dur(1);
    while (ros::ok()) {
        // Publish all the obstacles with associated topics
        for (auto k = collider.begin(); k != collider.end(); ++k)
            obstacle_topics[k->first].publish(k->second.getOctomapMsg());
        ros::spinOnce();
        dur.sleep();
    }
    return 0;
}

int SmallATC::addObstacle(const DynamicOctoMap &obstacle) {
    collider[++last_id] = obstacle;
    Publisher pub = node_handle.advertise<octomap_msgs::Octomap>(
            "/obstacle/" + std::to_string(last_id), 1);
    obstacle_topics[last_id] = pub;
    std::cout << "Obstacle #" << last_id << " registered!" << std::endl;
    return last_id;
}

bool SmallATC::removeObstacle(int id) {
    return collider.erase(id) + obstacle_topics.erase(id);
}
