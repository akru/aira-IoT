#include <small_atc/DynamicOctoMap.h>
#include <small_atc/CollisionCheckerFCL.h>
#include <octomap_msgs/conversions.h>
#include <boost/shared_ptr.hpp>
#include <fcl/octree.h>
#include <ros/ros.h>

using namespace boost;

DynamicOctoMap::DynamicOctoMap(int resolution)
    : octomap(new octomap::OcTree(resolution)) {
    std::cout << "Map with resolution=" << resolution << " created!" << std::endl;
    shared_ptr<fcl::CollisionGeometry> geometry(new fcl::OcTree(octomap));
    setCollisionObject(new fcl::CollisionObject(geometry));
}

DynamicOctoMap::DynamicOctoMap(const std::string &filename)
    : octomap(new octomap::OcTree(filename)) {
    std::cout << "File " << filename << " loaded!" << std::endl;
    shared_ptr<fcl::CollisionGeometry> geometry(new fcl::OcTree(octomap));
    setCollisionObject(new fcl::CollisionObject(geometry));
}

octomap_msgs::Octomap DynamicOctoMap::getOctomapMsg() const 
{
    octomap_msgs::Octomap msg;
    octomap_msgs::binaryMapToMsg(*octomap, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "/map";
    return msg; 
}

void DynamicOctoMap::drawAABB(const geometry_msgs::Point &AA,
                              const geometry_msgs::Point &BB)
{
    octomap::Pointcloud cloud;
    for (int x = AA.x; x < BB.x; x++)
        for (int z = AA.z; z < BB.z; z++) {
            cloud.push_back(x, AA.y, z); 
            cloud.push_back(x, BB.y, z); 
        }
    for (int y = AA.y; y < BB.y; y++)
        for (int z = AA.z; z < BB.z; z++) {
            cloud.push_back(AA.x, y, z); 
            cloud.push_back(BB.x, y, z); 
        }
    for (int x = AA.x; x < BB.x; x++)
        for (int y = AA.y; y < BB.y; y++) {
            cloud.push_back(x, y, AA.z); 
            cloud.push_back(x, y, BB.z); 
        }
    octomap->insertPointCloud(cloud, octomap::point3d());
}

void DynamicOctoMap::drawSphere(const geometry_msgs::Point &center, double R)
{
    octomap::Pointcloud cloud;
    for (double fi = 0; fi < 2 * M_PI; fi += 0.1)
        for (double teta = 0; teta < 2 * M_PI; teta += 0.1)
            cloud.push_back(center.x + R * sin(teta) * cos(fi),
                            center.y + R * sin(teta) * sin(fi),
                            center.z + R * cos(teta));
    octomap->insertPointCloud(cloud, octomap::point3d());
}
