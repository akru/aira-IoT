#ifndef __OBSTACLE_PROVIDER_H__
#define __OBSTACLE_PROVIDER_H__

#include "HasCollision.h" 
#include <vector>

typedef std::vector<const HasCollision*> ObstacleVector;

class ObstacleProvider
{
public:
    virtual ObstacleVector getObstacles() const = 0; 
};

#endif
