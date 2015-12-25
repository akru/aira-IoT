#include <ros/ros.h>
#include <small_atc/GeoHelpers.h>

class GeoHelper
{
public:
    GeoHelper(const MapMetaData &meta);
    ~GeoHelper();
    
    std::vector<SatFix>
    local2global(const std::Vector<Point> &path) {
        std::vector<SatFix> sat_path;
        for (auto i : path)
            sat_path.push_back(point2SatFix(i, map_meta)); 
        return sat_path;
    }

    std::vector<Point>
    global2local(const std::Vector<SatFix> &sat_path,
             const MapMetaData &map_meta) {
        std::vector<Point> path;
        for (auto i : sat_path)
            sat_path.push_back(satFix2Point(i, map_meta)); 
        return path;
    }

private:
    MapMetaData map_meta;
};


int main(int argc, const char *argv[])
{
    
    return 0;
}
