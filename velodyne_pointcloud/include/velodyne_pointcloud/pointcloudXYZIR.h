//
// Created by pedro on 07-03-2019.
//

#ifndef PROJECT_POINTCLOUDXYZIR_H
#define PROJECT_POINTCLOUDXYZIR_H

// System Includes
#include <velodyne_pointcloud/rawdata.h>

namespace velodyne_pointcloud
{
    class PointcloudXYZIR : public velodyne_rawdata::DataContainerBase
    {
        public:
            velodyne_rawdata::VPointCloud::Ptr pc;

            PointcloudXYZIR() : pc(new velodyne_rawdata::VPointCloud) {}

            virtual void addPoint(const float& x, const float& y, const float& z, const uint16_t& ring, const uint16_t& azimuth, const float& distance, const float& intensity);
    };
}

#endif //PROJECT_POINTCLOUDXYZIR_H
