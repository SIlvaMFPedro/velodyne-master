//
// Created by pedro on 07-03-2019.
//

#ifndef PROJECT_DATACONTAINERBASE_H
#define PROJECT_DATACONTAINERBASE_H

// System Includes
#include <ros/ros.h>

namespace velodyne_rawdata
{
    class DataContainerBase
    {
        public:
            virtual void addPoint(const float& x, const float& y, const float& z, const uint16_t& ring, const uint16_t& azimuth, const float& distance, const float& intensity) = 0;
    };
}

#endif //PROJECT_DATACONTAINERBASE_H
