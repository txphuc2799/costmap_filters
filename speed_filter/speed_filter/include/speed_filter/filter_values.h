#ifndef FILTER_VALUES_H_
#define FILTER_VALUES_H_

#include <ros/ros.h>

namespace costmap_2d
{
    static constexpr uint8_t SPEED_FILTER_PERCENT = 1;
    static constexpr uint8_t SPEED_FILTER_ABSOLUTE = 2;

    /** Default values for base and multiplier */
    static constexpr double BASE_DEFAULT = 100.0;
    static constexpr double MULTIPLIER_DEFAULT = -1.0;

    /** Speed filter constants */
    static constexpr int8_t SPEED_MASK_UNKNOWN = 99;
    static constexpr int8_t SPEED_MASK_NO_LIMIT = 1;
    static constexpr double NO_SPEED_LIMIT = 100.0;
} // namespace costmap_2d

#endif  // FILTER_VALUES_H_