#ifndef BINARY_FILTER_H_
#define BINARY_FILTER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Config.h>


namespace costmap_2d
{
static constexpr int8_t STATE_ON = 99;
static constexpr int8_t STATE_OFF = 1;

class BinaryFilter : public Layer
{
public:
    BinaryFilter();

    virtual ~BinaryFilter();

    virtual void onInitialize();
    virtual void activate();
    virtual void deactivate();
    virtual void reset();

    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                              double* max_x, double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    /**
     * @brief Process the keepout layer at the current pose / bounds / grid
     */
    void process(costmap_2d::Costmap2D& master_grid,
                int min_i, int min_j, int max_i, int max_j,
                const geometry_msgs::Pose2D & pose);

    /**
     * @brief:  Transforms robot pose from current layer frame to mask frame
     * @param:  global_frame Costmap frame to transform from
     * @param:  global_pose Robot pose in costmap frame
     * @param:  mask_frame Filter mask frame to transform to
     * @param:  mask_pose Output robot pose in mask frame
     * @return: True if the transformation was successful, false otherwise
     */
    bool transformPose(
        const std::string global_frame,
        const geometry_msgs::Pose2D & global_pose,
        const std::string mask_frame,
        geometry_msgs::Pose2D & mask_pose) const;

    /**
     * @brief: Convert from world coordinates to mask coordinates.
         Similar to Costmap2D::worldToMap() method but works directly with OccupancyGrid-s.
    * @param  filter_mask Filter mask on which to convert
    * @param  wx The x world coordinate
    * @param  wy The y world coordinate
    * @param  mx Will be set to the associated mask x coordinate
    * @param  my Will be set to the associated mask y coordinate
    * @return True if the conversion was successful (legal bounds) false otherwise
    */
    bool worldToMask(nav_msgs::OccupancyGrid::ConstPtr& filter_mask,
        double wx, double wy, unsigned int & mx, unsigned int & my) const;

    /**
     * @brief  Get the data of a cell in the filter mask
     * @param  filter_mask Filter mask to get the data from
     * @param  mx The x coordinate of the cell
     * @param  my The y coordinate of the cell
     * @return The data of the selected cell
    */
    inline int8_t getMaskData(nav_msgs::OccupancyGrid::ConstPtr& filter_mask,
        const unsigned int mx, const unsigned int my) const
    {
        return filter_mask->data[my * filter_mask->info.width + mx];
    }
    
private:
    void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
    void changeState(int state);

private:
    ros::Subscriber map_sub_;
    ros::Publisher binary_state_pub_;
    std::vector<int8_t> map_data_;
    nav_msgs::OccupancyGrid::ConstPtr filter_mask_;
    geometry_msgs::Pose2D latest_pose_;
    std::string map_topic_, global_frame_;
    std::string binary_state_topic_;
    bool map_received_;
    int binary_state_, prev_binary_state_;
};
} // namespace costmap_2d

#endif  // BINARY_FILTER_H_