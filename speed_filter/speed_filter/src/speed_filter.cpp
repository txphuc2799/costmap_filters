#include <speed_filter/speed_filter.h>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(costmap_2d::SpeedFilter, costmap_2d::Layer)

namespace costmap_2d
{
SpeedFilter::SpeedFilter():
    speed_limit_(NO_SPEED_LIMIT),
    speed_limit_prev_(NO_SPEED_LIMIT),
    percentage_(false),
    filter_mask_(nullptr)
{
}

SpeedFilter::~SpeedFilter() {}

void SpeedFilter::onInitialize()
{
    ros::NodeHandle nh_private_("~" + name_), nh_;

    // Initialize parameters
    nh_private_.param("speed_limit_topic", speed_limit_topic_, std::string("speed_limit"));
    nh_private_.param("map_topic", map_topic_, std::string("/map"));
    nh_private_.param("base", base_, 100.0);
    nh_private_.param("multiplier", multiplier_, -1.0);
    nh_private_.param("type", type_, 1);
    nh_private_.param("enable", enabled_, true);

    global_frame_ = layered_costmap_->getGlobalFrameID();

    // Only resubscribe if topic has changed
    if (map_sub_.getTopic() != ros::names::resolve(map_topic_))
    {
        // we'll subscribe to the latched topic that the map server uses
        ROS_INFO("Requesting the map...");
        map_sub_ = nh_.subscribe(map_topic_, 1, &SpeedFilter::mapCallback, this);
        map_received_ = false;

        ros::Rate r(10);
        while (!map_received_ && nh_.ok())
        {
            ros::spinOnce();
            r.sleep();
        }
    }
    speed_limit_pub_ = nh_.advertise<speed_filter_msgs::SpeedLimit>(speed_limit_topic_, 1);
}

void SpeedFilter::activate()
{
    onInitialize();
}

void SpeedFilter::deactivate()
{
    map_sub_.shutdown();
}

void SpeedFilter::reset()
{
    onInitialize();
    current_ = false;
}

void SpeedFilter::mapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    unsigned int size_x = msg->info.width, size_y = msg->info.height;
    ROS_INFO("Received a %d X %d map at %f m/pix", size_x, size_y, msg->info.resolution);

    if (type_ == SPEED_FILTER_PERCENT) {
        // Using speed limit in % of maximum speed
        percentage_ = true;
        ROS_INFO("SpeedFilter: Using expressed in a percent from maximum speed"
        " speed_limit = %f + filter_mask_data * %f",
        base_, multiplier_);
    } else if (type_ == SPEED_FILTER_ABSOLUTE) {
        // Using speed limit in m/s
        percentage_ = false;
        ROS_INFO("SpeedFilter: Using absolute speed_limit = %f + filter_mask_data * %f",
        base_, multiplier_);
    } else {
        ROS_ERROR("SpeedFilter: Mode is not supported");
        return;
    }

    if (!filter_mask_) {
        ROS_INFO("SpeedFilter: Received filter mask from %s topic.", map_topic_.c_str());
    } else {
        ROS_INFO("SpeedFilter: New filter mask arrived from %s topic. Updating old filter mask.",
        map_topic_.c_str());
        filter_mask_.reset();
    }

    map_received_ = true;
    filter_mask_ = msg;
}

void SpeedFilter::process(costmap_2d::Costmap2D& master_grid,
                int min_i, int min_j, int max_i, int max_j,
                const geometry_msgs::Pose2D & pose)
{
    if (!filter_mask_){
        ROS_WARN("SpeedFilter: Filter mask was not received");
        return;
    }

    geometry_msgs::Pose2D mask_pose;  // robot coordinates in mask frame

    // Transforming robot pose from current layer frame to mask frame
    if (!transformPose(global_frame_, pose, filter_mask_->header.frame_id, mask_pose)) {
        return;
    }

    // Converting mask_pose robot position to filter_mask_ indexes (mask_robot_i, mask_robot_j)
    unsigned int mask_robot_i, mask_robot_j;
    if (!worldToMask(filter_mask_, mask_pose.x, mask_pose.y, mask_robot_i, mask_robot_j)) {
        return;
    }

    // Getting filter_mask data from cell where the robot placed and
    // calculating speed limit value
    int8_t speed_mask_data = getMaskData(filter_mask_, mask_robot_i, mask_robot_j);

    if (speed_mask_data == SPEED_MASK_NO_LIMIT) {
        // Corresponding filter mask cell is free.
        // Setting no speed limit there.
        speed_limit_ = NO_SPEED_LIMIT;
    } else if (speed_mask_data == SPEED_MASK_UNKNOWN) {
        // Corresponding filter mask cell is unknown.
        // Do nothing.
        ROS_ERROR(
        "SpeedFilter: Found unknown cell in filter_mask[%i, %i], "
        "which is invalid for this kind of filter",
        mask_robot_i, mask_robot_j);
        return;
    } else {
        // Normal case: speed_mask_data in range of [1..100]
        speed_limit_ = speed_mask_data * multiplier_ + base_;
        if (percentage_) {
        if (speed_limit_ < 0.0 || speed_limit_ > 100.0) {
            ROS_WARN(
            "SpeedFilter: Speed limit in filter_mask[%i, %i] is %f%%, "
            "out of bounds of [0, 100]. Setting it to no-limit value.",
            mask_robot_i, mask_robot_j, speed_limit_);
            speed_limit_ = NO_SPEED_LIMIT;
        }
        } else {
        if (speed_limit_ < 0.0) {
            ROS_WARN(
            "SpeedFilter: Speed limit in filter_mask[%i, %i] is less than 0 m/s, "
            "which can not be true. Setting it to no-limit value.",
            mask_robot_i, mask_robot_j);
            speed_limit_ = NO_SPEED_LIMIT;
        }
        }
    }

    if (speed_limit_ != speed_limit_prev_) {
        if (speed_limit_ != NO_SPEED_LIMIT) {
            ROS_INFO("SpeedFilter: Set speed limit!");
        } else {
            ROS_INFO("SpeedFilter: Set normal speed");
        }

        // Forming and publishing new SpeedLimit message
        speed_filter_msgs::SpeedLimit msg;
        msg.header.frame_id = global_frame_;
        msg.header.stamp = ros::Time::now();
        msg.percentage = percentage_;
        msg.speed_limit = speed_limit_/100.0;
        speed_limit_pub_.publish(msg);

        speed_limit_prev_ = speed_limit_;
    }
}

void SpeedFilter::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                               double* max_x, double* max_y)
{
    if (!enabled_ || !map_received_){
        return;
    }

    latest_pose_.x = robot_x;
    latest_pose_.y = robot_y;
    latest_pose_.theta = robot_yaw;
}

void SpeedFilter::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
    if (!enabled_ || !map_received_){
        return;
    }

    process(master_grid, min_i, min_j, max_i, max_j, latest_pose_);
    current_ = true;
}

bool SpeedFilter::transformPose(
        const std::string global_frame,
        const geometry_msgs::Pose2D & global_pose,
        const std::string mask_frame,
        geometry_msgs::Pose2D & mask_pose) const
{
    if (mask_frame != global_frame) {
        // Filter mask and current layer are in different frames:
        // Transform (global_pose.x, global_pose.y) point from current layer frame (global_frame)
        // to mask_pose point in mask_frame
        geometry_msgs::TransformStamped transform;
        geometry_msgs::PointStamped in, out;
        in.header.stamp = ros::Time::now();
        in.header.frame_id = global_frame;
        in.point.x = global_pose.x;
        in.point.y = global_pose.y;
        in.point.z = 0;

        try {
            tf_->transform(in, out, mask_frame, ros::Duration(0.1));
        } catch (tf2::TransformException & ex) {
        ROS_ERROR(
            "CostmapFilter: failed to get costmap frame (%s) "
            "transformation to mask frame (%s) with error: %s",
            global_frame.c_str(), mask_frame.c_str(), ex.what());
        return false;
        }
        mask_pose.x = out.point.x;
        mask_pose.y = out.point.y;
    } else {
        // Filter mask and current layer are in the same frame:
        // Just use global_pose coordinates
        mask_pose = global_pose;
    }

    return true;
}

bool SpeedFilter::worldToMask(nav_msgs::OccupancyGrid::ConstPtr& filter_mask,
  double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  const double origin_x = filter_mask->info.origin.position.x;
  const double origin_y = filter_mask->info.origin.position.y;
  const double resolution = filter_mask->info.resolution;
  const unsigned int size_x = filter_mask->info.width;
  const unsigned int size_y = filter_mask->info.height;

  if (wx < origin_x || wy < origin_y) {
    return false;
  }

  mx = static_cast<unsigned int>((wx - origin_x) / resolution);
  my = static_cast<unsigned int>((wy - origin_y) / resolution);
  if (mx >= size_x || my >= size_y) {
    return false;
  }

  return true;
}

} // namespace costmap_2d