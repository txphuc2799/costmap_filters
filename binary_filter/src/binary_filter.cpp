#include <binary_filter/binary_filter.h>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(costmap_2d::BinaryFilter, costmap_2d::Layer)

namespace costmap_2d
{
BinaryFilter::BinaryFilter():
    filter_mask_(nullptr),
    binary_state_(STATE_OFF), prev_binary_state_(STATE_OFF)
{
}

BinaryFilter::~BinaryFilter() {}

void BinaryFilter::onInitialize()
{
    ros::NodeHandle nh_private_("~" + name_), nh_;

    // Initialize parameters
    nh_private_.param("map_topic", map_topic_, std::string("/map"));
    nh_private_.param("enable", enabled_, true);
    nh_private_.param("binary_state_topic", binary_state_topic_, std::string("/binary_state"));

    global_frame_ = layered_costmap_->getGlobalFrameID();

    // Only resubscribe if topic has changed
    if (map_sub_.getTopic() != ros::names::resolve(map_topic_))
    {
        // we'll subscribe to the latched topic that the map server uses
        ROS_INFO("Requesting the map...");
        map_sub_ = nh_.subscribe(map_topic_, 1, &BinaryFilter::mapCallback, this);
        map_received_ = false;

        ros::Rate r(10);
        while (!map_received_ && nh_.ok())
        {
            ros::spinOnce();
            r.sleep();
        }
    }

    binary_state_pub_ = nh_.advertise<std_msgs::Bool>(binary_state_topic_, 1);
}

void BinaryFilter::activate()
{
    onInitialize();
}

void BinaryFilter::deactivate()
{
    map_sub_.shutdown();
}

void BinaryFilter::reset()
{
    onInitialize();
    current_ = false;
}

void BinaryFilter::mapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    unsigned int size_x = msg->info.width, size_y = msg->info.height;
    ROS_INFO("Received a %d X %d map at %f m/pix", size_x, size_y, msg->info.resolution);

    map_received_ = true;
    filter_mask_ = msg;
}

void BinaryFilter::process(costmap_2d::Costmap2D& master_grid,
                int min_i, int min_j, int max_i, int max_j,
                const geometry_msgs::Pose2D & pose)
{
    if (!filter_mask_){
        ROS_WARN("BinaryFilter: Filter mask was not received");
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
    int8_t mask_data = getMaskData(filter_mask_, mask_robot_i, mask_robot_j);

    if (mask_data == STATE_ON) {
        binary_state_ = STATE_ON;
        if (binary_state_ != prev_binary_state_){
            changeState(STATE_ON);
        }
    }
    else{
        binary_state_ = STATE_OFF;
        if (binary_state_ != prev_binary_state_){
            changeState(STATE_OFF);
        }
    }
    prev_binary_state_ = binary_state_;
}

void BinaryFilter::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                               double* max_x, double* max_y)
{
    if (!enabled_ || !map_received_){
        return;
    }

    latest_pose_.x = robot_x;
    latest_pose_.y = robot_y;
    latest_pose_.theta = robot_yaw;
}

void BinaryFilter::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
    if (!enabled_ || !map_received_){
        return;
    }

    process(master_grid, min_i, min_j, max_i, max_j, latest_pose_);
    current_ = true;
}

bool BinaryFilter::transformPose(
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

bool BinaryFilter::worldToMask(nav_msgs::OccupancyGrid::ConstPtr& filter_mask,
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

void BinaryFilter::changeState(int state)
{
    std_msgs::Bool msg;

    if (state == STATE_ON) {
        msg.data = true;
        ROS_INFO("BinaryFilter: Switched ON");
        binary_state_pub_.publish(msg);
    }
    else if (state == STATE_OFF){
        msg.data = false;
        ROS_INFO("BinaryFilter: Switched OFF");
        binary_state_pub_.publish(msg);
    } 
}

} // namespace costmap_2d