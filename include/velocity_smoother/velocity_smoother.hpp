/**
 * Source file: yocs_velocity_smoother: https://github.com/yujinrobot/yujin_ocs/tree/devel/yocs_velocity_smoother
*/

#ifndef __VELOCITY_SMOOTHER__HPP__
#define __VELOCITY_SMOOTHER__HPP__

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <velocity_smoother/VelocitySmootherConfig.h>

#include <mutex>

#define PERIOD_RECORD_SIZE  5
#define ZERO_VEL_COMMAND    geometry_msgs::Twist();
#define IS_ZERO_VELOCITY(a) ((a.linear.x == 0.0) && (a.angular.z == 0.0))

namespace velocity_smoother
{
class VelocitySmoother
{

typedef velocity_smoother::VelocitySmootherConfig Config;
typedef dynamic_reconfigure::Server<Config> ParamterConfigServer;
typedef dynamic_reconfigure::Server<Config>::CallbackType CallbackType;

public:
    VelocitySmoother(const std::string &name);
    ~VelocitySmoother()
    {
        if (dynamic_srv_ != NULL)
        {
            delete dynamic_srv_;
        }
    }
    /**
     * Initialise from a nodelet's private nodehandle.
     * @param nh : private nodehandle
     * @return bool : success or failure
     */
    bool init(ros::NodeHandle& nh);

    /**
     * Main running
     */
    void run();

    void shutdown() { shutdown_req = true; };

    std::mutex locker;

private:
    void reconfigureCB(Config& config, uint32_t level);
    void robotVelCB(const geometry_msgs::Twist::ConstPtr& msg);
    void odometryCB(const nav_msgs::Odometry::ConstPtr& msg);

    double sign(double x){ return x < 0.0 ? -1.0 : 1.0; };

    double median(std::vector<double> values)
    {
        // Return the median element of an doubles vector
        nth_element(values.begin(), values.begin() + values.size()/2, values.end());
        return values[values.size()/2];
    };

private:
    std::string name;

    // Parameters
    double smoothing_frequency;
    double max_vel_x;
    double max_vel_theta;
    double max_accel_x;
    double max_accel_theta;
    double max_decel_x;
    double max_decel_theta;
    double decel_factor;

    bool shutdown_req;
    bool input_active;
    double cb_avg_time;
    ros::Time last_cb_time;
    std::vector<double> period_record;
    unsigned int pr_next;

    geometry_msgs::Twist last_cmd_vel;
    geometry_msgs::Twist current_vel;
    geometry_msgs::Twist target_vel_;

    // Subscribers:
    ros::Subscriber sub_odometry;
    ros::Subscriber sub_curr_vel;

    // Publishers:
    ros::Publisher pub_smoothed_vel;
    
    ParamterConfigServer* dynamic_srv_;
};    
} // namespace velocity_smoother

#endif // __VELOCITY_SMOOTHER__HPP__