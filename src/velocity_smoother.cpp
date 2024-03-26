#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ecl/threads/thread.hpp>

#include <velocity_smoother/velocity_smoother.hpp>

namespace velocity_smoother
{
VelocitySmoother::VelocitySmoother(const std::string &name):
    name(name),
    shutdown_req(false),
    input_active(false),
    pr_next(0),
    dynamic_srv_(NULL) {};

void VelocitySmoother::reconfigureCB(Config& config, uint32_t level)
{
    locker.lock();
    max_vel_x       = config.max_vel_x;
    max_vel_theta   = config.max_vel_theta;
    max_accel_x     = config.max_accel_x;
    max_accel_theta = config.max_accel_theta;
    decel_factor    = config.decel_factor;
    max_decel_x     = config.max_accel_x;
    max_decel_theta = config.max_accel_theta;
    locker.unlock();
}

void VelocitySmoother::robotVelCB(const geometry_msgs::Twist::ConstPtr& msg)
{
    if (period_record.size() < PERIOD_RECORD_SIZE)
    {
        period_record.push_back((ros::Time::now() - last_cb_time).toSec());
    }
    else
    {
        period_record[pr_next] = (ros::Time::now() - last_cb_time).toSec();
    }

    pr_next++;
    pr_next %= period_record.size();
    last_cb_time = ros::Time::now();

    if (period_record.size() <= PERIOD_RECORD_SIZE/2)
    {
        // wait until we have some values; make a reasonable assumption (10 Hz) meanwhile
        cb_avg_time = 0.1;
    }
    else
    {
        // enough; recalculate with the latest input
        cb_avg_time = median(period_record);
    }

    input_active = true;

    // Bound speed with the maximum values
    locker.lock();
    target_vel_.linear.x  =
        msg->linear.x  > 0.0 ? std::min(msg->linear.x, max_vel_x) : std::max(msg->linear.x, -max_vel_x);
    target_vel_.angular.z =
        msg->angular.z > 0.0 ? std::min(msg->angular.z, max_vel_theta) : std::max(msg->angular.z, -max_vel_theta);
    locker.unlock();
}

void VelocitySmoother::odometryCB(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_vel = msg->twist.twist;
}

// Main run
void VelocitySmoother::run()
{   
    double period = 1.0 / smoothing_frequency;
    ros::Rate rate(smoothing_frequency);

    while (!shutdown_req && ros::ok())
    {   
        locker.lock();
        double max_accel_x_(max_accel_x);
        double max_accel_theta_(max_accel_theta);
        double max_decel_x_(max_decel_x);
        double max_decel_theta_(max_decel_theta);
        locker.unlock();

        if ((input_active == true) && (cb_avg_time > 0.0) &&
            ((ros::Time::now() - last_cb_time).toSec() > std::min(3.0*cb_avg_time, 0.5)))
        {
            // Velocity input no active anymore; normally last command is a zero-velocity one, but reassure
            // this, just in case something went wrong with our input, or he just forgot good manners...
            // Issue #2, extra check in case cb_avg_time is very big, for example with several atomic commands
            // The cb_avg_time > 0 check is required to deal with low-rate simulated time, that can make that
            // several messages arrive with the same time and so lead to a zero median
            input_active = false;
            if (IS_ZERO_VELOCITY(target_vel_) == false)
            {
                ROS_WARN_STREAM("Velocity Smoother : input got inactive leaving us a non-zero target velocity ("
                    << target_vel_.linear.x << ", " << target_vel_.angular.z << "), zeroing...[" << name << "]");
                target_vel_ = ZERO_VEL_COMMAND;
            }
        }

        //check if the feedback is off from what we expect
        //don't care about min / max velocities here, just for rough checking
        double period_buffer = 2.0;

        double v_deviation_lower_bound = last_cmd_vel.linear.x - max_decel_x_ * period * period_buffer;
        double v_deviation_upper_bound = last_cmd_vel.linear.x + max_accel_x_ * period * period_buffer;

        double w_deviation_lower_bound = last_cmd_vel.angular.z - max_decel_theta_ * period * period_buffer;
        double angular_max_deviation = last_cmd_vel.angular.z + max_accel_theta_ * period * period_buffer;

        bool v_different_from_feedback = current_vel.linear.x < v_deviation_lower_bound || current_vel.linear.x > v_deviation_upper_bound;
        bool w_different_from_feedback = current_vel.angular.z < w_deviation_lower_bound || current_vel.angular.z > angular_max_deviation;

        if ((input_active == true) && (cb_avg_time > 0.0) &&
            (((ros::Time::now() - last_cb_time).toSec() > 5.0*cb_avg_time) ||
            v_different_from_feedback || w_different_from_feedback))
        {
            last_cmd_vel = current_vel;
        }

        geometry_msgs::TwistPtr cmd_vel;

        if ((target_vel_.linear.x  != last_cmd_vel.linear.x) ||
            (target_vel_.angular.z != last_cmd_vel.angular.z))
        {
            // Try to reach target velocity ensuring that we don't exceed the acceleration limits
            cmd_vel.reset(new geometry_msgs::Twist(target_vel_));

            double v_inc, w_inc, max_v_inc, max_w_inc;

            v_inc = target_vel_.linear.x - last_cmd_vel.linear.x;
            if (current_vel.linear.x*target_vel_.linear.x < 0.0)
            {
                // countermarch (on robots with significant inertia; requires odometry feedback to be detected)
                max_v_inc = max_decel_x_*period;
            }
            else
            {
                max_v_inc = ((v_inc*target_vel_.linear.x > 0.0)?max_accel_x:max_decel_x_)*period;
            }

            w_inc = target_vel_.angular.z - last_cmd_vel.angular.z;
            if (current_vel.angular.z*target_vel_.angular.z < 0.0)
            {
                // countermarch (on robots with significant inertia; requires odometry feedback to be detected)
                max_w_inc = max_decel_theta_*period;
            }
            else
            {
                max_w_inc = ((w_inc*target_vel_.angular.z > 0.0)?max_accel_theta_:max_decel_theta_)*period;
            }

            // Calculate and normalise vectors A (desired velocity increment) and B (maximum velocity increment),
            // where v acts as coordinate x and w as coordinate y; the sign of the angle from A to B determines
            // which velocity (v or w) must be overconstrained to keep the direction provided as command
            double MA = sqrt(    v_inc *     v_inc +     w_inc *     w_inc);
            double MB = sqrt(max_v_inc * max_v_inc + max_w_inc * max_w_inc);

            double Av = std::abs(v_inc) / MA;
            double Aw = std::abs(w_inc) / MA;
            double Bv = max_v_inc / MB;
            double Bw = max_w_inc / MB;
            double theta = atan2(Bw, Bv) - atan2(Aw, Av);

            if (theta < 0)
            {
                // overconstrain linear velocity
                max_v_inc = (max_w_inc*std::abs(v_inc))/std::abs(w_inc);
            }
            else
            {
                // overconstrain angular velocity
                max_w_inc = (max_v_inc*std::abs(w_inc))/std::abs(v_inc);
            }

            if (std::abs(v_inc) > max_v_inc)
            {
                // we must limit linear velocity
                cmd_vel->linear.x  = last_cmd_vel.linear.x  + sign(v_inc)*max_v_inc;
            }

            if (std::abs(w_inc) > max_w_inc)
            {
                // we must limit angular velocity
                cmd_vel->angular.z = last_cmd_vel.angular.z + sign(w_inc)*max_w_inc;
            }
            pub_smoothed_vel.publish(std::move(cmd_vel));
            last_cmd_vel = *cmd_vel;
        }
        else if (input_active == true)
        {
        // We already reached target velocity; just keep resending last command while input is active
        cmd_vel.reset(new geometry_msgs::Twist(last_cmd_vel));
        pub_smoothed_vel.publish(std::move(cmd_vel));
        }

        rate.sleep();
    }
}

bool VelocitySmoother::init(ros::NodeHandle& nh)
{   
    // Dynamic Reconfigure
    dynamic_srv_ = new ParamterConfigServer(ros::NodeHandle("~"));
    CallbackType cb = boost::bind(&VelocitySmoother::reconfigureCB,
                                  this, _1, _2);
    dynamic_srv_->setCallback(cb);

    // Optional parameters
    nh.param("smoothing_frequency", smoothing_frequency, 20.0);
    nh.param("decel_factor", decel_factor, 1.0);

    // Mandatory parameters
    if (!nh.getParam("max_vel_x", max_vel_x) ||
        !nh.getParam("max_vel_theta", max_vel_theta))
    {
        ROS_ERROR("Missing velocity limit parameter!");
        return false;
    }

    if (!nh.getParam("max_accel_x", max_accel_x) ||
        !nh.getParam("max_accel_theta", max_accel_theta))
    {
        ROS_ERROR("Missing acceleration limit parameter!");
        return false;
    }

    // Deceleration can be more aggressive, if necessary
    max_decel_x = decel_factor * max_accel_x;
    max_decel_theta = decel_factor * max_accel_theta;

    // Subscribers:
    sub_odometry = nh.subscribe("odom",    1, &VelocitySmoother::odometryCB, this);
    sub_curr_vel = nh.subscribe("cmd_vel", 1, &VelocitySmoother::robotVelCB, this);

    // Publishers:
    pub_smoothed_vel = nh.advertise <geometry_msgs::Twist> ("smoothed_vel", 1);

    return true;
}

// Nodelet
class VelocitySmootherNodelet : public nodelet::Nodelet
{
public:
  VelocitySmootherNodelet()  { }
  ~VelocitySmootherNodelet()
  {
    NODELET_DEBUG("Velocity Smoother : waiting for worker thread to finish...");
    vel_smoother_->shutdown();
    worker_thread_.join();
  }

  std::string unresolvedName(const std::string &name) const {
    size_t pos = name.find_last_of('/');
    return name.substr(pos + 1);
  }

  virtual void onInit()
  {
    ros::NodeHandle ph = getPrivateNodeHandle();
    std::string resolved_name = ph.getUnresolvedNamespace();
    std::string name = unresolvedName(resolved_name);
    NODELET_DEBUG_STREAM("Velocity Smoother : initialising nodelet...[" << name << "]");
    vel_smoother_.reset(new VelocitySmoother(name));
    if (vel_smoother_->init(ph))
    {
      NODELET_DEBUG_STREAM("Velocity Smoother : nodelet initialised [" << name << "]");
      worker_thread_.start(&VelocitySmoother::run, *vel_smoother_);
    }
    else
    {
      NODELET_ERROR_STREAM("Velocity Smoother : nodelet initialisation failed [" << name << "]");
    }
  }

private:
  boost::shared_ptr<VelocitySmoother> vel_smoother_;
  ecl::Thread worker_thread_;
};

} // namespace velocity_smoother

PLUGINLIB_EXPORT_CLASS(velocity_smoother::VelocitySmootherNodelet, nodelet::Nodelet);
