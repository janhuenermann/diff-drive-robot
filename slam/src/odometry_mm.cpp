#include <slam/odometry_mm.h>
#include <math.h>

double normalize_angle(double theta)
{
    if (theta > M_PI) return theta - 2 * M_PI;
    else if (theta < -M_PI) return theta + 2 * M_PI;
    else return theta;
}

OdometryMM::OdometryMM(geometry_msgs::Pose2D pos_init, double wl_init, double wr_init) :
    pos(pos_init),
    wl(wl_init),
    wr(wr_init),
    v(0),
    a(0),
    w(0),
    t(ros::Time(0))
{
    ros::NodeHandle nh;
    wheel_sub = nh.subscribe("joint_states",1,&OdometryMM::callback_wheels,this);
    imu_sub = nh.subscribe("imu",1,&OdometryMM::callback_imu,this);
    pos_pub = nh.advertise<geometry_msgs::Pose2D>("robot_pose", 5);
    vel_pub = nh.advertise<geometry_msgs::Twist>("robot_velocity", 5);
}

void OdometryMM::callback_imu(const sensor_msgs::Imu& msg)
{
    compass = tf::getYaw(msg.orientation);  //extract info from quaterion
    w = msg.angular_velocity.z;
    a = msg.linear_acceleration.x;
}

void OdometryMM::callback_wheels(const sensor_msgs::JointState& msg)
{
    if (t == ros::Time(0))
    {
        t = ros::Time::now();
        wl = msg.position[1];
        wr = msg.position[0];
        
        return ;
    }

    double dt = (ros::Time::now() - t).toSec();
    t = ros::Time::now();

    profiler.start();

    double dwl = msg.position[1] - wl,
           dwr = msg.position[0] - wr;

    double x_tm1 = pos.x,
           y_tm1 = pos.y,
           theta_tm1 = pos.theta;

    double w_t = WR / (L * dt) * (dwr - dwl), // angular velocity
           v_t = WR / (2 * dt) * (dwl + dwr); // forward velocity

    double dtheta = WR / L * (dwr - dwl); // w_t * dt;

    double x_t, y_t, theta_t;

    if (std::abs(w_t) > 1e-8)
    {
        double r_t = L * (dwr + dwl) / (2 * (dwr - dwl));
        
        // exact integration
        x_t = x_tm1 + r_t * (std::sin(theta_tm1 + dtheta) - std::sin(theta_tm1));
        y_t = y_tm1 - r_t * (std::cos(theta_tm1 + dtheta) - std::cos(theta_tm1));
    }
    else
    {
        // runge kutta integration
        x_t = x_tm1 + v_t * dt * std::cos(theta_tm1 + 0.5 * dtheta);
        y_t = y_tm1 + v_t * dt * std::sin(theta_tm1 + 0.5 * dtheta);
    }

    theta_t = theta_tm1 + dtheta;

    pos.x = x_t;
    pos.y = y_t;
    pos.theta = compass; // theta_t;

    publish_pos();
    publish_vel(w_t, v_t);

    wl = msg.position[1];
    wr = msg.position[0];
    
    // double omega = (2 * WR / (2 * L * dt)) * (dwr - dwl);
    // double theta_od = pos.theta;
    // double w_av;

    // if(std::abs(omega) < 1e-1)
    // {
    //     double v_od = ((2 * WR) / (4 * dt)) * (dwl + dwr);
    //     double v_imu = v + a * dt;
    //     v = WEIGHT_ODOMETRY * v_od + (1-WEIGHT_ODOMETRY) * v_imu;
    //     pos.x += v * dt * cos(pos.theta);
    //     pos.y += v * dt * sin(pos.theta);
    //     w_av = 0;
    // }
    // else
    // {
    //     double rt = (L / 2) * (dwl + dwr) / (dwr - dwl);    //turn radius
    //     double dphi_od = ((2 * WR) / (2 * L)) * (dwr - dwl);
    //     double dphi_imu = w * dt;
    //     double dphi_av = WEIGHT_ODOMETRY * dphi_od + (1-WEIGHT_ODOMETRY) * dphi_imu;
    //     pos.x += -rt * sin(pos.theta) + rt * sin(pos.theta + dphi_av);
    //     pos.y += rt * cos(pos.theta) -   rt * cos(pos.theta + dphi_av);
    //     theta_od += dphi_od;
    //     v = dphi_av*rt/dt;
    //     w_av = dphi_av/dt;
    // }

    // wl = wheel_l;
    // wr = wheel_r;

    // theta_od = normalize_angle(theta_od);
    // compass = normalize_angle(compass);

    // pos.theta = (1 - WEIGHT_COMPASS) * theta_od + WEIGHT_COMPASS * compass;

    // // publish
    // publish_pos();
    // publish_vel(w_av,v);

    profiler.stop();
    profiler.print("odometry_mm");
}

void OdometryMM::publish_pos()
{
    pos_pub.publish(pos);
}

void OdometryMM::publish_vel(double wz, double v){
    geometry_msgs::Twist vel;
    vel.linear.x = v;
    vel.angular.z = wz;
    vel_pub.publish(vel);
}
