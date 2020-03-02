#include <local_planner/local_planner.h>

void Spline::calculate(geometry_msgs::Pose2D pi,geometry_msgs::Pose2D dpi,geometry_msgs::Pose2D pf,geometry_msgs::Pose2D dpf){
  a[0] = pi.x;
  a[1] = dpi.x;
  a[2] = -3/std::pow(TF,2)*pi.x-2*dpi.x/TF+3*pf.x/std::pow(TF,2)-dpf.x/TF;
  a[3] = 2/std::pow(TF,3)*pi.x+dpi.x/std::pow(TF,2)-2*pf.x/std::pow(TF,3)+dpf.x/std::pow(TF,2);

  b[0] = pi.y;
  b[1] = dpi.y;
  b[2] = -3/std::pow(TF,2)*pi.y-2*dpi.y/TF+3*pf.y/std::pow(TF,2)-dpf.y/TF;
  b[3] = 2/std::pow(TF,3)*pi.y+dpi.y/std::pow(TF,2)-2*pf.y/std::pow(TF,3)+dpf.y/std::pow(TF,2);
}

geometry_msgs::Pose2D Spline::get_pos(double t){
  geometry_msgs::Pose2D res;
  res.x = a[0]+a[1]*t+a[2]*std::pow(t,2)+a[3]*std::pow(t,3);
  res.y = b[0]+b[1]*t+b[2]*std::pow(t,2)+b[3]*std::pow(t,3);
  return res;
}

geometry_msgs::Pose2D Spline::get_derivative(double t){
  geometry_msgs::Pose2D res;
  res.x = a[1]+2*a[2]*t+3*a[3]*pow(t,2);
  res.y = b[1]+2*b[2]*t+3*b[3]*pow(t,2);
  return res;
}
