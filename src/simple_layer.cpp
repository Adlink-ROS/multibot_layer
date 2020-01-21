#include<simple_layers/simple_layer.h>
#include <pluginlib/class_list_macros.h>
 
PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayer, costmap_2d::Layer)
 
using costmap_2d::LETHAL_OBSTACLE;
 
namespace simple_layer_namespace
{


SimpleLayer::SimpleLayer() {}
 
void SimpleLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &SimpleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}
 
 
void SimpleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}
 
void SimpleLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;
  
  nh.setParam("pose_x", origin_x);
  nh.setParam("pose_y", origin_y);

  for (int i = 0; i < 10; i++) {
    std::string robot_pose = "/tb3_";
    std::string robot_x;
    std::string robot_y;
    std::string this_bot;
    nh.searchParam("pose_x", this_bot);
    // ROS_INFO("I am %s", this_bot.c_str());

    robot_pose.append(std::to_string(i));
    robot_x = robot_y = robot_pose;
    robot_x.append("/pose_x");
    robot_y.append("/pose_y");
    
    if (nh.searchParam(robot_x, robot_x) && robot_x != this_bot) {
      nh.getParam(robot_x, mark[i][0]);
      nh.getParam(robot_y, mark[i][1]);
      ROS_INFO("I got %s at ( %f , %f )", robot_x.c_str(), mark[i][0], mark[i][1]);
    } else {
      mark[i][0] = 100;
      mark[i][1] = 100;
    } 

    *min_x = std::min(*min_x, mark[i][0]);
    *min_y = std::min(*min_y, mark[i][1]);
    *max_x = std::max(*max_x, mark[i][0]);
    *max_y = std::max(*max_y, mark[i][1]);
  }
}
 
void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;
  unsigned int mx;
  unsigned int my;
  for (int i = 0; i < 10; i++) {
    if(master_grid.worldToMap(mark[i][0], mark[i][1], mx, my)){
	    for (int x = 0; x < 6; x++) {
		    for (int y = 0; y < 6; y++) {
			    if ((mx - 6 > 0) && (my - 6 > 0)) {
     				master_grid.setCost(mx-3+x, my-3+y, LETHAL_OBSTACLE);
          }
	      }
      } 
    }
  }
} 

} // end namespace
