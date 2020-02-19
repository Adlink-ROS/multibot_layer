#include<multibot_layers/multibot_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(multibot_layer_namespace::MultibotLayer, costmap_2d::Layer)

	using costmap_2d::LETHAL_OBSTACLE;

	namespace multibot_layer_namespace
{
	MultibotLayer::MultibotLayer() {}

	void MultibotLayer::onInitialize()
	{
		ros::NodeHandle nh("~/" + name_);
		current_ = true;

		dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
		dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
				&MultibotLayer::reconfigureCB, this, _1, _2);
		dsrv_->setCallback(cb);
	}


	void MultibotLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
	{
		enabled_ = config.enabled;
	}

	void MultibotLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,
			double* min_y, double* max_x, double* max_y)
	{
		if (!enabled_)
			return;


		nh.setParam("pose_x", origin_x);
		nh.setParam("pose_y", origin_y);

		// dummy search for five robots. TODO: set dynamic robot array
		for (int i = 0; i < 5; i++) {
			std::string robot_pose = "/nb2_";
			std::string robot_x;
			std::string robot_y;
			std::string this_bot;
			// search for the full posename of the current robot. Eg, this_bot = "nb2_<current_robot_number>/pose_x".
			nh.searchParam("pose_x", this_bot);  


			robot_pose.append(std::to_string(i));  // set robot_pose to "/nb2_<i=1~5>"
			robot_x = robot_y = robot_pose;  // give robot_x and robot_y the same prefix
			robot_x.append("/pose_x");  // set robot_x to "/nb2_<i>/pose_x"
			robot_y.append("/pose_y");  // set robot_y to "/nb2_<i>/pose_y"

			// Save poses of every robot, except itself, in mark[][] array
			if (nh.searchParam(robot_x, robot_x) && robot_x != this_bot) {
				nh.getParam(robot_x, mark[i][0]);
				nh.getParam(robot_y, mark[i][1]);
			} else {
				mark[i][0] = 100;  // Set an unaffected pose for unused robots
				mark[i][1] = 100;  // Set an unaffected pose for unused robots
			} 

			*min_x = std::min(*min_x, mark[i][0]);
			*min_y = std::min(*min_y, mark[i][1]);
			*max_x = std::max(*max_x, mark[i][0]);
			*max_y = std::max(*max_y, mark[i][1]);
		}
	}

	void MultibotLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
			int max_j)
	{
		if (!enabled_)
			return;
		unsigned int mx;
		unsigned int my;
		for (int i = 0; i < 5; i++) {
			if(master_grid.worldToMap(mark[i][0], mark[i][1], mx, my)){
				master_grid.setCost(mx, my, LETHAL_OBSTACLE);
			}
		}
	} 

} // end namespace
