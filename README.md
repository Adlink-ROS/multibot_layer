## Multibot_layers
A costmap layer allows robot to communicate and share location to each others. Through the ROS parameter server, robots publish their pose to the corresponding rosparam under its own name prefix, for example, /omni_1/pose_x and /omni_1/pose_y, when the costmap updates.

## Implementation
Before diving into this repo, read this [wonderful tutorial](http://wiki.ros.org/costmap_2d/Tutorials/Creating%20a%20New%20Layer). The differences between the multibot_layers and the simple_layers represented in the ROS Wiki tutorial are changes in updateBounds() and updateCosts() functions.
* ***updateBounds()*** does not change the costmap. It just defines the area that will need to be updated. It first publish its own pose to the parameter server under the specific prefix. And then We calculate the point we want to change in mark[][] by checking the poses of other robots on parameter server. Finally, we expand the min/max bounds to be sure it includes the new point. 
* ***updateCosts()*** calculates which grid cell our point is in using worldToMap. Then we set the cost of that cell.

### pseudocode
```
# in updateBounds

set param -- /omni_num/pose_x -- to its pose_x relative to "map" frame
set param -- /omni_num/pose_x -- to its pose_x relative to "map" frame
find its own prefix by "searchParam"
for i (0 to 4) do 
    save the poses of omni_i, except itself, relative to "map" frame map[i]
end for
```
p s. [searchParam](http://wiki.ros.org/roscpp/Overview/Parameter%20Server) helps you to get a parameter from the closest namespace.  
```
# in updateCosts

for i (0 to 4) do
    set Lethal_obstacle to the poses occupied by other robots.
end for
```

## Usage

### Clone the pkg
```
cd ~/catkin_ws/src
git clone https://github.com/airuchen/multibot_layer.git 
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```
### Add the layer into the costmap yaml file
```
plugins:
- {name: multibot_layer,     type: "multibot_layer_namespace::MultibotLayer"}

```

## Try it yourself
Follow the tutorial [here](https://github.com/airuchen/turtlebot3) to try out this plugin with multiple turtlebot3.