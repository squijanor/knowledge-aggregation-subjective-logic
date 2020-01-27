#ifndef GRID_LAYER_H_
#define GRID_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/OccupancyGrid.h>

namespace proxy_layer_namespace
{

class GridLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  GridLayer();
  
  std::vector<signed char> costmap_layer_;
  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  bool isDiscretized()
  {
    return true;
  }

  virtual void matchSize();

private:
  ros::Subscriber grid_sub;
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  void gridCallback(const nav_msgs::OccupancyGrid &msg);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
};
}
#endif
