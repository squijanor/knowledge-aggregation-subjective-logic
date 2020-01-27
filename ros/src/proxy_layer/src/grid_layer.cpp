#include <grid_layer.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

PLUGINLIB_EXPORT_CLASS(proxy_layer_namespace::GridLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using namespace std;
namespace proxy_layer_namespace
{

unsigned int costn = 10;
unsigned int countit = 0;

void GridLayer::onInitialize()
{

  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = NO_INFORMATION;
  costmap_layer_ = vector<signed char>(203 * 203, 0);
  matchSize();

  grid_sub = nh.subscribe("/custom_layers/combined", 100, &GridLayer::gridCallback, this);

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &GridLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

GridLayer::GridLayer()
{
}

void GridLayer::gridCallback(const nav_msgs::OccupancyGrid &msg)
{ // TODO update origin etc.
  //ROSINFO("Got costmap update!");
  costmap_layer_ = msg.data;
}

void GridLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void GridLayer::matchSize()
{
  Costmap2D *master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}

void GridLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x,
                             double *min_y, double *max_x, double *max_y)
{
  if (!enabled_)
    return;

            //we know what size we want to update: TODO make dynamic
            *min_x = min(-5.1,*min_x);
            *min_y = min(-5.1,*min_y);
            *max_x = max(5.1,*max_x);
            *max_y = max(5.1,*max_y);

} // namespace simple_layer_namespace

void GridLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i,
                            int max_j)
{
  if (!enabled_)
    return;
  uint i_0, j_0;
  worldToMap(-5.1, -5.1, i_0, j_0);

  uint i_N = i_0 + 204;
  uint j_N = j_0 + 204;

  //cout << " min_j: " + std::to_string(min_j) + " max_j: " + std::to_string(max_j);
  for (int j = j_0; j < j_N; j++)
  {
    for (int i = i_0; i < i_N; i++)
    {
      int index= 51*((j-j_0)/4) + (i-i_0)/4;
      int c = master_grid.getCost(i, j);
      if ((c > 90)){
        int new_c = c;
        if (c<140){
        new_c =100;}
        else if(c<252){
            new_c +=20;


           master_grid.setCost(i, j, min(252,new_c));
        }
              continue;
      }

      master_grid.setCost(i, j, costmap_layer_[index]);
    }
  }
}

} // namespace simple_layer_namespace