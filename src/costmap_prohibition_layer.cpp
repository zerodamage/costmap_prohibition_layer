/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Stephan Kurzawe
 *********************************************************************/

#include <costmap_prohibition_layer/costmap_prohibition_layer.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/costmap_math.h>

PLUGINLIB_EXPORT_CLASS(costmap_prohibition_layer_namespace::CostmapProhibitionLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::Costmap2D;

namespace costmap_prohibition_layer_namespace
{
    
CostmapProhibitionLayer::CostmapProhibitionLayer() : _dsrv(NULL)
{
}

CostmapProhibitionLayer::~CostmapProhibitionLayer()
{
    if (_dsrv!=NULL)
        delete _dsrv;
}

void CostmapProhibitionLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  current_ = true;

  if (_dsrv)
  {
    delete _dsrv;
  }

  _dsrv = new dynamic_reconfigure::Server<CostmapProhibitionLayerConfig>(nh);
  dynamic_reconfigure::Server<CostmapProhibitionLayerConfig>::CallbackType cb =
      boost::bind(&CostmapProhibitionLayer::reconfigureCB, this, _1, _2);
  _dsrv->setCallback(cb);

  global_frame_ = layered_costmap_->getGlobalFrameID();

  std::string map_topic;
  nh.param("map_topic", map_topic, std::string("map"));

  // Only resubscribe if topic has changed
  if (map_sub_.getTopic() != ros::names::resolve(map_topic))
  {
    // we'll subscribe to the latched topic that the map server uses
    ROS_INFO("Requesting the map...");
    map_sub_ = g_nh.subscribe(map_topic, 1, &CostmapProhibitionLayer::incomingMap, this);
    map_received_ = false;
    has_updated_data_ = false;

    ros::Rate r(10);
    while (!map_received_ && g_nh.ok())
    {
      ros::spinOnce();
      r.sleep();
    }

    ROS_INFO("Received a %d X %d map at %f m/pix", getSizeInCellsX(), getSizeInCellsY(), getResolution());

    if (subscribe_to_updates_)
    {
      ROS_INFO("Subscribing to updates");
      map_update_sub_ = g_nh.subscribe(map_topic + "_updates", 10, &CostmapProhibitionLayer::incomingUpdate, this);

    }
  }
  else
  {
    has_updated_data_ = true;
  }

  // get a pointer to the layered costmap and save resolution
  costmap_2d::Costmap2D *costmap = layered_costmap_->getCostmap();
  _costmap_resolution = costmap->getResolution();

  // set initial bounds
  _min_x = _min_y = _max_x = _max_y = 0;
  
  // reading the prohibition areas out of the namespace of this plugin!
  // e.g.: "move_base/global_costmap/prohibition_layer/prohibition_areas"
  std::string params = "prohibition_areas";
  if (!parseProhibitionListFromYaml(&nh, params))
    ROS_ERROR_STREAM("Reading prohibition areas from '" << nh.getNamespace() << "/" << params << "' failed!");
  
  _fill_polygons = true;
  nh.param("fill_polygons", _fill_polygons, _fill_polygons);
  
  // compute map bounds for the current set of prohibition areas.
  computeMapBounds();
  
  ROS_INFO("CostmapProhibitionLayer initialized.");
}

void CostmapProhibitionLayer::reconfigureCB(CostmapProhibitionLayerConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
  _fill_polygons = config.fill_polygons;
}


void CostmapProhibitionLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, 
                                           double *min_x, double *min_y, double *max_x, double *max_y)
{
    if (!enabled_)
        return;
    
    std::lock_guard<std::mutex> l(_data_mutex);
    
    if (_prohibition_points.empty() && _prohibition_polygons.empty())
        return;

    *min_x = std::min(*min_x, _min_x);
    *min_y = std::min(*min_y, _min_y);
    *max_x = std::max(*max_x, _max_x);
    *max_y = std::max(*max_y, _max_y);

}

void CostmapProhibitionLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  std::lock_guard<std::mutex> l(_data_mutex);
  
  // set costs of polygons
  for (int i = 0; i < _prohibition_polygons.size(); ++i)
  {
      setPolygonCost(master_grid, _prohibition_polygons[i], LETHAL_OBSTACLE, min_i, min_j, max_i, max_j, _fill_polygons);
  }
      
  // set cost of points
  for (int i = 0; i < _prohibition_points.size(); ++i)
  {
    unsigned int mx;
    unsigned int my;
    if (master_grid.worldToMap(_prohibition_points[i].x, _prohibition_points[i].y, mx, my))
    {
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
    }
  }
}

void CostmapProhibitionLayer::computeMapBounds()
{
  std::lock_guard<std::mutex> l(_data_mutex);
    
  // reset bounds
  _min_x = _min_y = _max_x = _max_y = 0;
    
  // iterate polygons
  for (int i = 0; i < _prohibition_polygons.size(); ++i)
  {
    for (int j=0; j < _prohibition_polygons.at(i).points.size(); ++j)
    {
      double px = _prohibition_polygons.at(i).points.at(j).x;
      double py = _prohibition_polygons.at(i).points.at(j).y;
      _min_x = std::min(px, _min_x);
      _min_y = std::min(py, _min_y);
      _max_x = std::max(px, _max_x);
      _max_y = std::max(py, _max_y);
    }
  }

  // iterate points
  for (int i = 0; i < _prohibition_points.size(); ++i)
  {
      double px = _prohibition_points.at(i).x;
      double py = _prohibition_points.at(i).y;
      _min_x = std::min(px, _min_x);
      _min_y = std::min(py, _min_y);
      _max_x = std::max(px, _max_x);
      _max_y = std::max(py, _max_y);
  }
}


void CostmapProhibitionLayer::setPolygonCost(costmap_2d::Costmap2D &master_grid, const geometry_msgs::Polygon& polygon, unsigned char cost,
                                             int min_i, int min_j, int max_i, int max_j, bool fill_polygon)
{
    std::vector<PointInt> map_polygon;
    for (unsigned int i = 0; i < polygon.points.size(); ++i)
    {
        PointInt loc;
        master_grid.worldToMapNoBounds(polygon.points[i].x, polygon.points[i].y, loc.x, loc.y);
        map_polygon.push_back(loc);
    }

    std::vector<PointInt> polygon_cells;

    // get the cells that fill the polygon
    rasterizePolygon(map_polygon, polygon_cells, fill_polygon);

    // set the cost of those cells
    for (unsigned int i = 0; i < polygon_cells.size(); ++i)
    {
        int mx = polygon_cells[i].x;
        int my = polygon_cells[i].y;
        // check if point is outside bounds
        if (mx < min_i || mx >= max_i)
            continue;
        if (my < min_j || my >= max_j)
            continue;
        master_grid.setCost(mx, my, cost);
    }
}


void CostmapProhibitionLayer::polygonOutlineCells(const std::vector<PointInt>& polygon, std::vector<PointInt>& polygon_cells)
  {
     for (unsigned int i = 0; i < polygon.size() - 1; ++i)
     {
       raytrace(polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y, polygon_cells);
     }
     if (!polygon.empty())
     {
       unsigned int last_index = polygon.size() - 1;
       // we also need to close the polygon by going from the last point to the first
       raytrace(polygon[last_index].x, polygon[last_index].y, polygon[0].x, polygon[0].y, polygon_cells);
     }
  }

void CostmapProhibitionLayer::raytrace(int x0, int y0, int x1, int y1, std::vector<PointInt>& cells)
{
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    PointInt pt;
    pt.x = x0;
    pt.y = y0;
    int n = 1 + dx + dy;
    int x_inc = (x1 > x0) ? 1 : -1;
    int y_inc = (y1 > y0) ? 1 : -1;
    int error = dx - dy;
    dx *= 2;
    dy *= 2;
        
    for (; n > 0; --n)
    {
        cells.push_back(pt);

        if (error > 0)
        {
            pt.x += x_inc;
            error -= dy;
        }
        else
        {
            pt.y += y_inc;
            error += dx;
        }
    }
}

void CostmapProhibitionLayer::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map)
{
  unsigned int size_x = new_map->info.width, size_y = new_map->info.height;

  ROS_DEBUG("Received a %d X %d map at %f m/pix", size_x, size_y, new_map->info.resolution);

  // resize costmap if size, resolution or origin do not match
  Costmap2D* master = layered_costmap_->getCostmap();
  if (!layered_costmap_->isRolling() && (master->getSizeInCellsX() != size_x ||
      master->getSizeInCellsY() != size_y ||
      master->getResolution() != new_map->info.resolution ||
      master->getOriginX() != new_map->info.origin.position.x ||
      master->getOriginY() != new_map->info.origin.position.y ||
      !layered_costmap_->isSizeLocked()))
  {
    // Update the size of the layered costmap (and all layers, including this one)
    ROS_INFO("Resizing costmap to %d X %d at %f m/pix", size_x, size_y, new_map->info.resolution);
    layered_costmap_->resizeMap(size_x, size_y, new_map->info.resolution, new_map->info.origin.position.x,
                                new_map->info.origin.position.y, true);
  }

  unsigned int index = 0;

  // initialize the costmap with the prohibition regions
  /*for (unsigned int i = 0; i < size_y; ++i)
  {
    for (unsigned int j = 0; j < size_x; ++j)
    {
      unsigned char value = new_map->data[index];
      costmap_[index] = interpretValue(value);
      ++index;
    }
  }*/
  map_frame_ = new_map->header.frame_id;

  // we have a new map, update full size of map
  x_ = y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  map_received_ = true;
  has_updated_data_ = true;

  // shutdown the map subscrber if firt_map_only_ flag is on
  /*if (first_map_only_)
  {
    ROS_INFO("Shutting down the map subscriber. first_map_only flag is on");
    map_sub_.shutdown();
  }*/
}


void CostmapProhibitionLayer::rasterizePolygon(const std::vector<PointInt>& polygon, std::vector<PointInt>& polygon_cells, bool fill)
{
    // this implementation is a slighly modified version of Costmap2D::convexFillCells(...)

    //we need a minimum polygon of a traingle
    if(polygon.size() < 3)
        return;

    //first get the cells that make up the outline of the polygon
    polygonOutlineCells(polygon, polygon_cells);

    if (!fill)
        return;

    //quick bubble sort to sort points by x
    PointInt swap;
    unsigned int i = 0;
    while(i < polygon_cells.size() - 1)
    {
        if(polygon_cells[i].x > polygon_cells[i + 1].x)
        {
            swap = polygon_cells[i];
            polygon_cells[i] = polygon_cells[i + 1];
            polygon_cells[i + 1] = swap;

            if(i > 0)
            --i;
        }
        else
            ++i;
        }

        i = 0;
        PointInt min_pt;
        PointInt max_pt;
        int min_x = polygon_cells[0].x;
        int max_x = polygon_cells[(int)polygon_cells.size() -1].x;

        //walk through each column and mark cells inside the polygon
        for(int x = min_x; x <= max_x; ++x)
        {
            if(i >= (int)polygon_cells.size() - 1)
                break;

            if(polygon_cells[i].y < polygon_cells[i + 1].y)
            {
                min_pt = polygon_cells[i];
                max_pt = polygon_cells[i + 1];
            }
            else
            {
                min_pt = polygon_cells[i + 1];
                max_pt = polygon_cells[i];
            }

            i += 2;
            while(i < polygon_cells.size() && polygon_cells[i].x == x)
            {
                if(polygon_cells[i].y < min_pt.y)
                    min_pt = polygon_cells[i];
                else if(polygon_cells[i].y > max_pt.y)
                    max_pt = polygon_cells[i];
                ++i;
            }

            PointInt pt;
            //loop though cells in the column
            for(int y = min_pt.y; y < max_pt.y; ++y)
            {
                pt.x = x;
                pt.y = y;
                polygon_cells.push_back(pt);
            }
        }
  }

// load prohibition positions out of the rosparam server
bool CostmapProhibitionLayer::parseProhibitionListFromYaml(ros::NodeHandle *nhandle, const std::string &param)
{
  std::lock_guard<std::mutex> l(_data_mutex);
  std::unordered_map<std::string, geometry_msgs::Pose> map_out;

  XmlRpc::XmlRpcValue param_yaml;

  bool ret_val = true;

  if (nhandle->getParam(param, param_yaml))
  {
    if (param_yaml.getType() == XmlRpc::XmlRpcValue::TypeArray)  // list of goals
    {
      for (int i = 0; i < param_yaml.size(); ++i)
      {
        if (param_yaml[i].getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
          geometry_msgs::Polygon polygon_to_add;

          /* **************************************
           * differ between points and polygons
           * lines get to a polygon with the resolution
           * of the costmap
           **************************************** */

          // add a point
          if (param_yaml[i].size() == 1)
          {
            geometry_msgs::Point point;
            ret_val = getPoint(param_yaml[i][0], point);
            _prohibition_points.push_back(point);
          }
          // add a line
          else if (param_yaml[i].size() == 2)
          {
            if (param_yaml[i][0].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
	      param_yaml[i][0].getType() == XmlRpc::XmlRpcValue::TypeInt)
            {
              // add a lonely point
              geometry_msgs::Point point;
              ret_val = getPoint(param_yaml[i], point);
              _prohibition_points.push_back(point);
            }
            else
            {
              // add a line!
              geometry_msgs::Point32 point_A;
              ret_val = getPoint(param_yaml[i][0], point_A);
              polygon_to_add.points.push_back(point_A);

              geometry_msgs::Point32 point_B;
              ret_val = getPoint(param_yaml[i][1], point_B);
              polygon_to_add.points.push_back(point_B);

              // calculate the normal vector for AB
              geometry_msgs::Point point_N;
              point_N.x = point_B.y - point_A.y;
              point_N.y = point_A.x - point_B.x;

              // get the absolute value of N to normalize and get
              // it to the length of the costmap resolution
              double abs_N = sqrt(pow(point_N.x, 2) + pow(point_N.y, 2));
              point_N.x = point_N.x / abs_N * _costmap_resolution;
              point_N.y = point_N.y / abs_N * _costmap_resolution;

              // calculate the new points to get a polygon which can be filled
              geometry_msgs::Point32 point;
              point.x = point_A.x + point_N.x;
              point.y = point_A.y + point_N.y;
              polygon_to_add.points.push_back(point);

              point.x = point_B.x + point_N.x;
              point.y = point_B.y + point_N.y;
              polygon_to_add.points.push_back(point);

              _prohibition_polygons.push_back(polygon_to_add);
            }
          }
          // add a point or add a polygon
          else if (param_yaml[i].size() >= 3)
          {
            // add a polygon with any number of points
            for (int j = 0; j < param_yaml[i].size(); ++j)
            {
              geometry_msgs::Point32 point;
              ret_val = getPoint(param_yaml[i][j], point);
              polygon_to_add.points.push_back(point);
            }
            _prohibition_polygons.push_back(polygon_to_add);
          }
        }
        else
        {
          ROS_ERROR_STREAM("Prohibition Layer:" << param << " with index " << i << " is not correct.");
          ret_val = false;
        }
      }
    }
    else
    {
      ROS_ERROR_STREAM("Prohibition Layer: " << param << "struct is not correct.");
      ret_val = false;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Prohibition Layer: Cannot read " << param << " from parameter server");
    ret_val = false;
  }
  return ret_val;
}

// get a point out of the XML Type into a geometry_msgs::Point
bool CostmapProhibitionLayer::getPoint(XmlRpc::XmlRpcValue &val, geometry_msgs::Point &point)
{
  try
  {
    // check if there a two values for the coordinate
    if (val.getType() == XmlRpc::XmlRpcValue::TypeArray && val.size() == 2)
    {
      auto convDouble = [](XmlRpc::XmlRpcValue &val) -> double
      {
        if (val.getType() == XmlRpc::XmlRpcValue::TypeInt)  // XmlRpc cannot cast int to double
          return int(val);
        return val;  // if not double, an exception is thrown;
      };

      point.x = convDouble(val[0]);
      point.y = convDouble(val[1]);
      point.z = 0.0;
      return true;
    }
    else
    {
      ROS_ERROR_STREAM("Prohibition_Layer: A point has to consist two values!");
      return false;
    }
  }
  catch (const XmlRpc::XmlRpcException &ex)
  {
    ROS_ERROR_STREAM("Prohibition Layer: Cannot add current point: " << ex.getMessage());
    return false;
  }
}

bool CostmapProhibitionLayer::getPoint(XmlRpc::XmlRpcValue &val, geometry_msgs::Point32 &point)
{
  try
  {
    // check if there a two values for the coordinate
    if (val.getType() == XmlRpc::XmlRpcValue::TypeArray && val.size() == 2)
    {
      auto convDouble = [](XmlRpc::XmlRpcValue &val) -> double
      {
        if (val.getType() == XmlRpc::XmlRpcValue::TypeInt)  // XmlRpc cannot cast int to double
          return int(val);
        return val;  // if not double, an exception is thrown;
      };

      point.x = convDouble(val[0]);
      point.y = convDouble(val[1]);
      point.z = 0.0;
      return true;
    }
    else
    {
      ROS_ERROR_STREAM("Prohibition_Layer: A point has to consist two values!");
      return false;
    }
  }
  catch (const XmlRpc::XmlRpcException &ex)
  {
    ROS_ERROR_STREAM("Prohibition Layer: Cannot add current point: " << ex.getMessage());
    return false;
  }
}

}  // end namespace
