#ifndef CROP_HULL_FILTER_CROP_HULL_FILTER_H_INCLUDED
#define CROP_HULL_FILTER_CROP_HULL_FILTER_H_INCLUDED

// Headers in ROS
#include <pcl/filters/crop_hull.h>
#include <pcl_ros/filters/filter.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/pcl_nodelet.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

// Headers in Boost
#include <boost/optional.hpp>

namespace pcl_ros
{
    class CropHullFilter : public PCLNodelet
    {
    public:
        CropHullFilter();
    protected:
        void onInit();
        void subscribe();
        void unsubscribe();
    private:
        pcl::CropHull<pcl::PointXYZ> filter_;
        boost::shared_ptr<PointCloud> hull_cloud_;
        boost::shared_ptr<PointCloud> hull_points_;
        std::vector<pcl::Vertices> convex_hull_polygons_;
        XmlRpc::XmlRpcValue parameters_;
        std::string frame_id_;
        bool crop_outside_;
        double marker_line_width_;
        double marker_color_a_;
        double marker_color_r_;
        double marker_color_g_;
        double marker_color_b_;
        std::string output_frame_id_;
        ros::Subscriber pointcloud_sub_;
        ros::Publisher marker_pub_;
        void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr cloud);
        tf2_ros::Buffer buffer_;
        tf2_ros::TransformListener listener_;
        visualization_msgs::Marker marker_;
    };
}

#endif  //CROP_HULL_FILTER_CROP_HULL_FILTER_H_INCLUDED