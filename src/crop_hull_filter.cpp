#include <crop_hull_filter/crop_hull_filter.h>

#include <pcl/surface/convex_hull.h>

namespace pcl_ros
{
    CropHullFilter::CropHullFilter() : PCLNodelet(), listener_(buffer_)
    {

    }

    void CropHullFilter::onInit()
    {
        PCLNodelet::onInit();
        hull_cloud_ = boost::shared_ptr<PointCloud>(new PointCloud());
        hull_points_ = boost::shared_ptr<PointCloud>(new PointCloud());
        pcl::ConvexHull<pcl::PointXYZ> convex_hull;
        hull_cloud_->clear();
        pnh_->getParam("",parameters_);
        XmlRpc::XmlRpcValue points_param = parameters_["points"];
        pnh_->param<std::string>("frame_id", frame_id_, "map");
        pnh_->param<std::string>("output_frame_id", output_frame_id_, frame_id_);
        pnh_->param<bool>("crop_outside", crop_outside_, false);
        pnh_->param<double>("marker_line_width", marker_line_width_, 1.0);
        pnh_->param<double>("marker_color_r", marker_color_r_, 1.0);
        pnh_->param<double>("marker_color_g", marker_color_g_, 1.0);
        pnh_->param<double>("marker_color_b", marker_color_b_, 1.0);
        pnh_->param<double>("marker_color_a", marker_color_a_, 1.0);
        marker_.type = marker_.LINE_STRIP;
        marker_.action = marker_.ADD;
        marker_.header.frame_id = frame_id_;
        marker_.ns = "marker";
        marker_.id = 0;
        boost::optional<geometry_msgs::Point> first_point;
        std_msgs::ColorRGBA color;
        color.r = marker_color_r_;
        color.g = marker_color_g_;
        color.b = marker_color_b_;
        color.a = marker_color_a_;
        for(auto itr = points_param.begin(); itr != points_param.end(); itr++)
        {
            double x = itr->second["x"];
            double y = itr->second["y"];
            pcl::PointXYZ p;
            p.x = x;
            p.y = y;
            hull_cloud_->push_back(p);
            geometry_msgs::Point point;
            point.x = x;
            point.y = y;
            point.z = 0;
            if(!first_point)
            {
                first_point = point;
            }
            marker_.points.push_back(point);
            marker_.colors.push_back(color);
        }
        marker_.scale.x = marker_line_width_;
        marker_.points.push_back(*first_point);
        marker_.colors.push_back(color);
        marker_.frame_locked = true;
        convex_hull.setInputCloud(hull_cloud_);
        convex_hull.reconstruct(*hull_points_,convex_hull_polygons_);
        filter_.setHullIndices(convex_hull_polygons_);
        filter_.setHullCloud(hull_points_);
        filter_.setDim(2);
        filter_.setCropOutside(crop_outside_);
        subscribe();
        onInitPostProcess();
    }

    void CropHullFilter::subscribe()
    {
        marker_pub_ = pnh_->advertise<visualization_msgs::Marker>("marker",1);
        pub_output_ = advertise<PointCloud>(*pnh_, "output", 10);
        pointcloud_sub_ = pnh_->subscribe("input", 10, &CropHullFilter::pointCloudCallback, this);
    }

    void CropHullFilter::unsubscribe()
    {
        //pointcloud_sub_.shutdown();
    }

    void CropHullFilter::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr cloud)
    {
        try
        {
            geometry_msgs::TransformStamped transform_stamped 
                = buffer_.lookupTransform(frame_id_, cloud->header.frame_id, ros::Time(0), ros::Duration(2.0));
            geometry_msgs::TransformStamped transform_stamped_inv
                = buffer_.lookupTransform(output_frame_id_, frame_id_, ros::Time(0), ros::Duration(2.0));
            Eigen::Matrix4f mat = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
            Eigen::Matrix4f mat_inv = tf2::transformToEigen(transform_stamped_inv.transform).matrix().cast<float>();
            sensor_msgs::PointCloud2 cloud_transformed;
            pcl_ros::transformPointCloud(mat, *cloud, cloud_transformed);
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(cloud_transformed, *pcl_cloud);
            filter_.setInputCloud(pcl_cloud);
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
            filter_.filter(*filtered);
            sensor_msgs::PointCloud2 cloud_filtered,cloud_output;
            pcl::toROSMsg(*filtered, cloud_filtered);
            pcl_ros::transformPointCloud(mat_inv, cloud_filtered, cloud_output);
            pub_output_.publish(cloud_output);
            marker_pub_.publish(marker_);
        }
        catch(...)
        {
            return;
        }
        return;
    }
}

#include <pluginlib/class_list_macros.h>
typedef pcl_ros::CropHullFilter CropHullFilter;
PLUGINLIB_EXPORT_CLASS(CropHullFilter,nodelet::Nodelet);