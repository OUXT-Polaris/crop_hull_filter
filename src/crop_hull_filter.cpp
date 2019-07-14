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
        //double z;
        //pnh_->param<double>("z", z, 0.0);
        //double crop_height;
        //pnh_->param<double>("crop_height", crop_height, 1.0);
        pnh_->param<std::string>("frame_id", frame_id_, "map");
        pnh_->param<bool>("crop_outside", crop_outside_, false);
        for(auto itr = points_param.begin(); itr != points_param.end(); itr++)
        {
            double x = itr->second["x"];
            double y = itr->second["y"];
            pcl::PointXYZ p;
            p.x = x;
            p.y = y;
            //p.z = z + crop_height*0.5;
            hull_cloud_->push_back(p);
            //p.z = z - crop_height*0.5;
            //hull_cloud_->push_back(p);
        }
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
        pub_output_ = advertise<PointCloud>(*pnh_, "output", 10);
        pointcloud_sub_ = pnh_->subscribe("input", 10, &CropHullFilter::pointCloudCallback, this);
    }

    void CropHullFilter::unsubscribe()
    {
        pointcloud_sub_.shutdown();
    }

    void CropHullFilter::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr cloud)
    {
        geometry_msgs::TransformStamped transform_stamped 
            = buffer_.lookupTransform(cloud->header.frame_id, frame_id_, ros::Time(0), ros::Duration(2.0));
        Eigen::Matrix4f mat = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
        sensor_msgs::PointCloud2 cloud_transformed;
        pcl_ros::transformPointCloud(mat, *cloud, cloud_transformed);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(cloud_transformed, *pcl_cloud);
        filter_.setInputCloud(pcl_cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
        filter_.filter(*filtered);
        pub_output_.publish(filtered);
        return;
    }
}

#include <pluginlib/class_list_macros.h>
typedef pcl_ros::CropHullFilter CropHullFilter;
PLUGINLIB_EXPORT_CLASS(CropHullFilter,nodelet::Nodelet);