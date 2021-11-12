

/* Todo: 1. Add ros service to kill the process by calling the destructor 
*        2. Create Destructor
*/

/* System Header Files*/
#include <iostream>
#include <fstream>
#include <string>
#include <iostream>
#include <sys/types.h>

/* 3rd Party*/
#ifdef OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#endif
#include <boost/asio.hpp>

/* ROS */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/image_encodings.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/ChannelFloat32.h"
#include "std_srvs/Trigger.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_broadcaster.h"
#include "cv_bridge/cv_bridge.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

/* Oculii SDK Files */
#include "radar_system.h"

#define SAVE_RESULT_TO_CSV 0
#define PUBLISH_RATE_RADAR 100
#define PUBLISH_RATE_IMG 100
#define PUBLISH_RATE_AXIS 40
#define MAX_X 800
#define MAX_Y 800
#define RES 50
#define CAMERA_FLAG false
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, nav_msgs::Odometry> syncPolicy;

class SdkDriver
{
public:
    /*! Constructor to Initialize the SDK Driver and ROS variables. The Radar is started in this function.
    */
    SdkDriver(std::string configPath)//: sync_(imuSub_, odomSub_, 10)
    {
        /* Set Configuration path for radar system*/
        configPath_ = configPath;

        /* Create the radar System Object. It takes config path as input to configure the radar object*/
        radarSystem_ = oculii::RadarSystem::GetRadarSystemInstance(configPath_);

        /* Start Radar System */
        oculii::RadarErrorCode status = radarSystem_->StartSystem();

        /* Check if radar system start was successful */
        if (status == oculii::RadarErrorCode::SUCCESS)
            std::cout << "Start Radar System success! " << std::endl;
        else
        {
            std::cout << "Start Radar System fail with error " << oculii::ErrorToString(status) << std::endl;
            return;
        }

        /* Start Radar Data Recieve */
        radarSystem_->StartRadarReceive();

        /* PCL Publisher */
        detPub_ = n_.advertise<sensor_msgs::PointCloud>("radar_pcl", 1000);

        /* Tracker Publisher */
        trkPub_ = n_.advertise<sensor_msgs::PointCloud>("radar_trk", 1000);

        /* Enhanced Publisher */
        enhancedDetPub_ = n_.advertise<sensor_msgs::PointCloud>("radar_enhanced_pcl", 1000);

        /* Initialize the ros publisher for image */
        imgPub_ = n_.advertise<sensor_msgs::Image>("camera_image", 10);

        /* Initialize the ros publisher for axis */
        markerPub_ = n_.advertise<visualization_msgs::MarkerArray>("axis", 10);
        
        std::cout << "Init Completed" << std::endl;
    
    }
    /*! Function To initialize Marker to display text
    * @param marker, pointer of visualization_msgs::Marker type 
    */
    void MarkerInit(visualization_msgs::Marker *marker)
    {
        /* The Marker frame is linked to base_link tf created */
        marker->header.frame_id = "base_link";

        /* ROS time to add current time to the header */
        marker->header.stamp = ros::Time::now();

        marker->action = visualization_msgs::Marker::ADD;

        /* Orientation Set to 0 rpy */
        marker->pose.orientation.w = 1.0;

        /* Set Default Marker Id */
        marker->id = 0;

        /* Marker Type Text */
        marker->type = visualization_msgs::Marker::TEXT_VIEW_FACING;

        /* Set Size of Text */
        marker->scale.z = 10;

        /* White Color Text */
        marker->color.b = 1;
        marker->color.g = 1;
        marker->color.r = 1;

        /* Transperancy */
        marker->color.a = 1;
    }

    /*! Function To Publish Axis 
    */
    void PubAxis()
    {

        /* Set Publish Rate */
        int publishRate = PUBLISH_RATE_AXIS;
        ros::Rate loop_rate(publishRate);

        visualization_msgs::MarkerArray markerArray;

        /* x positive axis */
        for (int i = 0; i < (MAX_X / (2 * RES)); i++)
        {
            visualization_msgs::Marker marker;
            marker.ns = "basic_shapes" + std::to_string(i) + "xpos";
            MarkerInit(&marker);

            geometry_msgs::Pose pose;
            pose.position.x = -10;
            pose.position.y = -i * RES;
            pose.position.z = 0;

            marker.text = std::to_string(i * RES);
            marker.pose = pose;
            markerArray.markers.push_back(marker);
        }

        /* x negative axis */
        for (int i = 0; i < (MAX_X / (2 * RES)); i++)
        {
            visualization_msgs::Marker marker;
            marker.ns = "basic_shapes" + std::to_string(i) + "xneg";
            MarkerInit(&marker);

            geometry_msgs::Pose pose;
            pose.position.x = -10;
            pose.position.y = i * RES;
            pose.position.z = 0;

            marker.text = std::to_string(-i * RES);
            marker.pose = pose;
            markerArray.markers.push_back(marker);
        }

        /* y positive axis */
        for (int i = 1; i < (MAX_X / (2 * RES)); i++)
        {
            visualization_msgs::Marker marker;
            marker.ns = "basic_shapes" + std::to_string(i) + "ypos";
            MarkerInit(&marker);

            geometry_msgs::Pose pose;
            pose.position.x = -i * RES;
            pose.position.y = -10;
            pose.position.z = 0;

            marker.text = std::to_string(i * RES);
            marker.pose = pose;
            markerArray.markers.push_back(marker);
        }

        /* y negative axis */
        for (int i = 1; i < (MAX_X / (2 * RES)); i++)
        {
            visualization_msgs::Marker marker;
            marker.ns = "basic_shapes" + std::to_string(i) + "yneg";
            MarkerInit(&marker);

            geometry_msgs::Pose pose;
            pose.position.x = i * RES;
            pose.position.y = -10;
            pose.position.z = 0;

            marker.text = std::to_string(i * RES);
            marker.pose = pose;
            markerArray.markers.push_back(marker);
        }

        /* Publish Axis */
        while (ros::ok())
        {
            /* Publish the tf */
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(0, 0, 0));
            tf::Quaternion q(0, 0, 0, 1);
            transform.setRotation(q);
            tfBroadcast_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

            /* Publish Axis */
            markerPub_.publish(markerArray);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    /*! Function To Publish Image from Camera 
    */
    void PubImg()
    {
        /* Set Publish Rate */
        int publishRate = PUBLISH_RATE_IMG;
        ros::Rate loop_rate(publishRate);

        int seqCounter = 0;

        while (ros::ok())
        {
            /* tf frame */
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(0, 0, 0));
            tf::Quaternion q(0, 0, 0, 1);
            transform.setRotation(q);
            tfBroadcast_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

            /* Open CV bridge to convert CV image to ROS Image */
            cv_bridge::CvImage cvImage;

            /* Get Image frame from Oculii SDK */
            cvImage.image = radarSystem_->GetOneImg();

            /* set encoding Type for the image */
            cvImage.encoding = "rgb8";

            /* Ros Message Image Type */
            sensor_msgs::Image rosImage;

            /* Set Publish Time */
            ros::Time scan_time = ros::Time::now();
            rosImage.header.stamp = scan_time;

            /* Set Image Sequence */
            rosImage.header.seq = seqCounter++;

            /* Set Image Frame Id reference*/
            rosImage.header.frame_id = "base_link";
            cvImage.toImageMsg(rosImage);

            /* Publish Message */
            imgPub_.publish(rosImage);

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    /*! Function To Publish Radar Point Cloud and Tracker Data  
    */
    void PubRadarData()
    {
        /* Declare Containers for Point Cloud */
        std::map< int, oculii::RadarDetectionPacket > pcl;

        /* Declare Containers for tracker */
        std::map< int, oculii::RadarTrackerPacket > trk;

        /* Set Publish Rate */
        int publishRate = PUBLISH_RATE_RADAR;
        ros::Rate loop_rate(publishRate);

        while (ros::ok())
        {
            /* set Transformation Frame */
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(0, 0, 0));
            tf::Quaternion q(0, 0, 0, 1);
            transform.setRotation(q);
            tfBroadcast_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

            /* Get PCL data */
            oculii::RadarErrorCode pclStatus = radarSystem_->GetPointcloud(pcl);

            /* Get Tracker data */
            oculii::RadarErrorCode trackerStatus = radarSystem_->GetTracker(trk);

            /*Publish detection data for each sensor respectively*/
            if(pclStatus == oculii::RadarErrorCode::SUCCESS)
            {
		
		        sensor_msgs::PointCloud oneFrameDetections;
                for (auto it = pcl.begin(); it != pcl.end(); ++it)
                {
		    
                    ros::Time scan_time((it->second.timestamp) / 1e6);
                    oneFrameDetections.header.stamp = scan_time;
                    oneFrameDetections.header.frame_id = "base_link";
                    
                    geometry_msgs::Point32 coord;
                    sensor_msgs::ChannelFloat32 doppler, range, power, alpha, beta;
                    doppler.name = "Doppler";
                    range.name = "Range";
                    power.name = "Power";
                    alpha.name = "Alpha";
                    beta.name = "Beta";

                    for (int i = 0; i < it->second.data.size(); ++i)
                    {

                        coord.x = it->second.data[i].z;
                        coord.y = -it->second.data[i].x;
                        coord.z = -it->second.data[i].y;
                        oneFrameDetections.points.push_back(coord);

                        doppler.values.push_back(it->second.data[i].doppler);
                        range.values.push_back(it->second.data[i].range);
                        power.values.push_back(it->second.data[i].power);
                        alpha.values.push_back(it->second.data[i].alpha);
                        beta.values.push_back(it->second.data[i].beta);
                    }

                    oneFrameDetections.channels.push_back(doppler);
                    oneFrameDetections.channels.push_back(range);
                    oneFrameDetections.channels.push_back(power);
                    oneFrameDetections.channels.push_back(alpha);
                    oneFrameDetections.channels.push_back(beta);

                    
                }
		        detPub_.publish(oneFrameDetections);
            }

            if(trackerStatus == oculii::RadarErrorCode::SUCCESS)
            {
		        sensor_msgs::PointCloud oneFrameTrackers;
                /*Publish tracker data for each sensor respectively*/
                for (auto it = trk.begin(); it != trk.end(); ++it)
                {
                    
                    ros::Time scan_time_trk(it->second.timestamp / 1e6);
                    oneFrameTrackers.header.stamp = scan_time_trk;
                    oneFrameTrackers.header.frame_id = "base_link";

                    geometry_msgs::Point32 coord_trk;
                    sensor_msgs::ChannelFloat32 vx, vy, vz, trkclass, confidence;
                    vx.name = "VeloX";
                    vy.name = "VeloY";
                    vz.name = "VeloZ";
                    trkclass.name = "Class";
                    confidence.name = "Confidence";
                    for (int i = 0; i < it->second.data.size(); ++i)
                    {
                        coord_trk.x = it->second.data[i].z;
                        coord_trk.y = -it->second.data[i].x;
                        coord_trk.z = -it->second.data[i].y;
                        oneFrameTrackers.points.push_back(coord_trk);

                        vx.values.push_back(it->second.data[i].vz);
                        vy.values.push_back(-it->second.data[i].vx);
                        vz.values.push_back(-it->second.data[i].vy);
                        trkclass.values.push_back(it->second.data[i].trkClass);
                        confidence.values.push_back(it->second.data[i].confidence);
                    }
                    oneFrameTrackers.channels.push_back(vx);
                    oneFrameTrackers.channels.push_back(vy);
                    oneFrameTrackers.channels.push_back(vz);
                    oneFrameTrackers.channels.push_back(trkclass);
                    oneFrameTrackers.channels.push_back(confidence);

                    
                }
		        trkPub_.publish(oneFrameTrackers);
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    /*! Function To Publish Radar Enhanced Point Cloud
    */
    void PubRadarEnhancedPcl()
    {
        /* Declare Containers for Enhanced Point Cloud */
        std::map< int, std::vector<oculii::RadarDetectionPacket> > pcl;

        /* Declare Containers for tracker */
        std::map< int, oculii::RadarTrackerPacket > trk;

        /* Set Publish Rate */
        int publishRate = PUBLISH_RATE_RADAR;
        ros::Rate loop_rate(publishRate);

        while (ros::ok())
        {
            /* set Transformation Frame */
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(0, 0, 0));
            tf::Quaternion q(0, 0, 0, 1);
            transform.setRotation(q);
            tfBroadcast_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
            /*Get Enhanced Pcl data */
            oculii::RadarErrorCode enhancedPclStatus = radarSystem_->GetEnhancedPointcloud(pcl);
            /* Get Tracker data */
            //oculii::RadarErrorCode trackerStatus = radarSystem_->GetTracker(trk);

            /*Get Enhanced Pcl data */
            if(enhancedPclStatus == oculii::RadarErrorCode::SUCCESS)
            {
		        sensor_msgs::PointCloud oneFrameDetections;
                for (auto it = pcl.begin(); it != pcl.end(); ++it)
                {
                    
                    ros::Time scan_time = ros::Time::now();
                    oneFrameDetections.header.stamp = scan_time;
                    oneFrameDetections.header.frame_id = "base_link";
                    geometry_msgs::Point32 coord;
                    sensor_msgs::ChannelFloat32 doppler, range, power, alpha, beta;
                    doppler.name = "Doppler";
                    range.name = "Range";
                    power.name = "Power";
                    alpha.name = "Alpha";
                    beta.name = "Beta";
                    for(auto& packet : it->second)
                        for (int i = 0; i < packet.data.size(); ++i)
                        {
                            if (packet.data[i].denoiseFlag != 0 && packet.data[i].power >= 0)
                            {
                                coord.x = packet.data[i].z;
                                coord.y = -packet.data[i].x;
                                coord.z = -packet.data[i].y;
                                oneFrameDetections.points.push_back(coord);

                                doppler.values.push_back(packet.data[i].doppler);
                                range.values.push_back(packet.data[i].range);
                                power.values.push_back(packet.data[i].power);
                                alpha.values.push_back(packet.data[i].alpha);
                                beta.values.push_back(packet.data[i].beta);
                            }
                        }
                    oneFrameDetections.channels.push_back(doppler);
                    oneFrameDetections.channels.push_back(range);
                    oneFrameDetections.channels.push_back(power);
                    oneFrameDetections.channels.push_back(alpha);
                    oneFrameDetections.channels.push_back(beta);

                    
                }
		        enhancedDetPub_.publish(oneFrameDetections);
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    /*! node handle*/
    ros::NodeHandle n_;

    /*! ROS publisher for Radar Point Cloud */
    ros::Publisher detPub_;

    /*! ROS publisher for Radar Enhanced Point Cloud */
    ros::Publisher enhancedDetPub_;

    /*! ROS publisher for Radar Tracker */
    ros::Publisher trkPub_;

    /*! ROS publisher for Radar Image */
    ros::Publisher imgPub_;

    /*! ROS publisher for Axis and Markers */
    ros::Publisher markerPub_;

    /*! Radar IDs_ Containers */
    std::vector<int> IDs_;

    /*! Pointer to Radar System Object */
    oculii::RadarSystem *radarSystem_;

    /*! Path To config File */
    std::string configPath_;

    /*! Tf frame Broadcaster*/
    tf::TransformBroadcaster tfBroadcast_;
};
