#ifndef CONFIGSERVER_H
#define CONFIGSERVER_H

#include <QObject>
#include <QTimer>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "mtmsl_common/mirrorConfig.h"
#include "mtmsl_common/visionHSVConfig.h"
#include "mtmsl_common/imageConfig.h"
#include "mtmsl_common/requestOmniVisionConf.h"
#include "mtmsl_common/requestImage.h"
#include "mtmsl_common/ControlerError.h"
#include "mtmsl_common/cameraProperty.h"
#include "mtmsl_common/PID.h"
#include "mtmsl_common/requestCamProperty.h"
#include "mtmsl_common/requestCamPID.h"
#include "mtmsl_common/requestROI.h"
#include "mtmsl_common/ROI.h"
#include "mtmsl_common/worldConfig.h"
#include "Utils/types.h"
#include "ros/topic_manager.h"
#include <QTime>
#include <vector>

// Config messages
using mtmsl_common::mirrorConfig;
using mtmsl_common::visionHSVConfig;
using mtmsl_common::imageConfig;
using mtmsl_common::ControlerError;
using mtmsl_common::cameraProperty;
using mtmsl_common::PID;
using mtmsl_common::ROI;
using mtmsl_common::worldConfig;

using mtmsl_common::requestImage;
using mtmsl_common::requestCamPID;
using mtmsl_common::requestCamProperty;
using mtmsl_common::requestROI;

using namespace cv;
using namespace std;
class ConfigServer : public QObject
{
   Q_OBJECT
public:
   explicit ConfigServer(ros::NodeHandle *par, QObject *parent = 0); // Constructor
   ~ConfigServer();
   void assignImage(Mat *source);
   void setOmniVisionConf(mirrorConfig mirrmsg,visionHSVConfig vismsg,imageConfig imgmsg, worldConfig ballmsg, worldConfig obsmsg, worldConfig rlemsg);
   void setProps(vector<float> prop_values);
   void setPIDValues(vector<float> pid_values);
   void sendError(Point2d error);
   void setROIs(vector<ROI> rois);
   void setKalman(int Qx,int Qz,int Rx,int Rz);
   int prop_used;
   inline bool getCalib(){ return calib;}
private:
   // VARIABLES
   // ##############################################################
   // ##############################################################
   // Subscriptions and Requests monitor
   int n_subscriptions_;
   QTimer *watchdog_timer_;
   QTimer *image_timer_;
   int configured_frequency_;
   bool multiple_sends_,calib;


   // ROS
   ros::NodeHandle *parent_; // ROS node parent pointer
   image_transport::ImageTransport *it_; // To send images through ROS

   //SEND
   image_transport::Publisher image_pub_; // Publisher for image_transport
   ros::Publisher error_pub_;

   //RECEIVE
   ros::Subscriber mirror_sub_; // Subscriber for mirror configuration (mirrorConfig.msg)
   ros::Subscriber image_sub_; // Subscriber for image configuration (imageConfig.msg)
   ros::Subscriber vision_sub_; // Subscriber for vision[lut] configuration (visionHSVConfig.msg)
   ros::Subscriber property_sub_;//Subscriber for property configuration(cameraProperty.msg)
   ros::Subscriber pid_sub_;//Subscriber for pid configuration(PID.msg)
   ros::Subscriber roi_sub_;//Subscriber for ROI configuration(ROI.msg)
   ros::Subscriber world_sub_;//Subscriber for worldConfig configuration(worldConfig.msg)

   //SERVICES AND DATA
   ros::ServiceServer service_omniconf; // Service to relay the configuration
   ros::ServiceServer service_imgrequest; // Service to relay the configuration
   ros::ServiceServer service_propConf; // Service to relay Properties configuration
   ros::ServiceServer service_pidConf; // Service to relay PID Controler Configuration
   ros::ServiceServer service_roiConf;// Service to relay ROI Controler Configuration

   mirrorConfig mirrorConfmsg; // Data to hold current mirrorConfig
   visionHSVConfig visionConfmsg; // Data to hold current visionHSVConfig
   imageConfig imageConfmsg; // Data to hold current imageConfig
   sensor_msgs::ImagePtr image_;
   cv::Mat mock_image;

   cameraProperty props_msg [7];
   PID pid_msg[7];
   ROI whiteRoi,blackRoi;
   worldConfig ballConfmsg, obsConfmsg, RLEConfmsg, KalmanConfmsg;
   Point targets;

private slots:
   void getSubscribers();
   bool processImageRequest(requestImage::Request &req, requestImage::Response &res);
   void processMirrorConfig(const mirrorConfig::ConstPtr &msg);
   void processVisionConfig(const visionHSVConfig::ConstPtr &msg);
   void processImageConfig(const imageConfig::ConstPtr &msg);
   bool omniVisionConfService(mtmsl_common::requestOmniVisionConf::Request &req,mtmsl_common::requestOmniVisionConf::Response &res);
   void postRequestedImage();
   void init_mock_image();

   void processCamPropertyConfig(const cameraProperty::ConstPtr &msg);
   void processPIDConfig(const PID::ConstPtr &msg);
   bool camPropertyConfService(requestCamProperty::Request &req,requestCamProperty::Response &res);
   bool camPIDConfService(requestCamPID::Request &req,requestCamPID::Response &res);
   void processROIConfig(const ROI::ConstPtr &msg);
   void processWorldConfig(const worldConfig::ConstPtr &msg);
   bool camROIConfService(requestROI::Request &req,requestROI::Response &res);

signals:
   void stopImageAssigning();
   void changedImageRequest(uint8_t type);
   void changedMirrorConfiguration(mirrorConfig::ConstPtr msg);
   void changedLutConfiguration(visionHSVConfig::ConstPtr msg);
   void changedImageConfiguration(imageConfig::ConstPtr msg);
   void changedCameraProperties(cameraProperty::ConstPtr msg);
   void changedCamPID(PID::ConstPtr msg);
   void changedROIConfiguration(ROI::ConstPtr msg);
   void changedWorldConfiguration(worldConfig::ConstPtr msg);
};

#endif // CONFIGSERVER_H
