
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>

#include <iostream>
#include <thread>
#include <chrono>

#include "StereoCameraRealSense.h"
#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>


void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &_event, void* _mViewer);
void transformation();
float angle;
float translation;

pcl::PointCloud<pcl::PointXYZRGB> cumulative_cloud;
StereoCameraRealSense camera;


// This is the main function
int main (int argc, char** argv)
{
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor(0.05, 0.05, 0.05);//Setting backgroud to dark grey
    viewer.initCameraParameters();
    viewer.addCoordinateSystem (1.0);
    camera.init();
    viewer.registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

   while(true){

          viewer.removePointCloud("cumulative_cloud");
          viewer.addPointCloud (cumulative_cloud.makeShared(), "cumulative_cloud");
          viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cumulative_cloud");
          viewer.spinOnce(30);

          //         // pcl::PointCloud<pcl::PointXYZRGB>::Ptr fin_transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());


          //          //cout<< cloud.size() <<endl;



       }
 return 0;
}


void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &_event, void* _mViewer) {



    std::cout<<"Please Enter the value of angle at which camera is grabbing the image: ";

    if (_event.getKeySym() == "Return" && _event.keyUp()) {


      cin >> angle;
      cin >> translation;
      angle = angle * (M_PI/180);
      std::cout<<"The entered value in radian is: "<< angle <<endl;
      transformation();
    }

}

void transformation(){
    static pcl::PointCloud<pcl::PointXYZRGB> source_cloud;
    static pcl::PointCloud<pcl::PointXYZRGB> translated_cloud;
    static pcl::PointCloud<pcl::PointXYZRGB> transformed_cloud;

std::cout<<"The entered value in radian is: "<< angle <<endl;

       float Tx =0.0,Ty=translation,Tz=0.0;
       float Rz;
       float Rx = M_PI/2;
       Rz = angle;

std::cout<<"size cumulative1 " << cumulative_cloud.size();

       camera.grab();
       camera.cloud(source_cloud);

       Eigen::Affine3f translat = Eigen::Affine3f::Identity();
       Eigen::Affine3f trans_rotat = Eigen::Affine3f::Identity();



       translat.translation() << Tx, Ty, Tz;
       pcl::transformPointCloud (source_cloud, translated_cloud, translat);

       trans_rotat.rotate (Eigen::AngleAxisf (Rz,Eigen::Vector3f::UnitZ()));
       trans_rotat.rotate (Eigen::AngleAxisf (Rx,Eigen::Vector3f::UnitX()));

       pcl::transformPointCloud (translated_cloud, transformed_cloud, trans_rotat);

/*
       if(cumulative_cloud.size() > 0){

          pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
          icp.setInputSource(transformed_cloud.makeShared());
          icp.setInputTarget(cumulative_cloud.makeShared());
          pcl::PointCloud<pcl::PointXYZ> Final;
          std::cout<<"Aligining " << cumulative_cloud.size() <<" to " << transformed_cloud.size()<<endl;

          icp.align(Final);

          std::cout <<"Aligned" <<endl;

          //cumulative_cloud +=final;
       }
*/



       cumulative_cloud += transformed_cloud;
  std::cout<<"size cumulative2 " << cumulative_cloud.size();
       //std::clear.source_cloud;


}




