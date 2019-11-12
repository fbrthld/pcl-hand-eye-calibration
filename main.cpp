#include <iostream>
#include <thread>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/local_maximum.h>

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> CloudType;



using namespace std::chrono_literals;

void CalculateTranformation(CloudType::Ptr scan, Eigen::Matrix3f *rotation, Eigen::Vector3f *translation) {
    //CloudType::Ptr scan(new CloudType);


    CloudType::Ptr known(new CloudType);

    // A
    known->points.emplace_back(PointType(81.75, 25,  91.5));
    // B
    known->points.emplace_back(PointType(86.75, -75,  69));
    // C
    known->points.emplace_back(PointType( 79.25, 75, 69));
    // D
    known->points.emplace_back(PointType(89.25,  0,  31.5));


//    pcl::copyPointCloud(*known, *scan);
//
//    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
//    transform.translation() << 0, 100, 0;
//    pcl::transformPointCloud(*scan, *scan, transform);
//
//    transform = Eigen::Affine3f::Identity();
//    float rotationDeg = 90;
//    transform.rotate(Eigen::AngleAxisf(rotationDeg * M_PI / 180, Eigen::Vector3f::UnitZ()));
//
//    pcl::transformPointCloud(*scan, *scan, transform);

//    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Viewer"));
//
//    int vp0(0);
//    viewer->createViewPort(0, 0, 1, 1, vp0);
//    viewer->addText("transformed", 10, 10, "input_text", vp0);
//    viewer->setBackgroundColor (0, 0, 0, vp0);
//    viewer->addPointCloud<PointType> (known, "input", vp0);
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input", vp0);
//
//    for (int j = 0; j < known->points.size(); ++j) {
//        std::ostringstream sstream;
//        sstream << "Point " << j;
//        viewer->addText3D(sstream.str(), known->points[j]);
//    }
//
//    viewer->addCoordinateSystem(100);
//    viewer->initCameraParameters();
//
//    while (!viewer->wasStopped ())
//    {
//        viewer->spinOnce (100);
//        std::this_thread::sleep_for(100ms);
//    }


    Eigen::Matrix3f p;
    p(0, 0) = known->points[1].x - known->points[0].x;
    p(0, 1) = known->points[1].y - known->points[0].y;
    p(0, 2) = known->points[1].z - known->points[0].z;

    p(1, 0) = known->points[2].x - known->points[0].x;
    p(1, 1) = known->points[2].y - known->points[0].y;
    p(1, 2) = known->points[2].z - known->points[0].z;

    p(2, 0) = known->points[3].x - known->points[0].x;
    p(2, 1) = known->points[3].y - known->points[0].y;
    p(2, 2) = known->points[3].z - known->points[0].z;


    Eigen::Matrix3f q;
    q(0, 0) = scan->points[1].x - scan->points[0].x;
    q(0, 1) = scan->points[1].y - scan->points[0].y;
    q(0, 2) = scan->points[1].z - scan->points[0].z;

    q(1, 0) = scan->points[2].x - scan->points[0].x;
    q(1, 1) = scan->points[2].y - scan->points[0].y;
    q(1, 2) = scan->points[2].z - scan->points[0].z;

    q(2, 0) = scan->points[3].x - scan->points[0].x;
    q(2, 1) = scan->points[3].y - scan->points[0].y;
    q(2, 2) = scan->points[3].z - scan->points[0].z;

    *rotation = p.inverse() * q;

    auto transform = Eigen::Affine3f::Identity();
    transform.rotate(*rotation);
    pcl::transformPointCloud(*scan, *scan, transform);

    *translation = known->points[0].getVector3fMap() - scan->points[0].getVector3fMap();
}

CloudType::Ptr FindPeaks() {
    CloudType::Ptr input(new CloudType);
    CloudType::Ptr output(new CloudType);

    pcl::io::loadPCDFile("calibration +-0.pcd", *input);

    pcl::LocalMaximum<PointType> localMaximum;
    localMaximum.setInputCloud(input);
    localMaximum.setRadius(2);
    localMaximum.setNegative(true);

    localMaximum.filter(*output);


    std::cout << output->points.size() << " points" << std::endl;

    std::vector<PointType> peaks;
    peaks.emplace_back(output->points[0]);
    peaks.emplace_back(output->points[1]);
    peaks.emplace_back(output->points[2]);
    peaks.emplace_back(output->points[3]);
    peaks.emplace_back(output->points[4]);
    peaks.emplace_back(output->points[5]);

    int peaksMinimumIndex = 0;

    for (int j = 0; j < peaks.size(); ++j) {
        if(peaks[j].z < peaks[peaksMinimumIndex].z)
            peaksMinimumIndex = j;
    }

    for (int j = 6; j < output->points.size(); ++j) {
        if(output->points[j].z > peaks[peaksMinimumIndex].z) {
            peaks[peaksMinimumIndex] = output->points[j];

            for (int k = 0; k < peaks.size(); ++k) {
                if(peaks[k].z < peaks[peaksMinimumIndex].z)
                    peaksMinimumIndex = k;
            }
        }
    }

    output->points.clear();
    output->points.emplace_back(peaks[0]);
    output->points.emplace_back(peaks[1]);
    output->points.emplace_back(peaks[2]);
    output->points.emplace_back(peaks[3]);
    output->points.emplace_back(peaks[4]);
    output->points.emplace_back(peaks[5]);


    return output;
}

int main() {

    auto peakCloud = FindPeaks();

    peakCloud->points.erase(peakCloud->points.begin() + 5);
    peakCloud->points.erase(peakCloud->points.begin() + 1);

    CloudType::Ptr ordered(new CloudType);
    int peakSize = peakCloud->points.size();
    for (int j = 0; j < peakSize; ++j) {
        int smallestYIndex = 0;
        for (int k = 1; k < peakCloud->points.size(); ++k) {
            if(peakCloud->points[k].y < peakCloud->points[smallestYIndex].y) {
                smallestYIndex = k;
            }
        }
        ordered->points.emplace_back(peakCloud->points[smallestYIndex]);
        peakCloud->points.erase(peakCloud->points.begin() + smallestYIndex);
    }

    Eigen::Matrix3f rotation;
    Eigen::Vector3f translation;

    CalculateTranformation(ordered, &rotation, &translation);

    CloudType::Ptr input(new CloudType);
    // pcl::io::loadPCDFile("calibration +-0.pcd", *input);
    pcl::io::loadPCDFile("scans/box/short 1500sps 250mms.pcd", *input);

    auto transform = Eigen::Affine3f::Identity();
    transform.rotate(rotation);
    pcl::transformPointCloud(*input, *input, transform);
    transform = Eigen::Affine3f::Identity();

    transform.translate(translation);
    pcl::transformPointCloud(*input, *input, transform);


    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Viewer"));

    int vp0(0);
    viewer->createViewPort(0, 0, 1, 1, vp0);
    viewer->addText("transformed", 10, 10, "input_text", vp0);
    viewer->setBackgroundColor (0, 0, 0, vp0);
    viewer->addPointCloud<PointType> (input, "input", vp0);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input", vp0);

    PointType grabberOrigin(0, 0, 0);
    viewer->addCoordinateSystem(100, grabberOrigin.x, grabberOrigin.y, grabberOrigin.z);
    viewer->addText3D("    GrabberOrigin", grabberOrigin, 3);

    PointType robotOrigin(-980, 0, 37.5);
    viewer->addCoordinateSystem(100, robotOrigin.x, robotOrigin.y, robotOrigin.z);
    viewer->addText3D("    RobotOrigin", robotOrigin, 3);

    PointType sledOrigin(-1008.1, -165, -302.5);
    viewer->addCoordinateSystem(100, sledOrigin.x, sledOrigin.y, sledOrigin.z);
    viewer->addText3D("    SledOrigin", sledOrigin, 3);


    viewer->initCameraParameters();

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(100ms);
    }

    return 0;
}

