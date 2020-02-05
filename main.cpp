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

void CalculateTranformation(CloudType::Ptr scan, Eigen::Matrix3d *rotation, Eigen::Vector3d *translation) {

    CloudType::Ptr known(new CloudType);


//    // A
//    known->points.emplace_back(PointType(23.0, 194.0,  4.25));
//    // B
//    known->points.emplace_back(PointType(60.5, 244.0,  9.25));
//    // C
//    known->points.emplace_back(PointType(98.0, 169.0, -0.75));
//    // D
//    known->points.emplace_back(PointType(120.5,  219.0,  1.75));

    // A
    known->points.emplace_back(PointType(794.0, -50.0,  854.25));
    // B
    known->points.emplace_back(PointType(831.5, 00.0,  859.25));
    // C
    known->points.emplace_back(PointType(869.0, -75.0, 849.25));
    // D
    known->points.emplace_back(PointType(891.5,  -25.0,  851.75));


//    pcl::copyPointCloud(*known, *scan);
//
//    Eigen::Affine3d transform1 = Eigen::Affine3d::Identity();
//    transform1.translation() << 0, 100, 0;
//    pcl::transformPointCloud(*scan, *scan, transform1);
//
//    transform1 = Eigen::Affine3d::Identity();
//    float rotationDeg = 90;
//    transform1.rotate(Eigen::AngleAxisd(rotationDeg * M_PI / 180, Eigen::Vector3d::UnitZ()));
//    pcl::transformPointCloud(*scan, *scan, transform1);



//    pcl::copyPointCloud(*scan, *known);
//
//
//    auto transform1 = Eigen::Affine3d::Identity();
//    float rotationDeg = -90;
//    transform1.rotate(Eigen::AngleAxisd(rotationDeg * M_PI / 180, Eigen::Vector3d::UnitZ()));
//    pcl::transformPointCloud(*known, *known, transform1);
//
//    transform1 = Eigen::Affine3d::Identity();
//    transform1.translation() << 0, -100, 0;
//    pcl::transformPointCloud(*known, *known, transform1);



    Eigen::Matrix3d p;
    p(0, 0) = ((double)scan->points[1].x) - scan->points[0].x;
    p(0, 1) = ((double)scan->points[1].y) - scan->points[0].y;
    p(0, 2) = ((double)scan->points[1].z) - scan->points[0].z;

    p(1, 0) = ((double)scan->points[2].x) - scan->points[0].x;
    p(1, 1) = ((double)scan->points[2].y) - scan->points[0].y;
    p(1, 2) = ((double)scan->points[2].z) - scan->points[0].z;

    p(2, 0) = ((double)scan->points[3].x) - scan->points[0].x;
    p(2, 1) = ((double)scan->points[3].y) - scan->points[0].y;
    p(2, 2) = ((double)scan->points[3].z) - scan->points[0].z;


    Eigen::Matrix3d q;
    q(0, 0) = ((double)known->points[1].x) - known->points[0].x;
    q(0, 1) = ((double)known->points[1].y) - known->points[0].y;
    q(0, 2) = ((double)known->points[1].z) - known->points[0].z;

    q(1, 0) = ((double)known->points[2].x) - known->points[0].x;
    q(1, 1) = ((double)known->points[2].y) - known->points[0].y;
    q(1, 2) = ((double)known->points[2].z) - known->points[0].z;

    q(2, 0) = ((double)known->points[3].x) - known->points[0].x;
    q(2, 1) = ((double)known->points[3].y) - known->points[0].y;
    q(2, 2) = ((double)known->points[3].z) - known->points[0].z;

    cout << p << endl << endl << q << endl;

    *rotation = (p.inverse() * q).inverse();

    CloudType::Ptr transformedScan(new CloudType);

    auto transform = Eigen::Affine3d::Identity();
    transform.rotate(*rotation);
    pcl::transformPointCloud(*scan, *transformedScan, transform);

    cout << "transform:" << endl << transform.matrix() << endl;

    *translation = (known->points[0].getVector3fMap() - transformedScan->points[0].getVector3fMap()).cast<double>();

    transform = Eigen::Affine3d::Identity();
    transform.translate(*translation);
    pcl::transformPointCloud(*transformedScan, *transformedScan, transform);

    cout << *rotation << endl << endl << *translation << endl;

//region Visualization
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Viewer"));

    int vp0(0);
    viewer->createViewPort(0, 0, 0.5, 1, vp0);
    viewer->addText("known points", 10, 10, "known_text", vp0);
    viewer->setBackgroundColor (0, 0, 0, vp0);
    viewer->addPointCloud<PointType> (known, "known", vp0);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "known", vp0);

    int vp1(0);
    viewer->createViewPort(0.5, 0, 1, 1, vp1);
    viewer->addText("scanned peaks", 10, 10, "scan_text", vp1);
    viewer->setBackgroundColor (0, 0, 0, vp1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> scanColor(scan, 255, 0, 0);
    viewer->addPointCloud<PointType> (scan, scanColor, "scan", vp1);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scan", vp1);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformedScanColor(scan, 0, 255, 255);
    viewer->addPointCloud<PointType> (transformedScan, transformedScanColor, "transformedScan", vp1);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "transformedScan", vp1);

    for (int j = 0; j < known->points.size(); ++j) {
        std::ostringstream sstream;
        sstream << "Point " << j << "(" << known->points[j].x << ", " << known->points[j].y << ", " << known->points[j].z << ")";
        viewer->addText3D(sstream.str(), known->points[j], 1.0, 1.0, 1.0, 1.0, sstream.str(), vp0);
    }

    for (int j = 0; j < scan->points.size(); ++j) {
        std::ostringstream sstream;
        sstream << "Point " << j << "(" << scan->points[j].x << ", " << scan->points[j].y << ", " << scan->points[j].z << ")";
        viewer->addText3D(sstream.str(), scan->points[j], 1.0, 1.0, 1.0, 1.0, sstream.str(), vp1);
    }

    for (int j = 0; j < transformedScan->points.size(); ++j) {
        std::ostringstream sstream;
        sstream << "Point " << j << "(" << transformedScan->points[j].x << ", " << transformedScan->points[j].y << ", " << transformedScan->points[j].z << ")";
        viewer->addText3D(sstream.str(), known->points[j], 1.0, 1.0, 1.0, 1.0, sstream.str(), vp1);
    }

    viewer->addCoordinateSystem(100);
    viewer->initCameraParameters();

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(100ms);
    }
//endregion
}

CloudType::Ptr FindPeaks() {
    CloudType::Ptr input(new CloudType);
    CloudType::Ptr output(new CloudType);

    pcl::io::loadPCDFile("./output/calibration sample/scan10.pcd", *input);

    pcl::LocalMaximum<PointType> localMaximum;
    localMaximum.setInputCloud(input);
    localMaximum.setRadius(2);
    localMaximum.setNegative(true);

    localMaximum.filter(*output);


    std::cout << output->points.size() << " local maxima" << std::endl;

    std::vector<PointType> peaks;
    peaks.emplace_back(output->points[0]);
    peaks.emplace_back(output->points[1]);
    peaks.emplace_back(output->points[2]);
    peaks.emplace_back(output->points[3]);

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

//region Visualization
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Viewer"));

    int vp0(0);
    viewer->createViewPort(0, 0, 1, 1, vp0);
    viewer->addText("peaks", 10, 10, "input_text", vp0);
    viewer->setBackgroundColor (0, 0, 0, vp0);
    viewer->addPointCloud<PointType> (input, "input", vp0);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input", vp0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> outputColor(output, 0, 255, 255);
    viewer->addPointCloud<PointType> (output, outputColor, "output", vp0);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 40, "output", vp0);

    viewer->addCoordinateSystem(100);
    viewer->initCameraParameters();

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(100ms);
    }
//endregion

    return output;
}

int main() {
    auto peakCloud = FindPeaks();

    CloudType::Ptr ordered(new CloudType);
    int peakSize = peakCloud->points.size();
    for (int j = 0; j < peakSize; ++j) { // have to search in Y direction in scan, because the pcl is built up in y direction
        int smallestYIndex = 0;
        for (int k = 1; k < peakCloud->points.size(); ++k) {
            if(peakCloud->points[k].y < peakCloud->points[smallestYIndex].y) {
                smallestYIndex = k;
            }
        }
        ordered->points.emplace_back(peakCloud->points[smallestYIndex]);
        peakCloud->points.erase(peakCloud->points.begin() + smallestYIndex);

//        int largestYIndex = 0;
//        for (int k = 1; k < peakCloud->points.size(); ++k) {
//            if(peakCloud->points[k].y > peakCloud->points[largestYIndex].y) {
//                largestYIndex = k;
//            }
//        }
//        ordered->points.emplace_back(peakCloud->points[largestYIndex]);
//        peakCloud->points.erase(peakCloud->points.begin() + largestYIndex);
    }

    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;

    CalculateTranformation(ordered, &rotation, &translation);

    CloudType::Ptr input(new CloudType);
    pcl::io::loadPCDFile("./output/calibration sample/scan10.pcd", *input);
    // pcl::io::loadPCDFile("./output/box/1/scan2.pcd", *input);

    auto transform = Eigen::Affine3d::Identity();
    transform.rotate(rotation);
    pcl::transformPointCloud(*input, *input, transform);
    transform = Eigen::Affine3d::Identity();

    transform.translate(translation);
    pcl::transformPointCloud(*input, *input, transform);


//region Visualization
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Viewer"));

    int vp0(0);
    viewer->createViewPort(0, 0, 1, 1, vp0);
    viewer->addText("transformed", 10, 10, "input_text", vp0);
    viewer->setBackgroundColor (0, 0, 0, vp0);
    viewer->addPointCloud<PointType> (input, "input", vp0);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input", vp0);


    CloudType::Ptr known(new CloudType);
    // A
    known->points.emplace_back(PointType(794.0, -50.0,  854.25));
    // B
    known->points.emplace_back(PointType(831.5, 0.0,  859.25));
    // C
    known->points.emplace_back(PointType(869.0, -75.0, 849.25));
    // D
    known->points.emplace_back(PointType(891.5,  -25.0,  851.75));

    viewer->addPointCloud<PointType>(known, "known", vp0);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "known", vp0);

    for (int j = 0; j < known->points.size(); ++j) {
        std::ostringstream sstream;
        sstream << "Point " << j << "(" << known->points[j].x << ", " << known->points[j].y << ", " << known->points[j].z << ")";
        viewer->addText3D(sstream.str(), known->points[j]);
    }

//    PointType grabberOrigin(0, 0, 0);
//    viewer->addCoordinateSystem(100, grabberOrigin.x, grabberOrigin.y, grabberOrigin.z);
//    viewer->addText3D("    GrabberOrigin", grabberOrigin, 3);

    PointType robotOrigin(0, 0, 0);
    viewer->addCoordinateSystem(100, robotOrigin.x, robotOrigin.y, robotOrigin.z);
    viewer->addText3D("    RobotOrigin", robotOrigin, 3);
//
//    PointType sledOrigin(-1008.1, -165, -302.5);
//    viewer->addCoordinateSystem(100, sledOrigin.x, sledOrigin.y, sledOrigin.z);
//    viewer->addText3D("    SledOrigin", sledOrigin, 3);


    viewer->initCameraParameters();

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(100ms);
    }
//endregion

    return 0;
}

