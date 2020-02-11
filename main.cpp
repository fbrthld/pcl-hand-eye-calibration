#include <iostream>
#include <thread>
#include <chrono>

#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/local_maximum.h>



typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> CloudType;

using namespace std::chrono_literals;

void CalculateTranformation(CloudType::Ptr scan, Eigen::Matrix4f *transformation) {

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

    Eigen::Matrix<float, 3, 4> umeyamaSrc;
    umeyamaSrc(0, 0) = scan->points[0].x;
    umeyamaSrc(1, 0) = scan->points[0].y;
    umeyamaSrc(2, 0) = scan->points[0].z;

    umeyamaSrc(0, 1) = scan->points[1].x;
    umeyamaSrc(1, 1) = scan->points[1].y;
    umeyamaSrc(2, 1) = scan->points[1].z;

    umeyamaSrc(0, 2) = scan->points[2].x;
    umeyamaSrc(1, 2) = scan->points[2].y;
    umeyamaSrc(2, 2) = scan->points[2].z;

    umeyamaSrc(0, 3) = scan->points[3].x;
    umeyamaSrc(1, 3) = scan->points[3].y;
    umeyamaSrc(2, 3) = scan->points[3].z;

    Eigen::Matrix<float, 3, 4> umeyamaDest;
    umeyamaDest(0, 0) = known->points[0].x;
    umeyamaDest(1, 0) = known->points[0].y;
    umeyamaDest(2, 0) = known->points[0].z;

    umeyamaDest(0, 1) = known->points[1].x;
    umeyamaDest(1, 1) = known->points[1].y;
    umeyamaDest(2, 1) = known->points[1].z;

    umeyamaDest(0, 2) = known->points[2].x;
    umeyamaDest(1, 2) = known->points[2].y;
    umeyamaDest(2, 2) = known->points[2].z;

    umeyamaDest(0, 3) = known->points[3].x;
    umeyamaDest(1, 3) = known->points[3].y;
    umeyamaDest(2, 3) = known->points[3].z;



    *transformation = Eigen::umeyama(umeyamaSrc, umeyamaDest, true);

    CloudType::Ptr transformedScan(new CloudType);

    pcl::transformPointCloud(*scan, *transformedScan, *transformation);
    #pragma omp critical
    {
    cout << "transform:" << endl << transformation->matrix() << endl;
    cout << "errors: " << endl;

    for (int j = 0; j < known->points.size(); ++j) {
        auto difference = transformedScan->points[j].getVector3fMap() - known->points[j].getVector3fMap();
        cout << "\tp" << j << ": (" << difference.norm() << " mm)" << endl;
        // << difference << endl << endl;
    }

    cout << endl;
    };
/*
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
*/

}

CloudType::Ptr FindPeaks(std::string inputCloudFile, int nPeaks) {
    CloudType::Ptr input(new CloudType);
    CloudType::Ptr output(new CloudType);

    pcl::io::loadPCDFile(inputCloudFile, *input);

    pcl::LocalMaximum<PointType> localMaximum;
    localMaximum.setInputCloud(input);
    localMaximum.setRadius(2);
    localMaximum.setNegative(true);

    localMaximum.filter(*output);


    // std::cout << output->points.size() << " local maxima" << std::endl;

    std::vector<PointType> peaks;
    for (int l = 0; l < nPeaks; ++l) {
        peaks.emplace_back(output->points[l]);
    }


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
    for (int m = 0; m < nPeaks; ++m) {
        output->points.emplace_back(peaks[m]);
    }

/*
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
 */

    return output;
}

int main() {

    std::vector<CloudType::Ptr> peakClouds(10);

    #pragma omp parallel for default(none) shared(peakClouds)
    for (int j = 0; j < 10; ++j) {
        std::ostringstream sstream;
        sstream << "/home/fberthold/Documents/scans/scan" << j << ".pcd";

        auto currentPeaks = FindPeaks(sstream.str(), 4);

        peakClouds[j] = CloudType::Ptr(new CloudType);

        int nPeaks = currentPeaks->points.size();

        for (int k = 0; k < nPeaks; ++k) {
            int smallestYIndex = 0;
            for (int l = 1; l < currentPeaks->points.size(); ++l) {
                if(currentPeaks->points[l].y < currentPeaks->points[smallestYIndex].y) { // have to search in Y direction in scan, because the pcl is built up in y direction
                    smallestYIndex = l;
                }
            }
            peakClouds[j]->points.emplace_back(currentPeaks->points[smallestYIndex]);
            currentPeaks->points.erase(currentPeaks->points.begin() + smallestYIndex);
        }
    }

    CloudType::Ptr averagePeaks(new CloudType);

    for (int j = 0; j < 4; ++j) {
        const int nClouds = 10;

        auto meanPoint = peakClouds[0]->points[j].getVector3fMap();
        for (int k = 1; k < nClouds; ++k) {
            meanPoint += peakClouds[k]->points[j].getVector3fMap();
        }

        meanPoint /= nClouds;

        averagePeaks->points.emplace_back(PointType(meanPoint.x(), meanPoint.y(), meanPoint.z()));

    }

    Eigen::Matrix4f transformation;

    CalculateTranformation(averagePeaks, &transformation);

/*
    CloudType::Ptr input(new CloudType);
    pcl::io::loadPCDFile(SCAN_FILE, *input);
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

    return 0;*/
}

