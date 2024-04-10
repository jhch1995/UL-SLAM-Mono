/**
* This file is part of UL-SLAM based on ORB-SLAM2
* This file is a modified version of EPnP <http://cvlab.epfl.ch/EPnP/index.php>, see FreeBSD license below.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/raulmur/ORB_SLAM2>>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <iostream>

#include <cv.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/line_descriptor/descriptor.hpp>
using namespace cv;
using namespace cv::line_descriptor;
//using namespace line_descriptor;

#include <vector>
using namespace std;

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
using namespace Eigen;

typedef Matrix<double,6,1> Vector6d;
typedef Matrix<double,6,6> Matrix6d;

struct compare_descriptor_by_NN_dist
{
    inline bool operator()(const vector<DMatch>& a, const vector<DMatch>& b){
        return ( a[0].distance < b[0].distance);
    }
};

struct conpare_descriptor_by_NN12_dist
{
    inline bool operator()(const vector<DMatch>& a, const vector<DMatch>& b){
        return ((a[1].distance - a[0].distance) > (b[1].distance - b[0].distance));
    }
};

struct sort_descriptor_by_queryIdx
{
    inline bool operator()(const vector<DMatch>& a, const vector<DMatch>& b){
        return ( a[0].queryIdx < b[0].queryIdx );
    }
};

struct sort_lines_by_response
{
    inline bool operator()(const KeyLine& a, const KeyLine& b){
        return ( a.response > b.response );
    }
};

inline Mat SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<  0, -v.at<float>(2), v.at<float>(1),
                        v.at<float>(2),               0,-v.at<float>(0),
                       -v.at<float>(1),  v.at<float>(0),             0);
}

inline double vector_mad(vector<double> residues)
{
    if(residues.size()!=0)
    {
        // Return the standard deviation of vector with MAD estimation
        int n_samples = residues.size();
        sort(residues.begin(), residues.end());
        double median = residues[n_samples/2];
        for(int i=0; i<n_samples; i++)
            residues[i] = fabs(residues[i]-median);
        sort(residues.begin(), residues.end());
        double MAD = residues[n_samples/2];
        return 1.4826*MAD;
    } else
        return 0.0;
}
