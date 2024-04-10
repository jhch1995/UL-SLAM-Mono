/**
* This file is part of UL-SLAM based on ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef GEOMETRIC_TOOLS_H
#define GEOMETRIC_TOOLS_H

#include <opencv2/core/core.hpp>
#include <sophus/se3.hpp>
#include <Eigen/Core>

namespace ORB_SLAM3
{

class KeyFrame;

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector8d = Eigen::Matrix<double, 8, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

using Vector6f = Eigen::Matrix<float, 6, 1>;

class GeometricTools
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Compute the Fundamental matrix between KF1 and KF2
    static Eigen::Matrix3f ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

    //Triangulate point with KF1 and KF2
    static bool Triangulate(Eigen::Vector3f &x_c1, Eigen::Vector3f &x_c2,Eigen::Matrix<float,3,4> &Tc1w ,Eigen::Matrix<float,3,4> &Tc2w , Eigen::Vector3f &x3D);

    static bool TriangulatePlukerLine(Vector6f &line_c1, Vector6f &line_c2, Sophus::SE3<float> &Tc1w, Sophus::SE3<float> &Tc2w, Vector6f &line3D, Vector6f &pluker3D);

    template<int rows, int cols>
    static bool CheckMatrices(const cv::Mat &cvMat, const Eigen::Matrix<float,rows,cols> &eigMat) {
        const float epsilon = 1e-3;
        if(rows != cvMat.rows || cols != cvMat.cols) {
            std::cout << "wrong cvmat size\n";
            return false;
        }
        for(int i = 0; i < rows; i++)
            for(int j = 0; j < cols; j++)
                if ((cvMat.at<float>(i,j) > (eigMat(i,j) + epsilon)) ||
                    (cvMat.at<float>(i,j) < (eigMat(i,j) - epsilon))){
                    std::cout << "cv mat:\n" << cvMat << std::endl;
                    std::cout << "eig mat:\n" << eigMat << std::endl;
                    return false;
                }
        return true;
    }

    template<typename T, int rows, int cols>
    static bool CheckMatrices( const Eigen::Matrix<T,rows,cols> &eigMat1, const Eigen::Matrix<T,rows,cols> &eigMat2) {
        const float epsilon = 1e-3;
        for(int i = 0; i < rows; i++)
            for(int j = 0; j < cols; j++)
                if ((eigMat1(i,j) > (eigMat2(i,j) + epsilon)) ||
                    (eigMat1(i,j) < (eigMat2(i,j) - epsilon))){
                    std::cout << "eig mat 1:\n" << eigMat1 << std::endl;
                    std::cout << "eig mat 2:\n" << eigMat2 << std::endl;
                    return false;
                }
        return true;
    }

    // line geometry tools
    Eigen::Vector4d LineToOrth(Vector6d line);
    Vector6d OrthToLine(Eigen::Vector4d orth);
    static Eigen::Vector4d PlkToOrth(Vector6d plk);
    static Vector6d OrthToPlk(Eigen::Vector4d orth);

    static Eigen::Vector4d GetPlaneFromThreePoints(Eigen::Vector3d x1, Eigen::Vector3d x2, Eigen::Vector3d x3);
    static Vector6d GetPluckerFromTwoPlane(Eigen::Vector4d pi1, Eigen::Vector4d pi2);
    static Eigen::Vector3d GetPluckerOrigin(Eigen::Vector3d n, Eigen::Vector3d v);
    static Eigen::Matrix3d SkewMatFromVector3d(Eigen::Vector3d v );

    static Eigen::Vector3d TransInvPointFromPose( Eigen::Matrix3d Rcw, Eigen::Vector3d tcw, Eigen::Vector3d pt_c );
    static Eigen::Vector3d TransPointByPose( Eigen::Matrix3d Rcw, Eigen::Vector3d tcw , Eigen::Vector3d pt_w );
    Vector6d line_to_pose(Vector6d line_w, Eigen::Matrix3d Rcw, Eigen::Vector3d tcw);
    Vector6d line_from_pose(Vector6d line_c, Eigen::Matrix3d Rcw, Eigen::Vector3d tcw);

    static Vector6d TransPlkLineByPose(Vector6d plk_w, Eigen::Matrix3d Rcw, Eigen::Vector3d tcw );
    static Vector6d TransInvPlkLineFromPose(Vector6d plk_c, Eigen::Matrix3d Rcw, Eigen::Vector3d tcw );

    static Vector6d TrimLine(const Eigen::Vector3d &spt, const Eigen::Vector3d &ept, const Vector6d &plk);
};

}// namespace ORB_SLAM

#endif // GEOMETRIC_TOOLS_H
