//
// This file is part of UL-SLAM based on ORB-SLAM3
//

#ifndef HYBRID_VP_VPCOMPUTE_H
#define HYBRID_VP_VPCOMPUTE_H
#include <vector>
#include <iostream>
#include <map>
#include <tuple>
#include <cstring>
#include <cstdlib>
#include <ctime>

#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "Thirdparty/line_descriptor/include/line_descriptor_custom.hpp"
#include "GeometricCamera.h"

namespace SampleCommon{
    void d1_cal(float* aux, float* rot_axis, float* params_d1);

    void GenerateBin(const std::vector<float*>& all_2D_lines,
                     std::vector<int>& largest_bin_idxes, std::vector<int>& remain_bins_idxes, int flag);

    int ComputeIterTimes(float conf, int N1, int N2, int K, float r, int mode);


    int nchoosek(int n, int k);
}

class Sample {

public:
    void RunSampleByTwoOrthoLine(const std::vector<float*>& all_projPlane_normals, const std::vector<float*>& all_2D_lines,
                                 std::vector<float*>& all_tree_params);

    void SampleLinesOrtho(const std::vector<float*>& all_2D_lines, std::vector<std::pair<int, int>>& sample_pair_ids);

    void ParametrizeMFByTwoOrthoLine(float* s2, float* e2, float* n1, float* aux, float* params);

    void d2_cal_ortho(float* params_d1, float* e1, float* e2, float* params_d2);

    void d3_cal_ortho(float* params_d1, float* params_d2, float* params_d3);


    void RunSampleByTwoParalLine(const std::vector<float*>& all_projPlane_normals, const std::vector<float*>& all_2D_lines,
                                 std::vector<float*>& all_tree_params);

    void SampleLinesParal(const std::vector<float*>& all_2D_lines, std::vector<std::pair<int, int>>& sample_pair_ids);

    void ParametrizeMFByTwoParalLine(float* n1, float* n2, float* params);

    void d2_cal_paral(float* d2_, float* d3_, float* params_d2);

    void d3_cal_paral(float* d2_, float* d3_, float* params_d3);

};

namespace SearchCommon
{
    void bounds_by_aux_angle_lam(float a, float b, float* interval_lam, int label, float* bounds);
}

class SearchOrth {

public:
    bool RunSearchByTwoOrthoLine(const std::vector<float*>& all_tree_params, const std::vector<float*>& all_projPlane_normals, float error_thre,
                                 float* MF);

    void SearchOneTree(std::vector<float*>& candidateIntervals, std::vector<int>& lb_cards, std::vector<int>& ub_cards,
                       float* one_tree_params, const std::vector<float*>& all_projPlane_normals, float error_thre);

    void DivideParentInterval(int parentIdx, float* parentInterval,
                              float* params, const std::vector<float*>& all_projPlane_normals,
                              std::vector<float*>& candidateIntervals, std::vector<int>& lb_cards, std::vector<int>& ub_cards,
                              float error_thre);

    void ComputeBounds(float* params, float* lam_interval, float* projPlaneNormal, float error_thre,
                       int* numInlier_bounds);

    void error_denominator_bound(float* params, float* lam_interval,
                                 float* norm_bound_dd2_);

    void error_numerator_bound(float* params, float* orig_interval, float* projPlaneNormal,
                               float* bounds_wrt_lam);

    int ComputeCard_midPt(float* params, float mid_lam, const std::vector<float*>& all_projPlane_normals, float error_thre);

    void ComputeMFfromLam(float* params, float lam, float* MF);
};

class SearchParal {

public:
    bool RunSearchByTwoParalLine(const std::vector<float*>& all_tree_params, const std::vector<float*>& all_projPlane_normals, float error_thre, float* MF);

    void SearchOneTree(std::vector<float*>& candidateIntervals, std::vector<int>& lb_cards, std::vector<int>& ub_cards,
                       float* one_tree_params, const std::vector<float*>& all_projPlane_normals, float error_thre);

    void DivideParentInterval(int parentIdx, float* parentInterval,
                              float* params, const std::vector<float*>& all_projPlane_normals,
                              std::vector<float*>& candidateIntervals, std::vector<int>& lb_cards, std::vector<int>& ub_cards,
                              float error_thre);

    void ComputeBounds(float* params, float* lam_interval, float* projPlaneNormal, float error_thre, int* numInlier_bounds);

    void error_numerator_bound(float* params, float* orig_interval, float* projPlaneNormal, float* bounds_wrt_lam);

    int ComputeCard_midPt(float* params, float mid_lam, const std::vector<float*>& all_projPlane_normals, float error_thre);

    void ComputeMFfromLam(float* params, float lam, float* MF);
};

class VPCompute {
public:
    void Compute2dPlaneNormal(std::vector<float*>& all_2D_lines_norm, std::vector<float*>& all_projPlane_normals, ORB_SLAM3::GeometricCamera* pCamera, const std::vector<cv::line_descriptor::KeyLine>& keyLinesCur);
    void NormalizePt(const Eigen::Matrix<float, 3, 3>& K, const cv::Point2f& ordPt, float* normalizedPt);
    void Lines2Vps(const std::vector<cv::line_descriptor::KeyLine> cur_keyLine, double thAngle, std::vector<Eigen::Vector3d> &vps,
                   std::vector<std::vector<int>> &clusLineByVpId, std::vector<int> &lineToVpId, ORB_SLAM3::GeometricCamera* mpCamera);

    void VisualizeClusterLine(cv::Mat &inputImg, std::vector<cv::line_descriptor::KeyLine> &lines, std::vector<std::vector<int>> &clusters);

    bool RunVpCompute(double angle_threshold_deg, const cv::Mat& img, const std::vector<cv::line_descriptor::KeyLine>& cur_keyLine, ORB_SLAM3::GeometricCamera* pCamera);

public:
    std::vector<std::vector<int>> clusters;
    std::vector<int> local_vp_ids;
    std::vector<Eigen::Vector3d> vps;
};


#endif //HYBRID_VP_VPCOMPUTE_H
