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

#ifndef G2OTYPESLINE_H
#define G2OTYPESLINE_H

#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "Thirdparty/g2o/g2o/types/types_sba.h"
#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"

#include <opencv2/core/core.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <Frame.h>
#include <KeyFrame.h>

#include "Converter.h"
#include <math.h>

namespace ORB_SLAM3
{

class KeyFrame;
class Frame;
class GeometricCamera;

class VertexPlukerLine : public g2o::BaseVertex<4, Eigen::Vector4d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexPlukerLine() : g2o::BaseVertex<4,Eigen::Vector4d>() {};
    virtual bool read(std::istream& is) { return true; };
    virtual bool write(std::ostream& os) const { return true; };

    virtual void setToOriginImpl() {
        _estimate.fill(0.);
    }

    virtual void oplusImpl(const double* update)
    {
        Eigen::Map<const Eigen::Vector4d> delta(update);
        Eigen::Vector4d plusD;
        updateOrthCoord(_estimate, delta, plusD);
        _estimate = plusD;
        updateCache();
    }

    inline void updateOrthCoord(const Eigen::Vector4d& D, const Eigen::Vector4d& deltaD, Eigen::Vector4d& plusD){
        // ref: 2001, Adrien Bartol,Peter Sturm ,Structure-From-Motion Using Lines: Representation, Triangulation and Bundle Adjustment

        // theta --> U,  phi --> W
        Eigen::Vector3d theta = D.head(3);

        double phi = D(3);
        //Eigen::Vector3d theta = orth.head(3);
        //double phi = orth[3];
        double s1 = sin(theta[0]);
        double c1 = cos(theta[0]);
        double s2 = sin(theta[1]);
        double c2 = cos(theta[1]);
        double s3 = sin(theta[2]);
        double c3 = cos(theta[2]);
        Eigen::Matrix3d R;
        R <<
            c2 * c3,   s1 * s2 * c3 - c1 * s3,   c1 * s2 * c3 + s1 * s3,
            c2 * s3,   s1 * s2 * s3 + c1 * c3,   c1 * s2 * s3 - s1 * c3,
            -s2,                  s1 * c2,                  c1 * c2;
        double w1 = cos(phi);
        double w2 = sin(phi);

        // update
        Eigen::Vector3d _delta_theta = deltaD.head(3);
        double _delta_phi = deltaD(3);
        Eigen::Matrix3d Rz;
        Rz << cos(_delta_theta(2)), -sin(_delta_theta(2)), 0,
            sin(_delta_theta(2)), cos(_delta_theta(2)), 0,
            0, 0, 1;

        Eigen::Matrix3d Ry;
        Ry << cos(_delta_theta(1)), 0., sin(_delta_theta(1)),
            0., 1., 0.,
            -sin(_delta_theta(1)), 0., cos(_delta_theta(1));

        Eigen::Matrix3d Rx;
        Rx << 1., 0., 0.,
            0., cos(_delta_theta(0)), -sin(_delta_theta(0)),
            0., sin(_delta_theta(0)), cos(_delta_theta(0));
        R = R * Rx * Ry * Rz;

        Eigen::Matrix2d W;
        W << w1, -w2, w2, w1;
        Eigen::Matrix2d delta_W;
        delta_W << cos(_delta_phi), -sin(_delta_phi),sin(_delta_phi), cos(_delta_phi);
        W = W * delta_W;

        // U' -- > theta'. W' --> phi'

        Eigen::Vector3d u1 = R.col(0);
        Eigen::Vector3d u2 = R.col(1);
        Eigen::Vector3d u3 = R.col(2);
        plusD[0] = atan2( u2(2),u3(2) );
        plusD[1] = asin( -u1(2) );
        plusD[2] = atan2( u1(1),u1(0) );

        plusD[3] = asin( W(1,0) );
    }
};

class EdgeMonoLine : public g2o::BaseBinaryEdge<2, Eigen::Vector4d, VertexPlukerLine, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeMonoLine() : g2o::BaseBinaryEdge<2, Eigen::Vector4d, VertexPlukerLine, g2o::VertexSE3Expmap>() {}

    bool read(std::istream &is) { return true; }

    bool write(std::ostream &os) const { return true; }

    void SetParams(const double &fx_, const double &fy_, const double &cx_, const double &cy_) {
        fx = fx_;
        fy = fy_;
        cx = cx_;
        cy = cy_;
    }

    void computeError() {
        const VertexPlukerLine* vLine = dynamic_cast<const VertexPlukerLine* >(_vertices[0]);
        const g2o::VertexSE3Expmap* vPose = dynamic_cast<const g2o::VertexSE3Expmap* >(_vertices[1]);

        const Eigen::Vector4d obs(_measurement);

        const Matrix4d Tcw = vPose->estimate().to_homogeneous_matrix();
        const Eigen::Vector4d Lw = vLine->estimate();

        Vector6d plukerLw = changeOrthToPluker(Lw);
        Vector6d plukerLc = getTransformMatrix(Tcw) * plukerLw;

        Eigen::Vector3d plukerLcPixel = getPlukerK() * plukerLc.head(3);
        double lx = plukerLcPixel(0);
        double ly = plukerLcPixel(1);
        double lz = plukerLcPixel(2);

        double fenmu = sqrt( lx*lx + ly*ly);

        Eigen::Vector2d error;
        error(0) = ( lx * obs(0) + ly * obs(1) + lz ) / fenmu;
        error(1) = ( lx * obs(2) + ly * obs(3) + lz ) / fenmu;

        _error = error;
    }

    // check pass
    Eigen::Matrix3d getPlukerK(){
        Eigen::Matrix3d plukerK;
        plukerK << fy,     0,      0,
            0,      fx,     0,
            -fy*cx, -fx*cy, fx*fy;
        return plukerK;
    }

    Matrix6d getTransformMatrix(const Matrix4d& T){
        Matrix6d temp;
        temp.setZero();
        temp.block<3,3>(0,0) = T.block<3,3>(0,0);
        temp.block<3,3>(0,3) = Skew(T.block<3,1>(0,3)) * T.block<3,3>(0,0);
        temp.block<3,3>(3,3) = T.block<3,3>(0,0);

        return temp;
    }

    Vector6d changeOrthToPluker(const Eigen::Vector4d& orthLine ){
        double s1 = sin(orthLine[0]);
        double c1 = cos(orthLine[0]);
        double s2 = sin(orthLine[1]);
        double c2 = cos(orthLine[1]);
        double s3 = sin(orthLine[2]);
        double c3 = cos(orthLine[2]);
        Eigen::Matrix3d R;
        R <<
            c2 * c3,   s1 * s2 * c3 - c1 * s3,   c1 * s2 * c3 + s1 * s3,
            c2 * s3,   s1 * s2 * s3 + c1 * c3,   c1 * s2 * s3 - s1 * c3,
            -s2,                  s1 * c2,                  c1 * c2;

        double w1 = cos(orthLine[3]);
        double w2 = sin(orthLine[3]);

        Vector6d plukerLine;
        plukerLine.head(3) = w1 * R.col(0);
        plukerLine.tail(3) = w2 * R.col(1);
        return plukerLine;
    }

    virtual void linearizeOplus() {
        const VertexPlukerLine* vLine = dynamic_cast<const VertexPlukerLine* >(_vertices[0]);
        const g2o::VertexSE3Expmap* vPose = dynamic_cast<const g2o::VertexSE3Expmap* >(_vertices[1]);

        const Eigen::Vector4d obs(_measurement);

        const Eigen::Matrix4d Tcw = vPose->estimate().to_homogeneous_matrix();
        const Eigen::Vector4d Lw = vLine->estimate();

        const Eigen::Matrix3d Rcw = Tcw.block<3,3>(0,0);
        const Eigen::Vector3d Pcw = Tcw.block<3,1>(0,3);

        Vector6d plukerLw = changeOrthToPluker(Lw);
        Vector6d plukerLc = getTransformMatrix(Tcw) * plukerLw;

        Eigen::Vector3d plukerLcPixel = getPlukerK() * plukerLc.head(3);
        double lx = plukerLcPixel(0);
        double ly = plukerLcPixel(1);
        double lz = plukerLcPixel(2);
        double fenmu = sqrt(lx*lx + ly*ly);

        Eigen::Vector2d error;
        error(0) = ( lx * obs(0) + ly * obs(1) + lz ) / fenmu;
        error(1) = ( lx * obs(2) + ly * obs(3) + lz ) / fenmu;

        // check pass
        Matrix<double,1,3> jac_err0_lcPixel;
        jac_err0_lcPixel << -lx * error(0) / (fenmu*fenmu) + obs(0) /fenmu,
            -ly * error(0) / (fenmu*fenmu) + obs(1) /fenmu,
            1.0 / fenmu;
        Matrix<double,1,3> jac_err1_lcPixel;
        jac_err1_lcPixel << -lx * error(1) / (fenmu*fenmu) + obs(2) /fenmu,
            -ly * error(1) / (fenmu*fenmu) + obs(3) /fenmu,
            1.0 / fenmu;

        // check pass
        Matrix<double,3,6> jac_lcPixel_lc;
        jac_lcPixel_lc.setZero();
        jac_lcPixel_lc.block<3,3>(0,0) = getPlukerK();

        // check pass
        Matrix<double,6,6> jac_lc_lw;
        jac_lc_lw = getTransformMatrix(Tcw);

        Eigen::Matrix3d U = getOrhtRFromPluker(plukerLw);
        Matrix2d W = getOrthWFromPluker(plukerLw);

        Matrix<double,6,4> jac_lw_orth;
        jac_lw_orth = jacobianFromPlukerToOrth(U, W);

        Matrix<double,2,4> jac_error_orth;
        jac_error_orth.block<1,4>(0,0) = jac_err0_lcPixel * jac_lcPixel_lc * jac_lc_lw * jac_lw_orth;
        jac_error_orth.block<1,4>(1,0) = jac_err1_lcPixel * jac_lcPixel_lc * jac_lc_lw * jac_lw_orth;
        _jacobianOplusXi = jac_error_orth;

        Matrix<double,6,6> jac_lc_rt;
        jac_lc_rt.setZero();
        jac_lc_rt.block<3,3>(0,0) = - Sophus::SO3d::hat(Rcw*plukerLw.head(3)) - Sophus::SO3d::hat(Sophus::SO3d::hat(Pcw)*Rcw*plukerLw.tail(3));
        jac_lc_rt.block<3,3>(3,0) = - Sophus::SO3d::hat(Rcw*plukerLw.tail(3));
        jac_lc_rt.block<3,3>(0,3) = - Sophus::SO3d::hat(Rcw*plukerLw.tail(3));
        
        Matrix<double,2,6> jac_error_pose;
        jac_error_pose.block<1,6>(0,0) = jac_err0_lcPixel * jac_lcPixel_lc * jac_lc_rt;
        jac_error_pose.block<1,6>(1,0) = jac_err1_lcPixel * jac_lcPixel_lc * jac_lc_rt;
        _jacobianOplusXj = jac_error_pose;

    }

    bool isDepthPositive(MapLine* pML, KeyFrame* curKeyFrame) {
        const g2o::VertexSE3Expmap* vPose = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const VertexPlukerLine* vLine = static_cast<const VertexPlukerLine* >(_vertices[0]);
        Vector6d plukerLw = changeOrthToPluker(vLine->estimate());
        map<KeyFrame*, size_t> observations;
        Vector6f plukerPosW;
        {
            observations = pML->mObservations;
            plukerPosW = plukerLw.cast<float>();
        }

        KeyFrame* pKF = curKeyFrame;
        GeometricCamera* pCamera1 = pKF->mpCamera;
        const line_descriptor::KeyLine& kl1 = pKF->mvKeyLinesUn[observations[pKF]];

        Vector6d plukerLc1 = GeometricTools::TransPlkLineByPose(plukerPosW.cast<double>(), pKF->GetPose().cast<double>().rotationMatrix(), pKF->GetPose().cast<double>().translation());

        cv::Point2f sp1 = cv::Point2f(kl1.startPointX, kl1.startPointY);
        cv::Point2f ep1 = cv::Point2f(kl1.endPointX, kl1.endPointY);

        Eigen::Vector3d sp1_c = pCamera1->unprojectEig(sp1).cast<double>();
        Eigen::Vector3d ep1_c = pCamera1->unprojectEig(ep1).cast<double>();

        Vector6d line3Dc = GeometricTools::TrimLine(sp1_c, ep1_c, plukerLc1);
        Eigen::Vector3d csp = line3Dc.head(3);
        Eigen::Vector3d cep = line3Dc.tail(3);
        if (csp(2) <= 0 || cep(2) <= 0) {
            return false;
        }

        return true;
    }

    Matrix<double,6,4> jacobianFromPlukerToOrth(const Eigen::Matrix3d& u, const Matrix2d& w){
        double w1 = w(0,0);
        double w2 = w(1,0);
        Eigen::Vector3d u1 = u.col(0);
        Eigen::Vector3d u2 = u.col(1);
        Eigen::Vector3d u3 = u.col(2);
        Matrix<double,6,4> temp;
        temp.setZero();
        temp.block<3,1>(0,1) = -w1 * u3;
        temp.block<3,1>(0,2) = w1 * u2;
        temp.block<3,1>(0,3) = -w2 * u1;
        temp.block<3,1>(3,0) = w2 * u3;
        temp.block<3,1>(3,2) = -w2 * u1;
        temp.block<3,1>(3,3) = w1 * u2;
        return temp;
    }

    Matrix2d getOrthWFromPluker(const Vector6d& plukerLine)
    {
        Matrix2d temp;
        double nnorm = plukerLine.head(3).norm();
        double dnorm = plukerLine.tail(3).norm();
        double fenmu = sqrt(nnorm*nnorm + dnorm*dnorm);
        temp << nnorm / fenmu, -dnorm / fenmu, dnorm / fenmu, nnorm / fenmu;
        return temp;
    }

    Eigen::Matrix3d getOrhtRFromPluker(const Vector6d &plukerLine)
    {
        Eigen::Matrix3d R;
        Eigen::Vector3d n = plukerLine.head(3);
        Eigen::Vector3d d = plukerLine.tail(3);
        Eigen::Vector3d n0 = n;
        Eigen::Vector3d d0 = d;
        n.normalize();
        d.normalize();
        R.col(0) = n;
        R.col(1) = d;
        R.col(2) = n0.cross(d0) / (n0.cross(d0).norm());
        return R;
    }
    GeometricCamera* pCamera;

protected:
    double fx,fy,cx,cy;
};

class EdgeMonoVpLinePar : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexPlukerLine, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeMonoVpLinePar(Eigen::Vector3d obsVp) : obsVp_(obsVp){}

    bool read(std::istream &is) { return true; }

    bool write(std::ostream &os) const { return true; }

    void SetParams(const double &fx_, const double &fy_, const double &cx_, const double &cy_) {
        fx = fx_;
        fy = fy_;
        cx = cx_;
        cy = cy_;
    }

    void computeError() {
        const VertexPlukerLine* vLine = dynamic_cast<const VertexPlukerLine* >(_vertices[0]);
        const g2o::VertexSE3Expmap* vPose = dynamic_cast<const g2o::VertexSE3Expmap* >(_vertices[1]);

        const Eigen::Vector3d obs(_measurement);

        // transform se3 to a eigen matrix
        const Matrix4d Tcw = vPose->estimate().to_homogeneous_matrix();
        const Eigen::Vector4d Lw = vLine->estimate();

        Vector6d plukerLw = changeOrthToPluker(Lw);
        Vector6d plukerLc = getTransformMatrix(Tcw) * plukerLw;

        Eigen::Vector3d v_c = plukerLc.tail(3);

        Eigen::Vector3d v_c_norm = v_c / v_c.z();
        // vp loss
        double parAngleError = ComputeTwoLineAngleCam(v_c_norm, obsVp_);
        Eigen::Vector3d error = Eigen::Vector3d::Zero();
        error(0) = 1.0 - parAngleError;
        _error = error;
    }

    // check pass
    Eigen::Matrix3d getPlukerK(){
        Eigen::Matrix3d plukerK;
        plukerK << fy,     0,      0,
            0,      fx,     0,
            -fy*cx, -fx*cy, fx*fy;
        return plukerK;
    }

    Matrix6d getTransformMatrix(const Matrix4d& T){
        Matrix6d temp;
        temp.setZero();
        temp.block<3,3>(0,0) = T.block<3,3>(0,0);
        temp.block<3,3>(0,3) = Skew(T.block<3,1>(0,3)) * T.block<3,3>(0,0);
        temp.block<3,3>(3,3) = T.block<3,3>(0,0);

        return temp;
    }

    Vector6d changeOrthToPluker(const Eigen::Vector4d& orthLine ){
        double s1 = sin(orthLine[0]);
        double c1 = cos(orthLine[0]);
        double s2 = sin(orthLine[1]);
        double c2 = cos(orthLine[1]);
        double s3 = sin(orthLine[2]);
        double c3 = cos(orthLine[2]);
        Eigen::Matrix3d R;
        R <<
            c2 * c3,   s1 * s2 * c3 - c1 * s3,   c1 * s2 * c3 + s1 * s3,
            c2 * s3,   s1 * s2 * s3 + c1 * c3,   c1 * s2 * s3 - s1 * c3,
            -s2,                  s1 * c2,                  c1 * c2;

        double w1 = cos(orthLine[3]);
        double w2 = sin(orthLine[3]);

        Vector6d plukerLine;
        plukerLine.head(3) = w1 * R.col(0);
        plukerLine.tail(3) = w2 * R.col(1);
        return plukerLine;
    }

    bool isDepthPositive(MapLine* pML, KeyFrame* curKeyFrame) {
        const g2o::VertexSE3Expmap* vPose = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const VertexPlukerLine* vLine = static_cast<const VertexPlukerLine* >(_vertices[0]);
        Vector6d plukerLw = changeOrthToPluker(vLine->estimate());
        map<KeyFrame*, size_t> observations;
        Vector6f plukerPosW;
        {
            observations = pML->mObservations;
            plukerPosW = plukerLw.cast<float>();
        }

        KeyFrame* pKF = curKeyFrame;
        GeometricCamera* pCamera1 = pKF->mpCamera;
        const line_descriptor::KeyLine& kl1 = pKF->mvKeyLinesUn[observations[pKF]];

        Vector6d plukerLc1 = GeometricTools::TransPlkLineByPose(plukerPosW.cast<double>(), pKF->GetPose().cast<double>().rotationMatrix(), pKF->GetPose().cast<double>().translation());

        cv::Point2f sp1 = cv::Point2f(kl1.startPointX, kl1.startPointY);
        cv::Point2f ep1 = cv::Point2f(kl1.endPointX, kl1.endPointY);

        Eigen::Vector3d sp1_c = pCamera1->unprojectEig(sp1).cast<double>();
        Eigen::Vector3d ep1_c = pCamera1->unprojectEig(ep1).cast<double>();

        Vector6d line3Dc = GeometricTools::TrimLine(sp1_c, ep1_c, plukerLc1);
        Eigen::Vector3d csp = line3Dc.head(3);
        Eigen::Vector3d cep = line3Dc.tail(3);
        if (csp(2) <= 0 || cep(2) <= 0) {
            return false;
        }

        return true;
    }

    Matrix<double,6,4> jacobianFromPlukerToOrth(const Eigen::Matrix3d& u, const Matrix2d& w){
        double w1 = w(0,0);
        double w2 = w(1,0);
        Eigen::Vector3d u1 = u.col(0);
        Eigen::Vector3d u2 = u.col(1);
        Eigen::Vector3d u3 = u.col(2);
        Matrix<double,6,4> temp;
        temp.setZero();
        temp.block<3,1>(0,1) = -w1 * u3;
        temp.block<3,1>(0,2) = w1 * u2;
        temp.block<3,1>(0,3) = -w2 * u1;
        temp.block<3,1>(3,0) = w2 * u3;
        temp.block<3,1>(3,2) = -w2 * u1;
        temp.block<3,1>(3,3) = w1 * u2;
        return temp;
    }

    Matrix2d getOrthWFromPluker(const Vector6d& plukerLine)
    {
        Matrix2d temp;
        double nnorm = plukerLine.head(3).norm();
        double dnorm = plukerLine.tail(3).norm();
        double fenmu = sqrt(nnorm*nnorm + dnorm*dnorm);
        temp << nnorm / fenmu, -dnorm / fenmu, dnorm / fenmu, nnorm / fenmu;
        return temp;
    }

    Eigen::Matrix3d getOrhtRFromPluker(const Vector6d &plukerLine)
    {
        Eigen::Matrix3d R;
        Eigen::Vector3d n = plukerLine.head(3);
        Eigen::Vector3d d = plukerLine.tail(3);
        Eigen::Vector3d n0 = n;
        Eigen::Vector3d d0 = d;
        n.normalize();
        d.normalize();
        R.col(0) = n;
        R.col(1) = d;
        R.col(2) = n0.cross(d0) / (n0.cross(d0).norm());
        return R;
    }

    double ComputeTwoLineAngleCam(const Vector3d &l_a, const Vector3d &l_b)
    {
        double dot_product = l_b.dot(l_a);

        // Find magnitude of lines
        double magn_a = l_b.norm();
        double magn_b = l_a.norm();

        // Find the cosine of the angle formed
        return abs(dot_product / (magn_a * magn_b));
    }

    GeometricCamera* pCamera;

protected:
    double fx,fy,cx,cy;
    Eigen::Vector3d obsVp_;
};

class EdgeMonoVpOnlyLinePar : public g2o::BaseBinaryEdge<1, double, VertexPlukerLine, VertexPlukerLine>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeMonoVpOnlyLinePar(){}

    bool read(std::istream &is) { return true; }

    bool write(std::ostream &os) const { return true; }

    void SetParams(const double &fx_, const double &fy_, const double &cx_, const double &cy_) {
        fx = fx_;
        fy = fy_;
        cx = cx_;
        cy = cy_;
    }

    void computeError() {
        const VertexPlukerLine* vLine1 = dynamic_cast<const VertexPlukerLine* >(_vertices[0]);
        const VertexPlukerLine* vLine2 = dynamic_cast<const VertexPlukerLine* >(_vertices[1]);

        const double obs(_measurement);

        // transform se3 to a eigen matrix
        const Eigen::Vector4d Lw1 = vLine1->estimate();
        Vector6d plukerLw1 = changeOrthToPluker(Lw1);

        const Eigen::Vector4d Lw2 = vLine2->estimate();
        Vector6d plukerLw2 = changeOrthToPluker(Lw2);

        Eigen::Vector3d v_w1 = Lw1.tail(3);
        Eigen::Vector3d v_w1_norm = v_w1 / v_w1.z();

        Eigen::Vector3d v_w2 = Lw2.tail(3);
        Eigen::Vector3d v_w2_norm = v_w2 / v_w2.z();

        // vp loss
        double parAngleError = ComputeTwoLineAngleCam(v_w1_norm, v_w2_norm);
        double error = 1.0 - parAngleError;
        _error[0] = error;
    }

    Vector6d changeOrthToPluker(const Eigen::Vector4d& orthLine ){
        double s1 = sin(orthLine[0]);
        double c1 = cos(orthLine[0]);
        double s2 = sin(orthLine[1]);
        double c2 = cos(orthLine[1]);
        double s3 = sin(orthLine[2]);
        double c3 = cos(orthLine[2]);
        Eigen::Matrix3d R;
        R <<
            c2 * c3,   s1 * s2 * c3 - c1 * s3,   c1 * s2 * c3 + s1 * s3,
            c2 * s3,   s1 * s2 * s3 + c1 * c3,   c1 * s2 * s3 - s1 * c3,
            -s2,                  s1 * c2,                  c1 * c2;

        double w1 = cos(orthLine[3]);
        double w2 = sin(orthLine[3]);

        Vector6d plukerLine;
        plukerLine.head(3) = w1 * R.col(0);
        plukerLine.tail(3) = w2 * R.col(1);
        return plukerLine;
    }

    bool isDepthPositive(MapLine* pML, KeyFrame* curKeyFrame) {
        const g2o::VertexSE3Expmap* vPose = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const VertexPlukerLine* vLine = static_cast<const VertexPlukerLine* >(_vertices[0]);
        Vector6d plukerLw = changeOrthToPluker(vLine->estimate());
        map<KeyFrame*, size_t> observations;
        Vector6f plukerPosW;
        {
            observations = pML->mObservations;
            plukerPosW = plukerLw.cast<float>();
        }

        KeyFrame* pKF = curKeyFrame;
        GeometricCamera* pCamera1 = pKF->mpCamera;
        const line_descriptor::KeyLine& kl1 = pKF->mvKeyLinesUn[observations[pKF]];

        Vector6d plukerLc1 = GeometricTools::TransPlkLineByPose(plukerPosW.cast<double>(), pKF->GetPose().cast<double>().rotationMatrix(), pKF->GetPose().cast<double>().translation());

        cv::Point2f sp1 = cv::Point2f(kl1.startPointX, kl1.startPointY);
        cv::Point2f ep1 = cv::Point2f(kl1.endPointX, kl1.endPointY);

        Eigen::Vector3d sp1_c = pCamera1->unprojectEig(sp1).cast<double>();
        Eigen::Vector3d ep1_c = pCamera1->unprojectEig(ep1).cast<double>();

        Vector6d line3Dc = GeometricTools::TrimLine(sp1_c, ep1_c, plukerLc1);
        Eigen::Vector3d csp = line3Dc.head(3);
        Eigen::Vector3d cep = line3Dc.tail(3);
        if (csp(2) <= 0 || cep(2) <= 0) {
            return false;
        }

        return true;
    }

    Matrix<double,6,4> jacobianFromPlukerToOrth(const Eigen::Matrix3d& u, const Matrix2d& w){
        double w1 = w(0,0);
        double w2 = w(1,0);
        Eigen::Vector3d u1 = u.col(0);
        Eigen::Vector3d u2 = u.col(1);
        Eigen::Vector3d u3 = u.col(2);
        Matrix<double,6,4> temp;
        temp.setZero();
        temp.block<3,1>(0,1) = -w1 * u3;
        temp.block<3,1>(0,2) = w1 * u2;
        temp.block<3,1>(0,3) = -w2 * u1;
        temp.block<3,1>(3,0) = w2 * u3;
        temp.block<3,1>(3,2) = -w2 * u1;
        temp.block<3,1>(3,3) = w1 * u2;
        return temp;
    }

    Matrix2d getOrthWFromPluker(const Vector6d& plukerLine)
    {
        Matrix2d temp;
        double nnorm = plukerLine.head(3).norm();
        double dnorm = plukerLine.tail(3).norm();
        double fenmu = sqrt(nnorm*nnorm + dnorm*dnorm);
        temp << nnorm / fenmu, -dnorm / fenmu, dnorm / fenmu, nnorm / fenmu;
        return temp;
    }

    Eigen::Matrix3d getOrhtRFromPluker(const Vector6d &plukerLine)
    {
        Eigen::Matrix3d R;
        Eigen::Vector3d n = plukerLine.head(3);
        Eigen::Vector3d d = plukerLine.tail(3);
        Eigen::Vector3d n0 = n;
        Eigen::Vector3d d0 = d;
        n.normalize();
        d.normalize();
        R.col(0) = n;
        R.col(1) = d;
        R.col(2) = n0.cross(d0) / (n0.cross(d0).norm());
        return R;
    }

    double ComputeTwoLineAngleCam(const Vector3d &l_a, const Vector3d &l_b)
    {
        double dot_product = l_b.dot(l_a);

        // Find magnitude of lines
        double magn_a = l_b.norm();
        double magn_b = l_a.norm();

        // Find the cosine of the angle formed
        return abs(dot_product / (magn_a * magn_b));
    }

    GeometricCamera* pCamera;

protected:
    double fx,fy,cx,cy;
};

// add vanishing point perp constrain
class EdgeMonoVpOnlyLinePerp : public g2o::BaseBinaryEdge<1, double, VertexPlukerLine, VertexPlukerLine>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeMonoVpOnlyLinePerp(){}

    bool read(std::istream &is) { return true; }

    bool write(std::ostream &os) const { return true; }

    void SetParams(const double &fx_, const double &fy_, const double &cx_, const double &cy_) {
        fx = fx_;
        fy = fy_;
        cx = cx_;
        cy = cy_;
    }

    void computeError() {
        const VertexPlukerLine* vLine1 = dynamic_cast<const VertexPlukerLine* >(_vertices[0]);
        const VertexPlukerLine* vLine2 = dynamic_cast<const VertexPlukerLine* >(_vertices[1]);

        const double obs(_measurement);

        // transform se3 to a eigen matrix
        const Eigen::Vector4d Lw1 = vLine1->estimate();
        Vector6d plukerLw1 = changeOrthToPluker(Lw1);

        const Eigen::Vector4d Lw2 = vLine2->estimate();
        Vector6d plukerLw2 = changeOrthToPluker(Lw2);

        Eigen::Vector3d v_w1 = Lw1.tail(3);
        Eigen::Vector3d v_w1_norm = v_w1 / v_w1.z();

        Eigen::Vector3d v_w2 = Lw2.tail(3);
        Eigen::Vector3d v_w2_norm = v_w2 / v_w2.z();

        // vp loss
        double perpAngleError = ComputeTwoLineAngleCam(v_w1_norm, v_w2_norm);
        double error = perpAngleError;
        _error[0] = error;
    }

    Vector6d changeOrthToPluker(const Eigen::Vector4d& orthLine ){
        double s1 = sin(orthLine[0]);
        double c1 = cos(orthLine[0]);
        double s2 = sin(orthLine[1]);
        double c2 = cos(orthLine[1]);
        double s3 = sin(orthLine[2]);
        double c3 = cos(orthLine[2]);
        Eigen::Matrix3d R;
        R <<
            c2 * c3,   s1 * s2 * c3 - c1 * s3,   c1 * s2 * c3 + s1 * s3,
            c2 * s3,   s1 * s2 * s3 + c1 * c3,   c1 * s2 * s3 - s1 * c3,
            -s2,                  s1 * c2,                  c1 * c2;

        double w1 = cos(orthLine[3]);
        double w2 = sin(orthLine[3]);

        Vector6d plukerLine;
        plukerLine.head(3) = w1 * R.col(0);
        plukerLine.tail(3) = w2 * R.col(1);
        return plukerLine;
    }

    bool isDepthPositive(MapLine* pML, KeyFrame* curKeyFrame) {
        const g2o::VertexSE3Expmap* vPose = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const VertexPlukerLine* vLine = static_cast<const VertexPlukerLine* >(_vertices[0]);
        Vector6d plukerLw = changeOrthToPluker(vLine->estimate());
        map<KeyFrame*, size_t> observations;
        Vector6f plukerPosW;
        {
            observations = pML->mObservations;
            plukerPosW = plukerLw.cast<float>();
        }

        KeyFrame* pKF = curKeyFrame;
        GeometricCamera* pCamera1 = pKF->mpCamera;
        const line_descriptor::KeyLine& kl1 = pKF->mvKeyLinesUn[observations[pKF]];

        Vector6d plukerLc1 = GeometricTools::TransPlkLineByPose(plukerPosW.cast<double>(), pKF->GetPose().cast<double>().rotationMatrix(), pKF->GetPose().cast<double>().translation());

        cv::Point2f sp1 = cv::Point2f(kl1.startPointX, kl1.startPointY);
        cv::Point2f ep1 = cv::Point2f(kl1.endPointX, kl1.endPointY);

        Eigen::Vector3d sp1_c = pCamera1->unprojectEig(sp1).cast<double>();
        Eigen::Vector3d ep1_c = pCamera1->unprojectEig(ep1).cast<double>();

        Vector6d line3Dc = GeometricTools::TrimLine(sp1_c, ep1_c, plukerLc1);
        Eigen::Vector3d csp = line3Dc.head(3);
        Eigen::Vector3d cep = line3Dc.tail(3);
        if (csp(2) <= 0 || cep(2) <= 0) {
            return false;
        }

        return true;
    }

    Matrix<double,6,4> jacobianFromPlukerToOrth(const Eigen::Matrix3d& u, const Matrix2d& w){
        double w1 = w(0,0);
        double w2 = w(1,0);
        Eigen::Vector3d u1 = u.col(0);
        Eigen::Vector3d u2 = u.col(1);
        Eigen::Vector3d u3 = u.col(2);
        Matrix<double,6,4> temp;
        temp.setZero();
        temp.block<3,1>(0,1) = -w1 * u3;
        temp.block<3,1>(0,2) = w1 * u2;
        temp.block<3,1>(0,3) = -w2 * u1;
        temp.block<3,1>(3,0) = w2 * u3;
        temp.block<3,1>(3,2) = -w2 * u1;
        temp.block<3,1>(3,3) = w1 * u2;
        return temp;
    }

    Matrix2d getOrthWFromPluker(const Vector6d& plukerLine)
    {
        Matrix2d temp;
        double nnorm = plukerLine.head(3).norm();
        double dnorm = plukerLine.tail(3).norm();
        double fenmu = sqrt(nnorm*nnorm + dnorm*dnorm);
        temp << nnorm / fenmu, -dnorm / fenmu, dnorm / fenmu, nnorm / fenmu;
        return temp;
    }

    Eigen::Matrix3d getOrhtRFromPluker(const Vector6d &plukerLine)
    {
        Eigen::Matrix3d R;
        Eigen::Vector3d n = plukerLine.head(3);
        Eigen::Vector3d d = plukerLine.tail(3);
        Eigen::Vector3d n0 = n;
        Eigen::Vector3d d0 = d;
        n.normalize();
        d.normalize();
        R.col(0) = n;
        R.col(1) = d;
        R.col(2) = n0.cross(d0) / (n0.cross(d0).norm());
        return R;
    }

    double ComputeTwoLineAngleCam(const Vector3d &l_a, const Vector3d &l_b)
    {
        double dot_product = l_b.dot(l_a);

        // Find magnitude of lines
        double magn_a = l_b.norm();
        double magn_b = l_a.norm();

        // Find the cosine of the angle formed
        return abs(dot_product / (magn_a * magn_b));
    }

    GeometricCamera* pCamera;

protected:
    double fx,fy,cx,cy;
};


class EdgeMonoVpLinePerp : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexPlukerLine, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeMonoVpLinePerp(Eigen::Vector3d obsVp) : obsVp_(obsVp){}

    bool read(std::istream &is) { return true; }

    bool write(std::ostream &os) const { return true; }

    void SetParams(const double &fx_, const double &fy_, const double &cx_, const double &cy_) {
        fx = fx_;
        fy = fy_;
        cx = cx_;
        cy = cy_;
    }

    void computeError() {
        const VertexPlukerLine* vLine = dynamic_cast<const VertexPlukerLine* >(_vertices[0]);
        const g2o::VertexSE3Expmap* vPose = dynamic_cast<const g2o::VertexSE3Expmap* >(_vertices[1]);

        const Eigen::Vector3d obs(_measurement);

        // transform se3 to a eigen matrix
        const Matrix4d Tcw = vPose->estimate().to_homogeneous_matrix();
        const Eigen::Vector4d Lw = vLine->estimate();

        Vector6d plukerLw = changeOrthToPluker(Lw);
        Vector6d plukerLc = getTransformMatrix(Tcw) * plukerLw;

        Eigen::Vector3d v_c = plukerLc.tail(3);
        // vp loss
        double perpAngleError = ComputeTwoLineAngleCam(v_c, obsVp_);

        Eigen::Vector3d error = Eigen::Vector3d::Zero();
        error(0) = perpAngleError;
        _error = error;
    }

    // check pass
    Eigen::Matrix3d getPlukerK(){
        Eigen::Matrix3d plukerK;
        plukerK << fy,     0,      0,
            0,      fx,     0,
            -fy*cx, -fx*cy, fx*fy;
        return plukerK;
    }

    Matrix6d getTransformMatrix(const Matrix4d& T){
        Matrix6d temp;
        temp.setZero();
        temp.block<3,3>(0,0) = T.block<3,3>(0,0);
        temp.block<3,3>(0,3) = Skew(T.block<3,1>(0,3)) * T.block<3,3>(0,0);
        temp.block<3,3>(3,3) = T.block<3,3>(0,0);

        return temp;
    }

    Vector6d changeOrthToPluker(const Eigen::Vector4d& orthLine ){
        double s1 = sin(orthLine[0]);
        double c1 = cos(orthLine[0]);
        double s2 = sin(orthLine[1]);
        double c2 = cos(orthLine[1]);
        double s3 = sin(orthLine[2]);
        double c3 = cos(orthLine[2]);
        Eigen::Matrix3d R;
        R <<
            c2 * c3,   s1 * s2 * c3 - c1 * s3,   c1 * s2 * c3 + s1 * s3,
            c2 * s3,   s1 * s2 * s3 + c1 * c3,   c1 * s2 * s3 - s1 * c3,
            -s2,                  s1 * c2,                  c1 * c2;

        double w1 = cos(orthLine[3]);
        double w2 = sin(orthLine[3]);

        Vector6d plukerLine;
        plukerLine.head(3) = w1 * R.col(0);
        plukerLine.tail(3) = w2 * R.col(1);
        return plukerLine;
    }

    bool isDepthPositive(MapLine* pML, KeyFrame* curKeyFrame) {
        const g2o::VertexSE3Expmap* vPose = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const VertexPlukerLine* vLine = static_cast<const VertexPlukerLine* >(_vertices[0]);
        Vector6d plukerLw = changeOrthToPluker(vLine->estimate());
        map<KeyFrame*, size_t> observations;
        Vector6f plukerPosW;
        {
            observations = pML->mObservations;
            plukerPosW = plukerLw.cast<float>();
        }

        KeyFrame* pKF = curKeyFrame;
        GeometricCamera* pCamera1 = pKF->mpCamera;
        const line_descriptor::KeyLine& kl1 = pKF->mvKeyLinesUn[observations[pKF]];

        Vector6d plukerLc1 = GeometricTools::TransPlkLineByPose(plukerPosW.cast<double>(), pKF->GetPose().cast<double>().rotationMatrix(), pKF->GetPose().cast<double>().translation());

        cv::Point2f sp1 = cv::Point2f(kl1.startPointX, kl1.startPointY);
        cv::Point2f ep1 = cv::Point2f(kl1.endPointX, kl1.endPointY);

        Eigen::Vector3d sp1_c = pCamera1->unprojectEig(sp1).cast<double>();
        Eigen::Vector3d ep1_c = pCamera1->unprojectEig(ep1).cast<double>();

        Vector6d line3Dc = GeometricTools::TrimLine(sp1_c, ep1_c, plukerLc1);
        Eigen::Vector3d csp = line3Dc.head(3);
        Eigen::Vector3d cep = line3Dc.tail(3);
        if (csp(2) <= 0 || cep(2) <= 0) {
            return false;
        }

        return true;
    }

    Matrix<double,6,4> jacobianFromPlukerToOrth(const Eigen::Matrix3d& u, const Matrix2d& w){
        double w1 = w(0,0);
        double w2 = w(1,0);
        Eigen::Vector3d u1 = u.col(0);
        Eigen::Vector3d u2 = u.col(1);
        Eigen::Vector3d u3 = u.col(2);
        Matrix<double,6,4> temp;
        temp.setZero();
        temp.block<3,1>(0,1) = -w1 * u3;
        temp.block<3,1>(0,2) = w1 * u2;
        temp.block<3,1>(0,3) = -w2 * u1;
        temp.block<3,1>(3,0) = w2 * u3;
        temp.block<3,1>(3,2) = -w2 * u1;
        temp.block<3,1>(3,3) = w1 * u2;
        return temp;
    }

    Matrix2d getOrthWFromPluker(const Vector6d& plukerLine)
    {
        Matrix2d temp;
        double nnorm = plukerLine.head(3).norm();
        double dnorm = plukerLine.tail(3).norm();
        double fenmu = sqrt(nnorm*nnorm + dnorm*dnorm);
        temp << nnorm / fenmu, -dnorm / fenmu, dnorm / fenmu, nnorm / fenmu;
        return temp;
    }

    Eigen::Matrix3d getOrhtRFromPluker(const Vector6d &plukerLine)
    {
        Eigen::Matrix3d R;
        Eigen::Vector3d n = plukerLine.head(3);
        Eigen::Vector3d d = plukerLine.tail(3);
        Eigen::Vector3d n0 = n;
        Eigen::Vector3d d0 = d;
        n.normalize();
        d.normalize();
        R.col(0) = n;
        R.col(1) = d;
        R.col(2) = n0.cross(d0) / (n0.cross(d0).norm());
        return R;
    }

    double ComputeTwoLineAngleCam(const Vector3d &l_a, const Vector3d &l_b)
    {
        double dot_product = l_b.dot(l_a);

        // Find magnitude of lines
        double magn_a = l_b.norm();
        double magn_b = l_a.norm();

        // Find the cosine of the angle formed
        return abs(dot_product / (magn_a * magn_b));
    }
    GeometricCamera* pCamera;

protected:
    double fx,fy,cx,cy;
    Eigen::Vector3d obsVp_;
};


class  EdgeSE3ProjectXYZOnlyPoseLine: public  g2o::BaseUnaryEdge<2, Eigen::Vector4d, g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZOnlyPoseLine(){}

    bool read(std::istream &is) { return true; }

    bool write(std::ostream &os) const { return true; }

    void SetParams(const double &fx_, const double &fy_, const double &cx_, const double &cy_) {
        fx = fx_;
        fy = fy_;
        cx = cx_;
        cy = cy_;
    }

    virtual void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        Eigen::Vector4d obs(_measurement);

        const Matrix4d Tcw = v1->estimate().to_homogeneous_matrix();
        Vector6d plukerLc = getTransformMatrix(Tcw) * plukerLw;

        Eigen::Vector3d plukerLcPixel = getPlukerK() * plukerLc.head(3);
        double lx = plukerLcPixel(0);
        double ly = plukerLcPixel(1);
        double lz = plukerLcPixel(2);

        double fenmu = sqrt( lx*lx + ly*ly);

        Eigen::Vector2d error;
        error(0) = ( lx * obs(0) + ly * obs(1) + lz ) / fenmu;
        error(1) = ( lx * obs(2) + ly * obs(3) + lz ) / fenmu;

        _error = error;
    }

    bool isDepthPositive(MapLine* pML, KeyFrame* curKeyFrame) {
        const g2o::VertexSE3Expmap* vPose = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const VertexPlukerLine* vLine = static_cast<const VertexPlukerLine* >(_vertices[0]);
        Vector6d plukerLw = changeOrthToPluker(vLine->estimate());
        map<KeyFrame*, size_t> observations;
        Vector6f plukerPosW;
        {
            observations = pML->mObservations;
            plukerPosW = plukerLw.cast<float>();
        }

        KeyFrame* pKF = curKeyFrame;
        GeometricCamera* pCamera1 = pKF->mpCamera;
        const line_descriptor::KeyLine& kl1 = pKF->mvKeyLinesUn[observations[pKF]];

        Vector6d plukerLc1 = GeometricTools::TransPlkLineByPose(plukerPosW.cast<double>(), pKF->GetPose().cast<double>().rotationMatrix(), pKF->GetPose().cast<double>().translation());

        cv::Point2f sp1 = cv::Point2f(kl1.startPointX, kl1.startPointY);
        cv::Point2f ep1 = cv::Point2f(kl1.endPointX, kl1.endPointY);

        Eigen::Vector3d sp1_c = pCamera1->unprojectEig(sp1).cast<double>();
        Eigen::Vector3d ep1_c = pCamera1->unprojectEig(ep1).cast<double>();

        Vector6d line3Dc = GeometricTools::TrimLine(sp1_c, ep1_c, plukerLc1);
        Eigen::Vector3d csp = line3Dc.head(3);
        Eigen::Vector3d cep = line3Dc.tail(3);
        if (csp(2) <= 0 || cep(2) <= 0) {
            return false;
        }

        return true;
    }

    // check pass
    Eigen::Matrix3d getPlukerK(){
        Eigen::Matrix3d plukerK;
        plukerK << fy,     0,      0,
            0,      fx,     0,
            -fy*cx, -fx*cy, fx*fy;
        return plukerK;
    }

    Matrix6d getTransformMatrix(const Matrix4d& T){
        Matrix6d temp;
        temp.setZero();
        temp.block<3,3>(0,0) = T.block<3,3>(0,0);
        temp.block<3,3>(0,3) = Sophus::SO3d::hat(T.block<3,1>(0,3)) * T.block<3,3>(0,0);
        temp.block<3,3>(3,3) = T.block<3,3>(0,0);

        return temp;
    }

    Vector6d changeOrthToPluker(const Eigen::Vector4d& orthLine ){
        double s1 = sin(orthLine[0]);
        double c1 = cos(orthLine[0]);
        double s2 = sin(orthLine[1]);
        double c2 = cos(orthLine[1]);
        double s3 = sin(orthLine[2]);
        double c3 = cos(orthLine[2]);
        Eigen::Matrix3d R;
        R <<
            c2 * c3,   s1 * s2 * c3 - c1 * s3,   c1 * s2 * c3 + s1 * s3,
            c2 * s3,   s1 * s2 * s3 + c1 * c3,   c1 * s2 * s3 - s1 * c3,
            -s2,                  s1 * c2,                  c1 * c2;

        double w1 = cos(orthLine[3]);
        double w2 = sin(orthLine[3]);

        Vector6d plukerLine;
        plukerLine.head(3) = w1 * R.col(0);
        plukerLine.tail(3) = w2 * R.col(1);
        return plukerLine;
    }

    virtual void linearizeOplus()
    {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);

        const Eigen::Vector4d obs(_measurement);

        const Eigen::Matrix4d Tcw = v1->estimate().to_homogeneous_matrix();

        const Eigen::Matrix3d Rcw = Tcw.block<3,3>(0,0);
        const Eigen::Vector3d Pcw = Tcw.block<3,1>(0,3);

        Vector6d plukerLc = getTransformMatrix(Tcw) * plukerLw;

        Eigen::Vector3d plukerLcPixel = getPlukerK() * plukerLc.head(3);
        double lx = plukerLcPixel(0);
        double ly = plukerLcPixel(1);
        double lz = plukerLcPixel(2);
        double fenmu = sqrt(lx*lx + ly*ly);

        Eigen::Vector2d error;
        error(0) = ( lx * obs(0) + ly * obs(1) + lz ) / fenmu;
        error(1) = ( lx * obs(2) + ly * obs(3) + lz ) / fenmu;

        // check pass
        Matrix<double,1,3> jac_err0_lcPixel;
        jac_err0_lcPixel << -lx * error(0) / (fenmu*fenmu) + obs(0) /fenmu,
            -ly * error(0) / (fenmu*fenmu) + obs(1) /fenmu,
            1.0 / fenmu;
        Matrix<double,1,3> jac_err1_lcPixel;
        jac_err1_lcPixel << -lx * error(1) / (fenmu*fenmu) + obs(2) /fenmu,
            -ly * error(1) / (fenmu*fenmu) + obs(3) /fenmu,
            1.0 / fenmu;

        // check pass
        Matrix<double,3,6> jac_lcPixel_lc;
        jac_lcPixel_lc.setZero();
        jac_lcPixel_lc.block<3,3>(0,0) = getPlukerK();

        Matrix<double,6,6> jac_lc_rt;
        jac_lc_rt.setZero();
        jac_lc_rt.block<3,3>(0,0) = - Sophus::SO3d::hat(Rcw*plukerLw.head(3)) - Sophus::SO3d::hat(Sophus::SO3d::hat(Pcw)*Rcw*plukerLw.tail(3));
        jac_lc_rt.block<3,3>(3,0) = - Sophus::SO3d::hat(Rcw*plukerLw.tail(3));
        jac_lc_rt.block<3,3>(0,3) = - Sophus::SO3d::hat(Rcw*plukerLw.tail(3));

        Matrix<double,2,6> jac_error_pose;
        jac_error_pose.block<1,6>(0,0) = jac_err0_lcPixel * jac_lcPixel_lc * jac_lc_rt;
        jac_error_pose.block<1,6>(1,0) = jac_err1_lcPixel * jac_lcPixel_lc * jac_lc_rt;
        _jacobianOplusXi = jac_error_pose;

    }

    Vector6d plukerLw;
    GeometricCamera* pCamera;

protected:
    double fx,fy,cx,cy;
};


class EdgeMonoLineOnlyPose : public g2o::BaseUnaryEdge<2, Eigen::Vector4d, VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeMonoLineOnlyPose(const Vector6d &plukerLw_, int cam_idx_=0):plukerLw(plukerLw_), cam_idx(cam_idx_){}

    virtual bool read(std::istream& is){return false;}

    virtual bool write(std::ostream& os) const{return false;}

    void SetParams(const double &fx_, const double &fy_, const double &cx_, const double &cy_) {
        fx = fx_;
        fy = fy_;
        cx = cx_;
        cy = cy_;
    }

    void computeError() {
        const VertexPose* vPose = dynamic_cast<const VertexPose* >(_vertices[0]);


        const Eigen::Vector4d obs(_measurement);

        // transform se3 to a eigen matrix
        const Eigen::Matrix3d Rcw = vPose->estimate().Rcw[cam_idx];
        const Eigen::Vector3d tcw = vPose->estimate().tcw[cam_idx];
        Eigen::Matrix4d Tcw = Eigen::Matrix4d::Identity();
        Tcw.block<3,3>(0,0) = Rcw;
        Tcw.block<3,1>(0,3) = tcw;

        Vector6d plukerLc = getTransformMatrix(Tcw) * plukerLw;

        Eigen::Vector3d plukerLcPixel = getPlukerK() * plukerLc.head(3);
        double lx = plukerLcPixel(0);
        double ly = plukerLcPixel(1);
        double lz = plukerLcPixel(2);

        double fenmu = sqrt( lx*lx + ly*ly);

        Eigen::Vector2d error;
        error(0) = ( lx * obs(0) + ly * obs(1) + lz ) / fenmu;
        error(1) = ( lx * obs(2) + ly * obs(3) + lz ) / fenmu;

        _error = error;
    }

    // check pass
    Eigen::Matrix3d getPlukerK(){
        Eigen::Matrix3d plukerK;
        plukerK << fy,     0,      0,
            0,      fx,     0,
            -fy*cx, -fx*cy, fx*fy;
        return plukerK;
    }

    Matrix6d getTransformMatrix(const Matrix4d& T){
        Matrix6d temp;
        temp.setZero();
        temp.block<3,3>(0,0) = T.block<3,3>(0,0);
        temp.block<3,3>(0,3) = Skew(T.block<3,1>(0,3)) * T.block<3,3>(0,0);
        temp.block<3,3>(3,3) = T.block<3,3>(0,0);

        return temp;
    }

    Vector6d changeOrthToPluker(const Eigen::Vector4d& orthLine ){
        double s1 = sin(orthLine[0]);
        double c1 = cos(orthLine[0]);
        double s2 = sin(orthLine[1]);
        double c2 = cos(orthLine[1]);
        double s3 = sin(orthLine[2]);
        double c3 = cos(orthLine[2]);
        Eigen::Matrix3d R;
        R <<
            c2 * c3,   s1 * s2 * c3 - c1 * s3,   c1 * s2 * c3 + s1 * s3,
            c2 * s3,   s1 * s2 * s3 + c1 * c3,   c1 * s2 * s3 - s1 * c3,
            -s2,                  s1 * c2,                  c1 * c2;

        double w1 = cos(orthLine[3]);
        double w2 = sin(orthLine[3]);

        Vector6d plukerLine;
        plukerLine.head(3) = w1 * R.col(0);
        plukerLine.tail(3) = w2 * R.col(1);
        return plukerLine;
    }

    virtual void linearizeOplus() {
        const VertexPose* vPose = dynamic_cast<const VertexPose* >(_vertices[0]);

        const Eigen::Vector4d obs(_measurement);

        const Eigen::Matrix3d Rcw = vPose->estimate().Rcw[cam_idx];
        const Eigen::Vector3d tcw = vPose->estimate().tcw[cam_idx];
        const Eigen::Matrix3d Rcb = vPose->estimate().Rcb[cam_idx];
        const Eigen::Vector3d tcb = vPose->estimate().tcb[cam_idx];

        const Eigen::Matrix3d Rbw = vPose->estimate().Rwb.transpose();
        const Eigen::Vector3d tbw = -Rbw * vPose->estimate().twb;

        Eigen::Matrix4d Tcw = Eigen::Matrix4d::Identity();
        Tcw.block<3,3>(0,0) = Rcw;
        Tcw.block<3,1>(0,3) = tcw;

        Eigen::Matrix4d Tcb = Eigen::Matrix4d::Identity();
        Tcb.block<3,3>(0,0) = Rcb;
        Tcb.block<3,1>(0,3) = tcb;

        Vector6d plukerLc = getTransformMatrix(Tcw) * plukerLw;

        Eigen::Vector3d plukerLcPixel = getPlukerK() * plukerLc.head(3);
        double lx = plukerLcPixel(0);
        double ly = plukerLcPixel(1);
        double lz = plukerLcPixel(2);
        double fenmu = sqrt(lx*lx + ly*ly);

        Eigen::Vector2d error;
        error(0) = ( lx * obs(0) + ly * obs(1) + lz ) / fenmu;
        error(1) = ( lx * obs(2) + ly * obs(3) + lz ) / fenmu;

        // check pass
        Matrix<double,1,3> jac_err0_lcPixel;
        jac_err0_lcPixel << -lx * error(0) / (fenmu*fenmu) + obs(0) /fenmu,
            -ly * error(0) / (fenmu*fenmu) + obs(1) /fenmu,
            1.0 / fenmu;
        Matrix<double,1,3> jac_err1_lcPixel;
        jac_err1_lcPixel << -lx * error(1) / (fenmu*fenmu) + obs(2) /fenmu,
            -ly * error(1) / (fenmu*fenmu) + obs(3) /fenmu,
            1.0 / fenmu;

        // check pass
        Matrix<double,3,6> jac_lcPixel_lc;
        jac_lcPixel_lc.setZero();
        jac_lcPixel_lc.block<3,3>(0,0) = getPlukerK();

        Eigen::Matrix<double,6,6> jac_lc_lb;
        jac_lc_lb = getTransformMatrix(Tcb);
        jac_lc_lb = jac_lc_lb * (-1);

        Eigen::Matrix<double,6,6> jac_lb_rt;
        jac_lb_rt.setZero();
        jac_lb_rt.block<3,3>(0,0) = - Sophus::SO3d::hat(Rbw*plukerLw.head(3)) - Sophus::SO3d::hat(Sophus::SO3d::hat(tbw)*Rbw*plukerLw.tail(3));
        jac_lb_rt.block<3,3>(3,0) = - Sophus::SO3d::hat(Rbw*plukerLw.tail(3));
        jac_lb_rt.block<3,3>(0,3) = - Sophus::SO3d::hat(Rbw*plukerLw.tail(3));

        Eigen::Matrix<double,2,6> jac_error_pose;
        jac_error_pose.block<1,6>(0,0) = jac_err0_lcPixel * jac_lcPixel_lc * jac_lc_lb * jac_lb_rt;
        jac_error_pose.block<1,6>(1,0) = jac_err1_lcPixel * jac_lcPixel_lc * jac_lc_lb * jac_lb_rt;
        _jacobianOplusXi = jac_error_pose;
    }

    bool isDepthPositive(MapLine* pML, KeyFrame* curKeyFrame) {
        const g2o::VertexSE3Expmap* vPose = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const VertexPlukerLine* vLine = static_cast<const VertexPlukerLine* >(_vertices[0]);
        Vector6d plukerLw = changeOrthToPluker(vLine->estimate());
        map<KeyFrame*, size_t> observations;
        Vector6f plukerPosW;
        {
            observations = pML->mObservations;
            plukerPosW = plukerLw.cast<float>();
        }

        KeyFrame* pKF = curKeyFrame;
        GeometricCamera* pCamera1 = pKF->mpCamera;
        const line_descriptor::KeyLine& kl1 = pKF->mvKeyLinesUn[observations[pKF]];

        Vector6d plukerLc1 = GeometricTools::TransPlkLineByPose(plukerPosW.cast<double>(), pKF->GetPose().cast<double>().rotationMatrix(), pKF->GetPose().cast<double>().translation());

        cv::Point2f sp1 = cv::Point2f(kl1.startPointX, kl1.startPointY);
        cv::Point2f ep1 = cv::Point2f(kl1.endPointX, kl1.endPointY);

        Eigen::Vector3d sp1_c = pCamera1->unprojectEig(sp1).cast<double>();
        Eigen::Vector3d ep1_c = pCamera1->unprojectEig(ep1).cast<double>();

        Vector6d line3Dc = GeometricTools::TrimLine(sp1_c, ep1_c, plukerLc1);
        Eigen::Vector3d csp = line3Dc.head(3);
        Eigen::Vector3d cep = line3Dc.tail(3);
        if (csp(2) <= 0 || cep(2) <= 0) {
            return false;
        }

        return true;
    }

    Matrix<double,6,4> jacobianFromPlukerToOrth(const Eigen::Matrix3d& u, const Matrix2d& w){
        double w1 = w(0,0);
        double w2 = w(1,0);
        Eigen::Vector3d u1 = u.col(0);
        Eigen::Vector3d u2 = u.col(1);
        Eigen::Vector3d u3 = u.col(2);
        Matrix<double,6,4> temp;
        temp.setZero();
        temp.block<3,1>(0,1) = -w1 * u3;
        temp.block<3,1>(0,2) = w1 * u2;
        temp.block<3,1>(0,3) = -w2 * u1;
        temp.block<3,1>(3,0) = w2 * u3;
        temp.block<3,1>(3,2) = -w2 * u1;
        temp.block<3,1>(3,3) = w1 * u2;
        return temp;
    }

    Matrix2d getOrthWFromPluker(const Vector6d& plukerLine)
    {
        Matrix2d temp;
        double nnorm = plukerLine.head(3).norm();
        double dnorm = plukerLine.tail(3).norm();
        double fenmu = sqrt(nnorm*nnorm + dnorm*dnorm);
        temp << nnorm / fenmu, -dnorm / fenmu, dnorm / fenmu, nnorm / fenmu;
        return temp;
    }

    Eigen::Matrix3d getOrhtRFromPluker(const Vector6d &plukerLine)
    {
        Eigen::Matrix3d R;
        Eigen::Vector3d n = plukerLine.head(3);
        Eigen::Vector3d d = plukerLine.tail(3);
        Eigen::Vector3d n0 = n;
        Eigen::Vector3d d0 = d;
        n.normalize();
        d.normalize();
        R.col(0) = n;
        R.col(1) = d;
        R.col(2) = n0.cross(d0) / (n0.cross(d0).norm());
        return R;
    }

    Eigen::Matrix<double,6,6> GetHessian(){
        linearizeOplus();
        return _jacobianOplusXi.transpose()*information()*_jacobianOplusXi;
    }

public:
    double fx,fy,cx,cy;
    const Vector6d plukerLw;
    const int cam_idx;
};

class EdgeMonoLineInertial : public g2o::BaseBinaryEdge<2, Eigen::Vector4d, VertexPlukerLine, VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeMonoLineInertial(int cam_idx_=0) : cam_idx(cam_idx_){}

    bool read(std::istream &is) { return true; }

    bool write(std::ostream &os) const { return true; }

    void SetParams(const double &fx_, const double &fy_, const double &cx_, const double &cy_) {
        fx = fx_;
        fy = fy_;
        cx = cx_;
        cy = cy_;
    }

    void computeError() {
        const VertexPlukerLine* vLine = dynamic_cast<const VertexPlukerLine* >(_vertices[0]);
        const VertexPose* vPose = dynamic_cast<const VertexPose* >(_vertices[1]);

        const Eigen::Vector4d obs(_measurement);

        // transform se3 to a eigen matrix
        const Eigen::Matrix3d Rcw = vPose->estimate().Rcw[cam_idx];
        const Eigen::Vector3d tcw = vPose->estimate().tcw[cam_idx];
        Eigen::Matrix4d Tcw = Eigen::Matrix4d::Identity();
        Tcw.block<3,3>(0,0) = Rcw;
        Tcw.block<3,1>(0,3) = tcw;
        const Eigen::Vector4d LwOrth = vLine->estimate();

        Vector6d plukerLw = changeOrthToPluker(LwOrth);
        Vector6d plukerLc = getTransformMatrix(Tcw) * plukerLw;

        Eigen::Vector3d plukerLcPixel = getPlukerK() * plukerLc.head(3);
        double lx = plukerLcPixel(0);
        double ly = plukerLcPixel(1);
        double lz = plukerLcPixel(2);

        double fenmu = sqrt( lx*lx + ly*ly);

        Eigen::Vector2d error;
        error(0) = ( lx * obs(0) + ly * obs(1) + lz ) / fenmu;
        error(1) = ( lx * obs(2) + ly * obs(3) + lz ) / fenmu;

        _error = error;
    }

    // check pass
    Eigen::Matrix3d getPlukerK(){
        Eigen::Matrix3d plukerK;
        plukerK << fy,     0,      0,
            0,      fx,     0,
            -fy*cx, -fx*cy, fx*fy;
        return plukerK;
    }

    Matrix6d getTransformMatrix(const Matrix4d& T){
        Matrix6d temp;
        temp.setZero();
        temp.block<3,3>(0,0) = T.block<3,3>(0,0);
        temp.block<3,3>(0,3) = Skew(T.block<3,1>(0,3)) * T.block<3,3>(0,0);
        temp.block<3,3>(3,3) = T.block<3,3>(0,0);

        return temp;
    }

    Vector6d changeOrthToPluker(const Eigen::Vector4d& orthLine ){
        double s1 = sin(orthLine[0]);
        double c1 = cos(orthLine[0]);
        double s2 = sin(orthLine[1]);
        double c2 = cos(orthLine[1]);
        double s3 = sin(orthLine[2]);
        double c3 = cos(orthLine[2]);
        Eigen::Matrix3d R;
        R <<
            c2 * c3,   s1 * s2 * c3 - c1 * s3,   c1 * s2 * c3 + s1 * s3,
            c2 * s3,   s1 * s2 * s3 + c1 * c3,   c1 * s2 * s3 - s1 * c3,
            -s2,                  s1 * c2,                  c1 * c2;

        double w1 = cos(orthLine[3]);
        double w2 = sin(orthLine[3]);

        Vector6d plukerLine;
        plukerLine.head(3) = w1 * R.col(0);
        plukerLine.tail(3) = w2 * R.col(1);
        return plukerLine;
    }

    virtual void linearizeOplus() {
        const VertexPlukerLine* vLine = dynamic_cast<const VertexPlukerLine* >(_vertices[0]);
        const VertexPose* vPose = dynamic_cast<const VertexPose* >(_vertices[1]);

        const Eigen::Vector4d obs(_measurement);
        const Eigen::Vector4d LwOrth = vLine->estimate();

        const Eigen::Matrix3d Rcw = vPose->estimate().Rcw[cam_idx];
        const Eigen::Vector3d tcw = vPose->estimate().tcw[cam_idx];
        const Eigen::Matrix3d Rcb = vPose->estimate().Rcb[cam_idx];
        const Eigen::Vector3d tcb = vPose->estimate().tcb[cam_idx];

        const Eigen::Matrix3d Rbw = vPose->estimate().Rwb.transpose();
        const Eigen::Vector3d tbw = -Rbw * vPose->estimate().twb;

        Eigen::Matrix4d Tcw = Eigen::Matrix4d::Identity();
        Tcw.block<3,3>(0,0) = Rcw;
        Tcw.block<3,1>(0,3) = tcw;

        Eigen::Matrix4d Tcb = Eigen::Matrix4d::Identity();
        Tcb.block<3,3>(0,0) = Rcb;
        Tcb.block<3,1>(0,3) = tcb;

        Vector6d plukerLw = changeOrthToPluker(LwOrth);
        Vector6d plukerLc = getTransformMatrix(Tcw) * plukerLw;

        Eigen::Vector3d plukerLcPixel = getPlukerK() * plukerLc.head(3);
        double lx = plukerLcPixel(0);
        double ly = plukerLcPixel(1);
        double lz = plukerLcPixel(2);
        double fenmu = sqrt(lx*lx + ly*ly);

        Eigen::Vector2d error;
        error(0) = ( lx * obs(0) + ly * obs(1) + lz ) / fenmu;
        error(1) = ( lx * obs(2) + ly * obs(3) + lz ) / fenmu;

        Matrix<double,1,3> jac_err0_lcPixel;
        jac_err0_lcPixel << -lx * error(0) / (fenmu*fenmu) + obs(0) /fenmu,
            -ly * error(0) / (fenmu*fenmu) + obs(1) /fenmu,
            1.0 / fenmu;
        Matrix<double,1,3> jac_err1_lcPixel;
        jac_err1_lcPixel << -lx * error(1) / (fenmu*fenmu) + obs(2) /fenmu,
            -ly * error(1) / (fenmu*fenmu) + obs(3) /fenmu,
            1.0 / fenmu;

        // check pass
        Matrix<double,3,6> jac_lcPixel_lc;
        jac_lcPixel_lc.setZero();
        jac_lcPixel_lc.block<3,3>(0,0) = getPlukerK();

        Matrix<double,6,6> jac_lc_lw;
        jac_lc_lw = getTransformMatrix(Tcw);

        Eigen::Matrix3d U = getOrhtRFromPluker(plukerLw);
        Matrix2d W = getOrthWFromPluker(plukerLw);

        Matrix<double,6,4> jac_lw_orth;
        jac_lw_orth = jacobianFromPlukerToOrth(U, W);

        Matrix<double,2,4> jac_error_orth;
        jac_error_orth.block<1,4>(0,0) = jac_err0_lcPixel * jac_lcPixel_lc * jac_lc_lw * jac_lw_orth;
        jac_error_orth.block<1,4>(1,0) = jac_err1_lcPixel * jac_lcPixel_lc * jac_lc_lw * jac_lw_orth;
        _jacobianOplusXi = jac_error_orth;

        Eigen::Matrix<double,6,6> jac_lc_lb;
        jac_lc_lb = getTransformMatrix(Tcb);
        jac_lc_lb = jac_lc_lb * (-1);

        Eigen::Matrix<double,6,6> jac_lb_rt;
        jac_lb_rt.setZero();
        jac_lb_rt.block<3,3>(0,0) = - Sophus::SO3d::hat(Rbw*plukerLw.head(3)) - Sophus::SO3d::hat(Sophus::SO3d::hat(tbw)*Rbw*plukerLw.tail(3));
        jac_lb_rt.block<3,3>(3,0) = - Sophus::SO3d::hat(Rbw*plukerLw.tail(3));
        jac_lb_rt.block<3,3>(0,3) = - Sophus::SO3d::hat(Rbw*plukerLw.tail(3));

        Eigen::Matrix<double,2,6> jac_error_pose;
        jac_error_pose.block<1,6>(0,0) = jac_err0_lcPixel * jac_lcPixel_lc * jac_lc_lb * jac_lb_rt;
        jac_error_pose.block<1,6>(1,0) = jac_err1_lcPixel * jac_lcPixel_lc * jac_lc_lb * jac_lb_rt;
        _jacobianOplusXj = jac_error_pose;
    }

    bool isDepthPositive(MapLine* pML, KeyFrame* curKeyFrame) {
        const g2o::VertexSE3Expmap* vPose = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const VertexPlukerLine* vLine = static_cast<const VertexPlukerLine* >(_vertices[0]);
        Vector6d plukerLw = changeOrthToPluker(vLine->estimate());
        map<KeyFrame*, size_t> observations;
        Vector6f plukerPosW;
        {
            observations = pML->mObservations;
            plukerPosW = plukerLw.cast<float>();
        }

        KeyFrame* pKF = curKeyFrame;
        GeometricCamera* pCamera1 = pKF->mpCamera;
        const line_descriptor::KeyLine& kl1 = pKF->mvKeyLinesUn[observations[pKF]];

        Vector6d plukerLc1 = GeometricTools::TransPlkLineByPose(plukerPosW.cast<double>(), pKF->GetPose().cast<double>().rotationMatrix(), pKF->GetPose().cast<double>().translation());

        cv::Point2f sp1 = cv::Point2f(kl1.startPointX, kl1.startPointY);
        cv::Point2f ep1 = cv::Point2f(kl1.endPointX, kl1.endPointY);

        Eigen::Vector3d sp1_c = pCamera1->unprojectEig(sp1).cast<double>();
        Eigen::Vector3d ep1_c = pCamera1->unprojectEig(ep1).cast<double>();

        Vector6d line3Dc = GeometricTools::TrimLine(sp1_c, ep1_c, plukerLc1);
        Eigen::Vector3d csp = line3Dc.head(3);
        Eigen::Vector3d cep = line3Dc.tail(3);
        if (csp(2) <= 0 || cep(2) <= 0) {
            return false;
        }

        return true;
    }

    Matrix<double,6,4> jacobianFromPlukerToOrth(const Eigen::Matrix3d& u, const Matrix2d& w){
        double w1 = w(0,0);
        double w2 = w(1,0);
        Eigen::Vector3d u1 = u.col(0);
        Eigen::Vector3d u2 = u.col(1);
        Eigen::Vector3d u3 = u.col(2);
        Matrix<double,6,4> temp;
        temp.setZero();
        temp.block<3,1>(0,1) = -w1 * u3;
        temp.block<3,1>(0,2) = w1 * u2;
        temp.block<3,1>(0,3) = -w2 * u1;
        temp.block<3,1>(3,0) = w2 * u3;
        temp.block<3,1>(3,2) = -w2 * u1;
        temp.block<3,1>(3,3) = w1 * u2;
        return temp;
    }

    Matrix2d getOrthWFromPluker(const Vector6d& plukerLine)
    {
        Matrix2d temp;
        double nnorm = plukerLine.head(3).norm();
        double dnorm = plukerLine.tail(3).norm();
        double fenmu = sqrt(nnorm*nnorm + dnorm*dnorm);
        temp << nnorm / fenmu, -dnorm / fenmu, dnorm / fenmu, nnorm / fenmu;
        return temp;
    }

    Eigen::Matrix3d getOrhtRFromPluker(const Vector6d &plukerLine)
    {
        Eigen::Matrix3d R;
        Eigen::Vector3d n = plukerLine.head(3);
        Eigen::Vector3d d = plukerLine.tail(3);
        Eigen::Vector3d n0 = n;
        Eigen::Vector3d d0 = d;
        n.normalize();
        d.normalize();
        R.col(0) = n;
        R.col(1) = d;
        R.col(2) = n0.cross(d0) / (n0.cross(d0).norm());
        return R;
    }

    GeometricCamera* pCamera;

protected:
    double fx,fy,cx,cy;
    const int cam_idx;
};

} //namespace ORB_SLAM

#endif // G2OTYPESLINE
