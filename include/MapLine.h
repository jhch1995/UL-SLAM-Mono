/***************************************************************************************
 * Copyright(C) FuDan University Autonomous Driving Lab. 2021-2022. All rights reserved
 * Description: This is a part of UL-SLAM which is basic class about map line, it is
 *              similar to mappoint class in orb-slam3 framework.
 * Create: 2022-04-03
 * Last Modified: 2022-05-29
 * Notes: .h file(MapLine)
 **************************************************************************************/
#ifndef ORB_SLAM2_MAPLINE_H
#define ORB_SLAM2_MAPLINE_H

#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"
#include "Thirdparty/line_descriptor/include/line_descriptor_custom.hpp"

#include <opencv2/core/core.hpp>
#include <mutex>
#include <eigen3/Eigen/Core>
#include <map>

using namespace Eigen;
namespace ORB_SLAM3
{
class KeyFrame;
class Map;
class Frame;

using Vector6f = Eigen::Matrix<float,6,1>;

class MapLine
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapLine();

    MapLine(Vector6f &plukerLinePos, Vector6f &linePos3D, KeyFrame* pRefKF, Map* pMap);  
    MapLine(Vector6f &plukerLinePos, Vector6f &linePos3D, Map* pMap, Frame* pFrame, const int &idxF); 

    void SetWorldPos(const Vector6f &Pos1);
    Vector6f GetWorldPos();

    void SetPlukerWorldPos(const Vector6f &Pos1);
    Vector6f GetPlukerLineWordPos();

    Eigen::Vector3f GetNormal();
    KeyFrame* GetReferenceKeyFrame();

    map<KeyFrame*, size_t> GetObservations();
    int Observations();

    void AddObservation(KeyFrame* pKF, size_t idx);
    void EraseObservation(KeyFrame* pKF);

    void AddStructGroup(KeyFrame* pKF, size_t idx);
    int GetStructGroup(KeyFrame* pKF);

    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(MapLine* pML);
    MapLine* GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    void ComputeDistinctiveDescriptors(); 

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();

    void UpdateLineTrimming();

    void UpdatePlukerLinePos(float scale);

    void SetNormalVector(Eigen::Vector3f& normal);
    void UpdateMap(Map* pMap);

    void Update3DMapLine(Eigen::Vector3f& line3DStartPoint, Eigen::Vector3f& line3DEndPoint);

    void VisualizePluckerEnd(vector<MapLine*> vpML, Frame& F);

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();

    Map* GetMap();

    int PredictScale(const float &currentDist, const float &logScaleFactor);
    void PreSave(set<KeyFrame*>& spKF, set<MapLine*>& spML);

    void AddParObservation(KeyFrame *pKF, std::vector<int> idx);
    void AddPerpObservation(KeyFrame *pKF, std::vector<int> idx);

    void EraseParObs(KeyFrame *pKF, const int &idx);
    void ErasePerpObs(KeyFrame *pKF, const int &idx);

    std::map<KeyFrame*, std::vector<int>> GetPerpObservations();
    std::map<KeyFrame*, std::vector<int>> GetParObservations();

    static Eigen::Vector4d ChangePlukerToOrth(const Vector6d& plukerLine);
    static Vector6d ChangeOrthToPluker(const Vector4d& orthLine );
    static Eigen::Matrix3d GetOrthRFromPluker(const Vector6d& plukerLine);
    static Eigen::Matrix2d GetOrthWFromPluker(const Vector6d& plukerLine);


public:
    Vector6d NVw;               
    Eigen::Vector4d orthNVw;           

    vector<Vector3f> obs_list; 
    vector<Vector4f> pts_list;  

    vector<int> kf_obs_list;   
    vector<double> sigma_list;  
    bool viewer = true; 


public:
    long unsigned int mnId; 
    static long unsigned int nNextId;
    long unsigned int mnCornerId; 
    static long unsigned int nNextCornerId;
    const long int mnFirstKFid; 
    const long int mnFirstFrame; 
    int nObs;

    // Variables used by the tracking
    float mTrackProjX1;
    float mTrackProjY1;
    float mTrackProjX2;
    float mTrackProjY2;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    double mTrackAngle;

    bool mbTrackInView;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopLineForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;
    Vector6f mPosGBA; 
    long unsigned int mnBAGlobalForKF;

    static std::mutex mGlobalMutex;

public:
    // Position in absolute coordinates
    Vector6f mPlukerLinePosW;
    Vector6f mWorldPos;
    Vector3f mLineStart3D;
    Vector3f mLineEnd3D;

    // For save relation without pointer, this is necessary for save/load function
    std::map<long unsigned int, int> mBackupObservationsId1;
    std::map<long unsigned int, int> mBackupObservationsId2;

    // KeyFrames observing the line and associated index in keyframe
    map<KeyFrame*, size_t> mObservations;  

    map<KeyFrame*, size_t> mStructGroup;   

    std::map<KeyFrame*, std::vector<int>> mParObservations;
    std::map<KeyFrame*, std::vector<int>> mPerpObservations;

    Eigen::Vector3f mNormalVector;  

    cv::Mat mLDescriptor;  

    KeyFrame* mpRefKF;  
    long unsigned int mBackupRefKFId;

    vector<cv::Mat> mvDesc_list;  
    vector<Vector3f> mvdir_list;  

    //Tracking counters
    int mnVisible;
    int mnFound;

    // Bad flag , we don't currently erase MapPoint from memory
    bool mbBad;
    MapLine* mpReplaced;

    // For save relation without pointer, this is necessary for save/load function
    long long int mBackupReplacedId;

    float mfMinDistance;
    float mfMaxDistance;

    Map* mpMap;

    std::mutex mMutexPos;
    std::mutex mMutexMap;
    std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM


#endif // MAPLINE_H
