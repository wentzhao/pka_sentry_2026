#pragma once

#include <ctime>
#include <cassert>
#include <cmath>
#include <utility>
#include <vector>
#include <algorithm> 
#include <cstdlib>
#include <memory>
#include <iostream>
#include <fstream> // [新增] 必须包含，用于文件写入
#include <string>  // [新增] 用于路径字符串

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "nanoflann.hpp"
#include "KDTreeVectorOfVectorsAdaptor.h"
#include "tictoc.h"

using namespace Eigen;
using namespace nanoflann;

using std::cout;
using std::endl;
using std::make_pair;

using std::atan2;
using std::cos;
using std::sin;

using SCPointType = pcl::PointXYZI; 

using KeyMat = std::vector<std::vector<float> >;
using InvKeyTree = KDTreeVectorOfVectorsAdaptor< KeyMat, float >;

// namespace SC2
// {

void coreImportTest ( void );

// sc param-independent helper functions 
float xy2theta( const float & _x, const float & _y );
MatrixXd circshift( MatrixXd &_mat, int _num_shift );
std::vector<float> eig2stdvec( MatrixXd _eigmat );

class SCManager
{
public: 
    SCManager( ) = default; 

    Eigen::MatrixXd makeScancontext( pcl::PointCloud<SCPointType> & _scan_down );
    Eigen::MatrixXd makeRingkeyFromScancontext( Eigen::MatrixXd &_desc );
    Eigen::MatrixXd makeSectorkeyFromScancontext( Eigen::MatrixXd &_desc );

    int fastAlignUsingVkey ( MatrixXd & _vkey1, MatrixXd & _vkey2 ); 
    double distDirectSC ( MatrixXd &_sc1, MatrixXd &_sc2 ); 
    std::pair<double, int> distanceBtnScanContext ( MatrixXd &_sc1, MatrixXd &_sc2 ); 

    // =================================================================================
    // User-side API (修改区域)
    // =================================================================================
    void makeAndSaveScancontextAndKeys( pcl::PointCloud<SCPointType> & _scan_down );
    std::pair<int, float> detectLoopClosureID( void ); 
    std::pair<int, float> detectLoopClosureID(const int& count);

    // [新增 1] 保存指定索引的 ScanContext 到文件 (对应 Scancontext.cpp 的实现)
    void saveScanContextAndKeys(const std::string &save_dir, const int &index);

    // [新增 2] 获取指定索引的 ScanContext 矩阵 (安全访问接口)
    Eigen::MatrixXd getScanContext(int index) {
        if(index >= 0 && index < polarcontexts_.size())
            return polarcontexts_[index];
        return Eigen::MatrixXd::Zero(PC_NUM_RING, PC_NUM_SECTOR);
    }
    
    // for ltmapper 
    const Eigen::MatrixXd& getConstRefRecentSCD(void);

public:
    // hyper parameters ()
    const double LIDAR_HEIGHT = 2.0; 
    const int    PC_NUM_RING = 20; 
    const int    PC_NUM_SECTOR = 60; 
    const double PC_MAX_RADIUS = 80.0; 
    const double PC_UNIT_SECTORANGLE = 360.0 / double(PC_NUM_SECTOR);
    const double PC_UNIT_RINGGAP = PC_MAX_RADIUS / double(PC_NUM_RING);

    // tree
    const int    NUM_EXCLUDE_RECENT = 30; 
    const int    NUM_CANDIDATES_FROM_TREE = 3; 

    // loop thres
    const double SEARCH_RATIO = 0.1; 
    const double SC_DIST_THRES = 0.65;

    // config 
    const int    TREE_MAKING_PERIOD_ = 10; 
    int          tree_making_period_conter = 0;

    // data 
    std::vector<double> polarcontexts_timestamp_; 
    std::vector<Eigen::MatrixXd> polarcontexts_; // 注意：这里虽然在 public 下，但在某些编译器行为下，直接访问不如用 get 函数安全
    std::vector<Eigen::MatrixXd> polarcontext_invkeys_;
    std::vector<Eigen::MatrixXd> polarcontext_vkeys_;

    KeyMat polarcontext_invkeys_mat_;
    KeyMat polarcontext_invkeys_to_search_;

    std::unique_ptr<InvKeyTree> polarcontext_tree_;

}; // SCManager

// } // namespace SC2
