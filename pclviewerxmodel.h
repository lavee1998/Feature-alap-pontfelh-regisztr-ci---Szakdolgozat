#ifndef PCLVIEWERXMODEL_H
#define PCLVIEWERXMODEL_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_representation.h>
#include <pcl/console/parse.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/impl/sift_keypoint.hpp>
#include <chrono>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/shot_omp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <QObject>
#include <pcl/common/time.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/random_sample.h>

using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::registration;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
namespace pcl
{
    template<>
    struct SIFTKeypointFieldSelector<PointXYZ>
    {
        inline float
        operator () (const PointXYZ &p) const
        {
            return p.z;
        }
    };
}


class PCLViewerXModel : public QObject
{
    Q_OBJECT

public:
     PCLViewerXModel();
    ~PCLViewerXModel();

    //BETÖLTÉS/MENTÉS/TÖRLÉS

    int LoadSrcPointCloud(std::string path);

    int LoadTgtPointCloud(std::string path);

    int SavePreprocessedSource(QString filename);

    int SavePreprocessedTarget(QString filename);

    void DeletePreprocessedSource();

    void DeletePreprocessedTarget();

    int SaveTransformation(QString filename);

    //FEATURE SZÁMÍTÁSOK

    int EstimateNormals (const pcl::PointCloud<PointT>::Ptr &cloud, const pcl::PointCloud<PointT>::Ptr &keypoints_cloud,
                     pcl::PointCloud<pcl::Normal> &normals_cloud, double radiusSearch);

    int EstimateSHOT (const pcl::PointCloud<PointT>::Ptr &cloud,
                  const pcl::PointCloud<pcl::Normal>::Ptr &normals_cloud,
                  const pcl::PointCloud<PointT>::Ptr &keypoints_cloud,
                  pcl::PointCloud<pcl::SHOT352> &shot_cloud,
                 double radiusSearch);

    int EstimateFPFH (const pcl::PointCloud<PointT>::Ptr &cloud,
                  const pcl::PointCloud<pcl::Normal>::Ptr &normals_cloud,
                  const pcl::PointCloud<PointT>::Ptr &keypoints_cloud,
                  pcl::PointCloud<pcl::FPFHSignature33> &fpfhs_cloud,
                   double radiusSearch);

    int EstimatePFH(const pcl::PointCloud<PointT>::Ptr &cloud,
                const pcl::PointCloud<pcl::Normal>::Ptr &normals_cloud,
                const pcl::PointCloud<PointT>::Ptr &keypoints_cloud,
                pcl::PointCloud<pcl::PFHSignature125> &pfhs_cloud,
                double radiusSearch);

    //PÁROSÍTÁS ELUTASÍTÓK
    int RejectBadCorrespondencesBasedOnRANSAC (double _setinlierthreshold , double _setmaxit);

    int RejectBadCorrespondencesBasedOnSurfaceNormals( double threshold,double setKsearch);

    int RejectBadCorrespondencesBasedOnOneToOne();

    int RejectBadCorrespondencesBasedOnDistance (double max_distance);

   /* int RejectBadCorrespondencesWithCorrespondenceRejectorDistance(const pcl::CorrespondencesPtr &all_correspondences,
                                                                   const pcl::PointCloud<PointT>::Ptr &keypoints_src,
                                                                   const pcl::PointCloud<PointT>::Ptr &keypoints_tgt,
                                                                   pcl::Correspondences &remaining_correspondences);*/
    //downsampling
    int DownSamplingBasedOnVoxelGrid (const pcl::PointCloud<PointT>::Ptr &cloud,pcl::PointCloud<PointT> &downsampled_cloud,  double leaf);

    int DownSamplingBasedOnRandomSampling (const pcl::PointCloud<PointT>::Ptr &cloud, pcl::PointCloud<PointT> &downsampled_cloud, double setsample);

    //PÁROSÍTÁSOK KERESÉSE
    template<typename T>
    int FindCorrespondences (const typename pcl::PointCloud<T >::Ptr &feature_src, const typename pcl::PointCloud<T >::Ptr &feature_tgt,bool isDirectCorrespondences );


    //KULCSPONT DETEKTÁLÁS
    int
    EstimateKeypointsBasedOnISS (const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,pcl::PointCloud<pcl::PointXYZ> &keypoints_cloud,
                                              double _iss_gamma_21, double _iss_gamma_32, int _iss_min_neighbors,
                                              double _iss_SalientRadius, double _iss_NonMaxRadius);

    int
    EstimateKeypointsBasedOnHarrisKeypoint3D(const pcl::PointCloud<PointT>::Ptr &cloud,
                                         pcl::PointCloud<pcl::PointXYZ> &keypoints_cloud,
                                         double _setThreshold, bool _setNonMaxSuspression);

    int
    EstimateKeypointsBasedOnSIFT(const pcl::PointCloud<PointT>::Ptr &cloud,
                              pcl::PointCloud<pcl::PointXYZ> &keypoints_cloud,
                              const float _min_scale,
                              const int _n_octaves ,
                              const int _n_scales_per_octave,
                              const float _min_contrast);

    //FELHŐ MÉRTÉKEK BECSLÉSE
    double
    ComputeCloudResolution (const pcl::PointCloud<PointT>::ConstPtr &cloud);

    double GetFinalScore(const pcl::PointCloud<PointT>::Ptr &input_transformed,const pcl::PointCloud<PointT>::Ptr &target,double max_range = std::numeric_limits<double>::max());

    double ComputeCloudDiameter(const pcl::PointCloud<PointT>::Ptr &cloud);

    //REGISZTRÁCIÓK

    double ICPRegistration( double icpmaxdes, double setransacthreshold,double icpmaxit,double setransacit, double setEuclideanFitnessEpsilon);

    double RANSACRegistration(double _leafsizefordownsample,double _setradiussearchfornormals, double _setradiussearchforfeaters,int _setmaxit,int _setnumberofsamples,int _setCorrespondenceRandomness,double _setsimilaritythreshold,double _setInlierFraction, double _setInlierThreshold);

    double FeatureBasedRegistration(bool _HarrisKeypoints, bool _ISSKeypoints,//, bool _SIFTKeypoints,
                                 bool _FPFH, bool _PFH,//, bool _SHOT,
                                 bool IsDirectCorrespondences,
                                 bool _RANSACREJ, bool _distanceREJ, bool _onetooneREJ, bool _surfacenormalsREJ, bool _SVDTRANS,//, bool _LMTRANS,
                                 //HARRIS
                                 double _Harris_ThresholdS, double _Harris_ThresholdT, bool _Harris_setNonMaxSupS, bool _Harris_setNonMaxSupT,
                                 //ISS
                                 double _ISS_Gamma21S, double _ISS_Gamma21T, double _ISS_Gamma32S, double _ISS_Gamma32T, int _ISS_MINNS, int _ISS_MINNT,
                                 double _ISS_SalientRadS, double _ISS_SalientRadT, double _ISS_SetNonSupS, double _ISS_SetNonSupT,
                                 //SIFT
                                 int _SIFT_noctavesS, int _SIFT_noctavesT, int _SIFT_nscalesperoctavesS, int _SIFT_nscalesperoctavesT, double _SIFT_minscaleS, double _SIFT_minscaleT, double _SIFT_mincontrastS, double _SIFT_mincontrastT,
                                 //featers
                                 double _radiusSearchForFeaters, double _radiusSearchForNormals,
                                 //rejection
                                 double _RANSACREJ_setInlierThreshold, double _RANSACREJ_setMaximumIterations, double _distanceREJ_setMaxDistance, double _surfacenormalsREJ_setThresHold, double _surfacenormalsREJ_setKSearch
                                 );

    //ELŐFELDOLGOZÁS
    int Preprocessing(bool downsampling, bool downsampling_source, bool downsampling_target, bool voxeldownsampling, bool outliersremoval, bool outliersremoval_source, bool outliersremoval_target, bool removal_statistical,
                      bool smoothing, bool smoothing_source, bool smoothing_target,
                      //downsampling
                      double randommsampling_setsample_source, double randomsampling_setsample_target,
                      double voxel_leaf_source, double voxel_leaf_target,
                      //outliersrejection
                      int statistical_setmeanK_source, double statistical_setStddevMulThres_source, bool setNegative_source,
                      int statistical_setmeanK_target, double statistical_setStddevMulThres_target, bool setNegative_target,
                      //radiusrejection
                      int radius_setMinNeighborsInRadius_source, double setRadiusSearch_source,
                      int radius_setMinNeighborsInRadius_target, double setRadiusSearch_target,
                      //Smoothing
                      double setSearchRadius_source, double setSearchRadius_target);

    int
    OutliersRemovalBasedOnStatistical(
             pcl::PointCloud<PointT>::Ptr &cloud,  pcl::PointCloud<PointT> &filtered_cloud,
             int _setmeanK, double _setStddevMulThresh, bool setNegative);


    int
    OutliersRemovalBasedOnRadius(
            pcl::PointCloud<PointT>::Ptr &cloud,  pcl::PointCloud<PointT> &filtered_cloud,
            int _setMinNeighborsInRadius, double _setRadiusSearch);



    int
    SmoothingBasedOnMovingLeastSquares( pcl::PointCloud<PointT>::Ptr &cloud,  pcl::PointCloud<PointT> &filtered_cloud,
                                               double _setSearchRadius);


    //GETTERS

    PointCloudT::Ptr getSrc() const;

    PointCloudT::Ptr getTgt() const;

    PointCloudT::Ptr getPreprocessed_src() const;

    PointCloudT::Ptr getPreprocessed_tgt() const;

    pcl::PointCloud<pcl::Normal>::Ptr getNormals_src() const;

    pcl::PointCloud<pcl::Normal>::Ptr getNormals_tgt() const;

    PointCloudT::Ptr getKeypoints_src() const;

    PointCloudT::Ptr getKeypoints_tgt() const;

    pcl::CorrespondencesPtr getGood_correspondences() const;

    PointCloudT::Ptr getOutput() const;

signals:
    void SendMessage(QString message);

private:
    PointCloudT::Ptr _src;
    PointCloudT::Ptr _tgt;
    PointCloudT::Ptr _keypoints_src;
    PointCloudT::Ptr _keypoints_tgt;
    PointCloudT::Ptr _preprocessed_src;
    PointCloudT::Ptr _preprocessed_tgt;
    pcl::CorrespondencesPtr _correspondences;
    PointCloudT::Ptr _output;
    Eigen::Matrix4f _transform;
};

#endif // PCLVIEWERXMODEL_H
