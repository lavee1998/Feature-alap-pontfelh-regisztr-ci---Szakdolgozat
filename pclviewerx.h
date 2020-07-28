#pragma once

#include <iostream>
#include <pcl/io/pcd_io.h>
// Qt
#include <QMainWindow>
#include <pcl/point_representation.h>
#include <pcl/console/parse.h>
// Point Cloud Library
#include <chrono>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/common/time.h>
#include "icpparamsdialog.h"
#include "preprocessingparamsdialog.h"
//#include "ui_preprocessingparamsdialog.h"
#include <QSpinBox>

#include "featurebasedregistrationparamsdialog.h"
#include "ransacparamsdialog.h"
//#include "ui_ransacparamsdialog.h"
//#include "ui_featurebasedregistrationparamsdialog.h"
#include <QPushButton>
#include <QMessageBox>
#include <QFileDialog>
#include <QDateTime>

using namespace pcl::io;
using namespace pcl::console;
#include "ui_icpparamsdialog.h"
// Visualization Toolkit (VTK)
#include <QVTKWidget.h>
#include <vtkRenderWindow.h>
#include "pclviewerxmodel.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui
{
  class PCLViewerX;
}

class PCLViewerX : public QMainWindow
{
  Q_OBJECT

public:
  explicit PCLViewerX (QWidget *parent = 0);
  ~PCLViewerX ();

    int
    StartVisualization();
//visualization
    void SetVisualization();
    void
    SetVisualizationAfterPreprocessing();

public Q_SLOTS:

   void
   WriteMessage(QString message);

   void
   PreprocessingPushButton_Clicked();

   void
   RegistrationPushButton_Clicked();

   void
   DeletePreprocessedSourcePushButton_Clicked();

   void
   DeletePreprocessedTargetPushButton_Clicked();

   void
   SavePreprocessedSourcePushButton_Clicked();

   void
   SavePreprocessedTargetPushButton_Clicked();

   void
   SaveTranformationPushButton_Clicked();

   void
   LoadSourcePointCloudPushButton_Clicked();

   void
   LoadTargetPointCloudPushButton_Clicked();


private:
  Ui::PCLViewerX *ui;
  PCLViewerXModel *_model;
  ICPParamsDialog *_ICPParamsDialog;
  FeatureBasedRegistrationParamsDialog *_featurebasedParamsDialog;
  RANSACParamsDialog *_RANSACParamsDialog;
  PreprocessingParamsDialog *_preprocessingParamsDailog;
  pcl::visualization::PCLVisualizer::Ptr _viewer;
  int _viewport1;
  int _viewport2;
};
