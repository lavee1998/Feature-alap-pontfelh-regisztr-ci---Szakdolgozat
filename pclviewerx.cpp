#include "pclviewerx.h"
#include "ui_pclviewerx.h"


/*!
* \brief PCLViewerX::PCLViewerX
* \param parent
* A PCLViewerX konstruktora ami létrehozza a modelt, valamint a felhő megjelenítő panelt pcl által biztosított vizualizációs eszközével
* Az egyes funkciógombok és funkciók között kapcsolatot hoz létre, majd meghívja a StartVisualization függvényt.
*/
PCLViewerX::PCLViewerX (QWidget *parent) : QMainWindow (parent),
  ui (new Ui::PCLViewerX)
{
    ui->setupUi (this);
    this->setWindowTitle ("PCLViewerX");
    _model = new PCLViewerXModel();

    _featurebasedParamsDialog = new FeatureBasedRegistrationParamsDialog(this);
    _RANSACParamsDialog = new RANSACParamsDialog(this);
    _ICPParamsDialog = new ICPParamsDialog(this);
    _preprocessingParamsDailog = new PreprocessingParamsDialog(this);

    // Kapcsolatok felépítése
    connect(ui->pushbutton_preprocess,SIGNAL(clicked() ), this, SLOT(PreprocessingPushButton_Clicked()));
    connect(ui->pushbutton_registration,SIGNAL(clicked() ), this, SLOT(RegistrationPushButton_Clicked()));
    connect(ui->pushButton_DeletePreprocessedSource,SIGNAL(clicked() ), this, SLOT(DeletePreprocessedSourcePushButton_Clicked()));
    connect(ui->pushButton_DeletePreprocessedTarget,SIGNAL(clicked() ), this, SLOT(DeletePreprocessedTargetPushButton_Clicked()));
    connect(ui->pushButton_LoadSrcPointCloud,SIGNAL(clicked() ), this, SLOT(LoadSourcePointCloudPushButton_Clicked()));
    connect(ui->pushButton_LoadTgtPointCloud,SIGNAL(clicked() ), this, SLOT(LoadTargetPointCloudPushButton_Clicked()));
    connect(ui->pushButton_SavePreprocessedSource,SIGNAL(clicked() ), this, SLOT(SavePreprocessedSourcePushButton_Clicked()));
    connect(ui->pushButton_SavePreprocessedTarget,SIGNAL(clicked() ), this, SLOT(SavePreprocessedTargetPushButton_Clicked()));
    connect(ui->pushButton_savetransformation,SIGNAL(clicked() ), this, SLOT(SaveTranformationPushButton_Clicked()));
    connect(_model, SIGNAL(SendMessage(QString)), this, SLOT(WriteMessage(QString)),Qt::UniqueConnection);

    // A QVTK widget beállítása a felhő megjelenítőhöz
    _viewer.reset (new pcl::visualization::PCLVisualizer ("PCLViewerX", false));
    ui->qvtkWidget->SetRenderWindow (_viewer->getRenderWindow ());
    _viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    ui->qvtkWidget->update ();
    //a kezdeti ablak értékeinek beállítása a panelen
    StartVisualization();
    (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().
                                     toString("dd.MM.yyyy hh:mm:ss ") + "Welcome to PCLViewerX...\n");

}


/*!
* \brief PCLViewerX::WriteMessage
* \param message : a megjelenítendő QString típusú üzenet
* A függvény lekezeli a bejövő SendMessage üzenetet, és kiírja azt a szöveg megjelenítő panelre
*/
void PCLViewerX::WriteMessage(QString message)
{
    (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ") + message);
}


/*!
* \brief PCLViewerX::DeletePreprocessedSourcePushButton_Clicked
* Ha a felhasználó rámegy a Delete preprocessed Source gombra, akkor ez az eseménykezelő játszódik le, és meghívja az erre megfelelő metódust
* a modell esetén, majd közli ezt a felhasználóval, illetve frissíti a felhőmegjelenítő táblát is.
*/
void PCLViewerX::DeletePreprocessedSourcePushButton_Clicked()
{
    if(_model->getSrc() == nullptr || _model->getTgt() == nullptr || _model->getSrc()->size() == 0 || _model->getTgt()->size()==0)
    {
        QMessageBox::warning(this,"Warning!", "Uncorrect input clouds! Please give correct input clouds! (.pcd)!");
        (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ") + "Uncorrect input clouds! Please give correct input clouds!");

        return;
    }

    _model->DeletePreprocessedSource();
    (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime(). toString("dd.MM.yyyy hh:mm:ss ") + "Filtered source cloud has been deleted...\n");
    SetVisualization();
}


/*!
* \brief PCLViewerX::DeletePreprocessedTargetPushButton_Clicked
*Annak az eseménye, amikor a felhasználó rámegy a "Delete Filtered Target" nevű gombra. Meghívja a _model ehhez tartozó metódusát, majd közli a felhasználóval
* a szöveg megjelenítő panelen, illetve frissíti a felhőmegjelenítő táblát is.
*/
void PCLViewerX::DeletePreprocessedTargetPushButton_Clicked()
{
    if(_model->getSrc() == nullptr || _model->getTgt() == nullptr || _model->getSrc()->size() == 0 || _model->getTgt()->size()==0)
    {
        QMessageBox::warning(this,"Warning!", "Uncorrect input clouds! Please give correct input clouds! (.pcd)!");
        (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ") + "Uncorrect input clouds! Please give correct input clouds!");

        return;
    }

    _model->DeletePreprocessedTarget();
     (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().
                                          toString("dd.MM.yyyy hh:mm:ss ") + "Filtered target cloud has been deleted...\n");
     SetVisualization();
}

/*!
* \brief PCLViewerX::SavePreprocessedSourcePushButton_Clicked
* Meghívja a _model forrás felhő mentésére szolgáló metódusát. Annak hiba visszajelzése esetén, közli a felhasználóval a hibát.
*/
void PCLViewerX::SavePreprocessedSourcePushButton_Clicked()
{
    if(_model->getSrc() == nullptr || _model->getTgt() == nullptr || _model->getSrc()->size() == 0 || _model->getTgt()->size()==0)
    {
        QMessageBox::warning(this,"Warning!", "Uncorrect input clouds! Please give correct input clouds! (.pcd)!");
        (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ") + "Uncorrect input clouds! Please give correct input clouds!");

        return;
    }

    QString filename = QFileDialog::getSaveFileName(this, tr ("Save Source point cloud"), "/home", tr ("Point cloud data (*.pcd)"));

    int return_status = _model->SavePreprocessedSource(filename);

    if (return_status == -1)
    {
      (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().
                                           toString("dd.MM.yyyy hh:mm:ss ") + "Error writing point cloud...\n" );
      return;
    }
    else if(return_status == 0)
    {
        (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().
                                             toString("dd.MM.yyyy hh:mm:ss ") + "Saving filtered source cloud finished...\n" );
    }


}

/*!
* \brief PCLViewerX::SavePreprocessedTargetPushButton_Clicked
*Meghívja a _model célfelhő mentésére szolgáló metódusát. Ha hibát jelez vissza a _model, akkor a nézet jelzi ezt a felhasználónak.
*/
void PCLViewerX::SavePreprocessedTargetPushButton_Clicked()
{
    if(_model->getSrc() == nullptr || _model->getTgt() == nullptr || _model->getSrc()->size() == 0 || _model->getTgt()->size()==0)
    {
        QMessageBox::warning(this,"Warning!", "Uncorrect input clouds! Please give correct input clouds! (.pcd)!");
        (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ") + "Uncorrect input clouds! Please give correct input clouds!");

        return;
    }
    QString filename = QFileDialog::getSaveFileName(this, tr ("Save Target point cloud"), "/home/", tr ("Point cloud data (*.pcd)"));

    int return_status = _model->SavePreprocessedTarget(filename);

    if (return_status != 0)
    {
      (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().
                                           toString("dd.MM.yyyy hh:mm:ss ") + "Error writing point cloud...\n" );
      return;
    }
    (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().
                                         toString("dd.MM.yyyy hh:mm:ss ") + "Saving filtered target cloud finished...\n" );

}

/*!
* \brief PCLViewerX::SaveTranformationPushButton_Clicked
* Meghívja a _model Transzformációs mátrix mentésére szolgáló metódusát. Ha hibát kap vissza, közli a felhasználóval
*/
void PCLViewerX::SaveTranformationPushButton_Clicked()
{
    if(_model->getSrc() == nullptr || _model->getTgt() == nullptr || _model->getSrc()->size() == 0 || _model->getTgt()->size()==0)
    {
        QMessageBox::warning(this,"Warning!", "Uncorrect input clouds! Please give correct input clouds! (.pcd)!");
        (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ") + "Uncorrect input clouds! Please give correct input clouds!");

        return;
    }

    QString filename = QFileDialog::getSaveFileName(0, tr ("Save tranformation"), "/home/", tr ("Txt (*.txt)"));

    int return_status = _model->SaveTransformation(filename);
    if(return_status != 1)
    {
        (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().
                                                toString("dd.MM.yyyy hh:mm:ss ") + "Error Saving transformation... ");
        return;
    }
   (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().
                                           toString("dd.MM.yyyy hh:mm:ss ") + "Saving transformation finished... ");
}


/*!
* \brief PCLViewerX::StartVisualization
* \return 0
* Létrehozza az egymástól elszeparált részeit a felhő megjelenítő panelnek. Beállítja a szövegeket hozzá és az arányaikat.
*/
int PCLViewerX::StartVisualization()
{
    _viewer->createViewPort(0.0, 0.0, 0.5, 1.0, _viewport1);
    _viewer->setBackgroundColor (0, 0, 0, _viewport1);
    _viewer->addText("Input-Target Clouds", 10, 10, "viewport1 text", _viewport1);
    _viewer->createViewPort(0.5, 0.0, 1.0, 1.0, _viewport2);
    _viewer->setBackgroundColor (0.3, 0.3, 0.3, _viewport2);
    _viewer->addText("Transformated cloud", 10, 10, "viewport2 text", _viewport2);
    _viewer->resetCamera ();
    return 0;
}


/*!
* \brief PCLViewerX::~PCLViewerX
*Az osztály destruktora, ami törli a megfelelő pointereket.
*/
PCLViewerX::~PCLViewerX ()
{
  delete ui;
  delete _ICPParamsDialog;
  delete _model;
  delete _RANSACParamsDialog;
  delete _featurebasedParamsDialog;
  delete _preprocessingParamsDailog;

}

/*!
 * \brief PCLViewerX::SetVisualization
 *A felhő megjelenítő panel frissítésére szolgáló metódus. A bal oldalra felhelyezi a bemeneti felhőket,míg a jobb oldalra pedig a transzformált felhőt, valamint
 * a célfelhőt, az esetleges kulcspontokat illetve a párosításokat is bejelöli.
 */
void PCLViewerX::SetVisualization()
{
    _viewer->removeAllPointClouds(_viewport1);
    _viewer->removeAllPointClouds(_viewport2);
    _viewer->removeCorrespondences("Correspondences");
    _viewer->updateText("Transformated cloud", 10, 10, "viewport2 text");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color_filteredsource(_model->getPreprocessed_src() ,255,255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color_filteredtarget(_model->getPreprocessed_tgt(), 0, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color_target(_model->getTgt(), 0, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color_output(_model->getOutput(),  255, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color_keypointssource(_model->getKeypoints_src(),  255,0 , 255 );
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color_keypointstarget(_model->getKeypoints_tgt(),255, 179, 0);
    _viewer->addPointCloud<PointT> (_model->getKeypoints_src(), single_color_keypointssource, "keypoints source", _viewport1);
    _viewer->addPointCloud<PointT> (_model->getKeypoints_tgt(), single_color_keypointstarget, "keypoints target", _viewport1);
    _viewer->addPointCloud<PointT> (_model->getPreprocessed_src(), single_color_filteredsource, "source",_viewport1);
    _viewer->addPointCloud<PointT> (_model->getOutput(), single_color_output, "output", _viewport2);
    _viewer->addPointCloud<PointT> (_model->getTgt(), single_color_target, "target1",_viewport2);
    _viewer->addPointCloud<PointT> (_model->getPreprocessed_tgt(), single_color_filteredtarget, "target2",_viewport1);
    _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source");
    _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "keypoints source");
    _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "keypoints target");
    _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target1");
    _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target2");
    _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output");
    _viewer->addCorrespondences<PointT>(_model->getKeypoints_src(), _model->getKeypoints_tgt(), *(_model->getGood_correspondences()), "Correspondences", _viewport1);
    _viewer->resetCamera ();
    ui->qvtkWidget->update ();
}


/*!
 *\brief PCLViewerX::SetVisualizationAfterPreprocessing
 *A felhő megjelenítő frissítésére szolgáló metódus az előfeldolgozást követően. Ekkor az előfeldolgozott felhőket a panel jobb oldalára helyezi
 * míg az eredeti bemeneti felhőket pedig a bal oldalára. Mindenek elött eltávolítja a
 * két részről az összes felhőt, valamint a párosításokat.
 */
void PCLViewerX::SetVisualizationAfterPreprocessing()
{
    _viewer->removeAllPointClouds(_viewport1);
    _viewer->removeAllPointClouds(_viewport2);
    _viewer->removeCorrespondences("Correspondences");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color7(_model->getPreprocessed_src(), 255, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color8(_model->getPreprocessed_tgt(),  0, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color1(_model->getSrc(), 255, 255 , 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color2(_model->getTgt(),  0, 255, 255);
    _viewer->updateText("Filtered clouds", 10, 10, "viewport2 text");

    _viewer->addPointCloud<PointT> (_model->getSrc(), single_color1, "sample cloud1", _viewport1);
    _viewer->addPointCloud<PointT> (_model->getTgt(), single_color2, "sample cloud2",_viewport1);
    _viewer->addPointCloud<PointT> (_model->getPreprocessed_src(), single_color7, "sample cloud7", _viewport2);
    _viewer->addPointCloud<PointT> (_model->getPreprocessed_tgt(), single_color8, "sample cloud8",_viewport2);
    _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud1");
    _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud2");
    _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud7");
    _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud8");

     _viewer->resetCamera ();
     ui->qvtkWidget->update ();
}


/*!
* \brief PCLViewerX::RegistrationPushButton_Clicked
* Ha a felhasználó a regisztráció gombra megy, akkor ez a metódus fut le. A 3 ide tartozó rádiógombtól függ, hogy a  melyik regisztrációs
* folyamat hívódik meg. Ekkor megjelenik a megfelelő folyamat paramétereinek megadására szolgáló dialógus ablak, ahol a felhasználó megadhatja
* hogy milyen paraméterekkel szeretne regisztrálni. Helyes beolvasás esetén a megfelelő paraméterekkel
* az esemény meghívja a modell alkalmas regisztrációs folyamatát.  Az első a Feature alapú, a második az ICP alapú, míg a harmadik pedig a RANSAC alapú regisztrációs algoritmus. Ha üres a felhő
* egyből hibát jelez, és nem indítja el a folyamatokat
*/
void PCLViewerX::RegistrationPushButton_Clicked()
{
    double result_status = 0;
    if(_model->getSrc() == nullptr || _model->getTgt() == nullptr || _model->getSrc()->size() == 0 || _model->getTgt()->size()==0)
    {
        QMessageBox::warning(this,"Warning!", "Uncorrect input clouds! Please give correct input clouds! (.pcd)!");
        (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ") + "Uncorrect input clouds! Please give correct input clouds!");

        return;
    }

    if(ui->radioButton_FB->isChecked())
    {
        (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ") + " You choose Feature Based Registration!\n");
        _featurebasedParamsDialog->show();

        if(_featurebasedParamsDialog->exec() == QDialog::Accepted)
        {
            result_status = _model->FeatureBasedRegistration(
                   _featurebasedParamsDialog->getRadiobuttonsFeatureBasedRegistration()["KEYPOINTS_HARRIS"]->isChecked(),
                   _featurebasedParamsDialog->getRadiobuttonsFeatureBasedRegistration()["KEYPOINTS_ISS"]->isChecked(),
                   _featurebasedParamsDialog->getRadiobuttonsFeatureBasedRegistration()["FEATUREDESCRIPTOR_FPFH"]->isChecked(),
                   _featurebasedParamsDialog->getRadiobuttonsFeatureBasedRegistration()["FEATUREDESCRIPTOR_PFH"]->isChecked(),
                   _featurebasedParamsDialog->getRadiobuttonsFeatureBasedRegistration()["CORRESPONDENCES_DIRECT"]->isChecked(),
                   _featurebasedParamsDialog->getCheckboxesFeatureBasedRegistration()["REJECTION_RANSAC"]->isChecked(),
                   _featurebasedParamsDialog->getCheckboxesFeatureBasedRegistration()["REJECTION_DISTANCE"]->isChecked(),
                   _featurebasedParamsDialog->getCheckboxesFeatureBasedRegistration()["REJECTION_ONETOONE"]->isChecked(),
                   _featurebasedParamsDialog->getCheckboxesFeatureBasedRegistration()["REJECTION_SURFACENORMALS"]->isChecked(),
                   _featurebasedParamsDialog->getRadiobuttonsFeatureBasedRegistration()["TRANSFORMATION_SVD"]->isChecked(),
                   _featurebasedParamsDialog->getSpinboxesFeatureBasedRegistration()["KEYPOINTS_HARRIS_ThresHoldSource"]->value(),
                   _featurebasedParamsDialog->getSpinboxesFeatureBasedRegistration()["KEYPOINTS_HARRIS_ThresHoldTarget"]->value(),
                   _featurebasedParamsDialog->getCheckboxesFeatureBasedRegistration()["KEYPOINTS_HARRIS_SetNonMaxSupSource"]->isChecked(),
                   _featurebasedParamsDialog->getCheckboxesFeatureBasedRegistration()["KEYPOINTS_HARRIS_SetNonMaxSupTarget"]->isChecked(),
                   _featurebasedParamsDialog->getSpinboxesFeatureBasedRegistration()["KEYPOINTS_ISS_Gamma21Source"]->value(),
                   _featurebasedParamsDialog->getSpinboxesFeatureBasedRegistration()["KEYPOINTS_ISS_Gamma21Target"]->value(),
                   _featurebasedParamsDialog->getSpinboxesFeatureBasedRegistration()["KEYPOINTS_ISS_Gamma32Source"]->value(),
                   _featurebasedParamsDialog->getSpinboxesFeatureBasedRegistration()["KEYPOINTS_ISS_Gamma32Target"]->value(),
                   _featurebasedParamsDialog->getSpinboxesFeatureBasedRegistration()["KEYPOINTS_ISS_MinNeighborsSource"]->value(),
                   _featurebasedParamsDialog->getSpinboxesFeatureBasedRegistration()["KEYPOINTS_ISS_MinNeighborsTarget"]->value(),
                   _featurebasedParamsDialog->getSpinboxesFeatureBasedRegistration()["KEYPOINTS_ISS_SalientRadS"]->value(),
                   _featurebasedParamsDialog->getSpinboxesFeatureBasedRegistration()["KEYPOINTS_ISS_SalientRadT"]->value(),
                   _featurebasedParamsDialog->getSpinboxesFeatureBasedRegistration()["KEYPOINTS_ISS_NonMaxSupS"]->value(),
                   _featurebasedParamsDialog->getSpinboxesFeatureBasedRegistration()["KEYPOINTS_ISS_NonMaxSupT"]->value(),
                   _featurebasedParamsDialog->getSpinboxesFeatureBasedRegistration()["KEYPOINTS_SIFT_NoctaveSource"]->value(),
                   _featurebasedParamsDialog->getSpinboxesFeatureBasedRegistration()["KEYPOINTS_SIFT_NoctaveTarget"]->value(),
                   _featurebasedParamsDialog->getSpinboxesFeatureBasedRegistration()["KEYPOINTS_SIFT_NScalePerOctaveSource"]->value(),
                   _featurebasedParamsDialog->getSpinboxesFeatureBasedRegistration()["KEYPOINTS_SIFT_NScalePerOctaveTarget"]->value(),
                   _featurebasedParamsDialog->getSpinboxesFeatureBasedRegistration()["KEYPOINTS_SIFT_MinScaleSource"]->value() ,
                   _featurebasedParamsDialog->getSpinboxesFeatureBasedRegistration()["KEYPOINTS_SIFT_MinScaleTarget"]->value(),
                   _featurebasedParamsDialog->getSpinboxesFeatureBasedRegistration()["KEYPOINTS_SIFT_MinContrastSource"]->value(),
                   _featurebasedParamsDialog->getSpinboxesFeatureBasedRegistration()["KEYPOINTS_SIFT_MinContrastTarget"]->value(),
                   _featurebasedParamsDialog->getSpinboxesFeatureBasedRegistration()["FEATUREDESCRIPTOR_SetRadiusSearchForFeatures"]->value(),
                   _featurebasedParamsDialog->getSpinboxesFeatureBasedRegistration()["FEATUREDESCRIPTOR_SetRadiusSearchForNormals"]->value(),
                   _featurebasedParamsDialog->getSpinboxesFeatureBasedRegistration()["REJECTION_SetInlierThreshold_RANSAC"]->value(),
                   _featurebasedParamsDialog->getSpinboxesFeatureBasedRegistration()["REJECTION_SetMaximumIterations_RANSAC"]->value(),
                   _featurebasedParamsDialog->getSpinboxesFeatureBasedRegistration()["REJECTION_SetMaxDistance_DISTANCE"]->value(),
                   _featurebasedParamsDialog->getSpinboxesFeatureBasedRegistration()["REJECTION_SetThresHold_SURFACENORMALS"]->value(),
                   _featurebasedParamsDialog->getSpinboxesFeatureBasedRegistration()["REJECTION_SetRadiusSearch_SURFACENORMALS"]->value()
                   );

           if(result_status >= 0)
           {
                SetVisualization();
                (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ") + "Transformation Error Metric: " + QString::number(result_status) + "m!\n");
           }
           else if(result_status == -1)
           {
                 QMessageBox::critical(this,"Error","Problem during Normal estimation. (Maybe some of the clouds have any NAN point). \nPlease change the parameters and try again!");
                 (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ") + "Problem during Normal estimation. (Maybe some of the clouds have any NAN point). \nPlease change the parameters and try again!");

           }
           else if(result_status == -2)
           {
                 QMessageBox::critical(this,"Error","Problem during Keypoints Estimation. \n(Maybe Keypoints estimation doesn't found any key point in some of the clouds) \nPlease change the parameters and try again!");
                 (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ") + "Problem during Keypoints Estimation. \n(Maybe Keypoints estimation doesn't found any key point in some of the clouds) \nPlease change the parameters and try again!");

           }
           else if(result_status == -3)
           {
                QMessageBox::critical(this,"Error","Problem during Feature Descriptors Estimation. \nPlease change the parameters and try again!");
                (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ") + "Problem during Feature Descriptors Estimation. \nPlease change the parameters and try again!");

           }
           else if(result_status == -4)
           {
                QMessageBox::critical(this,"Error","Problem during Correspondences estimation. (Maybe there are less than 5 correspondences.) \nPlease change the parameters and try again!");
                (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ") + "Problem during Correspondences estimation. (Maybe there are less than 5 correspondences.) \nPlease change the parameters and try again!");

           }
           else if(result_status == -5)
           {
                QMessageBox::critical(this,"Error","Problem during Correspondences Rejection. (Maybe there are less than 5 correspondences after rejection.) \nPlease change the parameters and try again!");
                (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ") + "Problem during Correspondences Rejection. (Maybe there are less than 5 correspondences after rejection.) \nPlease change the parameters and try again!");

           }
           else if(result_status == -6)
           {
               QMessageBox::critical(this,"Error","Problem during Transformation. You need at least 5 correspondences to estimate transformion!\n");
               (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ") + "Problem during Transformation. You need at least 5 correspondences to estimate transformion!\n");

           }
           else if(result_status == -7)
           {
               QMessageBox::critical(this,"Error","Problem during Feature Based Registration. Some of the onput clouds is empty!\n");
               (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ") + "Problem during Feature Based Registration. Some of the onput clouds is empty!\n");

           }
        }
    }
    else if(ui->radioButton_ICP->isChecked())
    {
        (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ") + " You choose ICP Registration!\n");
        _ICPParamsDialog->show();
        if(_ICPParamsDialog->exec() == QDialog::Accepted)
        {

            result_status = _model->ICPRegistration(_ICPParamsDialog->getSpinboxesICP()["SetCorrMaxDist"]->value(),
                    _ICPParamsDialog->getSpinboxesICP()["SetRANSACThreshold"]->value(),
                    _ICPParamsDialog->getSpinboxesICP()["SetMaxIterations"]->value(),
                    _ICPParamsDialog->getSpinboxesICP()["SetRANSACIterations"]->value(),
                    _ICPParamsDialog->getSpinboxesICP()["SetEuclideanFitnessEpsilon"]->value()
            );

            if(result_status >= 0)
            {
                (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ") + "ICP Registration finished. \nTransformation Error Metric: " + QString::number(result_status) + "m!\n");
                SetVisualization();
            }
            else if(result_status == -1)
            {
                QMessageBox::critical(this,"Error","Some of input clouds are empty. Please give correct input clouds!\n");
                (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().
                                                     toString("dd.MM.yyyy hh:mm:ss ") + "Some of input clouds are empty. Please give correct input clouds!\n");
            }
            else
            {
                QMessageBox::critical(this,"Error","Error during ICP-Registration...\n");
                (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().
                                                     toString("dd.MM.yyyy hh:mm:ss ") + "Error during ICP-Registration...\n");
            }
            SetVisualization();
        }
    }
    else
    {
        (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ") + " You choose RANSAC Registration!\n");
        _RANSACParamsDialog->show();
        if(_RANSACParamsDialog->exec() == QDialog::Accepted)
        {
            result_status =_model->RANSACRegistration(_RANSACParamsDialog->getSpinboxesRANSAC()["SetLeafSize"]->value(),
                    _RANSACParamsDialog->getSpinboxesRANSAC()["SetRadiusforNormals"]->value(),
                    _RANSACParamsDialog->getSpinboxesRANSAC()["SetRadiusforFeatures"]->value(),
                    _RANSACParamsDialog->getSpinboxesRANSAC()["SetMaxIt"]->value(),
                    _RANSACParamsDialog->getSpinboxesRANSAC()["SetNumberofSamples"]->value(),
                    _RANSACParamsDialog->getSpinboxesRANSAC()["SetRandomness"]->value(),
                    _RANSACParamsDialog->getSpinboxesRANSAC()["SetSimThreshold"]->value(),
                    _RANSACParamsDialog->getSpinboxesRANSAC()["SetInlierFraction"]->value(),
                    _RANSACParamsDialog->getSpinboxesRANSAC()["SetInlierThreshold"]->value());
            if(result_status >= 0)
            {
                (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ") + "RANSAC-Registration finished. \nTransformation Error Metric: " + QString::number(result_status) + "m!\n");
                SetVisualization();
            }
            else if(result_status == -3)
            {
                  QMessageBox::critical(this,"Error","Problem during Normal estimation. (Maybe some of the clouds have any NAN point). \nPlease change the parameters and try again!");
                  (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ") + "Problem during Normal estimation. (Maybe some of the clouds have any NAN point). \nPlease change the parameters and try again!");

            }
            else if(result_status == -2)
            {
                  QMessageBox::critical(this,"Error","Problem during Keypoints Estimation. \n(Maybe Keypoints estimation doesn't found any key point in some of the clouds) \nPlease change the parameters and try again!");
                  (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ") + "Problem during Keypoints Estimation. \n(Maybe Keypoints estimation doesn't found any key point in some of the clouds) \nPlease change the parameters and try again!");

            }
            else if(result_status == -4)
            {
                 QMessageBox::critical(this,"Error","Problem during Feature Descriptors Estimation. \nPlease change the parameters and try again!");
                 (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ") + "Problem during Feature Descriptors Estimation. \nPlease change the parameters and try again!");

            }
            else if(result_status == -5)
            {
                 QMessageBox::critical(this,"Error","Problem during RANSAC-Registration. Your parameters are too strict. \nThe RANSAC-algorithm doesn't found any transformation, which is satisfies your criterions. \nPlease change the parameters and try again!\n");
                 (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ") + "Problem during RANSAC-Registration. Your parameters are too strict. \nThe RANSAC-algorithm doesn't found any transformation, which is satisfies your criterions. \nPlease change the parameters and try again!\n");

            }
            else if(result_status == -1)
            {
                QMessageBox::critical(this,"Error","Problem during RANSAC-Registration. Some of the onput clouds is empty!\n");
                (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ") + "Problem during RANSAC-Registration. Some of the onput clouds is empty!\n");

            }
            else
            {
                (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().
                                                     toString("dd.MM.yyyy hh:mm:ss ") + "Error during RANSAC-Registration...\n");
            }
        }
    }
}


/*!
* \brief PCLViewerX::LoadSourcePointCloudPushButton_Clicked()
* Ha a felhasználó a Load Source Cloud gombra megy, akkor ez a metódus fog lefutni. Meghívja a modellnek a forrás felhő betöltésére szolgáló
* metódusát, hiba esetén jelzi a felhasználónak. Helyes beolvasás esetén a szöveg megjelenik a megjelenítő panelen
*/
void PCLViewerX::LoadSourcePointCloudPushButton_Clicked()
{
    QString src_string = QFileDialog::getOpenFileName(this,"Load Source Point Cloud", "C://", "PCD Files (*.pcd)");
    int result = _model->LoadSrcPointCloud(src_string.toStdString());
    if(result == -1)
    {
        (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().
                                             toString("dd.MM.yyyy hh:mm:ss ") + "Error in source point cloud loading...\n");

        QMessageBox::warning(this,"Warning!", "Uncorrect input cloud! Please give correct input cloud! (.pcd)!");
        (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ") + "Uncorrect input cloud! Please give correct input cloud!");

        return;
    }
    else if(result == 0)
    {
        (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().
                                             toString("dd.MM.yyyy hh:mm:ss ") + "Source cloud has been successfully loaded...\n");
        (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().
                                             toString("dd.MM.yyyy hh:mm:ss ") + "Source has " + QString::number(_model->getSrc()->points.size()) + " points!\n");

        (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().
                                             toString("dd.MM.yyyy hh:mm:ss ") + "Source's diameter: " + QString::number(_model->ComputeCloudDiameter(_model->getSrc())) + " m!\nSource cloud's resolution: " + QString::number(_model->ComputeCloudResolution(_model->getSrc())) + " m!\n");
         SetVisualization();
    }
}



/*!
* \brief PCLViewerX::LoadTargetPointCloudPushButton_Clicked
* Ha a felhasználó a Load Target Cloud gombra megy, akkor ez a metódus fog lefutni. Meghívja a modellnek a forrás felhő betöltésére szolgáló
* metódusát, hiba esetén jelzi a felhasználónak, az eredményt kiírja szöveg megjelenítő panelre.
*/
void PCLViewerX::LoadTargetPointCloudPushButton_Clicked()
{
    QString tgt_string = QFileDialog::getOpenFileName(this,"Load Target Point Cloud", "C://", "PCD Files (*.pcd)");
    int result = _model->LoadTgtPointCloud(tgt_string.toStdString());

    if(result == -1)
    {
        (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().
                                             toString("dd.MM.yyyy hh:mm:ss ") + "Error in target point cloud loading...\n");
        return;
    }
    else if(result == 0)
    {
        (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().
                                             toString("dd.MM.yyyy hh:mm:ss ") + "Target cloud has been successfully loaded...\n");
        (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().
                                             toString("dd.MM.yyyy hh:mm:ss ") + "Target has " + QString::number(_model->getTgt()->points.size()) + " points!\n");
        (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().
                                             toString("dd.MM.yyyy hh:mm:ss ") + "Target's diameter: " + QString::number(_model->ComputeCloudDiameter(_model->getTgt())) + " m!\n");
        (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().
                                             toString("dd.MM.yyyy hh:mm:ss ") + "Target cloud's resolution: " + QString::number(_model->ComputeCloudResolution(_model->getTgt())) + " m!\n");
        SetVisualization();
    }
}



/*!
* \brief PCLViewerX::PreprocessingPushButton_Clicked
*Ha a felhasználó a Preprocessing gombra kattint, akkor ez a metódus fog lefutni. Helytelen bemeneti felhők esetén a program egyből hibát jelez. Megjeleníti
* Az előfeldolgozáshoz szükséges bemeneti paraméterek megadására szolgáló dialógus ablakot ahol af elhasználó megadhatja a kívánt paramétereket
* Sikeres paramétermegadás esetén a program elvégzi az előfeldolgozást (meghívja a modellnek az előfeldolgozásra szolgáló metódusát), majd az eredményt
* megjeleníti a képernyőn
*/
void PCLViewerX::PreprocessingPushButton_Clicked()
{
    int preprocessing_status;
    if(_model->getSrc() == nullptr || _model->getTgt() == nullptr || _model->getSrc()->size() == 0 || _model->getTgt()->size()==0)
    {
        (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ") + "Uncorrect input clouds! Please give correct input clouds!");
         QMessageBox::warning(this,"Warning!", "Uncorrect input clouds! Please give correct input clouds! (.pcd)!");
        return;
    }
    _preprocessingParamsDailog->show();

    if(_preprocessingParamsDailog->exec() == QDialog::Accepted)
    {
       (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ") + "Preprocessing has started...!\n");
       preprocessing_status = _model->Preprocessing(
               _preprocessingParamsDailog->getGroupboxesPreprocessing()["DOWNSAMPLING"]->isChecked(),
               _preprocessingParamsDailog->getCheckboxesPreprocessing()["Downsampling_OnSource"]->isChecked(),
               _preprocessingParamsDailog->getCheckboxesPreprocessing()["Downsampling_OnTarget"]->isChecked(),
               _preprocessingParamsDailog->getRadiobuttonsPreprocessing()["Downsampling_Voxel"]->isChecked(),
               _preprocessingParamsDailog->getGroupboxesPreprocessing()["OUTLIERSREMOVAL"]->isChecked(),
               _preprocessingParamsDailog->getCheckboxesPreprocessing()["OUTREM_OnSource"]->isChecked(),
               _preprocessingParamsDailog->getCheckboxesPreprocessing()["OUTREM_OnTarget"]->isChecked(),
               _preprocessingParamsDailog->getRadiobuttonsPreprocessing()["OUTREM_STAT_OUTREM"]->isChecked(),
               _preprocessingParamsDailog->getGroupboxesPreprocessing()["SMOOTHING"]->isChecked(),
               _preprocessingParamsDailog->getCheckboxesPreprocessing()["SMOOTHING_OnSource"]->isChecked(),
               _preprocessingParamsDailog->getCheckboxesPreprocessing()["SMOOTHING_OnTarget"]->isChecked(),
               //params
               //downsampling
               _preprocessingParamsDailog->getSpinboxesPreprocessing()["Downsampling_SetSampleforSource_Random"]->value(),
               _preprocessingParamsDailog->getSpinboxesPreprocessing()["Downsampling_SetSampleforTarget_Random"]->value(),
               _preprocessingParamsDailog->getSpinboxesPreprocessing()["Downsampling_SetLeafSizeforSource_Voxel"]->value(),
               _preprocessingParamsDailog->getSpinboxesPreprocessing()["Downsampling_SetLeafSizeforTarget_Voxel"]->value(),
               //removal STAT
               _preprocessingParamsDailog->getSpinboxesPreprocessing()["OUTREM_SetMeanKSource_STAT"]->value(),
               _preprocessingParamsDailog->getSpinboxesPreprocessing()["OUTREM_SetStddevMulThreshSource_STAT"]->value(),
               _preprocessingParamsDailog->getCheckboxesPreprocessing()["OUTREM_SetNegativeSource_STAT"],
               _preprocessingParamsDailog->getSpinboxesPreprocessing()["OUTREM_SetMeanKTarget_STAT"]->value(),
               _preprocessingParamsDailog->getSpinboxesPreprocessing()["OUTREM_SetStddevMulThreshTarget_STAT"]->value(),
               _preprocessingParamsDailog->getCheckboxesPreprocessing()["OUTREM_SetNegativeTarget_STAT"],
               //removal RAD
               _preprocessingParamsDailog->getSpinboxesPreprocessing()["OUTREM_SetMinNeighborsInRadiusSource_RAD"]->value(),
               _preprocessingParamsDailog->getSpinboxesPreprocessing()["OUTREM_SetRadiusSearchSource_RAD"]->value(),
               _preprocessingParamsDailog->getSpinboxesPreprocessing()["OUTREM_SetMinNeighborsInRadiusTarget_RAD"]->value(),
               _preprocessingParamsDailog->getSpinboxesPreprocessing()["OUTREM_SetRadiusSearchTarget_RAD"]->value(),
               //Smoothing
               _preprocessingParamsDailog->getSpinboxesPreprocessing()["SMOOTHING_SetSearchRadiusSource_MLS"]->value(),
               _preprocessingParamsDailog->getSpinboxesPreprocessing()["SMOOTHING_SetSearchRadiusTarget_MLS"]->value()
       );
       if(preprocessing_status == 0)
       {
           (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().
                                                toString("dd.MM.yyyy hh:mm:ss ") + "Preprocessing finished.\n");

           (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().
                                                toString("dd.MM.yyyy hh:mm:ss ") + "Source has " + QString::number(_model->getPreprocessed_src()->points.size()) + " points after preprocessing \nSource's diameter: " + QString::number(_model->ComputeCloudDiameter(_model->getPreprocessed_src())) + " m after preprocessing!\nSource cloud's resolution: " + QString::number(_model->ComputeCloudResolution(_model->getPreprocessed_src())) + " m after preprocessing!\n");
           (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().
                                                toString("dd.MM.yyyy hh:mm:ss ") + "Target has " + QString::number(_model->getPreprocessed_tgt()->points.size()) + " points after preprocessing \nTarget's diameter: " + QString::number(_model->ComputeCloudDiameter(_model->getPreprocessed_tgt())) + " m after preprocessing!\nTarget cloud's resolution: " + QString::number(_model->ComputeCloudResolution(_model->getPreprocessed_tgt())) + " m after preprocessing!\n");
       }
       else if(preprocessing_status == -1)
       {
           QMessageBox::critical(this,"Error","Problem during downsampling. \nPlease change the parameters and try again!\n");
           (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ") + "Problem during downsampling. \nPlease change the parameters and try again!\n");
       }
       else if(preprocessing_status == -2)
       {
           QMessageBox::critical(this,"Error","Problem during outliers removal. \nPlease change the parameters and try again!\n");
           (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ") + "Problem during outliers removal. \nPlease change the parameters and try again!\n");
       }
       else if(preprocessing_status == -3)
       {
           QMessageBox::critical(this,"Error","Problem during smoothing. \nPlease change the parameters and try again!\n");
           (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ") + "Problem during smoothing. \nPlease change the parameters and try again!\n");
       }
       else if(preprocessing_status == -4)
       {
           QMessageBox::critical(this,"Error","Problem during preprocessing. Some of your input clouds are empty. \nPlease give correct input clouds!\n");
           (ui->plainTextEdit)->appendPlainText(QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ") + "Problem during preprocessing. Some of your input clouds are empty. \nPlease give correct input clouds!\n");
       }
       SetVisualizationAfterPreprocessing();
    }
}
