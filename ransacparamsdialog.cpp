#include "ransacparamsdialog.h"
#include "ui_ransacparamsdialog.h"

/*!
* \brief RANSACParamsDialog::RANSACParamsDialog
* \param parent
*A RANSAC alapú regisztrációhoz paraméterek bekérésére szolgáló osztály konstruktora
* ahol egy csúsztatható területbe tesszük az objektumokat, hogy kis képernyőn is minden látható legyen, és meghívjuk a paraméter elemeket
* tartalmazó elem halmazt visszaadó függvényt
*/
RANSACParamsDialog::RANSACParamsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::RANSACParamsDialog)
{   
    ui->setupUi(this);
    _mainLayout = new QVBoxLayout(this);
    _scrollArea = new QScrollArea(this);
    QWidget* scrollAreaContent = new QWidget(this);
    _subGridLayout = new QGridLayout(scrollAreaContent);
    _scrollArea->setVerticalScrollBarPolicy( Qt::ScrollBarAsNeeded );
    _scrollArea->setWidgetResizable(true);
    setContentsMargins(0,0,0,0);
    _subGridLayout->addWidget(ParametersRANSAC());
    scrollAreaContent->setLayout(_subGridLayout);
    _scrollArea->setWidget(scrollAreaContent);
    _mainLayout->addWidget(_scrollArea);
    setLayout(_mainLayout);
    _subGridLayout->setSizeConstraint(QLayout::SetMinimumSize);
}


/*! \brief RANSACParamsDialog::SetupSpinBox
* \param spinbox
* \param max
* \param min
* \param decimals
* Az egyes SpinBoxok egyszerű paraméterezését teszi lehetővé a függvény
*/
void RANSACParamsDialog::SetupSpinBox(QDoubleSpinBox &spinbox, double max, double min, int decimals)
{
    //az egyes Spinboxok tulajdonságait állítjuk be
    spinbox.setMaximum(max); //maximumát
    spinbox.setMinimum(min); //minimumát
    spinbox.setDecimals(decimals); //a tizedesjegy utáni számokat
}


/*! \brief RANSACParamsDialog::paramsforransac
* \return
*A RANSAC alapú rendezéshez szükséges paraméterek megadását lehetővé tevő elemcsoportot ad vissza a függvény. Ezt az elemcsoportot tölti fel a megfelelő
* eszközökkel, elemekkel, valamint eltárolja ezek értékeit a későbbi használathoz
*/
QGroupBox *
RANSACParamsDialog:: ParametersRANSAC()
{
    //létrehozzuk a QGroupBox-ot és a szükséges modulokat
    _groupBoxRANSAC = new QGroupBox(tr("RANSAC registration's parameters"),this);
    QPushButton *_okButton = new QPushButton(_groupBoxRANSAC);
    _okButton->setText("Start registration");
    QPushButton *_cancelButton = new QPushButton(_groupBoxRANSAC);
    _cancelButton->setText("Back");
    QPushButton *_helpButton = new QPushButton(_groupBoxRANSAC);
    _helpButton->setText("Help");
    QDoubleSpinBox *spinbox_leafsize = new QDoubleSpinBox(_groupBoxRANSAC);
    QDoubleSpinBox *spinbox_setrad_norm = new QDoubleSpinBox(_groupBoxRANSAC);
    QDoubleSpinBox *spinbox_setradf = new QDoubleSpinBox(_groupBoxRANSAC);
    QDoubleSpinBox *spinbox_setmaxit = new QDoubleSpinBox(_groupBoxRANSAC);
    QDoubleSpinBox *spinbox_numberofsamples = new QDoubleSpinBox(_groupBoxRANSAC);
    QDoubleSpinBox *spinbox_setrandomness = new QDoubleSpinBox(_groupBoxRANSAC);
    QDoubleSpinBox *spinbox_setinlierthreshold = new QDoubleSpinBox(_groupBoxRANSAC);
    QDoubleSpinBox *spinbox_sim_threshold = new QDoubleSpinBox(_groupBoxRANSAC);
    QDoubleSpinBox *spinbox_inlierfraction = new QDoubleSpinBox(_groupBoxRANSAC);

    QLabel *label_leafsize = new QLabel("Leaf Size for Voxel downsampling: ",_groupBoxRANSAC);
    QLabel *label_setrad_norm = new QLabel("Sphere radius that is to be used for determining the nearest \nneighbors used for the normal estimation: ",_groupBoxRANSAC);
    QLabel *label_setradf = new QLabel("Sphere radius that is to be used for determining the nearest \nneighbors used for the feature(FPFH) estimation:",_groupBoxRANSAC);
    QLabel *label_setmaxit = new QLabel("Number of RANSAC iterations: ",_groupBoxRANSAC);
    QLabel *label_numberofsamples = new QLabel("Number of samples to use during each iteration: ",_groupBoxRANSAC);
    QLabel *label_setrandomness = new QLabel("Number of neighbors to use when selecting a random feature correspondence: ",_groupBoxRANSAC);
    QLabel *label_sim_threshold = new QLabel("Similarity threshold between edge lengths of the underlying polygonal correspondence rejector object: ",_groupBoxRANSAC);
    QLabel *label_inlierfraction = new QLabel("Required inlier fraction (of the input) : ",_groupBoxRANSAC);
    QLabel *label_setinlierthreshold = new QLabel("Maximum distance threshold between two correspondent points in Source <-> Target",_groupBoxRANSAC);;


    _spinBoxesRANSAC["SetLeafSize"] = spinbox_leafsize;
    _spinBoxesRANSAC["SetRadiusforNormals"] = spinbox_setrad_norm;
    _spinBoxesRANSAC["SetRadiusforFeatures"] = spinbox_setradf;
    _spinBoxesRANSAC["SetMaxIt"] = spinbox_setmaxit;
    _spinBoxesRANSAC["SetNumberofSamples"] = spinbox_numberofsamples;
    _spinBoxesRANSAC["SetRandomness"] = spinbox_setrandomness;
    _spinBoxesRANSAC["SetSimThreshold"] = spinbox_sim_threshold;
    _spinBoxesRANSAC["SetInlierFraction"] = spinbox_inlierfraction;
    _spinBoxesRANSAC["SetInlierThreshold"] = spinbox_setinlierthreshold;

    _labelsRANSAC["SetLeafSize"] = label_leafsize;
    _labelsRANSAC["SetRadiusforNormals"] = label_setrad_norm;
    _labelsRANSAC["SetRadiusforFeatures"] = label_setradf;
    _labelsRANSAC["SetMaxIt"] = label_setmaxit;
    _labelsRANSAC["SetNumberofSamples"] = label_numberofsamples;
    _labelsRANSAC["SetRandomness"] = label_setrandomness;
    _labelsRANSAC["SetSimThreshold"] = label_sim_threshold;
    _labelsRANSAC["SetInlierFraction"] = label_inlierfraction;
    _labelsRANSAC["SetInlierThreshold"] = label_setinlierthreshold;

    //Egyéni SpinBox paraméterek beállítása
    SetupSpinBox(* spinbox_leafsize, 2,0,6);
    SetupSpinBox(* spinbox_setrad_norm,5,0,6);
    SetupSpinBox(* spinbox_setradf,5,0,6)  ;
    SetupSpinBox(* spinbox_sim_threshold,1,0,6);
    SetupSpinBox(* spinbox_inlierfraction, 1,0,6 );
    SetupSpinBox(* spinbox_setmaxit, 100000, 0,0);
    SetupSpinBox(* spinbox_numberofsamples, 10,0,0);
    SetupSpinBox(* spinbox_setrandomness,10,0,0 ) ;
    SetupSpinBox(*spinbox_setinlierthreshold, 5,0,6);


    //Layout feltöltése
    QGridLayout *GridLayoutRANSAC = new QGridLayout(_groupBoxRANSAC);
    GridLayoutRANSAC->addWidget(label_leafsize,0,0);
    GridLayoutRANSAC->addWidget(spinbox_leafsize,0,1);
    GridLayoutRANSAC->addWidget(label_setrad_norm,2,0);
    GridLayoutRANSAC->addWidget(spinbox_setrad_norm,2,1);
    GridLayoutRANSAC->addWidget(label_setradf,3,0);
    GridLayoutRANSAC->addWidget(spinbox_setradf,3,1);
    GridLayoutRANSAC->addWidget(label_setmaxit,4,0);
    GridLayoutRANSAC->addWidget(spinbox_setmaxit,4,1);
    GridLayoutRANSAC->addWidget(label_numberofsamples,5,0);
    GridLayoutRANSAC->addWidget(spinbox_numberofsamples,5,1);
    GridLayoutRANSAC->addWidget(label_setrandomness,6,0);
    GridLayoutRANSAC->addWidget(spinbox_setrandomness,6,1);
    GridLayoutRANSAC->addWidget(label_sim_threshold,7,0);
    GridLayoutRANSAC->addWidget(spinbox_sim_threshold,7,1);
    GridLayoutRANSAC->addWidget(label_inlierfraction,9,0);
    GridLayoutRANSAC->addWidget(spinbox_inlierfraction,9,1);
    GridLayoutRANSAC->addWidget(label_setinlierthreshold,10,0);
    GridLayoutRANSAC->addWidget(spinbox_setinlierthreshold,10,1);

    GridLayoutRANSAC->addWidget(_okButton,12,1);
    GridLayoutRANSAC->addWidget(_cancelButton,13,1);
    GridLayoutRANSAC->addWidget(_helpButton,14,1);

    connect(_okButton, SIGNAL(clicked()),this, SLOT(OnAccepted()));
    connect(_cancelButton, SIGNAL(clicked()),this, SLOT(reject()));
    connect(_helpButton, SIGNAL(clicked()),this, SLOT(ClickedHelpButton()));

    _groupBoxRANSAC->setLayout(GridLayoutRANSAC);
    return _groupBoxRANSAC;
}


/*! \brief RANSACParamsDialog::ClickedHelpButton
*Ha a felhasználó rámegy a "Help" gombra, akkor egy súgó ablak jelenik meg, és ez a függvény felel érte.
*/
void
RANSACParamsDialog :: ClickedHelpButton()
{
    QMessageBox::information(this,"Help","RANSAC-Based Registration.\n" "\nOn this dialog, you can run a RANSAC registration using FPFH feature descriptors and Voxel grid to estimate keypoints.\n "
                                         "\nRANSAC Registration is a built-in registration algorithm, which assumes that every datas we are looking at is comprised of both inliers and outliers.\n"
                             "In order to robustly align partial/occluded models, this routine performs fit error evaluation using only inliers (points closer than a Euclidean threshold).\n"
                             "\n The first parameter is for keypoints detection using Voxel grid. \n"
                             "\n The second and third parameters are for normals and feature descriptors. \n"
                             "\n Others are for RANSAC prerejective registration. You can refine your registration and set the threshold.\n"
                             "\nAttention! If certain parameters' value was zero,\nthese'd set to default value!");
}


/*! \brief RANSACParamsDialog::getSpinboxesRANSAC
* \return
*A ransac paraméterekhez tartozó GETTER függvény
*/
std::map<std::string, QDoubleSpinBox *> RANSACParamsDialog::getSpinboxesRANSAC() const
{
    return _spinBoxesRANSAC;
}



/*! \brief RANSACParamsDialog::~RANSACParamsDialog
*Destruktor
*/
RANSACParamsDialog::~RANSACParamsDialog()
{

    std::map<std::string, QDoubleSpinBox*>::iterator it;

    for ( it = _spinBoxesRANSAC.begin(); it != _spinBoxesRANSAC.end(); it++ )
    {
        it->second->deleteLater();
    }
    std::map<std::string, QLabel*>::iterator it2;

    for ( it2 = _labelsRANSAC.begin(); it2 != _labelsRANSAC.end(); it2++ )
    {
        it2->second->deleteLater();
    }
    _subGridLayout->deleteLater();
    delete ui;


}


/*! \brief RANSACParamsDialog::OnAccepted
*Ha valamilyen paramétert a felhasználó üresen hagyott, mielött megkezdené a regisztrációt a program, figyelmezteti erre a felhasználót
* ekkor lehetőség van visszalépni, vagy folytatni a regisztrációt
*/
void
RANSACParamsDialog::OnAccepted()
{
    bool ithasanynull = false;
    std::map<std::string, QDoubleSpinBox*>::iterator it;

    for ( it = _spinBoxesRANSAC.begin(); it != _spinBoxesRANSAC.end(); it++ )
    {
        if(it->second->value() == 0)
        {
            ithasanynull = true;
        }

    }

    if(ithasanynull)
    {
        QMessageBox::StandardButton reply = QMessageBox::question(this,"Warning!",
                                                                  "If any parameter's value was null, it'd set to default value! \n Want to continue?",
                                                                  QMessageBox::Yes | QMessageBox::No);
        if(reply == QMessageBox::Yes)
        {
            accept();

        }
        else
            return;
    }

    else
         accept();
}
