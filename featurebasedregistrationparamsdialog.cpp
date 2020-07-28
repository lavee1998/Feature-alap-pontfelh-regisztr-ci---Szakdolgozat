#include "featurebasedregistrationparamsdialog.h"
#include "ui_featurebasedregistrationparamsdialog.h"


/*!
* \brief FeatureBasedRegistrationParamsDialog::FeatureBasedRegistrationParamsDialog
* \param parent : a szülő paramétere
* Az osztály Konstruktora
* A külön folyamatokhoz külön GroupBoxokat hozunk létre, ezzel elszeparálva őket egymástól a kezelhető felület megvalósításának érdekében, ezeket
* egy GridLayoutba tesszük, majd mivel a szövegek helyigényesek, csúsztathatóvá tesszük a réteget, hogy kisebb képernyőkőn is látható legyen minden
*/
FeatureBasedRegistrationParamsDialog::FeatureBasedRegistrationParamsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::FeatureBasedRegistrationParamsDialog)
{
    ui->setupUi(this);

    _mainLayout = new QVBoxLayout(this);
    _scrollArea = new QScrollArea(this);

    QWidget* scrollAreaContent = new QWidget(_scrollArea);
    _subGridLayout = new QGridLayout(scrollAreaContent);
    _scrollArea->setVerticalScrollBarPolicy( Qt::ScrollBarAsNeeded );
    _scrollArea->setWidgetResizable(true);
    setContentsMargins(0,0,0,0);

    _subGridLayout->addWidget(ParametersKeypoints(), 0,0);
    _subGridLayout->addWidget(ParametersFeatureDescriptors(), 0,1);
    _subGridLayout->addWidget(ParametersCorrespondences(), 0,2);
    _subGridLayout->addWidget(ParametersRejection(), 1,0);
    _subGridLayout->addWidget(ParametersTransformation(),1,1);
    _subGridLayout->addWidget(Actionbuttons(),1,2);

    //egyes betűméretek beállítása
    std::map<std::string, QLabel*>::iterator it;

    for ( it = _labelsFeatureBasedRegistration.begin(); it != _labelsFeatureBasedRegistration.end(); it++ )
    {
        it->second->setStyleSheet("font: 9pt;");
    }

   scrollAreaContent->setLayout(_subGridLayout);
   _scrollArea->setWidget(scrollAreaContent);
   _mainLayout->addWidget(_scrollArea);
   setLayout(_mainLayout);
}


/*!
* \brief FeatureBasedRegistrationParamsDialog::SetupSpinBox
* \param spinbox : a módosítani kívánt spinbox
* \param max : a maximuma
* \param min : a minimuma
* \param decimals : a decimálisok száma a tizedesjegy után
* Az egyes QDoubleSpinBox típusú elemek speciális beállításai
*/
void FeatureBasedRegistrationParamsDialog::SetupSpinBox(QDoubleSpinBox &spinbox, double max, double min, int decimals)
{
    //az egyes Spinboxok tulajdonságait állítjuk be
    spinbox.setMaximum(max); //maximumát
    spinbox.setMinimum(min); //minimumát
    spinbox.setDecimals(decimals); //a tizedesjegy utáni számokat
}


/*!
* \brief FeatureBasedRegistrationParamsDialog::Actionbuttons
* \return : a visszaadni kívánt QGroupBox* típusú paraméter csoport mutatója
*Az elfogadó illetve elutasító gombok megjelenítésére szolgáló objektum csoportot adja vissza layouttal ellátva
*/
QGroupBox*
FeatureBasedRegistrationParamsDialog:: Actionbuttons()
{
    //létrehozzuk a QGroupBox típusú objektumot, majd az egyes gombokat, és beállítjuk az értékeiket
    QGroupBox* _actionButtons=new QGroupBox(this);
    _actionButtons->setTitle("Action buttons");
    QPushButton* _okbutton = new QPushButton(_actionButtons);
    _okbutton->setText("Start Registration");
    QPushButton* _cancelbutton = new QPushButton(_actionButtons);
    _cancelbutton->setText("Back");
    QPushButton* _helpbutton = new QPushButton(_actionButtons);
    _helpbutton->setText("Help");

    //felépítjük a kapcsolatot az egyes gombok és eseménykezelők között
    connect(_okbutton,SIGNAL(clicked() ),this, SLOT(accept()));
    connect(_cancelbutton,SIGNAL(clicked() ),this, SLOT(reject()));
    connect(_helpbutton,SIGNAL(clicked() ),this, SLOT(ClickedHelpButton()));

    QFormLayout *formLayout = new QFormLayout(_actionButtons);
    formLayout->addRow(_okbutton);
    formLayout->addRow(_cancelbutton);
    formLayout->addRow(_helpbutton);

    _actionButtons->setLayout(formLayout);
    return _actionButtons;
}

/*!
 * \brief FeatureBasedRegistrationParamsDialog::ClickedHelpButton
 *Ha a felhasználó a HELP feliratú gombra kattint akkor ez a metódus játszódik le, ami megjelenít egy üzenet dobozt a benne lévő
 * tanácsokkal, útbaigazításokkal, rövid információkkal.
 */
void
FeatureBasedRegistrationParamsDialog :: ClickedHelpButton()
{
    QMessageBox::information(this,"Help","Feature Based Registration.\n" "\nOn this dialog, you can build your own feature based registration pipeline."
                             "\n1. Step : select a keypoints detection method using the radiobuttons. After you have chosen, set the parameters which belong to your selected detector.\n"
                             "\n2. Step : select a Feature estimation method using the radiobuttons. After you have chosen, set the parameters which belong to your selected algorithm.\n"
                             "\n3. Step : select a correspondence estimation method using the radiobuttons.\n"
                             "\n4. Step : select some correspondence rejection methods using the checkboxes. After you have chosen, set the parameters which belong to your selected rejectors.\n"
                             "\n5. Step : select a transformation estimation method using the radiobuttons.\n"
                             "\nFinally : If you're ready, click on the 'Start Registration' button.\n"
                             "\nAttention! If certain parameters' value was zero,\nthese'd set to default value!");
}

/*!
* \brief FeatureBasedRegistrationParamsDialog::getRadiobuttonsFeatureBasedRegistration
* \return a rádiógombokhoz tartozó map tárolót adja vissza
* GETTER függvény a rádiógombokhoz
*/
std::map<std::string, QRadioButton *> FeatureBasedRegistrationParamsDialog::getRadiobuttonsFeatureBasedRegistration() const
{
    return _radiobuttonsFeatureBasedRegistration;
}


/*!
* \brief FeatureBasedRegistrationParamsDialog::getCheckboxesFeatureBasedRegistration
* \return a Checkboxokhoz tartozó map tárolót adja vissza
*GETTER függvény a jelőlő négyzetekhez
*/
std::map<std::string, QCheckBox *> FeatureBasedRegistrationParamsDialog::getCheckboxesFeatureBasedRegistration() const
{
    return _checkboxesFeatureBasedRegistration;
}


/*!
* \brief FeatureBasedRegistrationParamsDialog::getSpinboxesFeatureBasedRegistration
* \return a SpinBoxokhoz tartozó map tárolót adja vissza
*GETTER függvény a SpinBox-okhoz
*/
std::map<std::string, QDoubleSpinBox *> FeatureBasedRegistrationParamsDialog::getSpinboxesFeatureBasedRegistration() const
{
    return _spinboxesFeatureBasedRegistration;
}


/*!
* \brief FeatureBasedRegistrationParamsDialog::ParametersforKeypoints
* \return: a kulcspontok elrendezésére szolgáló QGroupBox* típusú paraméter csoport pointere
* A kulcspontdetektálókhoz tartozó paraméter csoportok inicializálása, majd értékének visszaadása, illetve a megfelelő tárolóban való eltárolásuk
* a későbbi használat végett
*/
QGroupBox *
FeatureBasedRegistrationParamsDialog::ParametersKeypoints()
{
    _groupBoxesFeatureBasedRegistration["KEYPOINTS"] = new QGroupBox(tr("Keypoints estimation"));

    //Harris-hez tartozó paraméterek inicializálása
    QRadioButton *radio_Harris = new QRadioButton(tr("Harris Keypoints"));
    _radiobuttonsFeatureBasedRegistration["KEYPOINTS_HARRIS"] = radio_Harris;
    radio_Harris->setChecked(true);
    QDoubleSpinBox *spinbox_Harris_ThresHoldS = new QDoubleSpinBox();
    QDoubleSpinBox *spinbox_Harris_ThresHoldT = new QDoubleSpinBox();
    SetupSpinBox(*spinbox_Harris_ThresHoldS,2,0,9);
    SetupSpinBox(*spinbox_Harris_ThresHoldT,2,0,9);
    QLabel *label_Harris_ThresHoldS = new QLabel("Threshold value for detecting corners (Source): ");
    QLabel *label_Harris_ThresHoldT = new QLabel("Threshold value for detecting corners (Target): ");
    QLabel *label_Harris_setNonMaxSupS = new QLabel("Whether non maxima suppression \nshould be applied for Source: ");
    QLabel *label_Harris_setNonMaxSupT = new QLabel("Whether non maxima suppression \nshould be applied for Target: ");
    QCheckBox *checkbox_Harris_setNonMaxSupS = new QCheckBox();
    QCheckBox *checkbox_Harris_setNonMaxSupT = new QCheckBox();
    _labelsFeatureBasedRegistration["KEYPOINTS_HARRIS_ThresHoldSource"] = label_Harris_ThresHoldS;
    _labelsFeatureBasedRegistration["KEYPOINTS_HARRIS_ThresHoldTarget"] = label_Harris_ThresHoldT;
    _labelsFeatureBasedRegistration["KEYPOINTS_HARRIS_SetNonMaxSupSource"] = label_Harris_setNonMaxSupS;
    _labelsFeatureBasedRegistration["KEYPOINTS_HARRIS_SetNonMaxSupTarget"] = label_Harris_setNonMaxSupT;

    _checkboxesFeatureBasedRegistration["KEYPOINTS_HARRIS_SetNonMaxSupSource"] = checkbox_Harris_setNonMaxSupS;
    _checkboxesFeatureBasedRegistration["KEYPOINTS_HARRIS_SetNonMaxSupTarget"] = checkbox_Harris_setNonMaxSupT;
    _spinboxesFeatureBasedRegistration["KEYPOINTS_HARRIS_ThresHoldSource"] = spinbox_Harris_ThresHoldS;
    _spinboxesFeatureBasedRegistration["KEYPOINTS_HARRIS_ThresHoldTarget"] = spinbox_Harris_ThresHoldT;

    //ISS-hez tartozó paraméterek inicializálása

    QRadioButton *radio_ISS = new QRadioButton(tr("ISS KeyPoints Estimation"));
    _radiobuttonsFeatureBasedRegistration["KEYPOINTS_ISS"] = radio_ISS;
    QDoubleSpinBox *spinbox_ISS_Gamma21S = new QDoubleSpinBox();
    QDoubleSpinBox *spinbox_ISS_Gamma32S = new QDoubleSpinBox();
    QDoubleSpinBox *spinbox_ISS_Gamma21T = new QDoubleSpinBox();
    QDoubleSpinBox *spinbox_ISS_Gamma32T = new QDoubleSpinBox();
    QDoubleSpinBox *spinbox_ISS_SalientRadT = new QDoubleSpinBox();
    QDoubleSpinBox *spinbox_ISS_SalientRadS = new QDoubleSpinBox();
    QDoubleSpinBox *spinbox_ISS_NonMaxSupT = new QDoubleSpinBox();
    QDoubleSpinBox *spinbox_ISS_NonMaxSupS = new QDoubleSpinBox();
    QDoubleSpinBox *spinbox_ISS_MINNS = new QDoubleSpinBox();
    QDoubleSpinBox *spinbox_ISS_MINNT = new QDoubleSpinBox();
    SetupSpinBox(*spinbox_ISS_Gamma21S,5,0,5);
    SetupSpinBox(*spinbox_ISS_Gamma21T,5,0,5);
    SetupSpinBox(*spinbox_ISS_Gamma32S,5,0,5);
    SetupSpinBox(*spinbox_ISS_Gamma32T,5,0,5);
    SetupSpinBox(*spinbox_ISS_MINNS,60,0,0);
    SetupSpinBox(*spinbox_ISS_MINNT,60,0,0);
    SetupSpinBox(*spinbox_ISS_SalientRadS,10,0,5);
    SetupSpinBox(*spinbox_ISS_SalientRadT,10,0,5);
    SetupSpinBox(*spinbox_ISS_NonMaxSupS,10,0,5);
    SetupSpinBox(*spinbox_ISS_NonMaxSupT,10,0,5);

    QLabel *label_ISS_Gamma21S = new QLabel("Upper bound on the ratio \nbetween the 2. and the 1. eigenvalue (Source): ");
    QLabel *label_ISS_Gamma32S = new QLabel("Upper bound on the ratio \nbetween the 3. and the 2. eigenvalue (Source): ");
    QLabel *label_ISS_MINNS = new QLabel("Minimum number of neighbors that has to be found (Source): ");
    QLabel *label_ISS_Gamma21T = new QLabel("Upper bound on the ratio \nbetween the 2. and the 1. eigenvalue (Target): ");
    QLabel *label_ISS_Gamma32T = new QLabel("Upper bound on the ratio \nbetween the 3. and the 2. eigenvalue (Target): ");
    QLabel *label_ISS_MINNT = new QLabel("Minimum number of neighbors that has to be found (Target): ");
    QLabel *label_ISS_SalientRadiusS = new QLabel("Radius of the spherical neighborhood (Source): ");
    QLabel *label_ISS_SalientRadiusT = new QLabel("Radius of the spherical neighborhood (Target): ");
    QLabel *label_ISS_NonMaxSupS = new QLabel("Non maxima suppression radius (Source): ");
    QLabel *label_ISS_NonMaxSupT = new QLabel("Non maxima suppression radius (Target): ");

    _labelsFeatureBasedRegistration["KEYPOINTS_ISS_Gamma21Source"] =label_ISS_Gamma21S ;
    _labelsFeatureBasedRegistration["KEYPOINTS_ISS_Gamma21Target"] = label_ISS_Gamma21T;
    _labelsFeatureBasedRegistration["KEYPOINTS_ISS_Gamma32Source"] = label_ISS_Gamma32S;
    _labelsFeatureBasedRegistration["KEYPOINTS_ISS_Gamma32Target"] = label_ISS_Gamma32T;
    _labelsFeatureBasedRegistration["KEYPOINTS_ISS_MinNeighborsSource"] = label_ISS_MINNS;
    _labelsFeatureBasedRegistration["KEYPOINTS_ISS_MinNeighborsTarget"] =label_ISS_MINNT ;
    _labelsFeatureBasedRegistration["KEYPOINTS_ISS_SalientRadS"] =label_ISS_SalientRadiusS ;
    _labelsFeatureBasedRegistration["KEYPOINTS_ISS_SalientRadT"] =label_ISS_SalientRadiusT ;
    _labelsFeatureBasedRegistration["KEYPOINTS_ISS_NonMaxSupS"] =label_ISS_NonMaxSupS ;
    _labelsFeatureBasedRegistration["KEYPOINTS_ISS_NonMaxSupT"] =label_ISS_NonMaxSupT ;


    _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_Gamma21Source"] = spinbox_ISS_Gamma21S;
    _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_Gamma21Target"] = spinbox_ISS_Gamma21T;
    _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_Gamma32Source"] = spinbox_ISS_Gamma32S;
    _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_Gamma32Target"] = spinbox_ISS_Gamma32T;
    _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_MinNeighborsSource"] =spinbox_ISS_MINNS;
    _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_MinNeighborsTarget"] =spinbox_ISS_MINNT;
    _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_SalientRadS"] =spinbox_ISS_SalientRadS;
    _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_SalientRadT"] =spinbox_ISS_SalientRadT;
    _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_NonMaxSupS"] =spinbox_ISS_NonMaxSupS;
    _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_NonMaxSupT"] =spinbox_ISS_NonMaxSupT;

    //SIFT-hez tartozó paraméterek inicializálása
    QRadioButton *radio_SIFT = new QRadioButton(tr("SIFT KeyPoints Estimation"));
    _radiobuttonsFeatureBasedRegistration["KEYPOINTS_SIFT"] = radio_SIFT;
    QDoubleSpinBox *spinbox_SIFT_noctavesS = new QDoubleSpinBox();
    QDoubleSpinBox *spinbox_SIFT_noctavesT = new QDoubleSpinBox();
    QDoubleSpinBox *spinbox_SIFT_nscalesperoctaveS = new QDoubleSpinBox();
    QDoubleSpinBox *spinbox_SIFT_nscalesperoctaveT = new QDoubleSpinBox();
    QDoubleSpinBox *spinbox_SIFT_minscaleS = new QDoubleSpinBox();
    QDoubleSpinBox *spinbox_SIFT_minscaleT = new QDoubleSpinBox();
    QDoubleSpinBox *spinbox_SIFT_mincontrastS = new QDoubleSpinBox();
    QDoubleSpinBox *spinbox_SIFT_mincontrastT = new QDoubleSpinBox();

    SetupSpinBox(*spinbox_SIFT_noctavesS ,20,0,0);
    SetupSpinBox(*spinbox_SIFT_noctavesT ,20,0,0);
    SetupSpinBox(*spinbox_SIFT_nscalesperoctaveS ,10,0,0);
    SetupSpinBox(* spinbox_SIFT_nscalesperoctaveT,10,0,0);
    SetupSpinBox(*spinbox_SIFT_minscaleS ,0.5,0,4);
    SetupSpinBox(* spinbox_SIFT_minscaleT,0.5,0,4);
    SetupSpinBox(* spinbox_SIFT_mincontrastS,1,0,6);
    SetupSpinBox(* spinbox_SIFT_mincontrastT,1,0,6);

    QLabel *label_SIFT_minscaleS = new QLabel("Standard deviation of the smallest scale(Source): ");
    QLabel *label_SIFT_minscaleT = new QLabel("Standard deviation of the smallest scale(Target): ");
    QLabel *label_SIFT_noctavesS = new QLabel("Number of octaves to compute(Source): ");
    QLabel *label_SIFT_noctavesT = new QLabel("Number of octaves to compute(Target): ");
    QLabel *label_SIFT_nscalesperoctaveS = new QLabel("Number of scales to compute within each octave(Source): ");
    QLabel *label_SIFT_nscalesperoctaveT = new QLabel("Number of scales to compute within each octave(Target): ");
    QLabel *label_SIFT_minconstrastS = new QLabel("Threshold to limit detection of keypoints(Source): ");
    QLabel *label_SIFT_minconstrastT= new QLabel("Threshold to limit detection of keypoints(Target): ");

    _labelsFeatureBasedRegistration["KEYPOINTS_SIFT_NoctaveSource"] = label_SIFT_noctavesS;
    _labelsFeatureBasedRegistration["KEYPOINTS_SIFT_NoctaveTarget"] = label_SIFT_noctavesT;
    _labelsFeatureBasedRegistration["KEYPOINTS_SIFT_NScalePerOctaveSource"] = label_SIFT_nscalesperoctaveS;
    _labelsFeatureBasedRegistration["KEYPOINTS_SIFT_NScalePerOctaveTarget"] = label_SIFT_nscalesperoctaveT;
    _labelsFeatureBasedRegistration["KEYPOINTS_SIFT_MinScaleSource"] =label_SIFT_minscaleS ;
    _labelsFeatureBasedRegistration["KEYPOINTS_SIFT_MinScaleTarget"] = label_SIFT_minscaleT;
    _labelsFeatureBasedRegistration["KEYPOINTS_SIFT_MinContrastSource"] =label_SIFT_minconstrastS ;
    _labelsFeatureBasedRegistration["KEYPOINTS_SIFT_MinContrastTarget"] = label_SIFT_minconstrastT;

    _spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_NoctaveSource"] = spinbox_SIFT_noctavesS;
    _spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_NoctaveTarget"] = spinbox_SIFT_noctavesT;
    _spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_NScalePerOctaveSource"] = spinbox_SIFT_nscalesperoctaveS;
    _spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_NScalePerOctaveTarget"] = spinbox_SIFT_nscalesperoctaveT;
    _spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_MinScaleSource"] = spinbox_SIFT_minscaleS;
    _spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_MinScaleTarget"] = spinbox_SIFT_minscaleT;
    _spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_MinContrastSource"] = spinbox_SIFT_mincontrastS;
    _spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_MinContrastTarget"] = spinbox_SIFT_mincontrastT;

    //kapcsolatok felépítése az elemek és az események között
    connect(radio_Harris,SIGNAL(clicked() ),this, SLOT(ClickedRadioButtonKeypointsHarris3D() ) );
    connect(radio_ISS,SIGNAL(clicked() ),this, SLOT(ClickedRadioButtonKeypointsISS() ) );
    connect(radio_SIFT,SIGNAL(clicked()) ,this ,SLOT(ClickedRadioButtonKeypointsSIFT()) );

    QFormLayout *formLayout = new QFormLayout;
    formLayout->addRow(radio_Harris);
    formLayout->addRow(radio_ISS);
    formLayout->addRow(radio_SIFT);
    formLayout->addRow(label_Harris_ThresHoldS,spinbox_Harris_ThresHoldS);
    formLayout->addRow(label_Harris_ThresHoldT,spinbox_Harris_ThresHoldT);
    formLayout->addRow(label_Harris_setNonMaxSupS,checkbox_Harris_setNonMaxSupS);
    formLayout->addRow(label_Harris_setNonMaxSupT,checkbox_Harris_setNonMaxSupT);

    //megjelöljük, hogy az előző lépésben mi volt bejelölve
    IsLastSelectedHarris = true;
    IsLastSelectedISS = false;
    IsLastSelectedSIFT = false;
    _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->setLayout(formLayout);
    return _groupBoxesFeatureBasedRegistration["KEYPOINTS"];

}


/*!
* \brief FeatureBasedRegistrationParamsDialog::ClickedRadioButtonKeypointsISS
* Az ISS alapú algoritmushoz tartozó paraméterek beállítása ha a felhasználó a hozzá tartozó rádiógombra megy. Láthatóvá tesszük ezeket az értékeket
* valamint, amik eddig láthatók voltak, azokat eltüntetjük
*/
void
FeatureBasedRegistrationParamsDialog::ClickedRadioButtonKeypointsISS()
{
    QFormLayout *formLayout = new QFormLayout;
    formLayout->addRow(_radiobuttonsFeatureBasedRegistration["KEYPOINTS_HARRIS"]);
    formLayout->addRow(_radiobuttonsFeatureBasedRegistration["KEYPOINTS_ISS"]);
    formLayout->addRow(_radiobuttonsFeatureBasedRegistration["KEYPOINTS_SIFT"]);

    formLayout->addRow(_labelsFeatureBasedRegistration["KEYPOINTS_ISS_Gamma21Source"],_spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_Gamma21Source"]);
    formLayout->addRow(_labelsFeatureBasedRegistration["KEYPOINTS_ISS_Gamma21Target"],_spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_Gamma21Target"]);
    formLayout->addRow(_labelsFeatureBasedRegistration["KEYPOINTS_ISS_Gamma32Source"],_spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_Gamma32Source"]);
    formLayout->addRow(_labelsFeatureBasedRegistration["KEYPOINTS_ISS_Gamma32Target"],_spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_Gamma32Target"]);
    formLayout->addRow(_labelsFeatureBasedRegistration["KEYPOINTS_ISS_MinNeighborsSource"],_spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_MinNeighborsSource"]);
    formLayout->addRow(_labelsFeatureBasedRegistration["KEYPOINTS_ISS_MinNeighborsTarget"],_spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_MinNeighborsTarget"]);
    formLayout->addRow(_labelsFeatureBasedRegistration["KEYPOINTS_ISS_SalientRadS"],_spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_SalientRadS"]);
    formLayout->addRow(_labelsFeatureBasedRegistration["KEYPOINTS_ISS_SalientRadT"],_spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_SalientRadT"]);
    formLayout->addRow(_labelsFeatureBasedRegistration["KEYPOINTS_ISS_NonMaxSupS"],_spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_NonMaxSupS"]);
    formLayout->addRow(_labelsFeatureBasedRegistration["KEYPOINTS_ISS_NonMaxSupT"],_spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_NonMaxSupT"]);

    _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_Gamma21Source"]->setVisible(true);
    _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_Gamma21Target"]->setVisible(true);
    _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_Gamma32Source"]->setVisible(true);
    _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_Gamma32Target"]->setVisible(true);
    _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_MinNeighborsSource"]->setVisible(true);
    _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_MinNeighborsTarget"]->setVisible(true);
    _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_SalientRadS"]->setVisible(true);
    _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_SalientRadT"]->setVisible(true);
    _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_NonMaxSupS"]->setVisible(true);
    _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_NonMaxSupT"]->setVisible(true);

    _labelsFeatureBasedRegistration["KEYPOINTS_ISS_Gamma21Source"]->setVisible(true);
    _labelsFeatureBasedRegistration["KEYPOINTS_ISS_Gamma21Target"]->setVisible(true);
    _labelsFeatureBasedRegistration["KEYPOINTS_ISS_Gamma32Source"]->setVisible(true);
    _labelsFeatureBasedRegistration["KEYPOINTS_ISS_Gamma32Target"]->setVisible(true);
    _labelsFeatureBasedRegistration["KEYPOINTS_ISS_MinNeighborsSource"]->setVisible(true);
    _labelsFeatureBasedRegistration["KEYPOINTS_ISS_MinNeighborsTarget"]->setVisible(true);
    _labelsFeatureBasedRegistration["KEYPOINTS_ISS_SalientRadS"]->setVisible(true);
    _labelsFeatureBasedRegistration["KEYPOINTS_ISS_SalientRadT"]->setVisible(true);
    _labelsFeatureBasedRegistration["KEYPOINTS_ISS_NonMaxSupS"]->setVisible(true);
    _labelsFeatureBasedRegistration["KEYPOINTS_ISS_NonMaxSupT"]->setVisible(true);

    if(IsLastSelectedHarris)
    {
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_HARRIS_ThresHoldSource"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_HARRIS_ThresHoldTarget"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_checkboxesFeatureBasedRegistration["KEYPOINTS_HARRIS_SetNonMaxSupSource"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_checkboxesFeatureBasedRegistration["KEYPOINTS_HARRIS_SetNonMaxSupTarget"]);

        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_HARRIS_ThresHoldSource"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_HARRIS_ThresHoldTarget"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_HARRIS_SetNonMaxSupSource"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_HARRIS_SetNonMaxSupTarget"]);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_HARRIS_ThresHoldSource"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_HARRIS_ThresHoldTarget"]->setVisible(false);
        _checkboxesFeatureBasedRegistration["KEYPOINTS_HARRIS_SetNonMaxSupSource"]->setVisible(false);
        _checkboxesFeatureBasedRegistration["KEYPOINTS_HARRIS_SetNonMaxSupTarget"]->setVisible(false);

        _labelsFeatureBasedRegistration["KEYPOINTS_HARRIS_ThresHoldSource"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_HARRIS_ThresHoldTarget"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_HARRIS_SetNonMaxSupSource"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_HARRIS_SetNonMaxSupTarget"]->setVisible(false);


    }
    else if(IsLastSelectedSIFT)
    {
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_MinScaleSource"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_MinScaleTarget"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_MinContrastSource"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_MinContrastTarget"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_NoctaveSource"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_NoctaveTarget"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_NScalePerOctaveSource"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_NScalePerOctaveTarget"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_SIFT_NoctaveSource"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_SIFT_NoctaveTarget"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_SIFT_NScalePerOctaveSource"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_SIFT_NScalePerOctaveTarget"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_SIFT_MinScaleSource"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_SIFT_MinScaleTarget"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_SIFT_MinContrastSource"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_SIFT_MinContrastTarget"]);

        _spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_MinScaleSource"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_MinScaleTarget"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_MinContrastSource"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_MinContrastTarget"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_NoctaveSource"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_NoctaveTarget"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_NScalePerOctaveSource"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_NScalePerOctaveTarget"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_SIFT_NoctaveSource"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_SIFT_NoctaveTarget"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_SIFT_NScalePerOctaveSource"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_SIFT_NScalePerOctaveTarget"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_SIFT_MinScaleSource"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_SIFT_MinScaleTarget"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_SIFT_MinContrastSource"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_SIFT_MinContrastTarget"]->setVisible(false);
    }
    _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_radiobuttonsFeatureBasedRegistration["KEYPOINTS_HARRIS"]);
    _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_radiobuttonsFeatureBasedRegistration["KEYPOINTS_ISS"]);
    _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_radiobuttonsFeatureBasedRegistration["KEYPOINTS_SIFT"]);
    IsLastSelectedISS = true;
    delete _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout();
    _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->setLayout(formLayout);
}


/*!
* \brief FeatureBasedRegistrationParamsDialog::ClickedRadioButtonKeypointsHarris3D
** A Harris alapú algoritmushoz tartozó paraméterek beállítása ha a felhasználó a hozzá tartozó rádiógombra megy. Láthatóvá tesszük ezeket az értékeket
* valamint, amik eddig láthatók voltak, azokat eltüntetjük
*/
void
FeatureBasedRegistrationParamsDialog::ClickedRadioButtonKeypointsHarris3D()
{
    QFormLayout *formLayout = new QFormLayout;
    formLayout->addRow(_radiobuttonsFeatureBasedRegistration["KEYPOINTS_HARRIS"]);
    formLayout->addRow(_radiobuttonsFeatureBasedRegistration["KEYPOINTS_ISS"]);
    formLayout->addRow(_radiobuttonsFeatureBasedRegistration["KEYPOINTS_SIFT"]);
    formLayout->addRow(_labelsFeatureBasedRegistration["KEYPOINTS_HARRIS_ThresHoldSource"],_spinboxesFeatureBasedRegistration["KEYPOINTS_HARRIS_ThresHoldSource"]);
    formLayout->addRow(_labelsFeatureBasedRegistration["KEYPOINTS_HARRIS_ThresHoldTarget"],_spinboxesFeatureBasedRegistration["KEYPOINTS_HARRIS_ThresHoldTarget"]);
    formLayout->addRow(_labelsFeatureBasedRegistration["KEYPOINTS_HARRIS_SetNonMaxSupSource"],_checkboxesFeatureBasedRegistration["KEYPOINTS_HARRIS_SetNonMaxSupSource"]);
    formLayout->addRow(_labelsFeatureBasedRegistration["KEYPOINTS_HARRIS_SetNonMaxSupTarget"],_checkboxesFeatureBasedRegistration["KEYPOINTS_HARRIS_SetNonMaxSupTarget"]);

    _spinboxesFeatureBasedRegistration["KEYPOINTS_HARRIS_ThresHoldSource"]->setVisible(true);
    _spinboxesFeatureBasedRegistration["KEYPOINTS_HARRIS_ThresHoldTarget"]->setVisible(true);
    _checkboxesFeatureBasedRegistration["KEYPOINTS_HARRIS_SetNonMaxSupSource"]->setVisible(true);
    _checkboxesFeatureBasedRegistration["KEYPOINTS_HARRIS_SetNonMaxSupTarget"]->setVisible(true);
    _labelsFeatureBasedRegistration["KEYPOINTS_HARRIS_ThresHoldSource"]->setVisible(true);
    _labelsFeatureBasedRegistration["KEYPOINTS_HARRIS_ThresHoldTarget"]->setVisible(true);
    _labelsFeatureBasedRegistration["KEYPOINTS_HARRIS_SetNonMaxSupSource"]->setVisible(true);
    _labelsFeatureBasedRegistration["KEYPOINTS_HARRIS_SetNonMaxSupTarget"]->setVisible(true);

    if(IsLastSelectedISS)
    {
        IsLastSelectedISS = false;

        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_Gamma21Source"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_Gamma21Target"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_Gamma32Source"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_Gamma32Target"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_MinNeighborsSource"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_MinNeighborsTarget"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_SalientRadS"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_SalientRadT"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_NonMaxSupS"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_NonMaxSupT"]);

        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_ISS_Gamma21Source"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_ISS_Gamma21Target"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_ISS_Gamma32Source"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_ISS_Gamma32Target"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_ISS_MinNeighborsSource"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_ISS_MinNeighborsTarget"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_ISS_SalientRadS"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_ISS_SalientRadT"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_ISS_NonMaxSupS"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_ISS_NonMaxSupT"]);

        _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_Gamma21Source"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_Gamma21Target"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_Gamma32Source"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_Gamma32Target"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_MinNeighborsSource"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_MinNeighborsTarget"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_SalientRadS"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_SalientRadT"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_NonMaxSupS"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_NonMaxSupT"]->setVisible(false);

        _labelsFeatureBasedRegistration["KEYPOINTS_ISS_Gamma21Source"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_ISS_Gamma21Target"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_ISS_Gamma32Source"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_ISS_Gamma32Target"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_ISS_MinNeighborsSource"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_ISS_MinNeighborsTarget"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_ISS_SalientRadS"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_ISS_SalientRadT"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_ISS_NonMaxSupS"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_ISS_NonMaxSupT"]->setVisible(false);

    }
    if(IsLastSelectedSIFT)
    {
        IsLastSelectedSIFT = false;

        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_MinScaleSource"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_MinScaleTarget"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_MinContrastSource"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_MinContrastTarget"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_NoctaveSource"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_NoctaveTarget"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_NScalePerOctaveSource"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_NScalePerOctaveTarget"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_SIFT_NoctaveSource"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_SIFT_NoctaveTarget"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_SIFT_NScalePerOctaveSource"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_SIFT_NScalePerOctaveTarget"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_SIFT_MinScaleSource"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_SIFT_MinScaleTarget"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_SIFT_MinContrastSource"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_SIFT_MinContrastTarget"]);

        _spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_MinScaleSource"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_MinScaleTarget"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_MinContrastSource"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_MinContrastTarget"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_NoctaveSource"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_NoctaveTarget"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_NScalePerOctaveSource"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_NScalePerOctaveTarget"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_SIFT_NoctaveSource"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_SIFT_NoctaveTarget"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_SIFT_NScalePerOctaveSource"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_SIFT_NScalePerOctaveTarget"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_SIFT_MinScaleSource"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_SIFT_MinScaleTarget"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_SIFT_MinContrastSource"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_SIFT_MinContrastTarget"]->setVisible(false);
    }
    _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_radiobuttonsFeatureBasedRegistration["KEYPOINTS_HARRIS"]);
    _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_radiobuttonsFeatureBasedRegistration["KEYPOINTS_ISS"]);
    _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_radiobuttonsFeatureBasedRegistration["KEYPOINTS_SIFT"]);

    IsLastSelectedHarris = true;
    delete _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout();
    _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->setLayout(formLayout);
}


/*!
* \brief FeatureBasedRegistrationParamsDialog::ClickedRadioButtonKeypointsSIFT
* A SIFT alapú algoritmushoz tartozó paraméterek beállítása ha a felhasználó a hozzá tartozó rádiógombra megy. Láthatóvá tesszük ezeket az értékeket
* valamint, amik eddig láthatók voltak, azokat eltüntetjük
*/
void
FeatureBasedRegistrationParamsDialog::ClickedRadioButtonKeypointsSIFT()
{
    QFormLayout *formLayout = new QFormLayout;
    formLayout->addRow(_radiobuttonsFeatureBasedRegistration["KEYPOINTS_HARRIS"]);
    formLayout->addRow(_radiobuttonsFeatureBasedRegistration["KEYPOINTS_ISS"]);
    formLayout->addRow(_radiobuttonsFeatureBasedRegistration["KEYPOINTS_SIFT"]);
    formLayout->addRow(_labelsFeatureBasedRegistration["KEYPOINTS_SIFT_NoctaveSource"],_spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_NoctaveSource"]);
    formLayout->addRow(_labelsFeatureBasedRegistration["KEYPOINTS_SIFT_NoctaveTarget"],_spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_NoctaveTarget"]);
    formLayout->addRow(_labelsFeatureBasedRegistration["KEYPOINTS_SIFT_NScalePerOctaveSource"],_spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_NScalePerOctaveSource"]);
    formLayout->addRow(_labelsFeatureBasedRegistration["KEYPOINTS_SIFT_NScalePerOctaveTarget"],_spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_NScalePerOctaveTarget"]);
    formLayout->addRow(_labelsFeatureBasedRegistration["KEYPOINTS_SIFT_MinScaleSource"],_spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_MinScaleSource"]);
    formLayout->addRow(_labelsFeatureBasedRegistration["KEYPOINTS_SIFT_MinScaleTarget"],_spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_MinScaleTarget"]);
    formLayout->addRow(_labelsFeatureBasedRegistration["KEYPOINTS_SIFT_MinContrastSource"],_spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_MinContrastSource"]);
    formLayout->addRow(_labelsFeatureBasedRegistration["KEYPOINTS_SIFT_MinContrastTarget"],_spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_MinContrastTarget"]);

    _spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_NoctaveSource"]->setVisible(true);
    _spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_NoctaveTarget"]->setVisible(true);
    _spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_NScalePerOctaveSource"]->setVisible(true);
    _spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_NScalePerOctaveTarget"]->setVisible(true);
    _spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_MinScaleSource"]->setVisible(true);
    _spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_MinScaleTarget"]->setVisible(true);
    _spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_MinContrastSource"]->setVisible(true);
    _spinboxesFeatureBasedRegistration["KEYPOINTS_SIFT_MinContrastTarget"]->setVisible(true);

    _labelsFeatureBasedRegistration["KEYPOINTS_SIFT_NoctaveSource"]->setVisible(true);
    _labelsFeatureBasedRegistration["KEYPOINTS_SIFT_NoctaveTarget"]->setVisible(true);
    _labelsFeatureBasedRegistration["KEYPOINTS_SIFT_NScalePerOctaveSource"]->setVisible(true);
    _labelsFeatureBasedRegistration["KEYPOINTS_SIFT_NScalePerOctaveTarget"]->setVisible(true);
    _labelsFeatureBasedRegistration["KEYPOINTS_SIFT_MinScaleSource"]->setVisible(true);
    _labelsFeatureBasedRegistration["KEYPOINTS_SIFT_MinScaleTarget"]->setVisible(true);
    _labelsFeatureBasedRegistration["KEYPOINTS_SIFT_MinContrastSource"]->setVisible(true);
    _labelsFeatureBasedRegistration["KEYPOINTS_SIFT_MinContrastTarget"]->setVisible(true);

    if(IsLastSelectedISS)
    {
        IsLastSelectedISS = false;
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_Gamma21Source"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_Gamma21Target"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_Gamma32Source"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_Gamma32Target"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_MinNeighborsSource"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_MinNeighborsTarget"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_SalientRadS"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_SalientRadT"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_NonMaxSupS"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_NonMaxSupT"]);

        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_ISS_Gamma21Source"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_ISS_Gamma21Target"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_ISS_Gamma32Source"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_ISS_Gamma32Target"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_ISS_MinNeighborsSource"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_ISS_MinNeighborsTarget"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_ISS_SalientRadS"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_ISS_SalientRadT"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_ISS_NonMaxSupS"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_ISS_NonMaxSupT"]);

        _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_Gamma21Source"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_Gamma21Target"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_Gamma32Source"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_Gamma32Target"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_MinNeighborsSource"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_MinNeighborsTarget"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_SalientRadS"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_SalientRadT"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_NonMaxSupS"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_ISS_NonMaxSupT"]->setVisible(false);

        _labelsFeatureBasedRegistration["KEYPOINTS_ISS_Gamma21Source"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_ISS_Gamma21Target"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_ISS_Gamma32Source"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_ISS_Gamma32Target"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_ISS_MinNeighborsSource"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_ISS_MinNeighborsTarget"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_ISS_SalientRadS"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_ISS_SalientRadT"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_ISS_NonMaxSupS"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_ISS_NonMaxSupT"]->setVisible(false);
    }
    if(IsLastSelectedHarris)
    {
        IsLastSelectedHarris = false;
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_HARRIS_ThresHoldSource"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["KEYPOINTS_HARRIS_ThresHoldTarget"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_checkboxesFeatureBasedRegistration["KEYPOINTS_HARRIS_SetNonMaxSupSource"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_checkboxesFeatureBasedRegistration["KEYPOINTS_HARRIS_SetNonMaxSupTarget"]);

        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_HARRIS_ThresHoldSource"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_HARRIS_ThresHoldTarget"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_HARRIS_SetNonMaxSupSource"]);
        _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["KEYPOINTS_HARRIS_SetNonMaxSupTarget"]);

        _spinboxesFeatureBasedRegistration["KEYPOINTS_HARRIS_ThresHoldSource"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["KEYPOINTS_HARRIS_ThresHoldTarget"]->setVisible(false);
        _checkboxesFeatureBasedRegistration["KEYPOINTS_HARRIS_SetNonMaxSupSource"]->setVisible(false);
        _checkboxesFeatureBasedRegistration["KEYPOINTS_HARRIS_SetNonMaxSupTarget"]->setVisible(false);

        _labelsFeatureBasedRegistration["KEYPOINTS_HARRIS_ThresHoldSource"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_HARRIS_ThresHoldTarget"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_HARRIS_SetNonMaxSupSource"]->setVisible(false);
        _labelsFeatureBasedRegistration["KEYPOINTS_HARRIS_SetNonMaxSupTarget"]->setVisible(false);

    }
    _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_radiobuttonsFeatureBasedRegistration["KEYPOINTS_HARRIS"]);
    _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_radiobuttonsFeatureBasedRegistration["KEYPOINTS_ISS"]);
    _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout()->removeWidget(_radiobuttonsFeatureBasedRegistration["KEYPOINTS_SIFT"]);

    IsLastSelectedSIFT = true;
    delete _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->layout();
    _groupBoxesFeatureBasedRegistration["KEYPOINTS"]->setLayout(formLayout);
}


/*!
* \brief FeatureBasedRegistrationParamsDialog::ParametersforFeatureDescriptors
* \return
* A Feature leírókhoz tartozó paraméterek beállítása a Layout számára, majd ennek a Layoutnak visszaadása a függvény feladata,
* hogy a konstruktorban megfelelően megjelenjenek a paraméter csoportok
*/
QGroupBox *
FeatureBasedRegistrationParamsDialog:: ParametersFeatureDescriptors()
{
    // létrehozzuk a 3 rádiógombot és a feature leíró algoritmusokhoz használt paramétereket reprezentáló elemeket,
    //beállítjuk a megfelelő értékeiket, ezt követően pedig hozzáadjuk a visszaadni kívánt layouthoz ezeket az elemeket

    QGroupBox *groupBox = new QGroupBox(tr("Feature Descriptors"));
    QRadioButton *radio_FPFH = new QRadioButton(tr("FPFH"));
    radio_FPFH->setChecked(true);
    QRadioButton *radio_PFH = new QRadioButton(tr("PFH"));
    QRadioButton *radio_SHOT = new QRadioButton(tr("Shot Estimation"));

    _radiobuttonsFeatureBasedRegistration["FEATUREDESCRIPTOR_FPFH"] = radio_FPFH;
    _radiobuttonsFeatureBasedRegistration["FEATUREDESCRIPTOR_PFH"] = radio_PFH;
    _radiobuttonsFeatureBasedRegistration["FEATUREDESCRIPTOR_SHOT"] = radio_SHOT;

    //A paraméterek inicializálása, eltárolása, és egyes értékeinek beállítása
    QDoubleSpinBox *spinbox_SetRadiusSearchforFeatures = new QDoubleSpinBox();
    QDoubleSpinBox *spinbox_SetRadiusSearchforNormals = new QDoubleSpinBox();
    _spinboxesFeatureBasedRegistration["FEATUREDESCRIPTOR_SetRadiusSearchForFeatures"] = spinbox_SetRadiusSearchforFeatures;
    _spinboxesFeatureBasedRegistration["FEATUREDESCRIPTOR_SetRadiusSearchForNormals"] = spinbox_SetRadiusSearchforNormals;
    SetupSpinBox(* spinbox_SetRadiusSearchforFeatures, 5,0,5);
    SetupSpinBox(* spinbox_SetRadiusSearchforNormals, 5,0,5);

    //A paraméterekhez tartozó rövid leírások inicializálása, értékeik eltárolása
    QLabel *label_SetRadiusSearchforFeatures = new QLabel("Sphere radius to determine the nearest neighbors \nfor Features descriptors: ");
    QLabel *label_SetRadiusSearchforNormals = new QLabel("Sphere radius to determine the nearest neighbors \nfor normals: ");
    _labelsFeatureBasedRegistration["FEATUREDESCRIPTOR_SetRadiusSearchForFeatures"] = label_SetRadiusSearchforFeatures;
    _labelsFeatureBasedRegistration["FEATUREDESCRIPTOR_SetRadiusSearchForNormals"] = label_SetRadiusSearchforNormals;

    //A felület feltöltése
    QFormLayout *formLayout = new QFormLayout;
    formLayout->addRow(radio_FPFH);
    formLayout->addRow(radio_PFH);
    formLayout->addRow(radio_SHOT);
    formLayout->addRow(label_SetRadiusSearchforFeatures,spinbox_SetRadiusSearchforFeatures);
    formLayout->addRow(label_SetRadiusSearchforNormals,spinbox_SetRadiusSearchforNormals);
    groupBox->setLayout(formLayout);
    return groupBox;
}


/*!
* \brief FeatureBasedRegistrationParamsDialog::ParametersforCorrespondences
* \return A párosításokhoz tartozó paraméterekkel ellátott modul csoportot adja vissza QGroupBox* típus formájában
*A párosítás kiválasztásához (esetleg paramétereihez) szükséges elemek megjelenítésére szolgáló layout-ot ad vissza
* a függvény feladata ennek a layoutnak az inicializálása
*/
QGroupBox *
FeatureBasedRegistrationParamsDialog:: ParametersCorrespondences()
{
    QGroupBox *groupBox = new QGroupBox(tr("Correspondence estimation"));
    QRadioButton *radio_direct = new QRadioButton(tr("Direct Correspondence Estimation"));
    QRadioButton *radio_reciprocal = new QRadioButton(tr("Reciprocal Correspondence Estimation"));

    _radiobuttonsFeatureBasedRegistration["CORRESPONDENCES_DIRECT"] = radio_direct;
    _radiobuttonsFeatureBasedRegistration["CORRESPONDENCES_RECIPROCAL"] = radio_direct;
    radio_direct->setChecked(true);
    //a layout feltöltése az elemekkel
    QFormLayout *formLayout = new QFormLayout;
    formLayout->addRow(radio_direct);
    formLayout->addRow(radio_reciprocal);

    groupBox->setLayout(formLayout);
    return groupBox;
}


/*!
* \brief FeatureBasedRegistrationParamsDialog::ParametersforRejection
* \return
* QGroupBox* típusú mutató, aminek segítségével megjelenítjük az elutasításhoz szükséges paramétereket
* A különböző módszerű párosítás elutasító algoritmusokhoz szükséges paraméterek megjelenítésére, bekérésére szolgáló modul csoportot biztosít
* megfelelő layout-tal a dialógus ablaknak. Feladata, ezek inicializálása, eltárolása, majd a megfelelően layouttal beállítáva, vissza
* adja a csoporthoz tartozó mutatót
*/
QGroupBox *
FeatureBasedRegistrationParamsDialog:: ParametersRejection()
{
    //checkboxok beállítása, eltárolása
    _groupBoxesFeatureBasedRegistration["REJECTORS"] = new QGroupBox(tr("Correspondence rejection"));
    QCheckBox *check_RANSAC = new QCheckBox("Rejection using RANSAC");
    QCheckBox *check_DISTANCE = new QCheckBox("Rejection using distance");
    QCheckBox *check_ONETOONE = new QCheckBox("Rejection based on 'One to One' rule");
    QCheckBox *check_SURFACENORMALS = new QCheckBox("Rejecton using surface normals");
    _checkboxesFeatureBasedRegistration["REJECTION_RANSAC"] = check_RANSAC;
    _checkboxesFeatureBasedRegistration["REJECTION_DISTANCE"] = check_DISTANCE;
    _checkboxesFeatureBasedRegistration["REJECTION_ONETOONE"] = check_ONETOONE;
    _checkboxesFeatureBasedRegistration["REJECTION_SURFACENORMALS"] = check_SURFACENORMALS;

    //kapcsolatok felépítése az egyes checkboxok és az események között
    connect(check_RANSAC,SIGNAL(clicked(bool)) ,this ,SLOT(ClickedRadioButtonRejectionBasedOnRANSAC(bool)) );
    connect(check_DISTANCE,SIGNAL(clicked(bool)) ,this ,SLOT(ClickedRadioButtonRejectionBasedOnDistance(bool)) );
    connect(check_SURFACENORMALS,SIGNAL(clicked(bool)) ,this ,SLOT(ClickedRadioButtonRejectionBasedOnSurfaceNormals(bool)) );

    //Spinboxok eltárolása, beállítása
    QDoubleSpinBox *spinbox_setInlierThreshold = new QDoubleSpinBox();
    QDoubleSpinBox *spinbox_setMaximumIterations = new QDoubleSpinBox();
    QDoubleSpinBox *spinbox_setMaxDistance = new QDoubleSpinBox();
    QDoubleSpinBox *spinbox_setThresHold = new QDoubleSpinBox();
    QDoubleSpinBox *spinbox_setRadiusSearch = new QDoubleSpinBox();

    SetupSpinBox(*spinbox_setRadiusSearch, 5,0,5);
    SetupSpinBox(*spinbox_setMaximumIterations, 100000,0,0);
    SetupSpinBox(*spinbox_setInlierThreshold, 3,0,5);
    SetupSpinBox(*spinbox_setMaxDistance, 30,0,5);
    SetupSpinBox(*spinbox_setThresHold, 360,0,2);

    _spinboxesFeatureBasedRegistration["REJECTION_SetInlierThreshold_RANSAC"] =spinbox_setInlierThreshold ;
    _spinboxesFeatureBasedRegistration["REJECTION_SetMaximumIterations_RANSAC"] = spinbox_setMaximumIterations;
    _spinboxesFeatureBasedRegistration["REJECTION_SetMaxDistance_DISTANCE"] =spinbox_setMaxDistance ;
    _spinboxesFeatureBasedRegistration["REJECTION_SetThresHold_SURFACENORMALS"] = spinbox_setThresHold;
    _spinboxesFeatureBasedRegistration["REJECTION_SetRadiusSearch_SURFACENORMALS"] = spinbox_setRadiusSearch;

    //leírósok eltárolása, inicializálása
    QLabel *label_setInlierThreshold = new QLabel("Maximum distance between corresponding points: ");
    QLabel *label_setMaximumIterations = new QLabel("Maximum number of iterations.: ");
    QLabel *label_setMaxDistance = new QLabel("Maximum distance between points: ");
    QLabel *label_setThresHold = new QLabel("The thresholding angle between the normals for rejection: ");
    QLabel *label_setRadiusSearch = new QLabel("The maximum distance to consider a point a neighbor for each keypoints: ");
    _labelsFeatureBasedRegistration["REJECTION_SetInlierThreshold_RANSAC"] = label_setInlierThreshold;
    _labelsFeatureBasedRegistration["REJECTION_SetMaximumIterations_RANSAC"] = label_setMaximumIterations;
    _labelsFeatureBasedRegistration["REJECTION_SetMaxDistance_DISTANCE"] = label_setMaxDistance;
    _labelsFeatureBasedRegistration["REJECTION_SetThresHold_SURFACENORMALS"] = label_setThresHold;
    _labelsFeatureBasedRegistration["REJECTION_SetRadiusSearch_SURFACENORMALS"] = label_setRadiusSearch;

    //elemek hozzáadása a layouthoz
    QFormLayout *formLayout = new QFormLayout;
    formLayout->addRow(check_RANSAC);
    formLayout->addRow(check_DISTANCE);
    formLayout->addRow(check_ONETOONE);
    formLayout->addRow(check_SURFACENORMALS);
    //layout beállítása, majd értékének visszaadása
    _groupBoxesFeatureBasedRegistration["REJECTORS"]->setLayout(formLayout);
    return _groupBoxesFeatureBasedRegistration["REJECTORS"];
}


/*!
* \brief FeatureBasedRegistrationParamsDialog::ClickedRadioButtonRejectionBasedOnRANSAC
* \param ischecked : a jelölő négyzet eseményéhez társuló bool típusú paraméter, ami megmondja, hogy éppen bejelölték, vagy kijelölték az adott jelölőnégyzetet
* Esemény függvény, ami beállítja az adott Layout megfelelő elemeit, annak érdekében, hogy a
* RANSAC elutasító algoritmushoz tartozó paraméterek jelenjenek meg a felületen. Ekkor a nem odatartozó
* elemeket levesszük a layoutról, elemeiket láthatatlanná tesszük, majd az új layoutra áttesszük őket és láthatóvá, végül beállítjuk a a modul
* csoport layoutját az új layoutra
*/
void
FeatureBasedRegistrationParamsDialog:: ClickedRadioButtonRejectionBasedOnRANSAC(bool ischecked)
{
    QFormLayout *formLayout = new QFormLayout;
    formLayout->addRow(_checkboxesFeatureBasedRegistration["REJECTION_RANSAC"]);
    formLayout->addRow(_checkboxesFeatureBasedRegistration["REJECTION_DISTANCE"]);
    formLayout->addRow(_checkboxesFeatureBasedRegistration["REJECTION_ONETOONE"]);
    formLayout->addRow(_checkboxesFeatureBasedRegistration["REJECTION_SURFACENORMALS"]);
    if(ischecked)
    {
        _checkboxesFeatureBasedRegistration["REJECTION_RANSAC"]->setChecked(true);
        formLayout->addRow(_labelsFeatureBasedRegistration["REJECTION_SetInlierThreshold_RANSAC"],_spinboxesFeatureBasedRegistration["REJECTION_SetInlierThreshold_RANSAC"]);
        formLayout->addRow(_labelsFeatureBasedRegistration["REJECTION_SetMaximumIterations_RANSAC"],_spinboxesFeatureBasedRegistration["REJECTION_SetMaximumIterations_RANSAC"]);
        _spinboxesFeatureBasedRegistration["REJECTION_SetInlierThreshold_RANSAC"]->setVisible(true);
        _labelsFeatureBasedRegistration["REJECTION_SetInlierThreshold_RANSAC"]->setVisible(true);
        _spinboxesFeatureBasedRegistration["REJECTION_SetMaximumIterations_RANSAC"]->setVisible(true);
        _labelsFeatureBasedRegistration["REJECTION_SetMaximumIterations_RANSAC"]->setVisible(true);
    }
    else
    {
        _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["REJECTION_SetInlierThreshold_RANSAC"]);
        _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["REJECTION_SetInlierThreshold_RANSAC"]);
        _spinboxesFeatureBasedRegistration["REJECTION_SetInlierThreshold_RANSAC"]->setVisible(false);
        _labelsFeatureBasedRegistration["REJECTION_SetInlierThreshold_RANSAC"]->setVisible(false);
        _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["REJECTION_SetMaximumIterations_RANSAC"]);
        _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["REJECTION_SetMaximumIterations_RANSAC"]);
        _spinboxesFeatureBasedRegistration["REJECTION_SetMaximumIterations_RANSAC"]->setVisible(false);
        _labelsFeatureBasedRegistration["REJECTION_SetMaximumIterations_RANSAC"]->setVisible(false);
    }
    if(_checkboxesFeatureBasedRegistration["REJECTION_DISTANCE"]->isChecked())
    {
        _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["REJECTION_SetMaxDistance_DISTANCE"]);
        _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["REJECTION_SetMaxDistance_DISTANCE"]);
        _spinboxesFeatureBasedRegistration["REJECTION_SetMaxDistance_DISTANCE"]->setVisible(false);
        _labelsFeatureBasedRegistration["REJECTION_SetMaxDistance_DISTANCE"]->setVisible(false);
        formLayout->addRow(_labelsFeatureBasedRegistration["REJECTION_SetMaxDistance_DISTANCE"],_spinboxesFeatureBasedRegistration["REJECTION_SetMaxDistance_DISTANCE"]);
        _spinboxesFeatureBasedRegistration["REJECTION_SetMaxDistance_DISTANCE"]->setVisible(true);
        _labelsFeatureBasedRegistration["REJECTION_SetMaxDistance_DISTANCE"]->setVisible(true);
    }
    if(_checkboxesFeatureBasedRegistration["REJECTION_ONETOONE"]->isChecked())
    {
        _checkboxesFeatureBasedRegistration["REJECTION_ONETOONE"]->setChecked(true);

    }
    if(_checkboxesFeatureBasedRegistration["REJECTION_SURFACENORMALS"]->isChecked())
    {
        _checkboxesFeatureBasedRegistration["REJECTION_SURFACENORMALS"]->setChecked(true);
       _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["REJECTION_SetThresHold_SURFACENORMALS"]);
        _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["REJECTION_SetThresHold_SURFACENORMALS"]);
        _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["REJECTION_SetRadiusSearch_SURFACENORMALS"]);
        _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["REJECTION_SetRadiusSearch_SURFACENORMALS"]);
        _spinboxesFeatureBasedRegistration["REJECTION_SetThresHold_SURFACENORMALS"]->setVisible(false);
        _labelsFeatureBasedRegistration["REJECTION_SetThresHold_SURFACENORMALS"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["REJECTION_SetRadiusSearch_SURFACENORMALS"]->setVisible(false);
        _labelsFeatureBasedRegistration["REJECTION_SetRadiusSearch_SURFACENORMALS"]->setVisible(false);

        formLayout->addRow(_labelsFeatureBasedRegistration["REJECTION_SetThresHold_SURFACENORMALS"],_spinboxesFeatureBasedRegistration["REJECTION_SetThresHold_SURFACENORMALS"]);
        formLayout->addRow(_labelsFeatureBasedRegistration["REJECTION_SetRadiusSearch_SURFACENORMALS"],_spinboxesFeatureBasedRegistration["REJECTION_SetRadiusSearch_SURFACENORMALS"]);
        _spinboxesFeatureBasedRegistration["REJECTION_SetThresHold_SURFACENORMALS"]->setVisible(true);
        _labelsFeatureBasedRegistration["REJECTION_SetThresHold_SURFACENORMALS"]->setVisible(true);
        _spinboxesFeatureBasedRegistration["REJECTION_SetRadiusSearch_SURFACENORMALS"]->setVisible(true);
        _labelsFeatureBasedRegistration["REJECTION_SetRadiusSearch_SURFACENORMALS"]->setVisible(true);
    }
    delete _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout();
    _groupBoxesFeatureBasedRegistration["REJECTORS"]->setLayout(formLayout);
}


/*!
* \brief FeatureBasedRegistrationParamsDialog::ClickedRadioButtonRejectionBasedOnDistance
* \param ischecked : a jelölő négyzet eseményéhez társuló bool típusú paraméter, ami megmondja, hogy éppen bejelölték, vagy kijelölték az adott jelölőnégyzetet
* Esemény függvény, ami beállítja az adott Layout megfelelő elemeit, annak érdekében, hogy a
*  távolság alapú elutasító algoritmushoz tartozó paraméterek jelenjenek meg a felületen, ha a felhasználó rákattint. Ekkor a nem odatartozó
* elemeket levesszük a layoutról, elemeiket láthatatlanná tesszük, majd az új layoutra áttesszük őket és láthatóvá, végül beállítjuk a a modul
* csoport layoutját az új layoutra
*/
void
FeatureBasedRegistrationParamsDialog:: ClickedRadioButtonRejectionBasedOnDistance(bool ischecked)
{
    QFormLayout *formLayout = new QFormLayout;
    formLayout->addRow(_checkboxesFeatureBasedRegistration["REJECTION_RANSAC"]);
    formLayout->addRow(_checkboxesFeatureBasedRegistration["REJECTION_DISTANCE"]);
    formLayout->addRow(_checkboxesFeatureBasedRegistration["REJECTION_ONETOONE"]);
    formLayout->addRow(_checkboxesFeatureBasedRegistration["REJECTION_SURFACENORMALS"]);

    if(_checkboxesFeatureBasedRegistration["REJECTION_RANSAC"]->isChecked())
    {
        _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["REJECTION_SetInlierThreshold_RANSAC"]);
        _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["REJECTION_SetInlierThreshold_RANSAC"]);
        _spinboxesFeatureBasedRegistration["REJECTION_SetInlierThreshold_RANSAC"]->setVisible(false);
        _labelsFeatureBasedRegistration["REJECTION_SetInlierThreshold_RANSAC"]->setVisible(false);
        _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["REJECTION_SetMaximumIterations_RANSAC"]);
        _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["REJECTION_SetMaximumIterations_RANSAC"]);
        _spinboxesFeatureBasedRegistration["REJECTION_SetMaximumIterations_RANSAC"]->setVisible(false);
        _labelsFeatureBasedRegistration["REJECTION_SetMaximumIterations_RANSAC"]->setVisible(false);
        formLayout->addRow(_labelsFeatureBasedRegistration["REJECTION_SetInlierThreshold_RANSAC"],_spinboxesFeatureBasedRegistration["REJECTION_SetInlierThreshold_RANSAC"]);
        formLayout->addRow(_labelsFeatureBasedRegistration["REJECTION_SetMaximumIterations_RANSAC"],_spinboxesFeatureBasedRegistration["REJECTION_SetMaximumIterations_RANSAC"]);
        _spinboxesFeatureBasedRegistration["REJECTION_SetInlierThreshold_RANSAC"]->setVisible(true);
        _labelsFeatureBasedRegistration["REJECTION_SetInlierThreshold_RANSAC"]->setVisible(true);
        _spinboxesFeatureBasedRegistration["REJECTION_SetMaximumIterations_RANSAC"]->setVisible(true);
        _labelsFeatureBasedRegistration["REJECTION_SetMaximumIterations_RANSAC"]->setVisible(true);
    }
    if(ischecked)
    {
        formLayout->addRow(_labelsFeatureBasedRegistration["REJECTION_SetMaxDistance_DISTANCE"],_spinboxesFeatureBasedRegistration["REJECTION_SetMaxDistance_DISTANCE"]);
        _spinboxesFeatureBasedRegistration["REJECTION_SetMaxDistance_DISTANCE"]->setVisible(true);
        _labelsFeatureBasedRegistration["REJECTION_SetMaxDistance_DISTANCE"]->setVisible(true);

    }
    else
    {
        _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["REJECTION_SetMaxDistance_DISTANCE"]);
        _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["REJECTION_SetMaxDistance_DISTANCE"]);
        _spinboxesFeatureBasedRegistration["REJECTION_SetMaxDistance_DISTANCE"]->setVisible(false);
        _labelsFeatureBasedRegistration["REJECTION_SetMaxDistance_DISTANCE"]->setVisible(false);
    }

    if(_checkboxesFeatureBasedRegistration["REJECTION_SURFACENORMALS"]->isChecked())
    {
        _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["REJECTION_SetThresHold_SURFACENORMALS"]);
        _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["REJECTION_SetThresHold_SURFACENORMALS"]);
        _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["REJECTION_SetRadiusSearch_SURFACENORMALS"]);
        _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["REJECTION_SetRadiusSearch_SURFACENORMALS"]);
        _labelsFeatureBasedRegistration["REJECTION_SetThresHold_SURFACENORMALS"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["REJECTION_SetThresHold_SURFACENORMALS"]->setVisible(false);
        _labelsFeatureBasedRegistration["REJECTION_SetRadiusSearch_SURFACENORMALS"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["REJECTION_SetRadiusSearch_SURFACENORMALS"]->setVisible(false);

        formLayout->addRow(_labelsFeatureBasedRegistration["REJECTION_SetThresHold_SURFACENORMALS"],_spinboxesFeatureBasedRegistration["REJECTION_SetThresHold_SURFACENORMALS"]);
        formLayout->addRow(_labelsFeatureBasedRegistration["REJECTION_SetRadiusSearch_SURFACENORMALS"],_spinboxesFeatureBasedRegistration["REJECTION_SetRadiusSearch_SURFACENORMALS"]);


        _labelsFeatureBasedRegistration["REJECTION_SetThresHold_SURFACENORMALS"]->setVisible(true);
        _spinboxesFeatureBasedRegistration["REJECTION_SetThresHold_SURFACENORMALS"]->setVisible(true);
        _labelsFeatureBasedRegistration["REJECTION_SetRadiusSearch_SURFACENORMALS"]->setVisible(true);
        _spinboxesFeatureBasedRegistration["REJECTION_SetRadiusSearch_SURFACENORMALS"]->setVisible(true);
    }
     delete _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout();
    _groupBoxesFeatureBasedRegistration["REJECTORS"]->setLayout(formLayout);
}


/*!
* \brief FeatureBasedRegistrationParamsDialog::ClickedRadioButtonRejectionBasedOnSurfaceNormals
* \param ischecked : a jelölő négyzet eseményéhez társuló bool típusú paraméter, ami megmondja, hogy éppen bejelölték, vagy kijelölték az adott jelölőnégyzetet
**Esemény függvény, ami beállítja az adott Layout megfelelő elemeit, annak érdekében, hogy a
*  felületi normákat használó elutasító algoritmushoz tartozó paraméterek jelenjenek meg a felületen. Ekkor a nem odatartozó
* elemeket levesszük a layoutról, elemeiket láthatatlanná tesszük, majd az új layoutra áttesszük őket és láthatóvá, végül beállítjuk a a modul
* csoport layoutját az új layoutra
*/
void
FeatureBasedRegistrationParamsDialog:: ClickedRadioButtonRejectionBasedOnSurfaceNormals(bool ischecked)
{
    QFormLayout *formLayout = new QFormLayout;
    formLayout->addRow(_checkboxesFeatureBasedRegistration["REJECTION_RANSAC"]);
    formLayout->addRow(_checkboxesFeatureBasedRegistration["REJECTION_DISTANCE"]);
    formLayout->addRow(_checkboxesFeatureBasedRegistration["REJECTION_ONETOONE"]);
    formLayout->addRow(_checkboxesFeatureBasedRegistration["REJECTION_SURFACENORMALS"]);

    if(_checkboxesFeatureBasedRegistration["REJECTION_RANSAC"]->isChecked())
    {
        _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["REJECTION_SetInlierThreshold_RANSAC"]);
        _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["REJECTION_SetInlierThreshold_RANSAC"]);
        _spinboxesFeatureBasedRegistration["REJECTION_SetInlierThreshold_RANSAC"]->setVisible(false);
        _labelsFeatureBasedRegistration["REJECTION_SetInlierThreshold_RANSAC"]->setVisible(false);
        _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["REJECTION_SetMaximumIterations_RANSAC"]);
        _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["REJECTION_SetMaximumIterations_RANSAC"]);
        _spinboxesFeatureBasedRegistration["REJECTION_SetMaximumIterations_RANSAC"]->setVisible(false);
        _labelsFeatureBasedRegistration["REJECTION_SetMaximumIterations_RANSAC"]->setVisible(false);
        formLayout->addRow(_labelsFeatureBasedRegistration["REJECTION_SetInlierThreshold_RANSAC"],_spinboxesFeatureBasedRegistration["REJECTION_SetInlierThreshold_RANSAC"]);
        formLayout->addRow(_labelsFeatureBasedRegistration["REJECTION_SetMaximumIterations_RANSAC"],_spinboxesFeatureBasedRegistration["REJECTION_SetMaximumIterations_RANSAC"]);
        _spinboxesFeatureBasedRegistration["REJECTION_SetInlierThreshold_RANSAC"]->setVisible(true);
        _labelsFeatureBasedRegistration["REJECTION_SetInlierThreshold_RANSAC"]->setVisible(true);
        _spinboxesFeatureBasedRegistration["REJECTION_SetMaximumIterations_RANSAC"]->setVisible(true);
        _labelsFeatureBasedRegistration["REJECTION_SetMaximumIterations_RANSAC"]->setVisible(true);
    }
    if(_checkboxesFeatureBasedRegistration["REJECTION_DISTANCE"]->isChecked())
    {
        _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["REJECTION_SetMaxDistance_DISTANCE"]);
        _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["REJECTION_SetMaxDistance_DISTANCE"]);
        _spinboxesFeatureBasedRegistration["REJECTION_SetMaxDistance_DISTANCE"]->setVisible(false);
        _labelsFeatureBasedRegistration["REJECTION_SetMaxDistance_DISTANCE"]->setVisible(false);
        formLayout->addRow(_labelsFeatureBasedRegistration["REJECTION_SetMaxDistance_DISTANCE"],_spinboxesFeatureBasedRegistration["REJECTION_SetMaxDistance_DISTANCE"]);
        _spinboxesFeatureBasedRegistration["REJECTION_SetMaxDistance_DISTANCE"]->setVisible(true);
        _labelsFeatureBasedRegistration["REJECTION_SetMaxDistance_DISTANCE"]->setVisible(true);
    }

    if(ischecked)
    {
        formLayout->addRow(_labelsFeatureBasedRegistration["REJECTION_SetThresHold_SURFACENORMALS"],_spinboxesFeatureBasedRegistration["REJECTION_SetThresHold_SURFACENORMALS"]);
        formLayout->addRow(_labelsFeatureBasedRegistration["REJECTION_SetRadiusSearch_SURFACENORMALS"], _spinboxesFeatureBasedRegistration["REJECTION_SetRadiusSearch_SURFACENORMALS"]);
        _labelsFeatureBasedRegistration["REJECTION_SetThresHold_SURFACENORMALS"]->setVisible(true);
        _spinboxesFeatureBasedRegistration["REJECTION_SetThresHold_SURFACENORMALS"]->setVisible(true);
        _labelsFeatureBasedRegistration["REJECTION_SetRadiusSearch_SURFACENORMALS"]->setVisible(true);
        _spinboxesFeatureBasedRegistration["REJECTION_SetRadiusSearch_SURFACENORMALS"]->setVisible(true);
    }
    else
    {
        _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["REJECTION_SetThresHold_SURFACENORMALS"]);
        _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["REJECTION_SetThresHold_SURFACENORMALS"]);
        _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout()->removeWidget(_labelsFeatureBasedRegistration["REJECTION_SetRadiusSearch_SURFACENORMALS"]);
        _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout()->removeWidget(_spinboxesFeatureBasedRegistration["REJECTION_SetRadiusSearch_SURFACENORMALS"]);
        _labelsFeatureBasedRegistration["REJECTION_SetThresHold_SURFACENORMALS"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["REJECTION_SetThresHold_SURFACENORMALS"]->setVisible(false);
        _labelsFeatureBasedRegistration["REJECTION_SetRadiusSearch_SURFACENORMALS"]->setVisible(false);
        _spinboxesFeatureBasedRegistration["REJECTION_SetRadiusSearch_SURFACENORMALS"]->setVisible(false);
    }
     delete _groupBoxesFeatureBasedRegistration["REJECTORS"]->layout();
    _groupBoxesFeatureBasedRegistration["REJECTORS"]->setLayout(formLayout);
}


/*!
* \brief FeatureBasedRegistrationParamsDialog::ParametersforTransformation
* \return : egy QGroupBox típusú pointer, amit vissza fog adni a függvény, miután ennek a csoportnak beállított megfelelő layoutot, illetve elemeit
* feltöltötte, eltárolta.
*a tranformációs módszer kiválasztására szóláló felület blokk, aminek feladata az egyes rádiógombok inicializálása, értékeinek eltárolása
*/
QGroupBox *
FeatureBasedRegistrationParamsDialog:: ParametersTransformation()
{
    //paraméterek létrehozása, értékeik beállítása, eltárolása
    _groupBoxesFeatureBasedRegistration["TRANSFORMATION"] = new QGroupBox(tr("Transformation Estimation"));
    QRadioButton *radio_SVD = new QRadioButton(tr("TransformationSVD"));
    QRadioButton *radio_LM = new QRadioButton(tr("TransformationLM"));
    radio_SVD->setChecked(true);
    _radiobuttonsFeatureBasedRegistration["TRANSFORMATION_SVD"] = radio_SVD;
    _radiobuttonsFeatureBasedRegistration["TRANSFORMATION_LM"] = radio_LM;
    //layout létrehozása, feltöltése a paraméterekkel
    QFormLayout *formLayout = new QFormLayout;
    formLayout->addRow(radio_SVD);
    formLayout->addRow(radio_LM);
    _groupBoxesFeatureBasedRegistration["TRANSFORMATION"]->setLayout(formLayout);
    return _groupBoxesFeatureBasedRegistration["TRANSFORMATION"];
}


/*! \brief FeatureBasedRegistrationParamsDialog::~FeatureBasedRegistrationParamsDialog
*Az osztály destruktora. Minden elemet törlünk, amit szükséges
*/
FeatureBasedRegistrationParamsDialog::~FeatureBasedRegistrationParamsDialog()
{
    std::map<std::string, QDoubleSpinBox*>::iterator it;

    for ( it = _spinboxesFeatureBasedRegistration.begin(); it != _spinboxesFeatureBasedRegistration.end(); it++ )
    {
        if(it->second != nullptr)
            it->second->deleteLater();
    }
    std::map<std::string, QCheckBox*>::iterator it2;

    for ( it2 = _checkboxesFeatureBasedRegistration.begin(); it2 != _checkboxesFeatureBasedRegistration.end(); it2++ )
    {
        if(it2->second != nullptr)
            it2->second->deleteLater();

    }
    std::map<std::string, QRadioButton*>::iterator it3;

    for ( it3 = _radiobuttonsFeatureBasedRegistration.begin(); it3 != _radiobuttonsFeatureBasedRegistration.end(); it3++ )
    {
        if(it3->second)
            it3->second->deleteLater();

    }
    std::map<std::string, QLabel*>::iterator it4;

    for ( it4 = _labelsFeatureBasedRegistration.begin(); it4 != _labelsFeatureBasedRegistration.end(); it4++ )
    {
        if(it4->second)
            it4->second->deleteLater();

    }

    std::map<std::string, QGroupBox*>::iterator it5;

    for ( it5 = _groupBoxesFeatureBasedRegistration.begin(); it5 != _groupBoxesFeatureBasedRegistration.end(); it5++ )
    {
        if(it5->second != nullptr)
        {
            it5->second->layout()->deleteLater();
            it5->second->deleteLater();
        }
    }

    _subGridLayout->deleteLater();
    delete ui;
}
