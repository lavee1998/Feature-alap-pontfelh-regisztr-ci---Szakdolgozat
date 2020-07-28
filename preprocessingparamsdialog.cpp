#include "preprocessingparamsdialog.h"
#include "ui_preprocessingparamsdialog.h"

/*!
 * \brief PreprocessingParamsDialog::PreprocessingParamsDialog
 * \param parent - a szülőként megjelölt QWidget
 * Az osztály konstruktora, ahol inicializáljuk az egyes értékeket, és a megfelelően elrendezzük őket
 */
PreprocessingParamsDialog::PreprocessingParamsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PreprocessingParamsDialog)
{
    //Az előfeldolgozás dialógusablakának konstruktora
    ui->setupUi(this);
    _mainLayout = new QVBoxLayout(this);
    _scrollArea = new QScrollArea(this);
    QWidget* scrollAreaContent = new QWidget(_scrollArea);
    _subGridlayout = new QGridLayout(scrollAreaContent);
    _scrollArea->setVerticalScrollBarPolicy( Qt::ScrollBarAsNeeded );
    _scrollArea->setWidgetResizable(true);
    setContentsMargins(0,0,0,0);
    //A felületre betöltjük az egyes funkciókhoz tartozó paramétereket tartalmazó csoportokat
    _subGridlayout->addWidget(ParametersDownsampling(), 0,0);
    _subGridlayout->addWidget(ParametersOutliersRemoval(), 0,1);
    _subGridlayout->addWidget(ParametersSmoothing(),1,0);
    _subGridlayout->addWidget(Actionbuttons(),1,1);
    scrollAreaContent->setLayout(_subGridlayout);
    _scrollArea->setWidget(scrollAreaContent);
    _mainLayout->addWidget(_scrollArea);
    setLayout(_mainLayout);
    _subGridlayout->setSizeConstraint(QLayout::SetMinimumSize);
}

/*!
 * \brief PreprocessingParamsDialog::SetupSpinBox
 * \param spinbox - a módosítani kívánt spinbox
 * \param max - a spinbox maximum értéke
 * \param min - a spinbox minimum értéke
 * \param decimals - a tizedesjegy utáni számok
 * Egy SpinBox paramétereinek módosítására szolgáló függvény
 */
void PreprocessingParamsDialog::SetupSpinBox(QDoubleSpinBox &spinbox, double max, double min, int decimals)
{
    //az egyes Spinboxok tulajdonságait állítjuk be
    spinbox.setMaximum(max); //maximumát
    spinbox.setMinimum(min); //minimumát
    spinbox.setDecimals(decimals); //a tizedesjegy utáni számokat
}


/*!
 * \brief PreprocessingParamsDialog::ParametersDownsampling
 * \return QGroupBox típusú pointert ad vissza, ami a már fel van töltve a megfelelő widget-ekkel. Ebben a függvényben létrehozzuk az egyes paraméterekhez
 * tartozó megjelenítő eszközöket, illetve eltároljuk őket, és egy layouttal ahhoz a QGroupBox-hoz adjunk, amelynek pointerét visszaadjuk.
 * A mintavételezéshez használt paraméterek, modulok inicializálására szolgáló függvény
 */
QGroupBox*
PreprocessingParamsDialog:: ParametersDownsampling()
{
    //a mintavételezéshez szükséges paraméterek beállítása
    _groupboxesPreprocessing["DOWNSAMPLING"] = new QGroupBox(tr("Downsampling"));
    _groupboxesPreprocessing["DOWNSAMPLING"]->setCheckable(true);
    _groupboxesPreprocessing["DOWNSAMPLING"]->setChecked(false);

    //ezekkel a checkboxokkal fogjuk jelezni, hogy melyik felhőn akarjuk véghez vinni a mintavételezést, kezdetbenmind a kettő érték ki van pipálva
    //eltároljuk a checkboxokat a későbbi használhathoz
    _checkboxesPreprocessing["Downsampling_OnSource"] = new QCheckBox("On Source");
    _checkboxesPreprocessing["Downsampling_OnTarget"] = new QCheckBox("On Target");
    _checkboxesPreprocessing["Downsampling_OnSource"]->setChecked(true);
    _checkboxesPreprocessing["Downsampling_OnTarget"]->setChecked(true);

    //A Voxel módszer paramétereinek inicializálása, értékeik eltárolása a megfelelő mapekbe
    _radiobuttonsPreprocessing["Downsampling_Voxel"] = new QRadioButton(tr("Voxel grid downsampling"));
    _radiobuttonsPreprocessing["Downsampling_Voxel"]->setChecked(true);

    _spinboxesPreprocessing["Downsampling_SetLeafSizeforSource_Voxel"] = new QDoubleSpinBox();
    _spinboxesPreprocessing["Downsampling_SetLeafSizeforTarget_Voxel"] = new QDoubleSpinBox();
    SetupSpinBox(* _spinboxesPreprocessing["Downsampling_SetLeafSizeforSource_Voxel"],1,0,6);
    SetupSpinBox(*_spinboxesPreprocessing["Downsampling_SetLeafSizeforTarget_Voxel"],1,0,6);

    _labelsPreprocessing["Downsampling_SetLeafSizeforSource_Voxel"] =  new QLabel("The voxel grid leaf size (Source): ");
    _labelsPreprocessing["Downsampling_SetLeafSizeforTarget_Voxel"] = new QLabel("The voxel grid leaf size (Target): ");

    //A Random mintavételezés paramétereinek inicializálása, értékeik eltárolása a megfelelő mapekbe
    _radiobuttonsPreprocessing["Downsampling_RandomSampling"] = new QRadioButton(tr("Random Sampling"));

    _spinboxesPreprocessing["Downsampling_SetSampleforSource_Random"] =new QDoubleSpinBox();
    _spinboxesPreprocessing["Downsampling_SetSampleforTarget_Random"] =new QDoubleSpinBox();
    SetupSpinBox(*_spinboxesPreprocessing["Downsampling_SetSampleforSource_Random"],100000,0,0);
    SetupSpinBox(*_spinboxesPreprocessing["Downsampling_SetSampleforTarget_Random"],100000,0,0);
    _labelsPreprocessing["Downsampling_SetSampleforSource_Random"] = new QLabel("Number of points to \nbe sampled (Source): ");;
    _labelsPreprocessing["Downsampling_SetSampleforTarget_Random"] = new QLabel("Number of points to \nbe sampled (Target): ");;


    //Az egyes gombok és az általuk kifejtett események beállítása
    connect(_radiobuttonsPreprocessing["Downsampling_Voxel"],SIGNAL(clicked() ),this, SLOT(ClickedRadioButtonDownsamplingVoxel() ) );
    connect(_radiobuttonsPreprocessing["Downsampling_RandomSampling"],SIGNAL(clicked() ),this, SLOT(ClickedRadioButtonDownsamplingRandom() ) );
    connect(_checkboxesPreprocessing["Downsampling_OnSource"],SIGNAL(clicked(bool) ), this, SLOT(ClickedCheckBoxDownsamplingSource(bool) ));
    connect(_checkboxesPreprocessing["Downsampling_OnTarget"],SIGNAL(clicked(bool) ), this, SLOT(ClickedCheckBoxDownsamplingTarget(bool) ));


    //A paraméter csoporthoz tartozó QFormLayout feltöltése. Kezdetben a Voxel paraméterei láthatóak csak
    _layoutsforpreprocessing["DOWNSAMPLING"] = new QFormLayout();
    _layoutsforpreprocessing["DOWNSAMPLING"]->addRow(_radiobuttonsPreprocessing["Downsampling_Voxel"]);
    _layoutsforpreprocessing["DOWNSAMPLING"]->addRow(_radiobuttonsPreprocessing["Downsampling_RandomSampling"]);
    _layoutsforpreprocessing["DOWNSAMPLING"]->addRow(_checkboxesPreprocessing["Downsampling_OnSource"], _checkboxesPreprocessing["Downsampling_OnTarget"]);
    _layoutsforpreprocessing["DOWNSAMPLING"]->addRow(_labelsPreprocessing["Downsampling_SetLeafSizeforSource_Voxel"],_spinboxesPreprocessing["Downsampling_SetLeafSizeforSource_Voxel"]);
    _layoutsforpreprocessing["DOWNSAMPLING"]->addRow(_labelsPreprocessing["Downsampling_SetLeafSizeforTarget_Voxel"],_spinboxesPreprocessing["Downsampling_SetLeafSizeforTarget_Voxel"]);
    _groupboxesPreprocessing["DOWNSAMPLING"]->setLayout(_layoutsforpreprocessing["DOWNSAMPLING"]);
    return _groupboxesPreprocessing["DOWNSAMPLING"];
}


/*!
* \brief PreprocessingParamsDialog::ClickedRadioButtonDownsamplingRandom
* Ha a felhasználó rámegy a "Random Sampling"-hez tartozó rádiógombra, akkor ez a metódus fog meghívódni. Ez eltünteti a Voxel paramétereit
* továbbá megjeleníti a Random Sampling-hez tartozó paramétereket megfelelően, az "On Source" és az "On Target" jelőlő dobozoktól függően
*/
void
PreprocessingParamsDialog:: ClickedRadioButtonDownsamplingRandom()
{
    QFormLayout *formLayout = new QFormLayout;
    formLayout->addRow(_radiobuttonsPreprocessing["Downsampling_Voxel"]);
    formLayout->addRow(_radiobuttonsPreprocessing["Downsampling_RandomSampling"]);
    formLayout->addRow(_checkboxesPreprocessing["Downsampling_OnSource"], _checkboxesPreprocessing["Downsampling_OnTarget"]);
    _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_checkboxesPreprocessing["Downsampling_OnSource"]);
    _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_checkboxesPreprocessing["Downsampling_OnTarget"]);
    _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_radiobuttonsPreprocessing["Downsampling_Voxel"]);
    _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_radiobuttonsPreprocessing["Downsampling_RandomSampling"]);

    if(_checkboxesPreprocessing["Downsampling_OnSource"]->isChecked())
    {
        formLayout->addRow(_labelsPreprocessing["Downsampling_SetSampleforSource_Random"],_spinboxesPreprocessing["Downsampling_SetSampleforSource_Random"]);
        _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget( _spinboxesPreprocessing["Downsampling_SetLeafSizeforSource_Voxel"]);
        _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_labelsPreprocessing["Downsampling_SetLeafSizeforSource_Voxel"]);
        _spinboxesPreprocessing["Downsampling_SetLeafSizeforSource_Voxel"]->setVisible(false);
        _labelsPreprocessing["Downsampling_SetLeafSizeforSource_Voxel"]->setVisible(false);
        _labelsPreprocessing["Downsampling_SetSampleforSource_Random"]->setVisible(true);
        _spinboxesPreprocessing["Downsampling_SetSampleforSource_Random"]->setVisible(true);
    }
    if(_checkboxesPreprocessing["Downsampling_OnTarget"]->isChecked())
    {
        formLayout->addRow( _labelsPreprocessing["Downsampling_SetSampleforTarget_Random"],_spinboxesPreprocessing["Downsampling_SetSampleforTarget_Random"]);
        _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_spinboxesPreprocessing["Downsampling_SetLeafSizeforTarget_Voxel"]);
        _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget( _labelsPreprocessing["Downsampling_SetLeafSizeforTarget_Voxel"]);
        _labelsPreprocessing["Downsampling_SetLeafSizeforTarget_Voxel"]->setVisible(false);
        _spinboxesPreprocessing["Downsampling_SetLeafSizeforTarget_Voxel"]->setVisible(false);
        _labelsPreprocessing["Downsampling_SetSampleforTarget_Random"]->setVisible(true);
        _spinboxesPreprocessing["Downsampling_SetSampleforTarget_Random"]->setVisible(true);
    }
     delete _groupboxesPreprocessing["DOWNSAMPLING"]->layout();
    _groupboxesPreprocessing["DOWNSAMPLING"]->setLayout(formLayout);
}


/*!
 * \brief PreprocessingParamsDialog::ClickedRadioButtonDownsamplingVoxel
* Ha a felhasználó rámegy a "Voxel based Downsampling"-hez tartozó rádiógombra, akkor ez a metódus fog meghívódni. Ez eltünteti a Random Sampling paramétereit
* továbbá megjeleníti a Voxel-hez tartozó paramétereket megfelelően, az "On Source" és az "On Target" jelőlő dobozoktól függően
*/
void
PreprocessingParamsDialog::ClickedRadioButtonDownsamplingVoxel()
{
    QFormLayout *formLayout = new QFormLayout;
    formLayout->addRow(_radiobuttonsPreprocessing["Downsampling_Voxel"]);
    formLayout->addRow(_radiobuttonsPreprocessing["Downsampling_RandomSampling"]);
    formLayout->addRow(_checkboxesPreprocessing["Downsampling_OnSource"], _checkboxesPreprocessing["Downsampling_OnTarget"]);
    _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_checkboxesPreprocessing["Downsampling_OnSource"]);
    _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_checkboxesPreprocessing["Downsampling_OnTarget"]);
    _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_radiobuttonsPreprocessing["Downsampling_Voxel"]);
    _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_radiobuttonsPreprocessing["Downsampling_RandomSampling"]);

    if(_checkboxesPreprocessing["Downsampling_OnSource"]->isChecked())
    {
        formLayout->addRow(_labelsPreprocessing["Downsampling_SetLeafSizeforSource_Voxel"],_spinboxesPreprocessing["Downsampling_SetLeafSizeforSource_Voxel"]);
        _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_spinboxesPreprocessing["Downsampling_SetSampleforSource_Random"]);
        _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_labelsPreprocessing["Downsampling_SetSampleforSource_Random"]);
        _spinboxesPreprocessing["Downsampling_SetSampleforSource_Random"]->setVisible(false);
        _spinboxesPreprocessing["Downsampling_SetLeafSizeforSource_Voxel"]->setVisible(true);
        _labelsPreprocessing["Downsampling_SetSampleforSource_Random"]->setVisible(false);
        _labelsPreprocessing["Downsampling_SetLeafSizeforSource_Voxel"]->setVisible(true);
    }
    if(_checkboxesPreprocessing["Downsampling_OnTarget"]->isChecked())
    {
        formLayout->addRow(_labelsPreprocessing["Downsampling_SetLeafSizeforTarget_Voxel"],_spinboxesPreprocessing["Downsampling_SetLeafSizeforTarget_Voxel"]);
        _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_spinboxesPreprocessing["Downsampling_SetSampleforTarget_Random"]);
        _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_labelsPreprocessing["Downsampling_SetSampleforTarget_Random"]);

        _spinboxesPreprocessing["Downsampling_SetSampleforTarget_Random"]->setVisible(false);
        _spinboxesPreprocessing["Downsampling_SetLeafSizeforTarget_Voxel"]->setVisible(true);
        _labelsPreprocessing["Downsampling_SetSampleforTarget_Random"]->setVisible(false);
        _labelsPreprocessing["Downsampling_SetLeafSizeforTarget_Voxel"]->setVisible(true);


    }
     delete _groupboxesPreprocessing["DOWNSAMPLING"]->layout();
    _groupboxesPreprocessing["DOWNSAMPLING"]->setLayout(formLayout);

}

/*!
* \brief PreprocessingParamsDialog::ClickedCheckBoxDownsamplingSource
* \param ischecked
* Ha a felhasználó az "On Source gombra megy" akkor ez a metódus fog lefutni, ekkor kicseréli a megfelelő paramétereket a rádiógomboktól függően
*/
void
PreprocessingParamsDialog::ClickedCheckBoxDownsamplingSource(bool ischecked)
{
    QFormLayout* formLayout = new QFormLayout();
    formLayout->addRow(_radiobuttonsPreprocessing["Downsampling_Voxel"]);
    formLayout->addRow(_radiobuttonsPreprocessing["Downsampling_RandomSampling"]);
    formLayout->addRow(_checkboxesPreprocessing["Downsampling_OnSource"], _checkboxesPreprocessing["Downsampling_OnTarget"]);
    _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_checkboxesPreprocessing["Downsampling_OnSource"]);
    _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_checkboxesPreprocessing["Downsampling_OnTarget"]);
    _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_radiobuttonsPreprocessing["Downsampling_Voxel"]);
    _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_radiobuttonsPreprocessing["Downsampling_RandomSampling"]);

    if(_radiobuttonsPreprocessing["Downsampling_Voxel"]->isChecked())
    {
        if(ischecked)
        {
            formLayout->addRow( _labelsPreprocessing["Downsampling_SetLeafSizeforSource_Voxel"], _spinboxesPreprocessing["Downsampling_SetLeafSizeforSource_Voxel"]);
             _labelsPreprocessing["Downsampling_SetLeafSizeforSource_Voxel"]->setVisible(true);
            _spinboxesPreprocessing["Downsampling_SetLeafSizeforSource_Voxel"]->setVisible(true);

        }
        else{
            _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget( _labelsPreprocessing["Downsampling_SetLeafSizeforSource_Voxel"]);
            _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_spinboxesPreprocessing["Downsampling_SetLeafSizeforSource_Voxel"]);
           _labelsPreprocessing["Downsampling_SetLeafSizeforSource_Voxel"]->setVisible(false);
           _spinboxesPreprocessing["Downsampling_SetLeafSizeforSource_Voxel"]->setVisible(false);
        }
        if(_checkboxesPreprocessing["Downsampling_OnTarget"]->isChecked())
        {
            formLayout->addRow( _labelsPreprocessing["Downsampling_SetLeafSizeforTarget_Voxel"],_spinboxesPreprocessing["Downsampling_SetLeafSizeforTarget_Voxel"]);
             _labelsPreprocessing["Downsampling_SetLeafSizeforTarget_Voxel"]->setVisible(true);
            _spinboxesPreprocessing["Downsampling_SetLeafSizeforTarget_Voxel"]->setVisible(true);
        }
    }
    else
    {
        if(ischecked)
        {
            formLayout->addRow(_labelsPreprocessing["Downsampling_SetSampleforSource_Random"],_spinboxesPreprocessing["Downsampling_SetSampleforSource_Random"]);
            _labelsPreprocessing["Downsampling_SetSampleforSource_Random"]->setVisible(true);
           _spinboxesPreprocessing["Downsampling_SetSampleforSource_Random"]->setVisible(true);
        }
        else{
            _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_labelsPreprocessing["Downsampling_SetSampleforSource_Random"]);
            _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_spinboxesPreprocessing["Downsampling_SetSampleforSource_Random"]);
            _labelsPreprocessing["Downsampling_SetSampleforSource_Random"]->setVisible(false);
            _spinboxesPreprocessing["Downsampling_SetSampleforSource_Random"]->setVisible(false);

        }
        if(_checkboxesPreprocessing["Downsampling_OnTarget"]->isChecked())
        {
            formLayout->addRow(_labelsPreprocessing["Downsampling_SetSampleforTarget_Random"],_spinboxesPreprocessing["Downsampling_SetSampleforTarget_Random"]);
            _labelsPreprocessing["Downsampling_SetSampleforTarget_Random"]->setVisible(true);
            _spinboxesPreprocessing["Downsampling_SetSampleforTarget_Random"]->setVisible(true);
        }
    }
    delete _groupboxesPreprocessing["DOWNSAMPLING"]->layout();
    _groupboxesPreprocessing["DOWNSAMPLING"]->setLayout(formLayout);
}


/*!
* \brief PreprocessingParamsDialog::ClickedCheckBoxDownsamplingTarget
* \param ischecked
* Ha a felhasználó az "On Source gombra megy" akkor ez a metódus fog lefutni, ekkor kicseréli a megfelelő paramétereket a rádiógomboktól függően
*/
void
PreprocessingParamsDialog::ClickedCheckBoxDownsamplingTarget(bool ischecked)
{
    QFormLayout* formLayout = new QFormLayout();
    formLayout->addRow(_radiobuttonsPreprocessing["Downsampling_Voxel"]);
    formLayout->addRow(_radiobuttonsPreprocessing["Downsampling_RandomSampling"]);
    formLayout->addRow(_checkboxesPreprocessing["Downsampling_OnSource"], _checkboxesPreprocessing["Downsampling_OnTarget"]);
    _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_checkboxesPreprocessing["Downsampling_OnSource"]);
    _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_checkboxesPreprocessing["Downsampling_OnTarget"]);
    _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_radiobuttonsPreprocessing["Downsampling_Voxel"]);
    _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_radiobuttonsPreprocessing["Downsampling_RandomSampling"]);
    if(_radiobuttonsPreprocessing["Downsampling_Voxel"]->isChecked())
    {
        if(_checkboxesPreprocessing["Downsampling_OnSource"]->isChecked())
        {
            formLayout->addRow(_labelsPreprocessing["Downsampling_SetLeafSizeforSource_Voxel"],_spinboxesPreprocessing["Downsampling_SetLeafSizeforSource_Voxel"]);
            _labelsPreprocessing["Downsampling_SetLeafSizeforSource_Voxel"]->setVisible(true);
            _spinboxesPreprocessing["Downsampling_SetLeafSizeforSource_Voxel"]->setVisible(true);
        }
        if(ischecked)
        {
            formLayout->addRow(_labelsPreprocessing["Downsampling_SetLeafSizeforTarget_Voxel"],_spinboxesPreprocessing["Downsampling_SetLeafSizeforTarget_Voxel"]);
            _labelsPreprocessing["Downsampling_SetLeafSizeforTarget_Voxel"]->setVisible(true);
            _spinboxesPreprocessing["Downsampling_SetLeafSizeforTarget_Voxel"]->setVisible(true);

        }
        else{
            _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_labelsPreprocessing["Downsampling_SetLeafSizeforTarget_Voxel"]);
            _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_spinboxesPreprocessing["Downsampling_SetLeafSizeforTarget_Voxel"]);
            _labelsPreprocessing["Downsampling_SetLeafSizeforTarget_Voxel"]->setVisible(false);
            _spinboxesPreprocessing["Downsampling_SetLeafSizeforTarget_Voxel"]->setVisible(false);
        }
    }
    else
    {
        if(_checkboxesPreprocessing["Downsampling_OnSource"]->isChecked())
        {
            formLayout->addRow(_labelsPreprocessing["Downsampling_SetSampleforSource_Random"],_spinboxesPreprocessing["Downsampling_SetSampleforSource_Random"]);
            _labelsPreprocessing["Downsampling_SetSampleforSource_Random"]->setVisible(true);
            _spinboxesPreprocessing["Downsampling_SetSampleforSource_Random"]->setVisible(true);
        }

        if(ischecked)
        {
            formLayout->addRow(_labelsPreprocessing["Downsampling_SetSampleforTarget_Random"], _spinboxesPreprocessing["Downsampling_SetSampleforTarget_Random"]);
            _labelsPreprocessing["Downsampling_SetSampleforTarget_Random"]->setVisible(true);
            _spinboxesPreprocessing["Downsampling_SetSampleforTarget_Random"]->setVisible(true);

        }
        else{
            _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget( _labelsPreprocessing["Downsampling_SetSampleforTarget_Random"]);

            _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_spinboxesPreprocessing["Downsampling_SetSampleforTarget_Random"]);
            _labelsPreprocessing["Downsampling_SetSampleforTarget_Random"]->setVisible(false);
            _spinboxesPreprocessing["Downsampling_SetSampleforTarget_Random"]->setVisible(false);

        }
    }
    delete _groupboxesPreprocessing["DOWNSAMPLING"]->layout();
    _groupboxesPreprocessing["DOWNSAMPLING"]->setLayout(formLayout);
}


/*!
* \brief PreprocessingParamsDialog::ParametersforRemoveOutliers
* \return
*Visszaadja a Külső adatok eltávolítására szolgáló függvények paramétereit
*/
QGroupBox *
PreprocessingParamsDialog:: ParametersOutliersRemoval()
{
    _groupboxesPreprocessing["OUTLIERSREMOVAL"] = new QGroupBox(tr("Remove Outliers"));
    _groupboxesPreprocessing["OUTLIERSREMOVAL"]->setCheckable(true);
    _groupboxesPreprocessing["OUTLIERSREMOVAL"]->setChecked(false);
    //jelőlő dobozok beállítása

    _checkboxesPreprocessing["OUTREM_OnSource"]=new QCheckBox("On Source");
    _checkboxesPreprocessing["OUTREM_OnTarget"]=new QCheckBox("On Target");
    _checkboxesPreprocessing["OUTREM_OnSource"]->setChecked(true);
    _checkboxesPreprocessing["OUTREM_OnTarget"]->setChecked(true);

    //statisztikailag kiálló részek eltávolításához szükséges paraméterek beállítása és eltárolása
    _radiobuttonsPreprocessing["OUTREM_STAT_OUTREM"] = new QRadioButton(tr("Remove outliers using statistics"));
    _radiobuttonsPreprocessing["OUTREM_STAT_OUTREM"]->setChecked(true);

    _labelsPreprocessing["OUTREM_SetMeanKSource_STAT"] = new QLabel("The number of points to use for mean distance estimation (Source): ");
    _labelsPreprocessing["OUTREM_SetMeanKTarget_STAT"] = new QLabel("The number of points to use for mean distance estimation (Target): ");
    _labelsPreprocessing["OUTREM_SetStddevMulThreshSource_STAT"] = new QLabel("Set the standard deviation multiplier \nfor the distance threshold calculation (Source): ");
    _labelsPreprocessing["OUTREM_SetStddevMulThreshTarget_STAT"] = new QLabel("Set the standard deviation multiplier \nfor the distance threshold calculation (Target): ");
    _labelsPreprocessing["OUTREM_SetNegativeSource_STAT"] = new QLabel("Inverted behavior (Source): ");
    _labelsPreprocessing["OUTREM_SetNegativeTarget_STAT"] = new QLabel("Inverted behavior (Target): ");
    _checkboxesPreprocessing["OUTREM_SetNegativeSource_STAT"] = new QCheckBox();
    _checkboxesPreprocessing["OUTREM_SetNegativeTarget_STAT"] = new QCheckBox();
    _spinboxesPreprocessing["OUTREM_SetMeanKSource_STAT"] = new QDoubleSpinBox();
    _spinboxesPreprocessing["OUTREM_SetMeanKTarget_STAT"] = new QDoubleSpinBox();
    _spinboxesPreprocessing["OUTREM_SetStddevMulThreshSource_STAT"] = new QDoubleSpinBox();
    _spinboxesPreprocessing["OUTREM_SetStddevMulThreshTarget_STAT"] = new QDoubleSpinBox();
    SetupSpinBox(*_spinboxesPreprocessing["OUTREM_SetMeanKSource_STAT"],30,0,0);
    SetupSpinBox(*_spinboxesPreprocessing["OUTREM_SetMeanKTarget_STAT"],30,0,0);
    SetupSpinBox(*_spinboxesPreprocessing["OUTREM_SetStddevMulThreshSource_STAT"],5,0,4);
    SetupSpinBox(*_spinboxesPreprocessing["OUTREM_SetStddevMulThreshTarget_STAT"],5,0,4);


    //sugár alapú kiálló részek eltávolításához szükséges paraméterek beállítása és eltárolása
    _radiobuttonsPreprocessing["OUTREM_RAD_OUTREM"] = new QRadioButton(tr("Remove outliers using radius"));
    _labelsPreprocessing["OUTREM_SetRadiusSearchSource_RAD"] = new QLabel("The radius of the sphere for nearest neighbor searching (Source): ");
    _labelsPreprocessing["OUTREM_SetRadiusSearchTarget_RAD"] = new QLabel("The radius of the sphere for nearest neighbor searching (Target): ");
    _labelsPreprocessing["OUTREM_SetMinNeighborsInRadiusSource_RAD"] = new QLabel("The minimum number of neighbors (Source): ");
    _labelsPreprocessing["OUTREM_SetMinNeighborsInRadiusTarget_RAD"] = new QLabel("sThe minimum number of neighbors (Target): ");
    _spinboxesPreprocessing["OUTREM_SetRadiusSearchSource_RAD"] = new QDoubleSpinBox();
    _spinboxesPreprocessing["OUTREM_SetRadiusSearchTarget_RAD"] = new QDoubleSpinBox();
    _spinboxesPreprocessing["OUTREM_SetMinNeighborsInRadiusSource_RAD"] = new QDoubleSpinBox();
    _spinboxesPreprocessing["OUTREM_SetMinNeighborsInRadiusTarget_RAD"] = new QDoubleSpinBox();
    SetupSpinBox(* _spinboxesPreprocessing["OUTREM_SetRadiusSearchSource_RAD"],3,0,4);
    SetupSpinBox(* _spinboxesPreprocessing["OUTREM_SetRadiusSearchTarget_RAD"],3,0,4);
    SetupSpinBox(*_spinboxesPreprocessing["OUTREM_SetMinNeighborsInRadiusSource_RAD"],30,0,0);
    SetupSpinBox(*_spinboxesPreprocessing["OUTREM_SetMinNeighborsInRadiusTarget_RAD"],30,0,0);

    //a kapcsolatok beállítása az egyes elemek és funkciók között
    connect(_radiobuttonsPreprocessing["OUTREM_STAT_OUTREM"],SIGNAL(clicked() ),this, SLOT(ClickedRadioButtonRemoveOutliersStatisticalOutlierRemoval() ) );
    connect(_radiobuttonsPreprocessing["OUTREM_RAD_OUTREM"],SIGNAL(clicked() ),this, SLOT(ClickedRadioButtonRemoveOutliersRadiusOutlierRemoval() ) );
    connect(_checkboxesPreprocessing["OUTREM_OnSource"],SIGNAL(clicked(bool) ) , this , SLOT(ClickedCheckBoxOutliersRemovalSource(bool)));
    connect(_checkboxesPreprocessing["OUTREM_OnTarget"],SIGNAL(clicked(bool)),this,SLOT(ClickedCheckBoxOutliersRemovalTarget(bool)));

    //a tábla feltöltése, majd értékének visszaadása
    QFormLayout *formLayout = new QFormLayout;
    formLayout->addRow(_radiobuttonsPreprocessing["OUTREM_STAT_OUTREM"]);
    formLayout->addRow(_radiobuttonsPreprocessing["OUTREM_RAD_OUTREM"]);
    formLayout->addRow(_checkboxesPreprocessing["OUTREM_OnSource"],_checkboxesPreprocessing["OUTREM_OnTarget"]);
    formLayout->addRow(_labelsPreprocessing["OUTREM_SetMeanKSource_STAT"],_spinboxesPreprocessing["OUTREM_SetMeanKSource_STAT"]);
    formLayout->addRow(_labelsPreprocessing["OUTREM_SetStddevMulThreshSource_STAT"], _spinboxesPreprocessing["OUTREM_SetStddevMulThreshSource_STAT"]);
    formLayout->addRow(_labelsPreprocessing["OUTREM_SetNegativeSource_STAT"], _checkboxesPreprocessing["OUTREM_SetNegativeSource_STAT"]);
    formLayout->addRow(_labelsPreprocessing["OUTREM_SetMeanKTarget_STAT"],_spinboxesPreprocessing["OUTREM_SetMeanKTarget_STAT"]);
    formLayout->addRow(_labelsPreprocessing["OUTREM_SetStddevMulThreshTarget_STAT"], _spinboxesPreprocessing["OUTREM_SetStddevMulThreshTarget_STAT"]);
    formLayout->addRow(_labelsPreprocessing["OUTREM_SetNegativeTarget_STAT"], _checkboxesPreprocessing["OUTREM_SetNegativeTarget_STAT"]);
    _groupboxesPreprocessing["OUTLIERSREMOVAL"]->setLayout(formLayout);
    return _groupboxesPreprocessing["OUTLIERSREMOVAL"];
}


/*!
* \brief PreprocessingParamsDialog::ClickedRadioButtonRemoveOutliersStatisticalOutlierRemoval
* Ha a felhasználó rámegy a Statisztikailag kiálló részek eltávolításához tartozó rádiógombra,
*  ez a függvény fog lefutni. A paraméterek beállítódnak az "On Source " és az "On Target értékeitől függően
*/
void
PreprocessingParamsDialog::ClickedRadioButtonRemoveOutliersStatisticalOutlierRemoval()
{
    QFormLayout *formLayout = new QFormLayout;
    formLayout->addRow(_radiobuttonsPreprocessing["OUTREM_STAT_OUTREM"]);
    formLayout->addRow(_radiobuttonsPreprocessing["OUTREM_RAD_OUTREM"]);
    formLayout->addRow( _checkboxesPreprocessing["OUTREM_OnSource"], _checkboxesPreprocessing["OUTREM_OnTarget"]);
    _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget(_radiobuttonsPreprocessing["OUTREM_STAT_OUTREM"]);
    _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget(_radiobuttonsPreprocessing["OUTREM_RAD_OUTREM"]);
    _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget( _checkboxesPreprocessing["OUTREM_OnSource"]);
    _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget( _checkboxesPreprocessing["OUTREM_OnTarget"]);

    if(_checkboxesPreprocessing["OUTREM_OnSource"]->isChecked())
    {
        formLayout->addRow(_labelsPreprocessing["OUTREM_SetMeanKSource_STAT"],_spinboxesPreprocessing["OUTREM_SetMeanKSource_STAT"]);
        formLayout->addRow(_labelsPreprocessing["OUTREM_SetStddevMulThreshSource_STAT"], _spinboxesPreprocessing["OUTREM_SetStddevMulThreshSource_STAT"]);
        formLayout->addRow( _labelsPreprocessing["OUTREM_SetNegativeSource_STAT"],_checkboxesPreprocessing["OUTREM_SetNegativeSource_STAT"]);
        _spinboxesPreprocessing["OUTREM_SetMeanKSource_STAT"]->setVisible(true);
        _labelsPreprocessing["OUTREM_SetMeanKSource_STAT"]->setVisible(true);
        _spinboxesPreprocessing["OUTREM_SetStddevMulThreshSource_STAT"]->setVisible(true);
        _labelsPreprocessing["OUTREM_SetStddevMulThreshSource_STAT"]->setVisible(true);
        _checkboxesPreprocessing["OUTREM_SetNegativeSource_STAT"]->setVisible(true);
        _labelsPreprocessing["OUTREM_SetNegativeSource_STAT"]->setVisible(true);

        _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_spinboxesPreprocessing["OUTREM_SetMinNeighborsInRadiusSource_RAD"]);
        _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_spinboxesPreprocessing["OUTREM_SetRadiusSearchSource_RAD"]);
        _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_labelsPreprocessing["OUTREM_SetMinNeighborsInRadiusSource_RAD"]);
        _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_labelsPreprocessing["OUTREM_SetRadiusSearchSource_RAD"]);
        _spinboxesPreprocessing["OUTREM_SetMinNeighborsInRadiusSource_RAD"]->setVisible(false);
        _spinboxesPreprocessing["OUTREM_SetRadiusSearchSource_RAD"]->setVisible(false);
        _labelsPreprocessing["OUTREM_SetMinNeighborsInRadiusSource_RAD"]->setVisible(false);
        _labelsPreprocessing["OUTREM_SetRadiusSearchSource_RAD"]->setVisible(false);
    }
    if(_checkboxesPreprocessing["OUTREM_OnTarget"]->isChecked())
    {
        formLayout->addRow(_labelsPreprocessing["OUTREM_SetMeanKTarget_STAT"], _spinboxesPreprocessing["OUTREM_SetMeanKTarget_STAT"]);
        formLayout->addRow(_labelsPreprocessing["OUTREM_SetStddevMulThreshTarget_STAT"], _spinboxesPreprocessing["OUTREM_SetStddevMulThreshTarget_STAT"]);
        formLayout->addRow(_labelsPreprocessing["OUTREM_SetNegativeTarget_STAT"],  _checkboxesPreprocessing["OUTREM_SetNegativeTarget_STAT"]);
        _labelsPreprocessing["OUTREM_SetMeanKTarget_STAT"]->setVisible(true);
        _labelsPreprocessing["OUTREM_SetStddevMulThreshTarget_STAT"]->setVisible(true);
        _labelsPreprocessing["OUTREM_SetNegativeTarget_STAT"]->setVisible(true);
        _spinboxesPreprocessing["OUTREM_SetMeanKTarget_STAT"]->setVisible(true);
        _spinboxesPreprocessing["OUTREM_SetStddevMulThreshTarget_STAT"]->setVisible(true);
        _checkboxesPreprocessing["OUTREM_SetNegativeTarget_STAT"]->setVisible(true);

        _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_labelsPreprocessing["OUTREM_SetMinNeighborsInRadiusTarget_RAD"]);
        _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_labelsPreprocessing["OUTREM_SetRadiusSearchTarget_RAD"]);
        _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_spinboxesPreprocessing["OUTREM_SetRadiusSearchTarget_RAD"]);
        _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_spinboxesPreprocessing["OUTREM_SetMinNeighborsInRadiusTarget_RAD"]);
        _labelsPreprocessing["OUTREM_SetMinNeighborsInRadiusTarget_RAD"]->setVisible(false);
        _labelsPreprocessing["OUTREM_SetRadiusSearchTarget_RAD"]->setVisible(false);
        _spinboxesPreprocessing["OUTREM_SetRadiusSearchTarget_RAD"]->setVisible(false);
        _spinboxesPreprocessing["OUTREM_SetMinNeighborsInRadiusTarget_RAD"]->setVisible(false);
    }
    delete _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout();
    _groupboxesPreprocessing["OUTLIERSREMOVAL"]->setLayout(formLayout);
}

/*!
* \brief PreprocessingParamsDialog::ClickedRadioButtonRemoveOutliersRadiusOutlierRemoval
* Ha a felhasználó rámegy a Sugár alapján mért kiálló részek eltávolításához tartozó rádiógombra,
*  ez a függvény fog lefutni. A paraméterek beállítódnak az "On Source " és az "On Target értékeitől függően
*/
void
PreprocessingParamsDialog::ClickedRadioButtonRemoveOutliersRadiusOutlierRemoval()
{
    QFormLayout *formLayout = new QFormLayout;
    formLayout->addRow(_radiobuttonsPreprocessing["OUTREM_STAT_OUTREM"]);
    formLayout->addRow(_radiobuttonsPreprocessing["OUTREM_RAD_OUTREM"]);
    formLayout->addRow( _checkboxesPreprocessing["OUTREM_OnSource"], _checkboxesPreprocessing["OUTREM_OnTarget"]);
    _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget(_radiobuttonsPreprocessing["OUTREM_STAT_OUTREM"]);
    _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget(_radiobuttonsPreprocessing["OUTREM_RAD_OUTREM"]);
    _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget( _checkboxesPreprocessing["OUTREM_OnSource"]);
    _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget( _checkboxesPreprocessing["OUTREM_OnTarget"]);

    if( _checkboxesPreprocessing["OUTREM_OnSource"]->isChecked())
    {
        formLayout->addRow(_labelsPreprocessing["OUTREM_SetMinNeighborsInRadiusSource_RAD"],_spinboxesPreprocessing["OUTREM_SetMinNeighborsInRadiusSource_RAD"]);
        formLayout->addRow(_labelsPreprocessing["OUTREM_SetRadiusSearchSource_RAD"], _spinboxesPreprocessing["OUTREM_SetRadiusSearchSource_RAD"]);

        _spinboxesPreprocessing["OUTREM_SetMinNeighborsInRadiusSource_RAD"]->setVisible(true);
        _labelsPreprocessing["OUTREM_SetRadiusSearchSource_RAD"]->setVisible(true);
        _labelsPreprocessing["OUTREM_SetMinNeighborsInRadiusSource_RAD"]->setVisible(true);
        _spinboxesPreprocessing["OUTREM_SetRadiusSearchSource_RAD"]->setVisible(true);

        _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget( _spinboxesPreprocessing["OUTREM_SetMeanKSource_STAT"]);
        _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget( _spinboxesPreprocessing["OUTREM_SetStddevMulThreshSource_STAT"]);
        _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_labelsPreprocessing["OUTREM_SetMeanKSource_STAT"]);
        _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_labelsPreprocessing["OUTREM_SetStddevMulThreshSource_STAT"]);
        _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget( _labelsPreprocessing["OUTREM_SetNegativeSource_STAT"]);
        _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_checkboxesPreprocessing["OUTREM_SetNegativeSource_STAT"]);
        _spinboxesPreprocessing["OUTREM_SetMeanKSource_STAT"]->setVisible(false);
        _spinboxesPreprocessing["OUTREM_SetStddevMulThreshSource_STAT"]->setVisible(false);
        _labelsPreprocessing["OUTREM_SetMeanKSource_STAT"]->setVisible(false);
        _labelsPreprocessing["OUTREM_SetStddevMulThreshSource_STAT"]->setVisible(false);
        _labelsPreprocessing["OUTREM_SetNegativeSource_STAT"]->setVisible(false);
        _checkboxesPreprocessing["OUTREM_SetNegativeSource_STAT"]->setVisible(false);
    }
    if(_checkboxesPreprocessing["OUTREM_OnTarget"]->isChecked())
    {
        formLayout->addRow(_labelsPreprocessing["OUTREM_SetMinNeighborsInRadiusTarget_RAD"],_spinboxesPreprocessing["OUTREM_SetMinNeighborsInRadiusTarget_RAD"]);
        formLayout->addRow(_labelsPreprocessing["OUTREM_SetRadiusSearchTarget_RAD"], _spinboxesPreprocessing["OUTREM_SetRadiusSearchTarget_RAD"]);

        _spinboxesPreprocessing["OUTREM_SetRadiusSearchTarget_RAD"]->setVisible(true);
        _spinboxesPreprocessing["OUTREM_SetMinNeighborsInRadiusTarget_RAD"]->setVisible(true);
        _labelsPreprocessing["OUTREM_SetMinNeighborsInRadiusTarget_RAD"]->setVisible(true);
        _labelsPreprocessing["OUTREM_SetRadiusSearchTarget_RAD"]->setVisible(true);

        _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_spinboxesPreprocessing["OUTREM_SetMeanKTarget_STAT"]);
        _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_spinboxesPreprocessing["OUTREM_SetStddevMulThreshTarget_STAT"]);
        _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_labelsPreprocessing["OUTREM_SetMeanKTarget_STAT"]);
        _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_labelsPreprocessing["OUTREM_SetStddevMulThreshTarget_STAT"]);
        _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget(_labelsPreprocessing["OUTREM_SetNegativeTarget_STAT"]);
        _groupboxesPreprocessing["DOWNSAMPLING"]->layout()->removeWidget( _checkboxesPreprocessing["OUTREM_SetNegativeTarget_STAT"]);
        _spinboxesPreprocessing["OUTREM_SetMeanKTarget_STAT"]->setVisible(false);
        _labelsPreprocessing["OUTREM_SetNegativeTarget_STAT"]->setVisible(false);
        _spinboxesPreprocessing["OUTREM_SetStddevMulThreshTarget_STAT"]->setVisible(false);
       _labelsPreprocessing["OUTREM_SetMeanKTarget_STAT"]->setVisible(false);
        _labelsPreprocessing["OUTREM_SetStddevMulThreshTarget_STAT"]->setVisible(false);
       _checkboxesPreprocessing["OUTREM_SetNegativeTarget_STAT"]->setVisible(false);
    }
     delete _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout();
    _groupboxesPreprocessing["OUTLIERSREMOVAL"]->setLayout(formLayout);
}


/*!
* \brief PreprocessingParamsDialog::ClickedCheckBoxOutliersRemovalSource
* \param ischecked
* A kiálló részek eltávolításához tartozó paraméterek beállításait kezeli, abban az esetben, ha a felhasználó az adott fülbel az "On Source"
* jelölőnégyzetre megy rá
*/
void
PreprocessingParamsDialog:: ClickedCheckBoxOutliersRemovalSource(bool ischecked)
{
    QFormLayout* formLayout = new QFormLayout();
    formLayout->addRow(_radiobuttonsPreprocessing["OUTREM_STAT_OUTREM"]);
    formLayout->addRow(_radiobuttonsPreprocessing["OUTREM_RAD_OUTREM"]);
    formLayout->addRow( _checkboxesPreprocessing["OUTREM_OnSource"], _checkboxesPreprocessing["OUTREM_OnTarget"]);
    _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget(_radiobuttonsPreprocessing["OUTREM_STAT_OUTREM"]);
    _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget(_radiobuttonsPreprocessing["OUTREM_RAD_OUTREM"]);
    _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget(_checkboxesPreprocessing["OUTREM_OnSource"]);
    _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget(_checkboxesPreprocessing["OUTREM_OnTarget"]);

    if(_radiobuttonsPreprocessing["OUTREM_STAT_OUTREM"]->isChecked())
    {
        if(ischecked)
        {
            formLayout->addRow(_labelsPreprocessing["OUTREM_SetMeanKSource_STAT"], _spinboxesPreprocessing["OUTREM_SetMeanKSource_STAT"]);
            formLayout->addRow(_labelsPreprocessing["OUTREM_SetStddevMulThreshSource_STAT"], _spinboxesPreprocessing["OUTREM_SetStddevMulThreshSource_STAT"]);
            formLayout->addRow( _labelsPreprocessing["OUTREM_SetNegativeSource_STAT"],_checkboxesPreprocessing["OUTREM_SetNegativeSource_STAT"]);
            _spinboxesPreprocessing["OUTREM_SetMeanKSource_STAT"]->setVisible(true);
            _labelsPreprocessing["OUTREM_SetMeanKSource_STAT"]->setVisible(true);
            _spinboxesPreprocessing["OUTREM_SetStddevMulThreshSource_STAT"]->setVisible(true);
            _labelsPreprocessing["OUTREM_SetStddevMulThreshSource_STAT"]->setVisible(true);
            _labelsPreprocessing["OUTREM_SetNegativeSource_STAT"]->setVisible(true);
            _checkboxesPreprocessing["OUTREM_SetNegativeSource_STAT"]->setVisible(true);
        }
        else
        {
            _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget(_labelsPreprocessing["OUTREM_SetMeanKSource_STAT"]);
            _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget(_spinboxesPreprocessing["OUTREM_SetMeanKSource_STAT"]);
            _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget(_labelsPreprocessing["OUTREM_SetStddevMulThreshSource_STAT"]);
            _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget(_spinboxesPreprocessing["OUTREM_SetStddevMulThreshSource_STAT"]);
            _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget(_labelsPreprocessing["OUTREM_SetNegativeSource_STAT"]);
            _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget(_checkboxesPreprocessing["OUTREM_SetNegativeSource_STAT"]);
            _labelsPreprocessing["OUTREM_SetMeanKSource_STAT"]->setVisible(false);
            _spinboxesPreprocessing["OUTREM_SetMeanKSource_STAT"]->setVisible(false);
            _labelsPreprocessing["OUTREM_SetStddevMulThreshSource_STAT"]->setVisible(false);
            _spinboxesPreprocessing["OUTREM_SetStddevMulThreshSource_STAT"]->setVisible(false);
            _labelsPreprocessing["OUTREM_SetNegativeSource_STAT"]->setVisible(false);
            _checkboxesPreprocessing["OUTREM_SetNegativeSource_STAT"]->setVisible(false);
        }
        if(_checkboxesPreprocessing["OUTREM_OnTarget"]->isChecked())
        {
            formLayout->addRow(_labelsPreprocessing["OUTREM_SetMeanKTarget_STAT"],_spinboxesPreprocessing["OUTREM_SetMeanKTarget_STAT"]);
            formLayout->addRow( _labelsPreprocessing["OUTREM_SetStddevMulThreshTarget_STAT"],_spinboxesPreprocessing["OUTREM_SetStddevMulThreshTarget_STAT"]);
            formLayout->addRow(_labelsPreprocessing["OUTREM_SetNegativeTarget_STAT"],_checkboxesPreprocessing["OUTREM_SetNegativeTarget_STAT"]);
            _labelsPreprocessing["OUTREM_SetMeanKTarget_STAT"]->setVisible(true);
            _spinboxesPreprocessing["OUTREM_SetMeanKTarget_STAT"]->setVisible(true);
            _labelsPreprocessing["OUTREM_SetStddevMulThreshTarget_STAT"]->setVisible(true);
            _spinboxesPreprocessing["OUTREM_SetStddevMulThreshTarget_STAT"]->setVisible(true);
            _labelsPreprocessing["OUTREM_SetNegativeTarget_STAT"]->setVisible(true);
            _checkboxesPreprocessing["OUTREM_SetNegativeTarget_STAT"]->setVisible(true);
        }
    }
    else
    {
        if(ischecked)
        {
            formLayout->addRow(_labelsPreprocessing["OUTREM_SetMinNeighborsInRadiusSource_RAD"],_spinboxesPreprocessing["OUTREM_SetMinNeighborsInRadiusSource_RAD"]);
            formLayout->addRow(_labelsPreprocessing["OUTREM_SetRadiusSearchSource_RAD"],_spinboxesPreprocessing["OUTREM_SetRadiusSearchSource_RAD"]);
            _labelsPreprocessing["OUTREM_SetMinNeighborsInRadiusSource_RAD"]->setVisible(true);
            _spinboxesPreprocessing["OUTREM_SetMinNeighborsInRadiusSource_RAD"]->setVisible(true);
            _labelsPreprocessing["OUTREM_SetRadiusSearchSource_RAD"]->setVisible(true);
            _spinboxesPreprocessing["OUTREM_SetRadiusSearchSource_RAD"]->setVisible(true);
        }
        else{
            _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget(_labelsPreprocessing["OUTREM_SetMinNeighborsInRadiusSource_RAD"]);
            _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget(_spinboxesPreprocessing["OUTREM_SetMinNeighborsInRadiusSource_RAD"]);
            _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget(_labelsPreprocessing["OUTREM_SetRadiusSearchSource_RAD"]);
            _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget(_spinboxesPreprocessing["OUTREM_SetRadiusSearchSource_RAD"]);

            _labelsPreprocessing["OUTREM_SetMinNeighborsInRadiusSource_RAD"]->setVisible(false);
            _spinboxesPreprocessing["OUTREM_SetMinNeighborsInRadiusSource_RAD"]->setVisible(false);
            _labelsPreprocessing["OUTREM_SetRadiusSearchSource_RAD"]->setVisible(false);
            _spinboxesPreprocessing["OUTREM_SetRadiusSearchSource_RAD"]->setVisible(false);
        }
        if(_checkboxesPreprocessing["OUTREM_OnTarget"]->isChecked())
        {
            formLayout->addRow(_labelsPreprocessing["OUTREM_SetMinNeighborsInRadiusTarget_RAD"],_spinboxesPreprocessing["OUTREM_SetMinNeighborsInRadiusTarget_RAD"]);
            formLayout->addRow(_labelsPreprocessing["OUTREM_SetRadiusSearchTarget_RAD"],_spinboxesPreprocessing["OUTREM_SetRadiusSearchTarget_RAD"]);

            _labelsPreprocessing["OUTREM_SetMinNeighborsInRadiusTarget_RAD"]->setVisible(true);
            _spinboxesPreprocessing["OUTREM_SetMinNeighborsInRadiusTarget_RAD"]->setVisible(true);
            _labelsPreprocessing["OUTREM_SetRadiusSearchTarget_RAD"]->setVisible(true);
            _spinboxesPreprocessing["OUTREM_SetRadiusSearchTarget_RAD"]->setVisible(true);
        }
    }
    delete _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout();
    _groupboxesPreprocessing["OUTLIERSREMOVAL"]->setLayout(formLayout);
}


/*!
* \brief PreprocessingParamsDialog::ClickedCheckBoxOutliersRemovalTarget
* \param ischecked
* A kiálló részek eltávolításához tartozó paraméterek beállításait kezeli, abban az esetben, ha a felhasználó az adott fülbel az "On Target"
* jelölőnégyzetre megy rá
*/
void
PreprocessingParamsDialog:: ClickedCheckBoxOutliersRemovalTarget(bool ischecked)
{
    QFormLayout* formLayout = new QFormLayout();
    formLayout->addRow(_radiobuttonsPreprocessing["OUTREM_STAT_OUTREM"]);
    formLayout->addRow(_radiobuttonsPreprocessing["OUTREM_RAD_OUTREM"]);
    formLayout->addRow( _checkboxesPreprocessing["OUTREM_OnSource"], _checkboxesPreprocessing["OUTREM_OnTarget"]);
    _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget(_radiobuttonsPreprocessing["OUTREM_STAT_OUTREM"]);
    _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget(_radiobuttonsPreprocessing["OUTREM_RAD_OUTREM"]);
    _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget( _checkboxesPreprocessing["OUTREM_OnSource"]);
    _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget( _checkboxesPreprocessing["OUTREM_OnTarget"]);
    if(_radiobuttonsPreprocessing["OUTREM_STAT_OUTREM"]->isChecked())
    {
        if( _checkboxesPreprocessing["OUTREM_OnSource"]->isChecked())
        {
            formLayout->addRow(_labelsPreprocessing["OUTREM_SetMeanKSource_STAT"],_spinboxesPreprocessing["OUTREM_SetMeanKSource_STAT"]);
            formLayout->addRow(_labelsPreprocessing["OUTREM_SetStddevMulThreshSource_STAT"],_spinboxesPreprocessing["OUTREM_SetStddevMulThreshSource_STAT"]);
            formLayout->addRow(_labelsPreprocessing["OUTREM_SetNegativeSource_STAT"],_checkboxesPreprocessing["OUTREM_SetNegativeSource_STAT"]);
            _labelsPreprocessing["OUTREM_SetMeanKSource_STAT"]->setVisible(true);
            _spinboxesPreprocessing["OUTREM_SetMeanKSource_STAT"]->setVisible(true);
            _labelsPreprocessing["OUTREM_SetStddevMulThreshSource_STAT"]->setVisible(true);
            _spinboxesPreprocessing["OUTREM_SetStddevMulThreshSource_STAT"]->setVisible(true);
            _labelsPreprocessing["OUTREM_SetNegativeSource_STAT"]->setVisible(true);
            _checkboxesPreprocessing["OUTREM_SetNegativeSource_STAT"]->setVisible(true);
        }
        if(ischecked)
        {
            formLayout->addRow(_labelsPreprocessing["OUTREM_SetMeanKTarget_STAT"],_spinboxesPreprocessing["OUTREM_SetMeanKTarget_STAT"]);
            formLayout->addRow(_labelsPreprocessing["OUTREM_SetStddevMulThreshTarget_STAT"],_spinboxesPreprocessing["OUTREM_SetStddevMulThreshTarget_STAT"]);
            formLayout->addRow(_labelsPreprocessing["OUTREM_SetNegativeTarget_STAT"],_checkboxesPreprocessing["OUTREM_SetNegativeTarget_STAT"]);
            _labelsPreprocessing["OUTREM_SetMeanKTarget_STAT"]->setVisible(true);
            _spinboxesPreprocessing["OUTREM_SetMeanKTarget_STAT"]->setVisible(true);
            _labelsPreprocessing["OUTREM_SetStddevMulThreshTarget_STAT"]->setVisible(true);
            _spinboxesPreprocessing["OUTREM_SetStddevMulThreshTarget_STAT"]->setVisible(true);
            _labelsPreprocessing["OUTREM_SetNegativeTarget_STAT"]->setVisible(true);
            _checkboxesPreprocessing["OUTREM_SetNegativeTarget_STAT"]->setVisible(true);
        }
        else{
            _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget(_labelsPreprocessing["OUTREM_SetMeanKTarget_STAT"]);
            _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget(_spinboxesPreprocessing["OUTREM_SetMeanKTarget_STAT"]);
            _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget(_labelsPreprocessing["OUTREM_SetStddevMulThreshTarget_STAT"]);
            _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget(_spinboxesPreprocessing["OUTREM_SetStddevMulThreshTarget_STAT"]);
            _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget(_labelsPreprocessing["OUTREM_SetNegativeTarget_STAT"]);
            _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget(_checkboxesPreprocessing["OUTREM_SetNegativeTarget_STAT"]);

            _labelsPreprocessing["OUTREM_SetMeanKTarget_STAT"]->setVisible(false);
            _spinboxesPreprocessing["OUTREM_SetMeanKTarget_STAT"]->setVisible(false);
            _labelsPreprocessing["OUTREM_SetStddevMulThreshTarget_STAT"]->setVisible(false);
            _spinboxesPreprocessing["OUTREM_SetStddevMulThreshTarget_STAT"]->setVisible(false);
            _labelsPreprocessing["OUTREM_SetNegativeTarget_STAT"]->setVisible(false);
            _checkboxesPreprocessing["OUTREM_SetNegativeTarget_STAT"]->setVisible(false);
        }
    }
    else
    {
        if(_checkboxesPreprocessing["OUTREM_OnSource"]->isChecked())
        {
            formLayout->addRow(_labelsPreprocessing["OUTREM_SetMinNeighborsInRadiusSource_RAD"],_spinboxesPreprocessing["OUTREM_SetMinNeighborsInRadiusSource_RAD"]);
            formLayout->addRow(_labelsPreprocessing["OUTREM_SetRadiusSearchSource_RAD"],_spinboxesPreprocessing["OUTREM_SetRadiusSearchSource_RAD"]);
            _labelsPreprocessing["OUTREM_SetMinNeighborsInRadiusSource_RAD"]->setVisible(true);
            _spinboxesPreprocessing["OUTREM_SetMinNeighborsInRadiusSource_RAD"]->setVisible(true);
            _labelsPreprocessing["OUTREM_SetRadiusSearchSource_RAD"]->setVisible(true);
            _spinboxesPreprocessing["OUTREM_SetRadiusSearchSource_RAD"]->setVisible(true);
        }
        if(ischecked)
        {
            formLayout->addRow(_labelsPreprocessing["OUTREM_SetMinNeighborsInRadiusTarget_RAD"],_spinboxesPreprocessing["OUTREM_SetMinNeighborsInRadiusTarget_RAD"]);
            formLayout->addRow(_labelsPreprocessing["OUTREM_SetRadiusSearchTarget_RAD"],_spinboxesPreprocessing["OUTREM_SetRadiusSearchTarget_RAD"]);

            _labelsPreprocessing["OUTREM_SetMinNeighborsInRadiusTarget_RAD"]->setVisible(true);
            _spinboxesPreprocessing["OUTREM_SetMinNeighborsInRadiusTarget_RAD"]->setVisible(true);
            _labelsPreprocessing["OUTREM_SetRadiusSearchTarget_RAD"]->setVisible(true);
            _spinboxesPreprocessing["OUTREM_SetRadiusSearchTarget_RAD"]->setVisible(true);
        }
        else{
            _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget(_labelsPreprocessing["OUTREM_SetMinNeighborsInRadiusTarget_RAD"]);
            _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget(_spinboxesPreprocessing["OUTREM_SetMinNeighborsInRadiusTarget_RAD"]);
            _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget(_labelsPreprocessing["OUTREM_SetRadiusSearchTarget_RAD"]);
            _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout()->removeWidget(_spinboxesPreprocessing["OUTREM_SetRadiusSearchTarget_RAD"]);
            _labelsPreprocessing["OUTREM_SetMinNeighborsInRadiusTarget_RAD"]->setVisible(false);
            _spinboxesPreprocessing["OUTREM_SetMinNeighborsInRadiusTarget_RAD"]->setVisible(false);
            _labelsPreprocessing["OUTREM_SetRadiusSearchTarget_RAD"]->setVisible(false);
            _spinboxesPreprocessing["OUTREM_SetRadiusSearchTarget_RAD"]->setVisible(false);
        }
    }
    delete _groupboxesPreprocessing["OUTLIERSREMOVAL"]->layout();
    _groupboxesPreprocessing["OUTLIERSREMOVAL"]->setLayout(formLayout);
}


/*!
* \brief PreprocessingParamsDialog::ParametersforSmoothing
* \return
* A simításhoz szükséges paraméterek inicializálását és az elemek eltárolását végzi ez a  függvény. Visszatérési értéke a megfelelően feltöltött
* elem csoport, ami a megfelelő helyen fog megjelenni a dialógusablakban
*/
QGroupBox *
PreprocessingParamsDialog:: ParametersSmoothing()
{
    _groupboxesPreprocessing["SMOOTHING"] = new QGroupBox(tr("Smoothing"));
    _groupboxesPreprocessing["SMOOTHING"]->setCheckable(true);
    _groupboxesPreprocessing["SMOOTHING"]->setChecked(false);

    _checkboxesPreprocessing["SMOOTHING_OnSource"] = new QCheckBox("On Source");
    _checkboxesPreprocessing["SMOOTHING_OnTarget"] = new QCheckBox("On Target");
    _checkboxesPreprocessing["SMOOTHING_OnSource"]->setChecked(true);
    _checkboxesPreprocessing["SMOOTHING_OnTarget"]->setChecked(true);

    _radiobuttonsPreprocessing["SMOOTHING_MLS"] = new QRadioButton(tr("Smoothing based on Moving Least Squares"));
    _radiobuttonsPreprocessing["SMOOTHING_MLS"]->setChecked(true);

    _labelsPreprocessing["SMOOTHING_SetSearchRadiusSource_MLS"] = new QLabel("The sphere radius that is to \ncontain all k-nearest neighbors (Source): ");
    _labelsPreprocessing["SMOOTHING_SetSearchRadiusTarget_MLS"] = new QLabel("the sphere radius that is to \ncontain all k-nearest neighbors (Target): ");
    _spinboxesPreprocessing["SMOOTHING_SetSearchRadiusSource_MLS"] = new QDoubleSpinBox();
    _spinboxesPreprocessing["SMOOTHING_SetSearchRadiusTarget_MLS"] = new QDoubleSpinBox();
    SetupSpinBox(*_spinboxesPreprocessing["SMOOTHING_SetSearchRadiusSource_MLS"],1,0,4);
    SetupSpinBox(*_spinboxesPreprocessing["SMOOTHING_SetSearchRadiusTarget_MLS"],1,0,4);

    //kapcsolatok beállítása az egyes gombok és funkciók között
    connect(_checkboxesPreprocessing["SMOOTHING_OnSource"],SIGNAL(clicked(bool)),this,SLOT(ClickedCheckBoxSmoothingSource(bool)));
    connect(_checkboxesPreprocessing["SMOOTHING_OnTarget"],SIGNAL(clicked(bool)),this,SLOT(ClickedCheckBoxSmoothingTarget(bool)));

    //a visszaadni kívánt Layout feltöltése a paraméterekhez tartozó elemekkel
    QFormLayout *formLayout = new QFormLayout;
    formLayout->addRow(_radiobuttonsPreprocessing["SMOOTHING_MLS"]);
    formLayout->addRow(_checkboxesPreprocessing["SMOOTHING_OnSource"],_checkboxesPreprocessing["SMOOTHING_OnTarget"]);
    formLayout->addRow(_labelsPreprocessing["SMOOTHING_SetSearchRadiusSource_MLS"],_spinboxesPreprocessing["SMOOTHING_SetSearchRadiusSource_MLS"]);
    formLayout->addRow(_labelsPreprocessing["SMOOTHING_SetSearchRadiusTarget_MLS"],_spinboxesPreprocessing["SMOOTHING_SetSearchRadiusTarget_MLS"]);
    _groupboxesPreprocessing["SMOOTHING"]->setLayout(formLayout);
    return _groupboxesPreprocessing["SMOOTHING"];
}


/*!
* \brief PreprocessingParamsDialog::ClickedCheckBoxSmoothingSource
* \param ischecked
*A simításhoz tartozó paraméterek beállításait kezeli, abban az esetben, ha a felhasználó az adott fülbel az "On Source"
* jelölőnégyzetre megy rá
*/
void
PreprocessingParamsDialog::ClickedCheckBoxSmoothingSource(bool ischecked)
{
    QFormLayout *formLayout = new QFormLayout;
    formLayout->addRow(_radiobuttonsPreprocessing["SMOOTHING_MLS"]);
    formLayout->addRow(_checkboxesPreprocessing["SMOOTHING_OnSource"],_checkboxesPreprocessing["SMOOTHING_OnTarget"]);

    if(ischecked)
    {
        formLayout->addRow(_labelsPreprocessing["SMOOTHING_SetSearchRadiusSource_MLS"],_spinboxesPreprocessing["SMOOTHING_SetSearchRadiusSource_MLS"]);
        _labelsPreprocessing["SMOOTHING_SetSearchRadiusSource_MLS"]->setVisible(true);
        _spinboxesPreprocessing["SMOOTHING_SetSearchRadiusSource_MLS"]->setVisible(true);
    }
    else
    {
        _groupboxesPreprocessing["SMOOTHING"]->layout()->removeWidget(_labelsPreprocessing["SMOOTHING_SetSearchRadiusSource_MLS"]);
        _groupboxesPreprocessing["SMOOTHING"]->layout()->removeWidget(_spinboxesPreprocessing["SMOOTHING_SetSearchRadiusSource_MLS"]);
        _labelsPreprocessing["SMOOTHING_SetSearchRadiusSource_MLS"]->setVisible(false);
        _spinboxesPreprocessing["SMOOTHING_SetSearchRadiusSource_MLS"]->setVisible(false);
    }
    if(_checkboxesPreprocessing["SMOOTHING_OnTarget"]->isChecked())
    {
        formLayout->addRow(_labelsPreprocessing["SMOOTHING_SetSearchRadiusTarget_MLS"],_spinboxesPreprocessing["SMOOTHING_SetSearchRadiusTarget_MLS"]);
        _labelsPreprocessing["SMOOTHING_SetSearchRadiusTarget_MLS"]->setVisible(true);
        _spinboxesPreprocessing["SMOOTHING_SetSearchRadiusTarget_MLS"]->setVisible(true);
    }
    delete _groupboxesPreprocessing["SMOOTHING"]->layout();
    _groupboxesPreprocessing["SMOOTHING"]->setLayout(formLayout);
}


/*! \brief PreprocessingParamsDialog::ClickedCheckBoxSmoothingTarget
* \param ischecked
*A simításhoz tartozó paraméterek beállításait kezeli, abban az esetben, ha a felhasználó az adott fülbel az "On Target"
* jelölőnégyzetre megy rá
*/
void
PreprocessingParamsDialog::ClickedCheckBoxSmoothingTarget(bool ischecked)
{
    QFormLayout *formLayout = new QFormLayout;
    formLayout->addRow(_radiobuttonsPreprocessing["SMOOTHING_MLS"]);
    formLayout->addRow(_checkboxesPreprocessing["SMOOTHING_OnSource"],_checkboxesPreprocessing["SMOOTHING_OnTarget"]);

    if(_checkboxesPreprocessing["SMOOTHING_OnSource"]->isChecked())
    {
         formLayout->addRow(_labelsPreprocessing["SMOOTHING_SetSearchRadiusSource_MLS"],_spinboxesPreprocessing["SMOOTHING_SetSearchRadiusSource_MLS"]);
         _labelsPreprocessing["SMOOTHING_SetSearchRadiusSource_MLS"]->setVisible(true);
         _spinboxesPreprocessing["SMOOTHING_SetSearchRadiusSource_MLS"]->setVisible(true);
     }
    if(ischecked)
    {
        formLayout->addRow(_labelsPreprocessing["SMOOTHING_SetSearchRadiusTarget_MLS"],_spinboxesPreprocessing["SMOOTHING_SetSearchRadiusTarget_MLS"]);
        _labelsPreprocessing["SMOOTHING_SetSearchRadiusTarget_MLS"]->setVisible(true);
        _spinboxesPreprocessing["SMOOTHING_SetSearchRadiusTarget_MLS"]->setVisible(true);
    }
    else
    {
        _groupboxesPreprocessing["SMOOTHING"]->layout()->removeWidget(_labelsPreprocessing["SMOOTHING_SetSearchRadiusTarget_MLS"]);
        _groupboxesPreprocessing["SMOOTHING"]->layout()->removeWidget(_spinboxesPreprocessing["SMOOTHING_SetSearchRadiusTarget_MLS"]);
        _labelsPreprocessing["SMOOTHING_SetSearchRadiusTarget_MLS"]->setVisible(false);
        _spinboxesPreprocessing["SMOOTHING_SetSearchRadiusTarget_MLS"]->setVisible(false);
    }
    delete _groupboxesPreprocessing["SMOOTHING"]->layout();
    _groupboxesPreprocessing["SMOOTHING"]->setLayout(formLayout);
}


/*!
* \brief PreprocessingParamsDialog::Actionbuttons
* \return QGroupBox* típusú paraméter csoportra mutató pointer, ami reprezentálja az adott modul csoportok megjelenítését a megfelelő layout-tal
* Az elfogadó illetve elutasító gombok megjelenítésére szolgáló layoutot adja vissza
*/
QGroupBox*
PreprocessingParamsDialog:: Actionbuttons()
{
    QGroupBox* _actionButtons=new QGroupBox(this);
    _actionButtons->setTitle("Action buttons");
  //  _groupboxesPreprocessing["ActionButtons"] = _actionButtons;
    QPushButton* _okbutton = new QPushButton(_actionButtons);
    _okbutton->setText("Start Preprocessing");
    QPushButton* _cancelbutton = new QPushButton(_actionButtons);
    _cancelbutton->setText("Back");
    QPushButton* _helpbutton = new QPushButton(_actionButtons);
    _helpbutton->setText("Help");


    //kapcsolatok felépítése, ha az "Ok" vagy a "Cancel" gombra kattint a felhasználó a dialógus ablak jelzést  küld a szülőjének
    connect(_okbutton,SIGNAL(clicked() ),this, SLOT(accept()));
    connect(_cancelbutton,SIGNAL(clicked() ),this, SLOT(reject()));
    connect(_helpbutton,SIGNAL(clicked() ),this, SLOT(ClickedHelpButton()));

    QFormLayout *formLayout = new QFormLayout();
    formLayout->addRow(_okbutton);
    formLayout->addRow(_cancelbutton);
    formLayout->addRow(_helpbutton);
    _actionButtons->setLayout(formLayout);
    return _actionButtons;
}


/*!
* \brief PreprocessingParamsDialog::ClickedHelpButton
*Ha a felhasználó a "Help" gombra kattint, akkor egy felugró ablakkal néhány tanácsot adunk neki üzenet doboz segítségével
*/
void
PreprocessingParamsDialog :: ClickedHelpButton()
{
    QMessageBox::information(this,"Help","Preprocessing for more accurate and faster registration processing\n" "\n1. If you want to reduce the density of your point clouds, use one of the downsampling methods.\n"
                                         "\n2. If your point clouds are noisy, and you want to remove outliers, use one of the outliers removal methods.\n"
                                         "\n3. If your point clouds are noisy, and you want to use smoothing to resample your point clouds, use Moving Least Square Smoothing method.\n"
                                         "\nAttention! If certain parameters' value was zero,\nthese'd set to default value!");
}


/*!
* \brief PreprocessingParamsDialog::getRadiobuttonsPreprocessing
* \return A rádiógombok tárolására szolgáló map tároló
*Radiógombokhoz tartozó Getter függvény
*/
std::map<std::string, QRadioButton *> PreprocessingParamsDialog::getRadiobuttonsPreprocessing() const
{
    return _radiobuttonsPreprocessing;
}


/*!
* \brief PreprocessingParamsDialog::getCheckboxesPreprocessing
* \return A Checkbox-ok tárolására szolgáló map tároló
*Jelölőnégyzetekhez tartozó Getter függvény
*/
std::map<std::string, QCheckBox *> PreprocessingParamsDialog::getCheckboxesPreprocessing() const
{
    return _checkboxesPreprocessing;
}


/*!
* \brief PreprocessingParamsDialog::getSpinboxesPreprocessing
* \return a spinboxokhoz tartozó map tároló
*SpinBox-okhoz tartozó Getter függvény
*/
std::map<std::string, QDoubleSpinBox *> PreprocessingParamsDialog::getSpinboxesPreprocessing() const
{
    return _spinboxesPreprocessing;
}


/*!
* \brief PreprocessingParamsDialog::getGroupboxesPreprocessing
* \return a GroupBox-okhoz tartozó map tárolót adja vissza
*A GroupBox-okhoz tartozó Getter függvény
*/
std::map<std::string, QGroupBox *> PreprocessingParamsDialog::getGroupboxesPreprocessing() const
{
    return _groupboxesPreprocessing;
}


/*!
* \brief PreprocessingParamsDialog::~PreprocessingParamsDialog
*Az osztály destruktora, ahol töröljük az egyes pointerekhez tartozó memóriahelyeket
*/
PreprocessingParamsDialog::~PreprocessingParamsDialog()
{
    delete ui;
    delete _subGridlayout;
    std::map<std::string, QDoubleSpinBox*>::iterator it;

   for ( it = _spinboxesPreprocessing.begin(); it != _spinboxesPreprocessing.end(); it++ )
    {
        it->second->deleteLater();

    }
    std::map<std::string, QCheckBox*>::iterator it2;

    for ( it2 = _checkboxesPreprocessing.begin(); it2 != _checkboxesPreprocessing.end(); it2++ )
    {
        it2->second->deleteLater();

    }

    std::map<std::string, QRadioButton*>::iterator it3;

    for ( it3 = _radiobuttonsPreprocessing.begin(); it3 != _radiobuttonsPreprocessing.end(); it3++ )
    {
        it3->second->deleteLater();

    }
    std::map<std::string, QLabel*>::iterator it4;

    for ( it4 = _labelsPreprocessing.begin(); it4 != _labelsPreprocessing.end(); it4++ )
    {
        it4->second->deleteLater();

    }
    std::map<std::string, QGroupBox*>::iterator it5;

    for ( it5 = _groupboxesPreprocessing.begin(); it5 != _groupboxesPreprocessing.end(); it5++ )
    {
         it5->second->layout()->deleteLater();
         it5->second->deleteLater();
    }
}
