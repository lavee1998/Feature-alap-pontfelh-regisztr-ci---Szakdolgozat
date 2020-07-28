#include "icpparamsdialog.h"
#include "ui_icpparamsdialog.h"


/*!
* \brief ICPParamsDialog::ICPParamsDialog
* \param parent : a szülő Widgetet megkapja, hogy tudja ki a szülője
* Létrehozzuk a felületet, csúsztatható ablak segítségével megoldjuk, hogy kis képernyő esetén is használható legyen a felület,
* meghívjuk az egyes függvényeket a felület feltöltésére.
*/
ICPParamsDialog::ICPParamsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ICPParamsDialog)
{
    ui->setupUi(this);

    _mainLayout = new QVBoxLayout(this);
    _scrollArea = new QScrollArea(this);
     QWidget* scrollAreaContent = new QWidget(_scrollArea);
    _subGridLayout = new QGridLayout(scrollAreaContent);
    _scrollArea->setVerticalScrollBarPolicy( Qt::ScrollBarAsNeeded );
    _scrollArea->setWidgetResizable(true);
    setContentsMargins(0,0,0,0);
    _subGridLayout->addWidget(ParametersICP());
    scrollAreaContent->setLayout(_subGridLayout);
    _scrollArea->setWidget(scrollAreaContent);
    _mainLayout->addWidget(_scrollArea);
    setLayout(_mainLayout);
    _subGridLayout->setSizeConstraint(QLayout::SetMinimumSize);
}


/*!
* \brief ICPParamsDialog::SetupSpinBox
* \param spinbox: a módosítani kívánt DoubleSpinBox
* \param max : a maximum értéke
* \param min: a minimum értéke
* \param decimals : a tizedes jegy utáni számjegyek száma
* Beállítja egey előre megadott DoubleSpinBox típusú elem egyes tulajdonságait.
*/
void
ICPParamsDialog::SetupSpinBox(QDoubleSpinBox &spinbox, double max, double min, int decimals)
{
    //az egyes Spinboxok tulajdonságait állítjuk be
    spinbox.setMaximum(max); //maximumát
    spinbox.setMinimum(min); //minimumát
    spinbox.setDecimals(decimals); //a tizedesjegy utáni számokat
}

/*!
 * \brief ICPParamsDialog::getSpinboxesICP
 * \return az ICP spinboxokhoz tartozó map tárolót adja vissza
 */
std::map<std::string, QDoubleSpinBox *> ICPParamsDialog::getSpinboxesICP() const
{
    return _spinboxesICP;
}


/*!
* \brief ICPParamsDialog::~ICPParamsDialog
*Az osztály destruktora, ahol minden szükséges pointer értéket törlünk
*/
ICPParamsDialog::~ICPParamsDialog()
{
    delete ui;
    _subGridLayout->deleteLater();

    std::map<std::string, QDoubleSpinBox*>::iterator it;

    for ( it = _spinboxesICP.begin(); it != _spinboxesICP.end(); it++ )
    {
        it->second->deleteLater();
    }
    std::map<std::string, QLabel*>::iterator it2;

    for ( it2 = _labelsICP.begin(); it2 != _labelsICP.end(); it2++ )
    {
        it2->second->deleteLater();
    }
    _groupBoxICP->layout()->deleteLater();
    _groupBoxICP->deleteLater();
}


/*!
* \brief ICPParamsDialog::paramsforicp
* \return QGroupBox* típusú elemet ad vissza, ami tartalmazza a megfelelő layouttal ellátott paramétereket, és azok leírását
* az ICP algoritmushoz szükséges paraméterek szemléltetése és megjelenítse,
* elemek eltárolása
*/
QGroupBox *
ICPParamsDialog:: ParametersICP()
{
    _groupBoxICP = new QGroupBox(this);
    _groupBoxICP->setTitle("Iterative Closest Points based Registration's parameters");

    QGridLayout *ICPGridLayout = new QGridLayout(_groupBoxICP);

    QDoubleSpinBox *spinbox_maxit = new QDoubleSpinBox(_groupBoxICP);
    QDoubleSpinBox *spinbox_corrmaxdist = new QDoubleSpinBox(_groupBoxICP);
    QDoubleSpinBox *spinbox_RANSACThreshold = new QDoubleSpinBox(_groupBoxICP);
    QDoubleSpinBox *spinbox_RANSAC_it = new QDoubleSpinBox(_groupBoxICP);
    QDoubleSpinBox *spinbox_EuclideanFitnessEpsilon = new QDoubleSpinBox(_groupBoxICP);

    //speciális beállítások a spinboxoknak
    SetupSpinBox(*spinbox_maxit,100000,0,0);
    SetupSpinBox(*spinbox_RANSAC_it,100000,0,0);
    SetupSpinBox(*spinbox_corrmaxdist,5,0,5);
    SetupSpinBox(*spinbox_RANSACThreshold,2,0,6);
    SetupSpinBox(*spinbox_EuclideanFitnessEpsilon,2,0,9);

   //spinboxok eltárolása egy map tárolóba
    _spinboxesICP["SetCorrMaxDist"] = spinbox_corrmaxdist;
    _spinboxesICP["SetMaxIterations"] = spinbox_maxit;
    _spinboxesICP["SetRANSACIterations"] = spinbox_RANSAC_it;
    _spinboxesICP["SetRANSACThreshold"] = spinbox_RANSACThreshold;
    _spinboxesICP["SetEuclideanFitnessEpsilon"] = spinbox_EuclideanFitnessEpsilon;
    _labelsICP["SetMaxIterations"] = new QLabel("Maximum number of iterations the internal optimization should run for: ",_groupBoxICP);
    _labelsICP["SetCorrMaxDist"] = new QLabel("Maximum distance threshold between two correspondent points in Source <-> Target",_groupBoxICP);
    _labelsICP["SetRANSACThreshold"] = new QLabel("Inlier distance threshold for the internal RANSAC outlier rejection loop: ",_groupBoxICP);
    _labelsICP["SetRANSACIterations"] =new QLabel("Number of iterations RANSAC should run for : ",_groupBoxICP) ;
    _labelsICP["SetEuclideanFitnessEpsilon"] =new QLabel("The maximum allowed distance error before the \nalgorithm will be considered to have converged  : ",_groupBoxICP) ;

    //a layout feltöltése
    ICPGridLayout->addWidget(_labelsICP["SetMaxIterations"],0,0);
    ICPGridLayout->addWidget(spinbox_maxit,0,1);
    ICPGridLayout->addWidget(_labelsICP["SetCorrMaxDist"],1,0);
    ICPGridLayout->addWidget(spinbox_corrmaxdist,1,1);
    ICPGridLayout->addWidget( _labelsICP["SetRANSACThreshold"],2,0);
    ICPGridLayout->addWidget(spinbox_RANSACThreshold,2,1);
    ICPGridLayout->addWidget(_labelsICP["SetRANSACIterations"],3,0);
    ICPGridLayout->addWidget(spinbox_RANSAC_it,3,1);
    ICPGridLayout->addWidget(_labelsICP["SetEuclideanFitnessEpsilon"],4,0);
    ICPGridLayout->addWidget(spinbox_EuclideanFitnessEpsilon,4,1);

    QPushButton* _okButton = new QPushButton(_groupBoxICP);
    _okButton->setText("Start Registration");
    QPushButton*  _cancelButton = new QPushButton(_groupBoxICP);
    _cancelButton->setText("Back");
    QPushButton* _helpButton = new QPushButton(_groupBoxICP);
    _helpButton->setText("Help");


    ICPGridLayout->addWidget(_okButton,10,1);
    ICPGridLayout->addWidget(_cancelButton,11,1);
    ICPGridLayout->addWidget(_helpButton,12,1);

    //Az egyes funkciógombok és a hozzájuk tartozó műveletek kapcsolatának leírása
    connect(_okButton, SIGNAL(clicked()),this, SLOT(OnAccepted()));
    connect(_cancelButton, SIGNAL(clicked()),this, SLOT(reject()));
    connect(_helpButton, SIGNAL(clicked()),this, SLOT(ClickedHelpButton()));

    _groupBoxICP->setLayout(ICPGridLayout);
    return _groupBoxICP;
}

/*!
 * \brief ICPParamsDialog::ClickedHelpButton
 * Ha a felhasználó a HELP feliratú gombra megy akkor ez a metódus játszódik le, ami megjelenít egy üzenet dobozt a benne lévő
 * tanácsokkal, útbaigazításokkal, rövid információkkal.
 */
void ICPParamsDialog::ClickedHelpButton()
{
    QMessageBox::information(this,"Help","Registration based on Iterative Closest Point algorithm\n"
                             "\nThis pre-built iterative registration based on Iterative closest point (ICP) algorithm is employed to minimize the difference between two clouds of points. \n"
                             "\n- You can set the maximum distance threshold between two correspondent points (Source <--> Target), which is used by the algorithm for each iterations.\n"
                             "\n- You can set the RANSAC rejection algorithm's parameters to reject bad correspondences.\n"
                             "\n- You can set the threshold, which is used to the ICP algorithm's converging.\n"
                             "\nWarning! It takes a lot of time if you use pointclouds,which have a lot of points!\n"
                             "\nAttention! If certain parameters' value was zero,\nthese'd set to default value!");
}


/*!
* \brief ICPParamsDialog::OnAccepted
* Ha valamilyen paramétert a felhasználó üresen hagyott, mielött megkezdené a regisztrációt a program, figyelmezteti erre a felhasználót
* ekkor lehetőség van visszalépni, vagy folytatni a regisztrációt
*/
void
ICPParamsDialog::OnAccepted()
{
    bool ithasanynull = false;

    std::map<std::string, QDoubleSpinBox*>::iterator it;

    for ( it = _spinboxesICP.begin(); it != _spinboxesICP.end(); it++ )
    {
        if(it->second->value() == 0)
        {
            ithasanynull = true;
        }

    }
    if(ithasanynull)
    {
        QMessageBox::StandardButton reply = QMessageBox::question(this,
          "Warning!", "If certain parameter's value was null, these'd set to default value! \n Want to continue?",
          QMessageBox::Yes | QMessageBox::No);
        if(reply == QMessageBox::Yes)
        {
            accept();
        }
        else
            return;
    }

    else
    {
         accept();
    }

}
