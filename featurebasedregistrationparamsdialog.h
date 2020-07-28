#ifndef FEATUREBASEDREGISTRATIONPARAMSDIALOG_H
#define FEATUREBASEDREGISTRATIONPARAMSDIALOG_H

#include <QDialog>
#include <QLayout>
#include <QFormLayout>
#include <QLabel>
#include <QRadioButton>
#include <QSpinBox>
#include <iostream>
#include <thread>

#include<QGroupBox>
#include<QDoubleSpinBox>
#include<QMouseEvent>
#include <QCheckBox>
#include <QPushButton>
#include <QMessageBox>
#include <QScrollArea>

namespace Ui {
class FeatureBasedRegistrationParamsDialog;
}

class FeatureBasedRegistrationParamsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit FeatureBasedRegistrationParamsDialog(QWidget *parent = nullptr);
    ~FeatureBasedRegistrationParamsDialog();

    //getter függvények
    Ui::FeatureBasedRegistrationParamsDialog *getUi() const;
    std::map<std::string, QDoubleSpinBox *> getSpinboxesFeatureBasedRegistration() const;
    std::map<std::string, QCheckBox *> getCheckboxesFeatureBasedRegistration() const;
    std::map<std::string, QLabel *> getLabelsFeatureBasedRegistration() const;
    std::map<std::string, QRadioButton *> getRadiobuttonsFeatureBasedRegistration() const;

public Q_SLOTS:
    //az egyes módszerek kattintásakor kiváltódó események
    void ClickedRadioButtonKeypointsHarris3D();
    void ClickedRadioButtonKeypointsSIFT();
    void ClickedRadioButtonKeypointsISS();
    void ClickedRadioButtonRejectionBasedOnRANSAC(bool ischecked);
    void ClickedRadioButtonRejectionBasedOnDistance(bool ischecked);
    void ClickedRadioButtonRejectionBasedOnSurfaceNormals(bool iscchecked);
    void ClickedHelpButton();

private:
    Ui::FeatureBasedRegistrationParamsDialog *ui; //felhasználói felület
    QGridLayout *_subGridLayout; //a felületi rács
    QVBoxLayout *_mainLayout;
    //legutoljára kiválasztott kulcspont módszerek
    bool IsLastSelectedISS;
    bool IsLastSelectedHarris;
    bool IsLastSelectedSIFT;
    //pereméter csoportokat visszaadó függvények
    QGroupBox* ParametersKeypoints();
    QGroupBox* ParametersFeatureDescriptors();
    QGroupBox* ParametersCorrespondences();
    QGroupBox* ParametersRejection();
    QGroupBox* ParametersTransformation();
    QGroupBox* Actionbuttons();
    void SetupSpinBox(QDoubleSpinBox &spinbox, double max, double min, int decimals); //egyes QDoubleSpinBox típusú gombok beállítására szolgáló függvény

    //paramétercsoportok
    QGroupBox *_groupBoxKeypoints;
    QGroupBox *_groupBoxFeatureDescriptors;
    QGroupBox *_groupBoxCorrespondences;
    QGroupBox *_groupBoxRejectors;
    QGroupBox *_groupBoxTransformation;
    //a felületi modulok eltárolására használatos tárolók
    std::map<std::string,QDoubleSpinBox*> _spinboxesFeatureBasedRegistration;
    std::map<std::string,QCheckBox*> _checkboxesFeatureBasedRegistration;
    std::map<std::string,QLabel*> _labelsFeatureBasedRegistration;
    std::map<std::string,QRadioButton*> _radiobuttonsFeatureBasedRegistration;
    std::map<std::string,QGroupBox*> _groupBoxesFeatureBasedRegistration;

    QScrollArea *_scrollArea;
};

#endif // FEATUREBASEDREGISTRATIONPARAMSDIALOG_H
