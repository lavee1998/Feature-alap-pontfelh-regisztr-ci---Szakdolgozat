#ifndef PREPROCESSINGPARAMSDIALOG_H
#define PREPROCESSINGPARAMSDIALOG_H

#include <QDialog>
#include <QGridLayout>
#include <QGroupBox>
#include <QRadioButton>
#include <QDoubleSpinBox>
#include <QLabel>
#include <iostream>
#include <thread>
#include <QFormLayout>
#include <QCheckBox>
#include <QPushButton>
#include <QMessageBox>
#include <QScrollArea>

namespace Ui {
class PreprocessingParamsDialog;
}

class PreprocessingParamsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit PreprocessingParamsDialog(QWidget *parent = nullptr);
    ~PreprocessingParamsDialog();

    void SetupSpinBox(QDoubleSpinBox &spinbox, double max, double min, int decimals);

    std::map<std::string, QGroupBox *> getGroupboxesPreprocessing() const;

    std::map<std::string, QDoubleSpinBox *> getSpinboxesPreprocessing() const;

    std::map<std::string, QCheckBox *> getCheckboxesPreprocessing() const;

    std::map<std::string, QRadioButton *> getRadiobuttonsPreprocessing() const;

public Q_SLOTS:
    void ClickedRadioButtonRemoveOutliersRadiusOutlierRemoval();
    void ClickedRadioButtonRemoveOutliersStatisticalOutlierRemoval();
    void ClickedRadioButtonDownsamplingVoxel();
    void ClickedRadioButtonDownsamplingRandom();
    void ClickedCheckBoxDownsamplingSource(bool ischecked);
    void ClickedCheckBoxDownsamplingTarget(bool ischecked);
    void ClickedCheckBoxOutliersRemovalSource(bool ischecked);
    void ClickedCheckBoxOutliersRemovalTarget(bool ischecked);
    void ClickedCheckBoxSmoothingSource(bool ischecked);
    void ClickedCheckBoxSmoothingTarget(bool ischecked);
    void ClickedHelpButton();

private:
    Ui::PreprocessingParamsDialog *ui;
    QGridLayout *_subGridlayout;
    QScrollArea *_scrollArea;
    QVBoxLayout *_mainLayout;
    QGroupBox * ParametersDownsampling();
    QGroupBox * ParametersOutliersRemoval();
    QGroupBox * ParametersSmoothing();
    QGroupBox * Actionbuttons();
    std::map<std::string,QGroupBox*> _groupboxesPreprocessing;
    std::map<std::string,QDoubleSpinBox*> _spinboxesPreprocessing;
    std::map<std::string,QCheckBox*> _checkboxesPreprocessing;
    std::map<std::string,QLabel*> _labelsPreprocessing;
    std::map<std::string,QRadioButton*> _radiobuttonsPreprocessing;
    std::map<std::string,QFormLayout*> _layoutsforpreprocessing;
};

#endif // PREPROCESSINGPARAMSDIALOG_H
