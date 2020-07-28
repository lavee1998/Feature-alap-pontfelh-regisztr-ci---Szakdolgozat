#ifndef RANSACPARAMSDIALOG_H
#define RANSACPARAMSDIALOG_H

#include <QDialog>
#include <QGroupBox>
#include <QSpinBox>
#include<QDoubleSpinBox>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QMessageBox>
#include <iostream>
#include <thread>
#include <QScrollArea>


namespace Ui {
class RANSACParamsDialog;
}

class RANSACParamsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit RANSACParamsDialog(QWidget *parent = nullptr);
    ~RANSACParamsDialog();
    void SetupSpinBox(QDoubleSpinBox &spinbox, double max, double min, int decimals);
    std::map<std::string, QDoubleSpinBox *> getSpinboxesRANSAC() const;


public Q_SLOTS:
    void OnAccepted();
    void ClickedHelpButton();

private:
    Ui::RANSACParamsDialog *ui;
    QGridLayout *_subGridLayout;
    QScrollArea *_scrollArea;
    QVBoxLayout *_mainLayout;

    QGroupBox *ParametersRANSAC();
    std::map<std::string,QDoubleSpinBox*> _spinBoxesRANSAC;
    std::map<std::string,QLabel*> _labelsRANSAC;
    QGroupBox *_groupBoxRANSAC;
};

#endif // RANSACPARAMSDIALOG_H
