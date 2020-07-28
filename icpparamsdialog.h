#ifndef ICPPARAMSDIALOG_H
#define ICPPARAMSDIALOG_H

#include <QDialog>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QMessageBox>
#include <iostream>
#include <thread>
#include <QGroupBox>
#include <QScrollArea>


namespace Ui {
class ICPParamsDialog;
}

class ICPParamsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ICPParamsDialog(QWidget *parent = nullptr);
    ~ICPParamsDialog();
    std::map<std::string, QDoubleSpinBox *> getSpinboxesICP() const;

public Q_SLOTS:
    void OnAccepted();
    void ClickedHelpButton();

private:
    Ui::ICPParamsDialog *ui;//felhasználói interfész
    //az egyes modulok eltárolására szolgáló tárolók
    std::map<std::string,QDoubleSpinBox*> _spinboxesICP;
    std::map<std::string,QLabel*> _labelsICP;
    QPushButton* _okButton;
    QPushButton* _cancelButton;
    QPushButton* _helpButton;
    QGridLayout *_subGridLayout;
    QVBoxLayout *_mainLayout;
    QScrollArea *_scrollArea;
    QGroupBox * _groupBoxICP;
    QGroupBox *ParametersICP();
    void SetupSpinBox(QDoubleSpinBox &spinbox, double max, double min, int decimals); //egyes QDoubleSpinBox típusú modulok beállítására szolgál

};

#endif // PARAMSDIALOG_ICP_H
