#include "burnvhfdialog.h"
#include "ui_burnvhfdialog.h"

burnvhfDialog::burnvhfDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::burnvhfDialog)
{
    ui->setupUi(this);
}

burnvhfDialog::~burnvhfDialog()
{
    delete ui;
}
