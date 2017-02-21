#ifndef BURNVHFDIALOG_H
#define BURNVHFDIALOG_H

#include <QDialog>

namespace Ui {
class burnvhfDialog;
}

class burnvhfDialog : public QDialog
{
    Q_OBJECT

public:
    explicit burnvhfDialog(QWidget *parent = 0);
    ~burnvhfDialog();

private:
    Ui::burnvhfDialog *ui;
};

#endif // BURNVHFDIALOG_H
