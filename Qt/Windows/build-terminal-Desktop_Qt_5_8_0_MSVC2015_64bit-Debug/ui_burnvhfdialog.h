/********************************************************************************
** Form generated from reading UI file 'burnvhfdialog.ui'
**
** Created by: Qt User Interface Compiler version 5.8.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_BURNVHFDIALOG_H
#define UI_BURNVHFDIALOG_H

#include <QtCore/QDate>
#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDateTimeEdit>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QRadioButton>

QT_BEGIN_NAMESPACE

class Ui_burnvhfDialog
{
public:
    QDialogButtonBox *buttonBox;
    QGroupBox *groupBox;
    QRadioButton *radioButton;
    QRadioButton *radioButton_2;
    QRadioButton *radioButton_3;
    QLineEdit *lineEdit;
    QDateTimeEdit *dateTimeEdit;
    QGroupBox *groupBox_2;
    QCheckBox *checkBox;
    QLineEdit *lineEdit_2;
    QLabel *descriptionLabel;

    void setupUi(QDialog *burnvhfDialog)
    {
        if (burnvhfDialog->objectName().isEmpty())
            burnvhfDialog->setObjectName(QStringLiteral("burnvhfDialog"));
        burnvhfDialog->resize(400, 300);
        buttonBox = new QDialogButtonBox(burnvhfDialog);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        buttonBox->setGeometry(QRect(30, 240, 341, 32));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);
        groupBox = new QGroupBox(burnvhfDialog);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(0, 0, 391, 141));
        radioButton = new QRadioButton(groupBox);
        radioButton->setObjectName(QStringLiteral("radioButton"));
        radioButton->setGeometry(QRect(20, 40, 161, 20));
        radioButton->setChecked(false);
        radioButton_2 = new QRadioButton(groupBox);
        radioButton_2->setObjectName(QStringLiteral("radioButton_2"));
        radioButton_2->setGeometry(QRect(20, 70, 141, 20));
        radioButton_3 = new QRadioButton(groupBox);
        radioButton_3->setObjectName(QStringLiteral("radioButton_3"));
        radioButton_3->setGeometry(QRect(20, 100, 141, 20));
        radioButton_3->setChecked(true);
        lineEdit = new QLineEdit(groupBox);
        lineEdit->setObjectName(QStringLiteral("lineEdit"));
        lineEdit->setGeometry(QRect(190, 40, 151, 21));
        dateTimeEdit = new QDateTimeEdit(groupBox);
        dateTimeEdit->setObjectName(QStringLiteral("dateTimeEdit"));
        dateTimeEdit->setGeometry(QRect(160, 70, 194, 24));
        dateTimeEdit->setDateTime(QDateTime(QDate(2017, 2, 1), QTime(5, 0, 0)));
        dateTimeEdit->setDate(QDate(2017, 2, 1));
        dateTimeEdit->setCalendarPopup(true);
        dateTimeEdit->setCurrentSectionIndex(0);
        dateTimeEdit->setTimeSpec(Qt::UTC);
        groupBox_2 = new QGroupBox(burnvhfDialog);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setGeometry(QRect(10, 140, 381, 80));
        checkBox = new QCheckBox(groupBox_2);
        checkBox->setObjectName(QStringLiteral("checkBox"));
        checkBox->setGeometry(QRect(10, 50, 151, 20));
        checkBox->setChecked(true);
        lineEdit_2 = new QLineEdit(groupBox_2);
        lineEdit_2->setObjectName(QStringLiteral("lineEdit_2"));
        lineEdit_2->setGeometry(QRect(100, 30, 113, 21));
        descriptionLabel = new QLabel(groupBox_2);
        descriptionLabel->setObjectName(QStringLiteral("descriptionLabel"));
        descriptionLabel->setGeometry(QRect(10, 30, 98, 16));

        retranslateUi(burnvhfDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), burnvhfDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), burnvhfDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(burnvhfDialog);
    } // setupUi

    void retranslateUi(QDialog *burnvhfDialog)
    {
        burnvhfDialog->setWindowTitle(QApplication::translate("burnvhfDialog", "Dialog", Q_NULLPTR));
        groupBox->setTitle(QApplication::translate("burnvhfDialog", "Burn Wire", Q_NULLPTR));
        radioButton->setText(QApplication::translate("burnvhfDialog", "Burn after x minutes", Q_NULLPTR));
        radioButton_2->setText(QApplication::translate("burnvhfDialog", "Burn at time (UTC)", Q_NULLPTR));
        radioButton_3->setText(QApplication::translate("burnvhfDialog", "Disable burn wire", Q_NULLPTR));
        lineEdit->setText(QApplication::translate("burnvhfDialog", "0", Q_NULLPTR));
        groupBox_2->setTitle(QApplication::translate("burnvhfDialog", "VHF", Q_NULLPTR));
        checkBox->setText(QApplication::translate("burnvhfDialog", "Enable", Q_NULLPTR));
        lineEdit_2->setText(QApplication::translate("burnvhfDialog", "1", Q_NULLPTR));
        descriptionLabel->setText(QApplication::translate("burnvhfDialog", "Turn on depth", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class burnvhfDialog: public Ui_burnvhfDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_BURNVHFDIALOG_H
