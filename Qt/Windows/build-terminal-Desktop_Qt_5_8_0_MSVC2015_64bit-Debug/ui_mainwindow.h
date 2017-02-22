/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.8.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionAbout;
    QAction *actionAboutQt;
    QAction *actionConnect;
    QAction *actionDisconnect;
    QAction *actionConfigure;
    QAction *actionClear;
    QAction *actionQuit;
    QAction *actionactionDownload;
    QAction *actionList;
    QAction *actionTest;
    QAction *actionSet_Time;
    QAction *actionDelete_All;
    QAction *actionDeploy;
    QAction *actionBurn;
    QAction *actionBurnDT;
    QWidget *centralWidget;
    QVBoxLayout *verticalLayout;
    QMenuBar *menuBar;
    QMenu *menuCalls;
    QMenu *menuTools;
    QMenu *menuHelp;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(638, 300);
        actionAbout = new QAction(MainWindow);
        actionAbout->setObjectName(QStringLiteral("actionAbout"));
        actionAboutQt = new QAction(MainWindow);
        actionAboutQt->setObjectName(QStringLiteral("actionAboutQt"));
        actionConnect = new QAction(MainWindow);
        actionConnect->setObjectName(QStringLiteral("actionConnect"));
        QIcon icon;
        icon.addFile(QStringLiteral(":/images/connect.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionConnect->setIcon(icon);
        actionDisconnect = new QAction(MainWindow);
        actionDisconnect->setObjectName(QStringLiteral("actionDisconnect"));
        QIcon icon1;
        icon1.addFile(QStringLiteral(":/images/disconnect.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionDisconnect->setIcon(icon1);
        actionConfigure = new QAction(MainWindow);
        actionConfigure->setObjectName(QStringLiteral("actionConfigure"));
        QIcon icon2;
        icon2.addFile(QStringLiteral(":/images/settings.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionConfigure->setIcon(icon2);
        actionClear = new QAction(MainWindow);
        actionClear->setObjectName(QStringLiteral("actionClear"));
        QIcon icon3;
        icon3.addFile(QStringLiteral(":/images/clear.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionClear->setIcon(icon3);
        actionQuit = new QAction(MainWindow);
        actionQuit->setObjectName(QStringLiteral("actionQuit"));
        QIcon icon4;
        icon4.addFile(QStringLiteral(":/images/application-exit.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionQuit->setIcon(icon4);
        actionactionDownload = new QAction(MainWindow);
        actionactionDownload->setObjectName(QStringLiteral("actionactionDownload"));
        actionList = new QAction(MainWindow);
        actionList->setObjectName(QStringLiteral("actionList"));
        actionTest = new QAction(MainWindow);
        actionTest->setObjectName(QStringLiteral("actionTest"));
        actionSet_Time = new QAction(MainWindow);
        actionSet_Time->setObjectName(QStringLiteral("actionSet_Time"));
        actionDelete_All = new QAction(MainWindow);
        actionDelete_All->setObjectName(QStringLiteral("actionDelete_All"));
        actionDeploy = new QAction(MainWindow);
        actionDeploy->setObjectName(QStringLiteral("actionDeploy"));
        actionBurn = new QAction(MainWindow);
        actionBurn->setObjectName(QStringLiteral("actionBurn"));
        actionBurnDT = new QAction(MainWindow);
        actionBurnDT->setObjectName(QStringLiteral("actionBurnDT"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        verticalLayout = new QVBoxLayout(centralWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 638, 22));
        menuCalls = new QMenu(menuBar);
        menuCalls->setObjectName(QStringLiteral("menuCalls"));
        menuTools = new QMenu(menuBar);
        menuTools->setObjectName(QStringLiteral("menuTools"));
        menuHelp = new QMenu(menuBar);
        menuHelp->setObjectName(QStringLiteral("menuHelp"));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        menuBar->addAction(menuCalls->menuAction());
        menuBar->addAction(menuTools->menuAction());
        menuBar->addAction(menuHelp->menuAction());
        menuCalls->addAction(actionConnect);
        menuCalls->addAction(actionDisconnect);
        menuCalls->addSeparator();
        menuCalls->addAction(actionQuit);
        menuTools->addAction(actionConfigure);
        menuTools->addAction(actionClear);
        menuHelp->addAction(actionAbout);
        menuHelp->addAction(actionAboutQt);
        mainToolBar->addAction(actionConnect);
        mainToolBar->addAction(actionDisconnect);
        mainToolBar->addAction(actionConfigure);
        mainToolBar->addAction(actionClear);
        mainToolBar->addAction(actionactionDownload);
        mainToolBar->addAction(actionList);
        mainToolBar->addAction(actionTest);
        mainToolBar->addAction(actionSet_Time);
        mainToolBar->addAction(actionDelete_All);
        mainToolBar->addAction(actionDeploy);
        mainToolBar->addAction(actionBurn);
        mainToolBar->addAction(actionBurnDT);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "iTag Terminal", Q_NULLPTR));
        actionAbout->setText(QApplication::translate("MainWindow", "&About", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        actionAbout->setToolTip(QApplication::translate("MainWindow", "About program", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        actionAbout->setShortcut(QApplication::translate("MainWindow", "Alt+A", Q_NULLPTR));
        actionAboutQt->setText(QApplication::translate("MainWindow", "About Qt", Q_NULLPTR));
        actionConnect->setText(QApplication::translate("MainWindow", "C&onnect", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        actionConnect->setToolTip(QApplication::translate("MainWindow", "Connect to serial port", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        actionConnect->setShortcut(QApplication::translate("MainWindow", "Ctrl+O", Q_NULLPTR));
        actionDisconnect->setText(QApplication::translate("MainWindow", "&Disconnect", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        actionDisconnect->setToolTip(QApplication::translate("MainWindow", "Disconnect from serial port", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        actionDisconnect->setShortcut(QApplication::translate("MainWindow", "Ctrl+D", Q_NULLPTR));
        actionConfigure->setText(QApplication::translate("MainWindow", "&Configure", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        actionConfigure->setToolTip(QApplication::translate("MainWindow", "Configure serial port", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        actionConfigure->setShortcut(QApplication::translate("MainWindow", "Alt+C", Q_NULLPTR));
        actionClear->setText(QApplication::translate("MainWindow", "C&lear", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        actionClear->setToolTip(QApplication::translate("MainWindow", "Clear data", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        actionClear->setShortcut(QApplication::translate("MainWindow", "Alt+L", Q_NULLPTR));
        actionQuit->setText(QApplication::translate("MainWindow", "&Quit", Q_NULLPTR));
        actionQuit->setShortcut(QApplication::translate("MainWindow", "Ctrl+Q", Q_NULLPTR));
        actionactionDownload->setText(QApplication::translate("MainWindow", "Download", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        actionactionDownload->setToolTip(QApplication::translate("MainWindow", "Download", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        actionList->setText(QApplication::translate("MainWindow", "List", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        actionList->setToolTip(QApplication::translate("MainWindow", "List Files", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        actionTest->setText(QApplication::translate("MainWindow", "Test", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        actionTest->setToolTip(QApplication::translate("MainWindow", "Sensor Test", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        actionSet_Time->setText(QApplication::translate("MainWindow", "Set Time", Q_NULLPTR));
        actionDelete_All->setText(QApplication::translate("MainWindow", "Delete All", Q_NULLPTR));
        actionDeploy->setText(QApplication::translate("MainWindow", "Deploy", Q_NULLPTR));
        actionBurn->setText(QApplication::translate("MainWindow", "Burn", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        actionBurn->setToolTip(QApplication::translate("MainWindow", "Setup Burn and VHF", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        actionBurnDT->setText(QApplication::translate("MainWindow", "BurnDT", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        actionBurnDT->setToolTip(QApplication::translate("MainWindow", "Burn wire datetime", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        menuCalls->setTitle(QApplication::translate("MainWindow", "Calls", Q_NULLPTR));
        menuTools->setTitle(QApplication::translate("MainWindow", "Tools", Q_NULLPTR));
        menuHelp->setTitle(QApplication::translate("MainWindow", "Help", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
