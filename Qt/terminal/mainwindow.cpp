/****************************************************************************
**
** Copyright (C) 2012 Denis Shienkov <denis.shienkov@gmail.com>
** Copyright (C) 2012 Laszlo Papp <lpapp@kde.org>
** Contact: https://www.qt.io/licensing/
**
** This file is part of the QtSerialPort module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "console.h"
#include "settingsdialog.h"
#include "burnvhfdialog.h"

#include <QMessageBox>
#include <QLabel>
#include <QtSerialPort/QSerialPort>
#include <QThread>
#include <QTime>
#include <QDateTime>
#include <QTextStream>
#include <QDir>
#include <QInputDialog>

int saveData = 0;
QString currentFile;
QString path = "/w/loggerhead/";
QString folder = "";
int newData = 0;
int nBytesInFile = 0;

//! [0]
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
//! [0]
    ui->setupUi(this);
    console = new Console;
    console->setEnabled(false);
    setCentralWidget(console);
//! [1]
    serial = new QSerialPort(this);
//! [1]
    settings = new SettingsDialog;
    burnvhf = new burnvhfDialog;

    ui->actionConnect->setEnabled(true);
    ui->actionDisconnect->setEnabled(false);
    ui->actionQuit->setEnabled(true);
    ui->actionConfigure->setEnabled(true);

    status = new QLabel;
    ui->statusBar->addWidget(status);

    initActionsConnections();

    connect(serial, static_cast<void (QSerialPort::*)(QSerialPort::SerialPortError)>(&QSerialPort::error),
            this, &MainWindow::handleError);

//! [2]
    connect(serial, &QSerialPort::readyRead, this, &MainWindow::readData);
//! [2]
    connect(console, &Console::getData, this, &MainWindow::writeData);
//! [3]
}
//! [3]

MainWindow::~MainWindow()
{
    delete settings;
    delete ui;
}

//! [4]
void MainWindow::openSerialPort()
{
    SettingsDialog::Settings p = settings->settings();
    serial->setPortName(p.name);
    serial->setBaudRate(p.baudRate);
    serial->setDataBits(p.dataBits);
    serial->setParity(p.parity);
    serial->setStopBits(p.stopBits);
    serial->setFlowControl(p.flowControl);
    if (serial->open(QIODevice::ReadWrite)) {
        console->setEnabled(true);
        console->setLocalEchoEnabled(p.localEchoEnabled);
        ui->actionConnect->setEnabled(false);
        ui->actionDisconnect->setEnabled(true);
        ui->actionConfigure->setEnabled(false);
        showStatusMessage(tr("Connected to %1 : %2, %3, %4, %5, %6")
                          .arg(p.name).arg(p.stringBaudRate).arg(p.stringDataBits)
                          .arg(p.stringParity).arg(p.stringStopBits).arg(p.stringFlowControl));
    } else {
        QMessageBox::critical(this, tr("Error"), serial->errorString());

        showStatusMessage(tr("Open error"));
    }
}
//! [4]

//! [5]
void MainWindow::closeSerialPort()
{
    if (serial->isOpen())
        serial->close();
    console->setEnabled(false);
    ui->actionConnect->setEnabled(true);
    ui->actionDisconnect->setEnabled(false);
    ui->actionConfigure->setEnabled(true);
    showStatusMessage(tr("Disconnected"));
}
//! [5]

void MainWindow::about()
{
    QMessageBox::about(this, tr("About iTag Terminal"),
                       tr("The <b>iTag Terminal</b> is used to configure and download data from the iTag."));
}

//! [6]
void MainWindow::writeData(const QByteArray &data)
{
   // serial->write(data);
}
//! [6]

//! [7]
void MainWindow::readData()
{
    QByteArray data = serial->readAll();
    console->putData(data);
    newData = 1;
    if(data.length() > 0 & saveData) writeDataFile(data);
}
//! [7]

//! [8]
void MainWindow::handleError(QSerialPort::SerialPortError error)
{
    if (error == QSerialPort::ResourceError) {
        QMessageBox::critical(this, tr("Critical Error"), serial->errorString());
        closeSerialPort();
    }
}
//! [8]

void MainWindow::initActionsConnections()
{
    connect(ui->actionConnect, &QAction::triggered, this, &MainWindow::openSerialPort);
    connect(ui->actionDisconnect, &QAction::triggered, this, &MainWindow::closeSerialPort);
    connect(ui->actionQuit, &QAction::triggered, this, &MainWindow::close);
    connect(ui->actionConfigure, &QAction::triggered, settings, &MainWindow::show);
    connect(ui->actionClear, &QAction::triggered, console, &Console::clear);
    connect(ui->actionAbout, &QAction::triggered, this, &MainWindow::about);
    connect(ui->actionAboutQt, &QAction::triggered, qApp, &QApplication::aboutQt);
}

void MainWindow::showStatusMessage(const QString &message)
{
    status->setText(message);
}


// download single file
void MainWindow::downloadData()
{
    // send 2
    QByteArray menuChoice;
    menuChoice.append('2');
    serial->write(menuChoice);
    serial->flush();
  //  QThread::msleep(10);

 //   QByteArray data = serial->readAll();  //clear buffer

    // send filename to download
    serial->write(currentFile.toLocal8Bit());
    serial->flush();
    saveData = 1;
}

// get file list and then call download file for each one

void MainWindow::on_actionactionDownload_triggered()
{
    QApplication::setOverrideCursor(Qt::WaitCursor);
    // get file list and bytes and tagID

    currentFile = "FILELIST.CSV";
    QFile file(path + currentFile); // remove list of files if exists
    if (file.exists())
        file.remove();
    else
        file.close();
    currentFile = "FILELIST.CSV";
    saveData = 1;
    QByteArray menuChoice;
    menuChoice.append('1');
    serial->write(menuChoice);
    serial->flush();
    waitUntilDone();

    QFile fileList(path + currentFile);
    fileList.open(QIODevice::ReadOnly | QIODevice::Text);
    QTextStream in(&fileList);

    // get serial number
    QString line = in.readLine();
    QStringList tagID = line.split(" ");
    folder = tagID.at(1) +"/";
    if(!QDir(path + folder).exists()) QDir().mkdir(path + folder);

    while(1){
        line = in.readLine();
        if (line == "") break;
        QStringList nextFile = line.split(",");
        currentFile = nextFile.at(0);
        console->putData(currentFile.toLocal8Bit());
        nBytesInFile = nextFile.at(1).toInt();
        console->putData(nextFile.at(1).toLocal8Bit());
        if(nBytesInFile>0){
            downloadData();
            waitUntilDone();
        }
    }
    fileList.close();
    saveData = 0;

    QApplication::restoreOverrideCursor();
}

void MainWindow::waitUntilDone(){
    QTime qtime;
    qtime.start();
    while(1){
        if (newData == 1) qtime.restart();
        if(qtime.elapsed() > 2000) break; // x ms with no new data get out
        qApp->processEvents();
        newData = 0;
    }
}

void MainWindow::writeDataFile(const QByteArray &data){
    //if there is an existing file, open it, append data, close
    QFile file(path + folder + currentFile);
    file.open(QIODevice::Append);
    file.write(data);
    //QDataStream out(&file);
    //out << data;
    file.close();
}

void MainWindow::on_actionList_triggered()
{
    saveData = 0;
    QByteArray menuChoice;
    menuChoice.append('1');
    serial->write(menuChoice);
    serial->flush();
    QThread::msleep(10);
}

void MainWindow::on_actionTest_triggered()
{
    saveData = 0;
    QByteArray menuChoice;
    menuChoice.append('4');
    serial->write(menuChoice);
    serial->flush();
}

void MainWindow::on_actionDeploy_triggered()
{
    saveData = 0;
    QByteArray menuChoice;
    menuChoice.append('6');
    serial->write(menuChoice);
    serial->flush();
}

void MainWindow::on_actionSet_Time_triggered()
{
    saveData = 0;
    QDateTime qdatetime;
    QByteArray menuChoice;
    menuChoice.append('3');
    serial->write(menuChoice);
    serial->flush();
    qdatetime = QDateTime::currentDateTimeUtc();
    QString newTime = qdatetime.toString("yyMMddhhmmss");
    QByteArray newTimeByte = newTime.toLocal8Bit();
    serial->write(newTimeByte);
    serial->flush();
}

void MainWindow::on_actionDelete_All_triggered()
{
    saveData = 0;
    QByteArray menuChoice;
    menuChoice.append('5');
    serial->write(menuChoice);
    serial->flush();
}

// set burn minutes
void MainWindow::on_actionBurn_triggered()
{
    //burnvhf->show();
    bool ok;
    QString text = QInputDialog::getText(this, tr("Burn Wire Setup"),
                                             tr("Minutes until trigger burn:"), QLineEdit::Normal,
                                             "", &ok);
    if (ok && !text.isEmpty()){
        saveData = 0;
        QByteArray menuChoice;
        menuChoice.append('7');
        menuChoice.append(text.toLocal8Bit());
        menuChoice.append('X'); //X to mark end of data
        serial->write(menuChoice);
        serial->flush();
    }
}

void MainWindow::on_actionBurnDT_triggered()
{
    bool ok;
    QString text = QInputDialog::getText(this, tr("Burn Wire Setup"),
                                             tr("Burn Datetime UTC (YYMMDDHHMMSS)"), QLineEdit::Normal,
                                             "", &ok);
    if (ok && !text.isEmpty()){
        saveData = 0;
        QByteArray menuChoice;
        menuChoice.append('8');
        menuChoice.append(text.toLocal8Bit());
        serial->write(menuChoice);
        serial->flush();
    }
}
