QT += widgets serialport

TARGET = iTag
TEMPLATE = app

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    settingsdialog.cpp \
    console.cpp \
    burnvhfdialog.cpp

HEADERS += \
    mainwindow.h \
    settingsdialog.h \
    console.h \
    burnvhfdialog.h

FORMS += \
    mainwindow.ui \
    settingsdialog.ui \
    burnvhfdialog.ui

RESOURCES += \
    terminal.qrc

INSTALLS += target

CONFIG += static
