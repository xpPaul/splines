#-------------------------------------------------
#
# Project created by QtCreator 2014-11-19T14:27:17
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = splines
TEMPLATE = app

INCLUDEPATH += ../../

SOURCES += main.cpp\
        mainwindow.cpp \
    ../../include/gui/glwidget/glwidget.cpp \

HEADERS  += mainwindow.h \
    ../../include/math/point2.h \
    ../../include/gui/glwidget/glwidget.h \
    ../../include/splines/arclength.h \
    ../../include/splines/builder.h \
    ../../include/splines/localization.h \
    ../../include/splines/segment.h \
    ../../include/splines/spline.h \
    ../../include/splines/splines_aux.h

FORMS    += mainwindow.ui
