/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.12.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QPushButton *detectCamera;
    QComboBox *comboBox;
    QPushButton *openCamera;
    QLabel *label;
    QLabel *showImage;
    QLineEdit *exposureTime;
    QLineEdit *minHeight;
    QLabel *label_2;
    QLineEdit *maxHeight;
    QLabel *label_3;
    QLineEdit *denRad;
    QLabel *label_4;
    QLineEdit *den1;
    QLineEdit *den2;
    QLineEdit *den3;
    QPushButton *scan3D;
    QCheckBox *depthBtn;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1070, 614);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        detectCamera = new QPushButton(centralWidget);
        detectCamera->setObjectName(QString::fromUtf8("detectCamera"));
        detectCamera->setGeometry(QRect(690, 20, 91, 41));
        QFont font;
        font.setPointSize(12);
        detectCamera->setFont(font);
        comboBox = new QComboBox(centralWidget);
        comboBox->setObjectName(QString::fromUtf8("comboBox"));
        comboBox->setGeometry(QRect(800, 20, 161, 41));
        QFont font1;
        font1.setPointSize(15);
        comboBox->setFont(font1);
        openCamera = new QPushButton(centralWidget);
        openCamera->setObjectName(QString::fromUtf8("openCamera"));
        openCamera->setGeometry(QRect(970, 20, 81, 41));
        openCamera->setFont(font);
        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(700, 80, 71, 41));
        label->setFont(font);
        label->setAlignment(Qt::AlignCenter);
        showImage = new QLabel(centralWidget);
        showImage->setObjectName(QString::fromUtf8("showImage"));
        showImage->setGeometry(QRect(20, 20, 621, 531));
        showImage->setPixmap(QPixmap(QString::fromUtf8("../build-GC3DExample-Desktop_Qt_5_12_0_MSVC2015_64bit-Release/1.jpg")));
        exposureTime = new QLineEdit(centralWidget);
        exposureTime->setObjectName(QString::fromUtf8("exposureTime"));
        exposureTime->setGeometry(QRect(800, 80, 161, 41));
        exposureTime->setFont(font1);
        minHeight = new QLineEdit(centralWidget);
        minHeight->setObjectName(QString::fromUtf8("minHeight"));
        minHeight->setGeometry(QRect(800, 140, 101, 41));
        minHeight->setFont(font1);
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(700, 140, 71, 41));
        label_2->setFont(font);
        label_2->setAlignment(Qt::AlignCenter);
        maxHeight = new QLineEdit(centralWidget);
        maxHeight->setObjectName(QString::fromUtf8("maxHeight"));
        maxHeight->setGeometry(QRect(940, 140, 101, 41));
        maxHeight->setFont(font1);
        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(900, 140, 31, 41));
        label_3->setFont(font);
        label_3->setAlignment(Qt::AlignCenter);
        denRad = new QLineEdit(centralWidget);
        denRad->setObjectName(QString::fromUtf8("denRad"));
        denRad->setGeometry(QRect(800, 200, 51, 41));
        denRad->setFont(font1);
        label_4 = new QLabel(centralWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(700, 200, 71, 41));
        label_4->setFont(font);
        label_4->setAlignment(Qt::AlignCenter);
        den1 = new QLineEdit(centralWidget);
        den1->setObjectName(QString::fromUtf8("den1"));
        den1->setGeometry(QRect(870, 200, 51, 41));
        den1->setFont(font1);
        den2 = new QLineEdit(centralWidget);
        den2->setObjectName(QString::fromUtf8("den2"));
        den2->setGeometry(QRect(940, 200, 51, 41));
        den2->setFont(font1);
        den3 = new QLineEdit(centralWidget);
        den3->setObjectName(QString::fromUtf8("den3"));
        den3->setGeometry(QRect(1010, 200, 51, 41));
        den3->setFont(font1);
        scan3D = new QPushButton(centralWidget);
        scan3D->setObjectName(QString::fromUtf8("scan3D"));
        scan3D->setGeometry(QRect(700, 280, 121, 71));
        QFont font2;
        font2.setPointSize(21);
        scan3D->setFont(font2);
        depthBtn = new QCheckBox(centralWidget);
        depthBtn->setObjectName(QString::fromUtf8("depthBtn"));
        depthBtn->setGeometry(QRect(850, 280, 101, 71));
        QFont font3;
        font3.setPointSize(14);
        depthBtn->setFont(font3);
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1070, 23));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "GC3DExample", nullptr));
        detectCamera->setText(QApplication::translate("MainWindow", "\346\243\200\346\265\213\347\233\270\346\234\272", nullptr));
        openCamera->setText(QApplication::translate("MainWindow", "\346\211\223\345\274\200", nullptr));
        label->setText(QApplication::translate("MainWindow", "\346\233\235\345\205\211\346\227\266\351\227\264", nullptr));
        showImage->setText(QString());
        exposureTime->setText(QApplication::translate("MainWindow", "2000", nullptr));
        minHeight->setText(QApplication::translate("MainWindow", "-1000", nullptr));
        label_2->setText(QApplication::translate("MainWindow", "\351\253\230\345\272\246\350\214\203\345\233\264", nullptr));
        maxHeight->setText(QApplication::translate("MainWindow", "1000", nullptr));
        label_3->setText(QApplication::translate("MainWindow", "-", nullptr));
        denRad->setText(QApplication::translate("MainWindow", "3", nullptr));
        label_4->setText(QApplication::translate("MainWindow", "\351\231\215\345\231\252\346\214\207\346\225\260", nullptr));
        den1->setText(QApplication::translate("MainWindow", "50", nullptr));
        den2->setText(QApplication::translate("MainWindow", "1", nullptr));
        den3->setText(QApplication::translate("MainWindow", "1", nullptr));
        scan3D->setText(QApplication::translate("MainWindow", "3D\346\211\253\346\217\217", nullptr));
        depthBtn->setText(QApplication::translate("MainWindow", "\346\267\261\345\272\246\345\233\276", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
