#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <gc3dAlgorithm.h>
#include<opencv2/opencv.hpp>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();


private slots:
    void on_detectCamera_clicked();

    void on_openCamera_clicked();

    void on_exposureTime_editingFinished();

    void on_minHeight_editingFinished();

    void on_maxHeight_editingFinished();

    void on_denRad_editingFinished();

    void on_den1_editingFinished();

    void on_den2_editingFinished();

    void on_den3_editingFinished();

    void on_scan3D_clicked();

    void on_depthBtn_clicked(bool checked);

private:
    Ui::MainWindow *ui;
    size_t devNum = 0;
    gc3d::GC3DDevice dev;
    gc3d::GC3DMetaData meta;
    gc3d::DeviceInformation* infos = nullptr;
    gc3d::GC3DCameraParameters params;
    float minH = 0;
    float maxH = 0;
    int denRad = 3;
    float den1=0;
    float den2=0;
    float den3=0;

};

#endif // MAINWINDOW_H
