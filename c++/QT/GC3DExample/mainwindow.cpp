#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "QMessageBox"




MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_detectCamera_clicked()
{

    uint32_t res = gc3d::initialDevice(infos,devNum);
    if(GC3D_SUCCESS == res){
        for(size_t i = 0;i<devNum;++i){
            ui->comboBox->addItem(QString::fromStdString(infos->serialNum));
        }
    }
}

void MainWindow::on_openCamera_clicked()
{
    QString camSerial = ui->comboBox->currentText();
    if(!camSerial.isEmpty()){
        QMessageBox box;
        if(GC3D_SUCCESS == dev.openDeviceBySerial(camSerial.toStdString())){
            box.information(this,"打开相机","成功",QMessageBox::Cancel|QMessageBox::Ok,QMessageBox::Ok);
        }else{
            box.information(this,"打开相机","失败",QMessageBox::Cancel|QMessageBox::Ok,QMessageBox::Ok);
        }
        box.show();
    }

}

void MainWindow::on_exposureTime_editingFinished()
{
    params.exposureTime = ui->exposureTime->text().toInt();
    dev.setCameraParameters(params);
}

void MainWindow::on_minHeight_editingFinished()
{
    dev.getHeightRange(minH,maxH);
    minH = ui->minHeight->text().toFloat();
    dev.setHeightRange(minH,maxH);

}

void MainWindow::on_maxHeight_editingFinished()
{
    dev.getHeightRange(minH,maxH);
    maxH = ui->maxHeight->text().toFloat();
    dev.setHeightRange(minH,maxH);
}

void MainWindow::on_denRad_editingFinished()
{
    dev.getDenoiseParameters(denRad,den1,den2,den3);
    denRad = ui->denRad->text().toInt();
    dev.setDenoiseParameters(denRad,den1,den2,den3);
}

void MainWindow::on_den1_editingFinished()
{
    dev.getDenoiseParameters(denRad,den1,den2,den3);
    den1 = ui->den1->text().toFloat();
    dev.setDenoiseParameters(denRad,den1,den2,den3);
}

void MainWindow::on_den2_editingFinished()
{
    dev.getDenoiseParameters(denRad,den1,den2,den3);
    den2 = ui->den2->text().toFloat();
    dev.setDenoiseParameters(denRad,den1,den2,den3);
}

void MainWindow::on_den3_editingFinished()
{
    dev.getDenoiseParameters(denRad,den1,den2,den3);
    den3 = ui->den3->text().toFloat();
    dev.setDenoiseParameters(denRad,den1,den2,den3);
}

void MainWindow::on_scan3D_clicked()
{
    if(GC3D_SUCCESS == dev.snapShot3D()){
        dev.getGC3DMetaData(meta);
        QImage image = QImage(meta.imgW,meta.imgH,QImage::Format_Indexed8);
        memcpy(image.bits(),meta.textureData,sizeof(uchar)*meta.imgH*meta.imgW);
        QPixmap pixmap = QPixmap::fromImage(image);
        ui->showImage->setPixmap(pixmap);
        ui->showImage->setScaledContents(true);
        ui->showImage->show();
    }else{
        QMessageBox box;
        box.information(this,"扫描","失败",QMessageBox::Cancel|QMessageBox::Ok,QMessageBox::Ok);
        box.show();
    }
}

void MainWindow::on_depthBtn_clicked(bool checked)
{
    if(meta.x!=nullptr){
        QImage image = QImage(meta.imgW,meta.imgH,QImage::Format_Indexed8);
        if(checked){
            memcpy(image.bits(),meta.depthImageData,sizeof(uchar)*meta.imgH*meta.imgW);
        }else{
            memcpy(image.bits(),meta.textureData,sizeof(uchar)*meta.imgH*meta.imgW);
        }
        QPixmap pixmap = QPixmap::fromImage(image);
        ui->showImage->setPixmap(pixmap);
        ui->showImage->setScaledContents(true);
        ui->showImage->show();
    }
}
