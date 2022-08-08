#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFile>
#include <math.h>
#include<QList>
#include<QTextStream>
#include<QtCharts>
#include <QWheelEvent>
#include <QMouseEvent>
#include"autorun.h"
using namespace QtCharts;

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
    void rec_cal_done_sig();

    void rec_data(double, double, double);

    void on_btn_start_clicked();

    void on_btn_clear_clicked();

    void on_btn_pause_clicked();

    void on_btn_showPoints_clicked();

protected:
    void wheelEvent(QWheelEvent* event);

    void mousePressEvent(QMouseEvent *event);

    void mouseMoveEvent(QMouseEvent *event);

    void mouseReleaseEvent(QMouseEvent *event);

private:
    bool drag_flag=false;
    QPoint m_lastPointF;
    double rangeX0=0,rangeX1=1,rangeY0=0,rangeY1=1;
    void init_chartAndComBox();

    QLineSeries *series=nullptr;
    Ui::MainWindow *ui;
    autoRun *autorun=nullptr;
};

#endif // MAINWINDOW_H
