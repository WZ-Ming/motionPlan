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
    void rec_data(int,int, int, int, double);

    void on_btn_Axis_StepF_clicked();

    void on_pause_clicked();

    void on_start_clicked();

    void on_btn_old_path_clicked();

    void on_exp_path_clicked();

protected:
    void wheelEvent(QWheelEvent* event);

    void mousePressEvent(QMouseEvent *event);

    void mouseMoveEvent(QMouseEvent *event);

    void mouseReleaseEvent(QMouseEvent *event);

private:
    double axis_time[32]={0};
    bool drag_flag=false;
    QPoint m_lastPointF;
    QList<QColor> color{Qt::red,Qt::black,Qt::blue,Qt::darkGreen,Qt::black,Qt::blue,Qt::darkGreen,Qt::red};
    void init_chart();
    bool checking_axis_data();

    Ui::MainWindow *ui;
    autoRun *autorun=nullptr;
};

#endif // MAINWINDOW_H
