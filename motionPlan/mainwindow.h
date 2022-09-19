#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFile>
#include <math.h>
#include<QList>
#include<QTextStream>
#include<QTableWidget>
#include<QPointer>
#include"autorun.h"
#include "mychart.h"

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

signals:
    void send_data(double, double, double,double,double);

private slots:
    void recMotionPlanMsg(const QString &arg);

    void rec_data(double, double, double , double , double);

    void showMenu();

    void cascadeShow();

    void tileShow();

    void rec_cal_done_sig();

    void on_btn_start_clicked();

    void on_btn_pause_clicked();

    void on_btn_showPoints_clicked();

    void on_btn_cancelPlan_clicked();

    void on_mdiArea_customContextMenuRequested(const QPoint &pos);

private:
    void initTableWidget();

    Ui::MainWindow *ui;
    QPointer<myChart>disChart,spdChart,accChart;
    QPointer<QTableWidget>dataTableWidget;
    //myChart *disChart=nullptr;
    //myChart* spdChart=nullptr;
    //myChart* accChart=nullptr;
    //QTabWidget* dataTabWidget=nullptr;
    autoRun *autorun=nullptr;
};

#endif // MAINWINDOW_H
