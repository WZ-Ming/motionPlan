#ifndef AUTORUN_H
#define AUTORUN_H

#include <QThread>
#include<QQueue>
#include<QDebug>
#include<QFile>
#include<QList>
#include<qmath.h>
#include<QTextStream>
#include <QTime>       //使用了QTime函数
#include <QtGlobal>    //qsrand和qrand这两个函数在这里面
#include"motionplan.h"

typedef struct{
    double Vmax=0 ;
    double maxAuv_acc=0;
    double Jerk=0 ;
    double cmd_pos=0 ;
}path_axis_motion;

class autoRun : public QThread
{
    Q_OBJECT

public:
    autoRun();
    ~autoRun();
    int cmd;
    bool run_var = true;
    QList<path_axis_motion>motion_data_pravete;
    double cal_delay,cal_time;
    bool path_pause=false;
    bool do_float;
    double v0,ve;
    double test_dis;
    double muti_T;

private:
    int old_path();
    int sync_path();
    int S_path();
    int exp_path();

signals:
    void send_data(int,int,int,int,double);

protected:
    void run();
};
#endif // AUTORUN_H
