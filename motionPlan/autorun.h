#ifndef AUTORUN_H
#define AUTORUN_H

#include <QThread>
#include<QQueue>
#include<QDebug>
#include<QFile>
#include<QList>
#include<qmath.h>
#include<QTextStream>
#include<QApplication>
#include <QTime>       //使用了QTime函数
#include <QtGlobal>    //qsrand和qrand这两个函数在这里面
#include<QScopedPointer>
#include"motionplan.h"

class autoRun : public QThread
{
    Q_OBJECT
public:
    autoRun();
    ~autoRun();
    pathInitData pathInit;
    bool path_pause=false;
    QString pathMode;

    bool forceQuit=false;

protected:
    void run();

private:
    motionPlan *motion=nullptr;

signals:
    void sendMotionPlanMsg(const QString &);
    void send_data(double,double,double,double,double);
    void send_cal_done_sig();

};
#endif // AUTORUN_H
