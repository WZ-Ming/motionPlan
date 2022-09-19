#ifndef S_MOTIONPLAN_H
#define S_MOTIONPLAN_H

#include<math.h>
#include<qmath.h>
#include<QDebug>
#include<QObject>

struct pathInitData{
    double max_acc=0;//最大加速度
    double Jerk=0 ;//加加速度
    double cmd_pos=0 ;//位移
    double v0=0;//初速度
    double VMax=0;//最大速度
    double ve=0;//末速度
    double deltaT=0.001;//规划时最小时间单位(默认0.001s)
};

class motionPlan : public QObject
{
   Q_OBJECT
public:
    motionPlan();
    ~motionPlan();
    bool performPath(bool path_pause=false);//执行规划
#ifdef motionDebug
    void get_move_msg(QString &motionMsg);
#endif
    void get_move_msg(double &curProportion,double &curPos_m,double &curVs_m, double &curAcc_m,double &curTime_m);//获得当前所走总位移、速度、加速度、时间
    void ini_path_data(const pathInitData &pathInit);//初始化参数
    bool pathBusy();//判断是否在规划中

signals:
    void sendMsg(const QString &);//发送调试信息

private:
    double deltaT=0.001;

    const double coverAccuracy=pow(10,-12);//弥补double精度
    const double maxProportion=1;

#ifdef motionDebug
    QString motionStatusMsg;
#endif

    double cmd_pos=0,remaining_pos=0,dec_move_pos=0,cur_move_pos=0;
    double v0=0,Vmax=0,ve=0,vs=0,dec_ve=0;
    double maxAcc=0,Jerk=0,val_Auv=0;
    double recordTime=0,ficureTimeRecord=0,recordDis=0;

    bool decInit=false, dec_finished=false,path_pause_cmd=false;
    bool path_busy=false,can_do_path=false;

    uint preView_phase=0;
    bool preViewDone=false;

    int moveDir=0;
    int ficureMoveDir=0;

    int acc_t=0;
    double ficureTime[4]={0,0,0,0};
    double ficureV[3]={0,0,0};
    double ficureAcc[3]={0,0,0};
    double ficureDis[3]={0,0,0};

    enum enumType{positiveAcceleration,steadyAcceleration,negetiveAcceleration};
    void judge_path_condition();//进行规划条件判断
    void doMotionPlan(bool doPath);//运行单位时间计算
    bool preView(enumType motionType);//前瞻下一个单位时间,并做出判断
    void preFicure();//前瞻完成,将后半程各个阶段的信息计算出来
    void movePreViewPos(enumType motionType);//前瞻后运动一个单位时间的计算
    void moveFicurePos(enumType motionType, const double time);//后半程运动一个单位时间的计算
    void get_time_point(const double vEnd,double &cal_vmax,double &cal_maxAcc,double &t1_tmp,double &t2_tmp,double &moveDis);//获取到达最大速度的时间,最大加速度,以及后半程各阶段时间
    void get_ficure_point(const double vBegin,const double vEnd,const double t1_tmp,const double t2_tmp);//获取后半程时间,位移,速度,加速度节点信息
};


#endif // S_MOTIONPLAN_H
