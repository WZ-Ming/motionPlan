#ifndef MOTIONPLAN_H
#define MOTIONPLAN_H

#include <QDebug>
#include<qmath.h>
#include<QFile>
#include<QTextStream>
#include<QMessageBox>

class MotionPlan
{
public:
    bool performPath(bool path_pause=false);//执行规划
    void get_move_proportion(double &curProportion);//获得当前所走总位移占全程位移的比例
    void get_move_msg(double &curPos);//获得当前所走总位移
    void get_move_msg(double &curPos,double &curTime);
    void get_move_msg(double &curPos_m,double &curVs_m, double &curAcc_m);//获得当前所走总位移、速度、加速度
    void get_move_msg(double &curPos_m,double &curVs_m, double &curAcc_m,double &curTime_m);//获得当前所走总位移、速度、加速度、时间
    bool get_fixed_spd(double &fixedVe,double &fixedVmax);//获得修正的末速度
    bool get_all_time(double &allTime);//获得走完全程所花时间
    bool dis_cal_time(double &dis);//根据理论位移修正实际到达的位置
    bool dis_cal_time(double &dis, double &cal_time);//根据位移计算所花时间以及实际位移
    void ini_path_data(double cmdPos_M, double v0_M, double Vmax_M, double ve_M, double maxAcc_M, double Jerk_M, double deltaT_M=0.001,double maxProportion_M=1,double dec_jerk_muti_M=1.5);
    bool pathBusy();//判断是否在规划中
    double dec_ve=0;
private:
    double cmd_pos=0,v0=0,Vmax=0,ve=0,maxAcc=0,Jerk=0;
    double deltaT=0.001,dec_jerk_muti=1.5,maxProportion=1;

    double remaining_pos=0,dec_move_pos=0;
    int path_busy=0;
    bool dec_finished=false,path_pause_cmd=false,can_do_path=false,judge_path=false;

    double extra_pos=0,per_add_pos=0,val_Auv=0,vs=0;
    int motion_phase=0,preView_phase=0;
    int acc_t3=0,even_t0=0,dec_t1=0,dec_t2=0,dec_t3=0;
    double cur_move_pos=0,recordTime=0;

    const double coverAccuracy=pow(10,-12);

    enum enumType{positiveAcceleration,steadyAcceleration,negetiveAcceleration};
    bool judge_path_condition();
    void doMotionPlan(bool doPath);
    bool preView(enumType motionType);
    void movePos(enumType motionType);
    void preFicure();
};

class MotionPlan_Exp
{
public:
    bool performPath(bool path_pause=false);//执行规划
    void get_move_proportion(double &curProportion);//获得当前所走总位移占全程位移的比例
    void get_move_msg(double &curPos);//获得当前所走总位移
    void get_move_msg(double &curPos,double &curTime);
    void get_move_msg(double &curPos_m,double &curVs_m, double &curAcc_m);//获得当前所走总位移、速度、加速度
    void get_move_msg(double &curPos_m,double &curVs_m, double &curAcc_m,double &curTime_m);//获得当前所走总位移、速度、加速度、时间
    bool get_all_time(double &allTime);//获得走完全程所花时间
    bool dis_cal_time(double &dis);//根据位移修正实际到达的位置
    bool dis_cal_time(double &dis, double &cal_time);//根据位移计算所花时间以及实际位移
    void ini_path_data(double cmdPos_M, double v0_M, double Vmax_M, double ve_M, double maxAcc_M, double Jerk_M, double deltaT_M=0.001, double maxProportion_M=1, double muti_T_M=1.5);
    bool pathBusy();//判断是否在规划中
private:
    double cmd_pos=0,v0=0,Vmax=0,ve=0,maxAcc=0,Jerk=0;
    double deltaT=0.001,maxProportion=1,muti_T=0;

    int path_busy=0;
    bool judge_path=false;
    double recordTime=0,cur_move_pos=0,val_Auv=0,vs=0,fixDis=0;
    double motion_time[4]={0,0,0,0};

    const double binaryOffsetDis=1,coverAccuracy=pow(10,-12);

    void get_time_point();
    void get_dis();
    bool judge_path_condition();//规划前进行初始条件判断
};

class MotionPlan_S
{
public:
    bool performPath(bool path_pause=false);//执行规划
    void get_move_proportion(double &curProportion);//获得当前所走总位移占全程位移的比例
    void get_move_msg(double &curPos);//获得当前所走总位移
    void get_move_msg(double &curPos,double &curTime);
    void get_move_msg(double &curPos_m,double &curVs_m, double &curAcc_m);//获得当前所走总位移、速度、加速度
    void get_move_msg(double &curPos_m,double &curVs_m, double &curAcc_m,double &curTime_m);//获得当前所走总位移、速度、加速度、时间
    bool get_all_time(double &allTime);//获得走完全程所花时间
    bool dis_cal_time(double &dis);//根据位移修正实际到达的位置
    bool dis_cal_time(double &dis, double &cal_time);//根据位移计算所花时间以及实际位移
    void ini_path_data(double cmdPos_M, double v0_M, double Vmax_M, double ve_M, double maxAcc_M, double Jerk_M, double deltaT_M=0.001, double maxProportion_M=1, double muti_T_M=1.5);
    bool pathBusy();//判断是否在规划中
private:

    double cmd_pos=0,v0=0,Vmax=0,ve=0,maxAcc=0,Jerk=0;
    double deltaT=0.001,maxProportion=1;
    int path_busy=0;
    bool judge_path=false,can_do_path=false;

    double vs=0,val_Auv=0,recordTime=0,cur_move_pos=0;
    double acc0=0,acc1=0,jerk0=0,jerk1=0,fixDis=0;
    double t1=0,t2=0,t3=0,t4=0,t5=0,t6=0,t7=0;
    double motion_time[8]={0,0,0,0,0,0,0,0},motion_v[8]={0,0,0,0,0,0,0,0},motion_dis[8]={0,0,0,0,0,0,0,0};

    const double binaryOffsetDis=1,coverAccuracy=pow(10,-12);

    void get_time_point();
    void get_dis();
    void get_note();
    bool judge_path_condition();//规划前进行初始条件判断
};

#endif // MOTIONPLAN_H
