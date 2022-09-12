#ifndef S_MOTIONPLAN_H
#define S_MOTIONPLAN_H

#include<math.h>
#include<qmath.h>
#include<QDebug>

struct pathInitData{
    double max_acc=0;//最大加速度
    double Jerk=0 ;//加加速度
    double cmd_pos=0 ;//位移
    double v0=0;//初速度
    double VMax=0;//最大速度
    double ve=0;//末速度
    double dec_jerk_muti=1;//暂停时加加速度比例
    double deltaT=0.001;//规划时最小时间单位(默认0.001s)
};

class motionPlan
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
    //void ini_path_data(double cmdPos_M, double v0_M, double Vmax_M, double ve_M, double maxAcc_M, double Jerk_M, double deltaT_M=0.001,double maxProportion_M=1,double dec_jerk_muti_M=1.5);
    void ini_path_data(const pathInitData &pathInit);
    bool pathBusy();//判断是否在规划中

private:
    double deltaT=0.001;
    double dec_ve=0,dec_jerk_muti=1;

    const double coverAccuracy=pow(10,-12);
    const double maxProportion=1;

    int path_busy=0;
    bool judge_path=false;

    double cmd_pos=0,v0=0,Vmax=0,ve=0,maxAcc=0,Jerk=0,val_Auv=0,vs=0;
    double cur_move_pos=0,recordTime=0;

    double remaining_pos=0,dec_move_pos=0;
    bool dec_finished=false,path_pause_cmd=false,can_do_path=false;
    double extra_pos=0,per_add_pos=0;
    int motion_phase=0,preView_phase=0;
    int acc_t3=0,even_t0=0,dec_t1=0,dec_t2=0,dec_t3=0;

    enum enumType{positiveAcceleration,steadyAcceleration,negetiveAcceleration};
    bool judge_path_condition();
    void doMotionPlan(bool doPath);
    bool preView(enumType motionType);
    void movePos(enumType motionType);
    void preFicure();
};


#endif // S_MOTIONPLAN_H
