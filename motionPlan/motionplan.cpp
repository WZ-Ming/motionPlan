#include "motionplan.h"

void MotionPlan::ini_path_data(double cmdPos_M, double v0_M, double Vmax_M, double ve_M, double maxAcc_M, double Jerk_M, double deltaT_M, double maxProportion_M, double dec_jerk_muti_M)//初始化参数
{
    cmd_pos=cmdPos_M; remaining_pos=cmdPos_M;
    v0=fabs(v0_M); Vmax=fabs(Vmax_M); ve=fabs(ve_M);
    maxAcc=fabs(maxAcc_M); Jerk=fabs(Jerk_M);
    deltaT=fabs(deltaT_M); maxProportion=fabs(maxProportion_M); dec_jerk_muti=fabs(dec_jerk_muti_M);

    path_busy=1;dec_move_pos=0;
    dec_finished=false;path_pause_cmd=false;
    can_do_path=false;judge_path=false;

    extra_pos=0;per_add_pos=0; vs=v0; val_Auv=0;
    preView_phase=0;motion_phase=0;
    acc_t3=0;even_t0=0;dec_t1=0;dec_t2=0;dec_t3=0;
    cur_move_pos=0; recordTime=0;
}

bool MotionPlan::pathBusy()//判断是否在规划中
{
    if(path_busy) return true;
    else return false;
}

bool MotionPlan::judge_path_condition()//规划前进行初始条件判断
{
    if(path_busy==0)return false;
    if(fabs(Vmax*deltaT/cmd_pos)>maxProportion)Vmax=fabs(maxProportion*cmd_pos/deltaT);
    if(Vmax<qMax(vs,ve)){
        path_busy=0;
        return false;
    }
    if(ve!=vs && fabs(ve-vs)>coverAccuracy){//初速度和末速度不相等的情况下
        double cal_ve=0,minDis=0,vmin=0;
        if(vs>ve)
            cal_ve=vs-Jerk*pow(deltaT,2);
        else
            cal_ve=vs+Jerk*pow(deltaT,2);
        minDis=(vs+cal_ve)*deltaT;
        vmin=qMax(vs,cal_ve);
        if(fabs(cmd_pos)<fabs(minDis) || maxAcc<Jerk*deltaT || Vmax<vmin){//距离太短或最大加速度太小或最大速度太小，退出S型速度规划
            can_do_path=false;
        }
        else{//判断是否需要修正末速度
            double cal_maxAcc=sqrt(fabs(vs-ve)*Jerk);
            if(cal_maxAcc>maxAcc)
                cal_maxAcc=maxAcc;
            cal_maxAcc=((int64_t)((cal_maxAcc+coverAccuracy)/(Jerk*deltaT)))*Jerk*deltaT;
            double t1_tmp=cal_maxAcc/Jerk;
            double t2_tmp=0;
            if(fabs(cal_maxAcc)>coverAccuracy)
                t2_tmp=(fabs(vs-ve)-pow(cal_maxAcc,2)/Jerk)/cal_maxAcc;
            t2_tmp=((int64_t)((t2_tmp+coverAccuracy)/deltaT))*deltaT;
            double fixed_ve=0;
            if(ve>vs)
                fixed_ve=vs+(pow(cal_maxAcc,2)/Jerk)+cal_maxAcc*t2_tmp;
            else
                fixed_ve=vs-(pow(cal_maxAcc,2)/Jerk)-cal_maxAcc*t2_tmp;
            double moveDis=(fixed_ve+vs)*t1_tmp+(fixed_ve+vs)*t2_tmp/2;
            if(fabs(moveDis)>fabs(cmd_pos)){//需要修正末速度
                double veTmp=fixed_ve,vsTmp=vs,vCenter=fixed_ve;
                double minus_dis=0;
                bool fixFinished=false;
                while(!fixFinished){
                    vCenter=(veTmp+vsTmp)/2;
                    cal_maxAcc=sqrt(fabs(vs-vCenter)*Jerk);
                    if(cal_maxAcc>maxAcc)
                        cal_maxAcc=maxAcc;
                    cal_maxAcc=((int64_t)((cal_maxAcc+coverAccuracy)/(Jerk*deltaT)))*Jerk*deltaT;
                    t1_tmp=cal_maxAcc/Jerk;
                    double t2_tmp=0;
                    if(fabs(cal_maxAcc)>coverAccuracy)
                        t2_tmp=(fabs(vs-vCenter)-pow(cal_maxAcc,2)/Jerk)/cal_maxAcc;
                    t2_tmp=((int64_t)((t2_tmp+coverAccuracy)/deltaT))*deltaT;
                    if(vCenter>vs)
                        vCenter=vs+(pow(cal_maxAcc,2)/Jerk)+cal_maxAcc*t2_tmp;
                    else
                        vCenter=vs-(pow(cal_maxAcc,2)/Jerk)-cal_maxAcc*t2_tmp;
                    moveDis=(vCenter+vs)*t1_tmp+(vCenter+vs)*t2_tmp/2;
                    if(fabs(minus_dis-(moveDis-fabs(cmd_pos)))<coverAccuracy)
                        fixFinished=true;
                    else
                        minus_dis=moveDis-fabs(cmd_pos);
                    if(fixFinished){
                        fixed_ve=vCenter;//修正的末速度
                        if(fixed_ve>=vs)
                            dec_t1=dec_t3=(int)((t1_tmp+coverAccuracy)/deltaT);
                        else
                            dec_t1=dec_t3=(int)(-(t1_tmp+coverAccuracy)/deltaT);
                        dec_t2=(int)((t2_tmp+coverAccuracy)/deltaT);
                        int moveDir= cmd_pos>=0 ? 1 : -1;
                        extra_pos=moveDir*fabs(moveDis-fabs(cmd_pos));
                        if(fabs(extra_pos)>pow(10,-3) && (fabs(dec_t1+dec_t3)+dec_t2)!=0)
                            per_add_pos=extra_pos/(fabs(dec_t1+dec_t3)+dec_t2);
                        else
                            extra_pos=0;
                        motion_phase=2;
                    }
                    else{
                        if(moveDis>fabs(cmd_pos)) veTmp=vCenter;
                        else vsTmp=vCenter;
                    }
                }
            }
            can_do_path=true;//末速度已满足要求，可以S型速度规划
        }
    }
    else{//初速度和末速度相等的情况下
        ve=v0;
        double cal_minDis=(vs+0.5*Jerk*pow(deltaT,2))*4.0*deltaT;
        double cal_vmin=vs+Jerk*pow(deltaT,2);
        if(cal_minDis>fabs(cmd_pos) || cal_vmin>Vmax || Jerk*deltaT>maxAcc){//距离太短或最大加速度太小或最大速度太小，退出S型速度规划
            can_do_path=false;
        }
        else can_do_path=true;//可以S型速度规划
    }
    return true;
}

bool MotionPlan::performPath(bool path_pause)//执行规划
{
    if(!judge_path){//进行S型判断
        if(!judge_path_condition())return false;
        judge_path=true;
    }
    int path_cmd=0;
    if(can_do_path){//S型规划模式
        if(!path_pause_cmd || dec_finished)path_pause_cmd=path_pause;
        if(path_busy){
            if(dec_finished){
                if(path_pause_cmd) path_cmd=0;
                else path_cmd=1;
            }
            else{
                if(path_pause_cmd) path_cmd=2;
                else path_cmd=1;
            }
            switch(path_cmd){
                case 1:{//正常模式
                    if(dec_finished) dec_finished=false;
                    doMotionPlan(can_do_path);
                }break;
                case 2:{//减速模式
                    if(!dec_finished){
                        double T1 = fabs(val_Auv / (Jerk*dec_jerk_muti));
                        if(vs - 0.5 * (Jerk*dec_jerk_muti) * pow(T1,2) >dec_ve){
                            if(vs-0.5*(Jerk*dec_jerk_muti)*pow(val_Auv/(Jerk*dec_jerk_muti),2)>dec_ve)
                                movePos(negetiveAcceleration);
                            else{
                                if(vs-0.5*(Jerk*dec_jerk_muti)*pow(val_Auv/(Jerk*dec_jerk_muti),2)>dec_ve)
                                    movePos(steadyAcceleration);
                                else
                                    movePos(positiveAcceleration);
                            }
                        }
                        else movePos(positiveAcceleration);
                        remaining_pos -= cur_move_pos;
                        if(vs<0.5*(Jerk*dec_jerk_muti)*pow(deltaT,2)){
                            dec_finished=true;
                            dec_move_pos+=(cmd_pos-remaining_pos);
                            cmd_pos=remaining_pos;
                            vs=dec_ve,val_Auv=0;
                            preView_phase=0,motion_phase=0;
                            can_do_path=false,judge_path=false;

                            extra_pos=0,per_add_pos=0;
                            acc_t3=0,even_t0=0,dec_t1=0,dec_t2=0,dec_t3=0;
                        }
                    }
                }break;
                default:break;
            }
        }
    }
    else{//匀速模式
        if(path_busy){
            path_pause_cmd=path_pause;
            if(path_pause_cmd) path_cmd=0;
            else path_cmd=1;
            switch(path_cmd){
                case 1:{
                    doMotionPlan(can_do_path);
                }break;
                default:break;
            }
        }
    }
    if(path_cmd==0) return false;
    else return true;
}

void MotionPlan::doMotionPlan(bool doPath)//进入运动模式
{
    if(doPath){
        switch(motion_phase){
            case 0:{
                switch(preView_phase){
                    case 0:{//加加速运动
                        if(preView(positiveAcceleration)){
                            recordTime+=deltaT;
                            movePos(positiveAcceleration);
                        }
                        else{
                            if(preView(steadyAcceleration)){
                                recordTime+=deltaT;
                                movePos(steadyAcceleration);
                                preView_phase=1;
                            }
                            else{
                                preFicure();
                                if(acc_t3==0) cur_move_pos=0;
                                else{
                                    recordTime+=deltaT;
                                    movePos(negetiveAcceleration);
                                    acc_t3--;
                                }
                                if(acc_t3==0){
                                    if(even_t0>0) motion_phase+=1;
                                    else if(dec_t1!=0) motion_phase+=2;
                                    else path_busy=0;
                                }
                                else preView_phase=2;
                            }
                        }
                    }break;
                    case 1:{//匀加速运动
                        if(preView(steadyAcceleration)){
                            recordTime+=deltaT;
                            movePos(steadyAcceleration);
                        }
                        else{
                            preFicure();
                            if(acc_t3==0) cur_move_pos=0;
                            else{
                                recordTime+=deltaT;
                                movePos(negetiveAcceleration);
                                acc_t3--;
                            }
                            if(acc_t3==0){
                                if(even_t0>0) motion_phase+=1;
                                else if(dec_t1!=0) motion_phase+=2;
                                else path_busy=0;
                            }
                            else preView_phase=2;
                        }
                    }break;
                    case 2:{//减加速运动
                        if(acc_t3!=0){
                            recordTime+=deltaT;
                            movePos(negetiveAcceleration);
                            acc_t3--;
                        }
                        if(acc_t3==0){
                            if(even_t0>0) motion_phase++;
                            else if(dec_t1!=0) motion_phase+=2;
                            else path_busy=0;
                        }
                    }break;
                }
            }break;
            case 1:{//匀速运动
                if(even_t0!=0){
                    recordTime+=deltaT;
                    movePos(steadyAcceleration);
                    even_t0--;
                }
                if(even_t0==0) motion_phase++;
            }break;
            case 2:{//加减速运动
                if(dec_t1>0){
                    recordTime+=deltaT;
                    movePos(positiveAcceleration);
                    dec_t1-=1;
                }
                else if(dec_t1<0){
                    recordTime+=deltaT;
                    movePos(negetiveAcceleration);
                    dec_t1+=1;
                }
                if(dec_t1==0){
                    if(dec_t2>0) motion_phase++;
                    else motion_phase+=2;
                }
            }break;
            case 3:{//匀减速运动
                if(dec_t2!=0){
                    recordTime+=deltaT;
                    movePos(steadyAcceleration);
                    dec_t2--;
                }
                if(dec_t2==0) motion_phase++;
            }break;
            case 4:{//减减速运动
                if(dec_t3>0){
                    recordTime+=deltaT;
                    movePos(negetiveAcceleration);
                    dec_t3-=1;
                }
                else if(dec_t3<0){
                    recordTime+=deltaT;
                    movePos(positiveAcceleration);
                    dec_t3+=1;
                }
                if(dec_t3==0) path_busy=0;
            }break;
        }
        if(path_busy){
            if(fabs(extra_pos)>coverAccuracy){
                extra_pos-=per_add_pos;
                cur_move_pos+=per_add_pos;
            }
        }
        else cur_move_pos=remaining_pos;
        remaining_pos -= cur_move_pos;
    }
    else{
        int moveDir = (remaining_pos >= 0) ? 1 : -1;
        if((moveDir*Vmax*deltaT)/(cmd_pos+dec_move_pos)>maxProportion){
            cur_move_pos=maxProportion*(cmd_pos+dec_move_pos);
        }
        else cur_move_pos=moveDir*Vmax*deltaT;
        recordTime+=deltaT;
        if(fabs(remaining_pos)>fabs(cur_move_pos))
            remaining_pos-=cur_move_pos;
        else{
            cur_move_pos=remaining_pos;
            remaining_pos=0;
            path_busy=0;
        }
    }
}

bool MotionPlan::preView(enumType motionType)//前瞻下一时刻
{
    double current_vs=0,current_acc=0,cal_vmax=0;
    double move_dis=0;
    int moveDir=remaining_pos>=0 ? 1 : -1;
    switch (motionType) {
        case steadyAcceleration:{//加速度增大
            move_dis=moveDir *(vs*deltaT+0.5*val_Auv*pow(deltaT,2));
            current_vs= vs+ val_Auv * deltaT;
            current_acc=val_Auv;
        }break;
        case positiveAcceleration:{//加速度不变
           move_dis=moveDir *(vs*deltaT+0.5*val_Auv*pow(deltaT,2)+1.0/6.0*Jerk*pow(deltaT,3));
           current_vs= vs+0.5*(val_Auv + val_Auv+Jerk * deltaT) * deltaT;
           current_acc=val_Auv+Jerk * deltaT;
        }break;
        default:break;
    }
    cal_vmax=current_vs+current_acc*current_acc/Jerk-0.5*Jerk*pow(current_acc/Jerk,2);
    if(cal_vmax>Vmax || current_acc>maxAcc) return false;
    else if(((moveDir*cal_vmax*deltaT)/(cmd_pos+dec_move_pos))>maxProportion) return false;
    else{
        move_dis+=moveDir*(current_vs*current_acc/Jerk+0.5*current_acc*pow(current_acc/Jerk,2)-Jerk*pow(current_acc/Jerk,3)*1.0/6.0);
        double cur_remaining_pos=remaining_pos-move_dis;
        double cal_maxAcc=sqrt(fabs(cal_vmax-ve)*Jerk);
        if(cal_maxAcc>maxAcc)
            cal_maxAcc=maxAcc;
        cal_maxAcc=((int64_t)((cal_maxAcc+coverAccuracy)/(Jerk*deltaT)))*Jerk*deltaT;
        double t1_tmp=cal_maxAcc/Jerk;
        double t2_tmp=0;
        if(fabs(cal_maxAcc)>coverAccuracy)
            t2_tmp=(fabs(cal_vmax-ve)-pow(cal_maxAcc,2)/Jerk)/cal_maxAcc;
        t2_tmp=((int64_t)((t2_tmp+coverAccuracy)/deltaT))*deltaT;
        double fixed_ve=0;
        if(ve>cal_vmax)
            fixed_ve=cal_vmax+(pow(cal_maxAcc,2)/Jerk)+cal_maxAcc*t2_tmp;
        else
            fixed_ve=cal_vmax-(pow(cal_maxAcc,2)/Jerk)-cal_maxAcc*t2_tmp;
        double moveDis=(fixed_ve+cal_vmax)*t1_tmp+(cal_vmax+fixed_ve)*t2_tmp/2;
        if(fabs(moveDis)<=fabs(cur_remaining_pos)) return true;
        else return false;
    }
}

void MotionPlan::preFicure()//全部前瞻完,结束前瞻
{
    int moveDir=remaining_pos>=0 ? 1 : -1;
    double cal_vmax=vs+val_Auv*val_Auv/Jerk-0.5*Jerk*pow(val_Auv/Jerk,2);
    double cal_acc_dis=moveDir*(vs*val_Auv/Jerk+0.5*val_Auv*pow(val_Auv/Jerk,2)-Jerk*pow(val_Auv/Jerk,3)*1.0/6.0);
    acc_t3=(int)((val_Auv+coverAccuracy)/Jerk/deltaT);
    double cal_maxAcc=sqrt(fabs(cal_vmax-ve)*Jerk);
    if(cal_maxAcc>maxAcc)
        cal_maxAcc=maxAcc;
    cal_maxAcc=((int64_t)((cal_maxAcc+coverAccuracy)/(Jerk*deltaT)))*Jerk*deltaT;
    double t1_tmp=cal_maxAcc/Jerk;
    double t2_tmp=0;
    if(fabs(cal_maxAcc)>coverAccuracy)
        t2_tmp=(fabs(cal_vmax-ve)-pow(cal_maxAcc,2)/Jerk)/cal_maxAcc;
    t2_tmp=((int64_t)((t2_tmp+coverAccuracy)/deltaT))*deltaT;
    double fixed_ve=0;
    if(ve>cal_vmax)
        fixed_ve=cal_vmax+(pow(cal_maxAcc,2)/Jerk)+cal_maxAcc*t2_tmp;
    else
        fixed_ve=cal_vmax-(pow(cal_maxAcc,2)/Jerk)-cal_maxAcc*t2_tmp;
    double moveDis=(fixed_ve+cal_vmax)*t1_tmp+(cal_vmax+fixed_ve)*t2_tmp/2;
    even_t0=(int)((fabs(moveDis-fabs(remaining_pos-cal_acc_dis))+coverAccuracy)/cal_vmax/deltaT);
    if(fixed_ve>=cal_vmax)
        dec_t1=dec_t3=(int)((t1_tmp+coverAccuracy)/deltaT);
    else
        dec_t1=dec_t3=(int)(-(t1_tmp+coverAccuracy)/deltaT);
    dec_t2=(int)((t2_tmp+coverAccuracy)/deltaT);
    extra_pos=moveDir*(fabs(remaining_pos-cal_acc_dis)-even_t0*deltaT*cal_vmax-moveDis);
    if(fabs(extra_pos)>pow(10,-3) && (acc_t3+even_t0+fabs(dec_t1)+dec_t2+fabs(dec_t3))!=0)
        per_add_pos=extra_pos/(acc_t3+even_t0+fabs(dec_t1)+dec_t2+fabs(dec_t3));
    else
        extra_pos=0;
}

void MotionPlan::movePos(enumType motionType)//下一时刻运动计算
{
    int moveDir = (remaining_pos >= 0) ? 1 : -1;
    switch (motionType){
    case negetiveAcceleration://加速度绝对值减小
        cur_move_pos=moveDir *(vs*deltaT+0.5*val_Auv*pow(deltaT,2)-1.0/6.0*Jerk*pow(deltaT,3));
        vs+=0.5*(val_Auv + val_Auv-Jerk * deltaT) * deltaT;
        val_Auv -= Jerk * deltaT;
        break;
    case steadyAcceleration://加速度不变
        cur_move_pos=moveDir *(vs*deltaT+0.5*val_Auv*pow(deltaT,2));
        vs+=val_Auv * deltaT;
        val_Auv = val_Auv;
        break;
    case positiveAcceleration://加速度绝对值增大
        cur_move_pos=moveDir *(vs*deltaT+0.5*val_Auv*pow(deltaT,2)+1.0/6.0*Jerk*pow(deltaT,3));
        vs+=0.5*(val_Auv + val_Auv+Jerk * deltaT) * deltaT;
        val_Auv += Jerk * deltaT;
        break;
    default:break;
    }
}

void MotionPlan::get_move_proportion(double &curProportion)//获得当前所走总位移占全程位移的比例
{
    if(fabs(cmd_pos+dec_move_pos)<=coverAccuracy) curProportion=0;
    else curProportion=curProportion=(cmd_pos-remaining_pos+dec_move_pos)/(cmd_pos+dec_move_pos);

}

void MotionPlan::get_move_msg(double &curPos)//获得当前所走总位移
{
    curPos=cmd_pos-remaining_pos+dec_move_pos;
}

void MotionPlan::get_move_msg(double &curPos,double &curTime)//获得当前所走总位移
{
    curPos=cmd_pos-remaining_pos+dec_move_pos;
    curTime=recordTime;
}

void MotionPlan::get_move_msg(double &curPos_m,double &curVs_m, double &curAcc_m)//获得当前所走总位移、速度、加速度
{
    curPos_m=cur_move_pos;
    curVs_m=vs;
    curAcc_m=val_Auv;
}

void MotionPlan::get_move_msg(double &curPos_m,double &curVs_m, double &curAcc_m,double &curTime_m)//获得当前所走总位移、速度、加速度、时间
{
    curPos_m=cmd_pos-remaining_pos+dec_move_pos;
    curVs_m=vs;
    curAcc_m=val_Auv;
    curTime_m=recordTime;
}

bool MotionPlan::get_fixed_spd(double &fixedVe,double &fixedVmax)//获得修正的末速度
{
    if(!judge_path_condition()){
        fixedVe=0;fixedVmax=0;
        return false;
    }
    double cmdPos_M=cmd_pos,v0_M=v0,Vmax_M=Vmax,ve_M=ve,maxAcc_M=maxAcc,Jerk_M=Jerk,deltaT_M=deltaT,maxProportion_M=maxProportion,dec_jerk_muti_M=dec_jerk_muti;
    fixedVmax=0;
    while(path_busy){
        doMotionPlan(can_do_path);
        if(fixedVmax<vs)fixedVmax=vs;
    }
    fixedVe=vs;
    ini_path_data(cmdPos_M,v0_M,Vmax_M,ve_M,maxAcc_M,Jerk_M,deltaT_M,maxProportion_M,dec_jerk_muti_M);
    return true;
}

bool MotionPlan::get_all_time(double &allTime)//获得走完全程所花时间
{
    if(!judge_path_condition()){
        allTime=0;
        return false;
    }
    double cmdPos_M=cmd_pos,v0_M=v0,Vmax_M=Vmax,ve_M=ve,maxAcc_M=maxAcc,Jerk_M=Jerk,deltaT_M=deltaT,maxProportion_M=maxProportion,dec_jerk_muti_M=dec_jerk_muti;
    while(path_busy){
        doMotionPlan(can_do_path);
    }
    allTime=recordTime;
    ini_path_data(cmdPos_M,v0_M,Vmax_M,ve_M,maxAcc_M,Jerk_M,deltaT_M,maxProportion_M,dec_jerk_muti_M);
    return true;
}

bool MotionPlan::dis_cal_time(double &dis,double &cal_time)//根据位移计算所花时间以及实际位移
{
    if(!judge_path_condition()){
        dis=0;cal_time=0;
        return false;//未初始化
    }
    double cmdPos_M=cmd_pos,v0_M=v0,Vmax_M=Vmax,ve_M=ve,maxAcc_M=maxAcc,Jerk_M=Jerk,deltaT_M=deltaT,maxProportion_M=maxProportion,dec_jerk_muti_M=dec_jerk_muti;
    bool cal_done=false;
    double before_dis=0,before_T=0;
    while(!cal_done){
        doMotionPlan(can_do_path);
        if(path_busy==0){
            cal_done=true;
            dis=cmd_pos-remaining_pos;
            cal_time=recordTime;
        }
        else if(fabs(cmd_pos-remaining_pos)>=fabs(dis)){
            cal_done=true;
            double dis_tmp=cmd_pos-remaining_pos;
            if(fabs((fabs(dis_tmp)-fabs(dis)))>fabs((fabs(dis)-fabs(before_dis)))){
                dis=before_dis;
                cal_time=before_T;
            }
            else{
                dis=cmd_pos-remaining_pos;
                cal_time=recordTime;
            }
        }
        before_dis=cmd_pos-remaining_pos;
        before_T=recordTime;
    }
    ini_path_data(cmdPos_M,v0_M,Vmax_M,ve_M,maxAcc_M,Jerk_M,deltaT_M,maxProportion_M,dec_jerk_muti_M);
    return true;
}

bool MotionPlan::dis_cal_time(double &dis)//根据位移计算所花时间以及实际位移
{
    if(!judge_path_condition()){
        dis=0;
        return false;//未初始化
    }
    double cmdPos_M=cmd_pos,v0_M=v0,Vmax_M=Vmax,ve_M=ve,maxAcc_M=maxAcc,Jerk_M=Jerk,deltaT_M=deltaT,maxProportion_M=maxProportion,dec_jerk_muti_M=dec_jerk_muti;
    bool cal_done=false;
    double before_dis=0;
    while(!cal_done){
        doMotionPlan(can_do_path);
        if(path_busy==0){
            cal_done=true;
            dis=cmd_pos-remaining_pos;
        }
        else if(fabs(cmd_pos-remaining_pos)>=fabs(dis)){
            cal_done=true;
            double dis_tmp=cmd_pos-remaining_pos;
            if(fabs((fabs(dis_tmp)-fabs(dis)))>fabs((fabs(dis)-fabs(before_dis)))){
                dis=before_dis;
            }
            else{
                dis=cmd_pos-remaining_pos;
            }
        }
        before_dis=cmd_pos-remaining_pos;
    }
    ini_path_data(cmdPos_M,v0_M,Vmax_M,ve_M,maxAcc_M,Jerk_M,deltaT_M,maxProportion_M,dec_jerk_muti_M);
    return true;
}


void MotionPlan_Exp::ini_path_data(double cmdPos_M, double v0_M, double Vmax_M, double ve_M, double maxAcc_M, double Jerk_M, double deltaT_M, double maxProportion_M, double muti_T_M)//初始化参数
{
    cmd_pos=cmdPos_M;
    v0=fabs(v0_M); Vmax=fabs(Vmax_M); ve=fabs(ve_M);
    maxAcc=fabs(maxAcc_M); Jerk=fabs(Jerk_M);
    deltaT=fabs(deltaT_M); maxProportion=fabs(maxProportion_M); muti_T=fabs(muti_T_M);

    path_busy=1; judge_path=false;
    vs=v0; val_Auv=0; recordTime=0; cur_move_pos=0; fixDis=0;
    motion_time[0]=0; motion_time[1]=0; motion_time[2]=0; motion_time[3]=0;
}

bool MotionPlan_Exp::pathBusy()//判断是否在规划中
{
    if(path_busy) return true;
    else return false;
}

void MotionPlan_Exp::get_move_proportion(double &curProportion)//获得当前所走总位移占全程位移的比例
{
    if(fabs(cmd_pos)<=coverAccuracy) curProportion=0;
    else curProportion=cur_move_pos/cmd_pos;
}

void MotionPlan_Exp::get_move_msg(double &curPos)//获得当前所走总位移
{
    curPos=cur_move_pos;
}

void MotionPlan_Exp::get_move_msg(double &curPos,double &curTime)//获得当前所走总位移
{
    curPos=cur_move_pos;
    curTime=recordTime;
}

void MotionPlan_Exp::get_move_msg(double &curPos_m,double &curVs_m, double &curAcc_m)//获得当前所走总位移、速度、加速度
{
    curPos_m=cur_move_pos;
    curVs_m=vs;
    curAcc_m=val_Auv;
}

void MotionPlan_Exp::get_move_msg(double &curPos_m,double &curVs_m, double &curAcc_m,double &curTime_m)//获得当前所走总位移、速度、加速度、时间
{
    curPos_m=cur_move_pos;
    curVs_m=vs;
    curAcc_m=val_Auv;
    curTime_m=recordTime;
}

bool MotionPlan_Exp::judge_path_condition()//规划前进行初始条件判断
{
    if(path_busy==0)return false;
    if(fabs(Vmax*deltaT/cmd_pos)>maxProportion)Vmax=fabs(maxProportion*cmd_pos/deltaT);
    if(Vmax<qMax(v0,ve)){
        path_busy=0;
        return false;
    }
    return true;
}

bool MotionPlan_Exp::performPath(bool path_pause)//执行规划
{
    if(!judge_path){
        if(!judge_path_condition())return false;
        get_time_point();
        judge_path=true;
    }
    if(path_busy){
        if(recordTime+deltaT>=motion_time[3]){
            recordTime=motion_time[3];
            cur_move_pos=cmd_pos;
            path_busy=0;
        }
        else {
            recordTime+=deltaT;
            get_dis();
        }
        return true;
    }
    else return false;
}

void MotionPlan_Exp::get_dis()
{
    if(recordTime<=motion_time[0]){
        cur_move_pos=(pow(M_E,muti_T)*Vmax-v0)/(pow(M_E,muti_T)-1)*recordTime-pow(M_E,muti_T)*(Vmax-v0)/(pow(M_E,muti_T)-1)*(1-pow(M_E,-recordTime/(motion_time[0]/muti_T)))*(motion_time[0]/muti_T);
        vs=v0+pow(M_E,muti_T)/(pow(M_E,muti_T)-1)*(Vmax-v0)*(1-pow(M_E,-recordTime/(motion_time[0]/muti_T)));
        val_Auv=pow(M_E,muti_T)/(pow(M_E,muti_T)-1)*(Vmax-v0)*(pow(M_E,-recordTime/(motion_time[0]/muti_T)))/(motion_time[0]/muti_T);
    }
    else if(recordTime>motion_time[0] && recordTime<=motion_time[0]+motion_time[1]){
        cur_move_pos=(pow(M_E,muti_T)*Vmax-v0)/(pow(M_E,muti_T)-1)*motion_time[0]-pow(M_E,muti_T)*(Vmax-v0)/(pow(M_E,muti_T)-1)*(1-pow(M_E,-muti_T))*(motion_time[0]/muti_T);
        cur_move_pos+=Vmax*(recordTime-motion_time[0]);
        vs=Vmax;
        val_Auv=0;
    }
    else{
        cur_move_pos=(pow(M_E,muti_T)*Vmax-v0)/(pow(M_E,muti_T)-1)*motion_time[0]-pow(M_E,muti_T)*(Vmax-v0)/(pow(M_E,muti_T)-1)*(1-pow(M_E,-muti_T))*(motion_time[0]/muti_T);
        cur_move_pos+=Vmax*motion_time[1];
        cur_move_pos+=(pow(M_E,muti_T)*ve-Vmax)/(pow(M_E,muti_T)-1)*(recordTime-motion_time[0]-motion_time[1])+pow(M_E,muti_T)*(Vmax-ve)/(pow(M_E,muti_T)-1)*(1-pow(M_E,-(recordTime-motion_time[0]-motion_time[1])/(motion_time[2]/muti_T)))*(motion_time[2]/muti_T);
        vs=Vmax+pow(M_E,muti_T)/(pow(M_E,muti_T)-1)*(Vmax-ve)*(pow(M_E,-(recordTime-motion_time[0]-motion_time[1])/(motion_time[2]/muti_T))-1);
        val_Auv=pow(M_E,muti_T)/(1-pow(M_E,muti_T))*(Vmax-ve)*(pow(M_E,-(recordTime-motion_time[0]-motion_time[1])/(motion_time[2]/muti_T)))/(motion_time[2]/muti_T);
    }
    int moveDir= (cmd_pos>=0) ? 1 : -1;
    cur_move_pos*=moveDir;
    cur_move_pos+=(fixDis*recordTime);
}

void MotionPlan_Exp::get_time_point()
{
    double T0_acc=pow(M_E,muti_T)*(Vmax-v0)/(pow(M_E,muti_T)-1)/maxAcc;
    double T1_acc=pow(M_E,muti_T)*(Vmax-ve)/(pow(M_E,muti_T)-1)/maxAcc;
    double T0_jerk=sqrt(pow(M_E,muti_T)*(Vmax-v0)/(pow(M_E,muti_T)-1)/Jerk);
    double T1_jerk=sqrt(pow(M_E,muti_T)*(Vmax-ve)/(pow(M_E,muti_T)-1)/Jerk);

    double T0=qMax(T0_acc,T0_jerk);
    double T1=qMax(T1_acc,T1_jerk);

    double s0=(pow(M_E,muti_T)*(Vmax)-v0)/(pow(M_E,muti_T)-1)*muti_T*T0-pow(M_E,muti_T)*(Vmax-v0)/(pow(M_E,muti_T)-1)*(1-pow(M_E,-muti_T))*T0;
    double s1=(pow(M_E,muti_T)*ve-Vmax)/(pow(M_E,muti_T)-1)*muti_T*T1+pow(M_E,muti_T)*(Vmax-ve)/(pow(M_E,muti_T)-1)*(1-pow(M_E,-muti_T))*T1;

    if((s0+s1)>fabs(cmd_pos)){
        double veTmp=Vmax,v0Tmp=qMax(v0,ve),vCenter=Vmax;
        bool fixFinished=false;
        while(!fixFinished){
            T0_acc=pow(M_E,muti_T)*(vCenter-v0)/(pow(M_E,muti_T)-1)/maxAcc;
            T1_acc=pow(M_E,muti_T)*(vCenter-ve)/(pow(M_E,muti_T)-1)/maxAcc;
            T0_jerk=sqrt(pow(M_E,muti_T)*(vCenter-v0)/(pow(M_E,muti_T)-1)/Jerk);
            T1_jerk=sqrt(pow(M_E,muti_T)*(vCenter-ve)/(pow(M_E,muti_T)-1)/Jerk);

            T0=qMax(T0_acc,T0_jerk);
            T1=qMax(T1_acc,T1_jerk);

            s0=(pow(M_E,muti_T)*vCenter-v0)/(pow(M_E,muti_T)-1)*muti_T*T0-pow(M_E,muti_T)*(vCenter-v0)/(pow(M_E,muti_T)-1)*(1-pow(M_E,-muti_T))*T0;
            s1=(pow(M_E,muti_T)*ve-vCenter)/(pow(M_E,muti_T)-1)*muti_T*T1+pow(M_E,muti_T)*(vCenter-ve)/(pow(M_E,muti_T)-1)*(1-pow(M_E,-muti_T))*T1;

            if((s0+s1)>fabs(cmd_pos)){
                veTmp=vCenter;
                vCenter=(veTmp+v0Tmp)/2;
            }
            else if((s0+s1)<fabs(cmd_pos)-binaryOffsetDis){
                v0Tmp=vCenter;
                vCenter=(veTmp+v0Tmp)/2;
            }
            else{
                fixFinished=true;
                Vmax=vCenter;
            }
        }
        motion_time[0]=muti_T*T0;
        motion_time[1]=0;
        motion_time[2]=muti_T*T1;
    }
    else{
        double T_vmax=(fabs(cmd_pos)-s0-s1)/(Vmax);
        motion_time[0]=muti_T*T0;
        motion_time[1]=T_vmax;
        motion_time[2]=muti_T*T1;
    }
    motion_time[3]=motion_time[0]+motion_time[1]+motion_time[2];
    int deltaT_cnt=(motion_time[3])/deltaT;
    motion_time[3]=deltaT_cnt*deltaT;
    double actual_dis=0;
    actual_dis=(pow(M_E,muti_T)*Vmax-v0)/(pow(M_E,muti_T)-1)*motion_time[0]-pow(M_E,muti_T)*(Vmax-v0)/(pow(M_E,muti_T)-1)*(1-pow(M_E,-muti_T))*(motion_time[0]/muti_T);
    actual_dis+=Vmax*motion_time[1];
    actual_dis+=(pow(M_E,muti_T)*ve-Vmax)/(pow(M_E,muti_T)-1)*(motion_time[3]-motion_time[0]-motion_time[1])+pow(M_E,muti_T)*(Vmax-ve)/(pow(M_E,muti_T)-1)*(1-pow(M_E,-(motion_time[3]-motion_time[0]-motion_time[1])/(motion_time[2]/muti_T)))*(motion_time[2]/muti_T);
    int moveDir= (cmd_pos>=0) ? 1 : -1;
    actual_dis*=moveDir;
    fixDis=(cmd_pos-actual_dis)/motion_time[3];
}

bool MotionPlan_Exp::dis_cal_time(double &dis,double &cal_time)//根据位移计算所花时间以及实际位移
{
    if(!judge_path_condition()){
        dis=0;cal_time=0;
        return false;
    }
    double cmdPos_M=cmd_pos,v0_M=v0,Vmax_M=Vmax,ve_M=ve,maxAcc_M=maxAcc,Jerk_M=Jerk,deltaT_M=deltaT,maxProportion_M=maxProportion,muti_T_M=muti_T;
    get_time_point();
    double dis_tmp=0,time_tmp=0;
    while(path_busy){
        if(recordTime+deltaT>=motion_time[3]){
            recordTime=motion_time[3];
            cur_move_pos=cmd_pos;
            dis=cur_move_pos;
            cal_time=recordTime;
            ini_path_data(cmdPos_M,v0_M,Vmax_M,ve_M,maxAcc_M,Jerk_M,deltaT_M,maxProportion_M,muti_T_M);
            return true;
        }
        else{
            recordTime+=deltaT;
            get_dis();
        }
        if(fabs(cur_move_pos)>=fabs(dis)){
            if(fabs(cur_move_pos-dis)<=fabs(cur_move_pos-dis_tmp)){
                dis=cur_move_pos;
                cal_time=recordTime;
            }
            else{
                dis=dis_tmp;
                cal_time=time_tmp;
            }
            path_busy=0;
            ini_path_data(cmdPos_M,v0_M,Vmax_M,ve_M,maxAcc_M,Jerk_M,deltaT_M,maxProportion_M,muti_T_M);
            return true;
        }
        else{
            dis_tmp=cur_move_pos;
            time_tmp=recordTime;
        }
    }
}

bool MotionPlan_Exp::dis_cal_time(double &dis)//根据位移计算所花时间以及实际位移
{
    if(!judge_path_condition()){
        dis=0;
        return false;
    }
    double cmdPos_M=cmd_pos,v0_M=v0,Vmax_M=Vmax,ve_M=ve,maxAcc_M=maxAcc,Jerk_M=Jerk,deltaT_M=deltaT,maxProportion_M=maxProportion,muti_T_M=muti_T;
    get_time_point();
    double dis_tmp=0;
    while(path_busy){
        if(recordTime+deltaT>=motion_time[3]){
            recordTime=motion_time[3];
            cur_move_pos=cmd_pos;
            dis=cur_move_pos;
            ini_path_data(cmdPos_M,v0_M,Vmax_M,ve_M,maxAcc_M,Jerk_M,deltaT_M,maxProportion_M,muti_T_M);
            return true;
        }
        else{
            recordTime+=deltaT;
            get_dis();
        }
        if(fabs(cur_move_pos)>=fabs(dis)){
            if(fabs(cur_move_pos-dis)<=fabs(cur_move_pos-dis_tmp)){
                dis=cur_move_pos;
            }
            else{
                dis=dis_tmp;
            }
            ini_path_data(cmdPos_M,v0_M,Vmax_M,ve_M,maxAcc_M,Jerk_M,deltaT_M,maxProportion_M,muti_T_M);
            return true;
        }
        else{
            dis_tmp=cur_move_pos;
        }
    }
}

bool MotionPlan_Exp::get_all_time(double &allTime)
{
    if(!judge_path_condition()){
        allTime=0;
        return false;
    }
    double cmdPos_M=cmd_pos,v0_M=v0,Vmax_M=Vmax,ve_M=ve,maxAcc_M=maxAcc,Jerk_M=Jerk,deltaT_M=deltaT,maxProportion_M=maxProportion,muti_T_M=muti_T;
    get_time_point();
    allTime=motion_time[3];
    ini_path_data(cmdPos_M,v0_M,Vmax_M,ve_M,maxAcc_M,Jerk_M,deltaT_M,maxProportion_M,muti_T_M);
}



void MotionPlan_S::ini_path_data(double cmdPos_M, double v0_M, double Vmax_M, double ve_M, double maxAcc_M, double Jerk_M, double deltaT_M, double maxProportion_M, double muti_T_M)//初始化参数
{
    cmd_pos=cmdPos_M;
    v0=fabs(v0_M); Vmax=fabs(Vmax_M); ve=fabs(ve_M);
    maxAcc=fabs(maxAcc_M); Jerk=fabs(Jerk_M);
    deltaT=fabs(deltaT_M); maxProportion=fabs(maxProportion_M);
    path_busy=1; judge_path=false; can_do_path=false;

    vs=v0; val_Auv=0; recordTime=0; cur_move_pos=0;
    acc0=0; acc1=0; jerk0=0; jerk1=0; fixDis=0;
    t1=0; t2=0; t3=0; t4=0; t5=0; t6=0; t7=0;
    for(int i=0;i<8;i++){
        motion_time[i]=0;
        motion_v[i]=0;
        motion_dis[i]=0;
    }
}

bool MotionPlan_S::pathBusy()//判断是否在规划中
{
    if(path_busy) return true;
    else return false;
}

void MotionPlan_S::get_move_proportion(double &curProportion)//获得当前所走总位移占全程位移的比例
{
    if(fabs(cmd_pos)<=coverAccuracy) curProportion=0;
    else curProportion=cur_move_pos/cmd_pos;

}

void MotionPlan_S::get_move_msg(double &curPos)//获得当前所走总位移
{
    curPos=cur_move_pos;
}

void MotionPlan_S::get_move_msg(double &curPos,double &curTime)//获得当前所走总位移
{
    curPos=cur_move_pos;
    curTime=recordTime;
}

void MotionPlan_S::get_move_msg(double &curPos_m,double &curVs_m, double &curAcc_m)//获得当前所走总位移、速度、加速度
{
    curPos_m=cur_move_pos;
    curVs_m=vs;
    curAcc_m=val_Auv;
}

void MotionPlan_S::get_move_msg(double &curPos_m,double &curVs_m, double &curAcc_m,double &curTime_m)//获得当前所走总位移、速度、加速度、时间
{
    curPos_m=cur_move_pos;
    curVs_m=vs;
    curAcc_m=val_Auv;
    curTime_m=recordTime;
}

bool MotionPlan_S::judge_path_condition()//规划前进行初始条件判断
{
    if(path_busy==0)return false;
    if(fabs(Vmax*deltaT/cmd_pos)>maxProportion)Vmax=fabs(maxProportion*cmd_pos/deltaT);
    if(Vmax<qMax(v0,ve)){
        path_busy=0;
        return false;
    }
    double vs=v0;
    if(ve!=vs && fabs(ve-vs)>coverAccuracy){//初速度和末速度不相等的情况下
        double cal_ve=0,minDis=0,vmin=0;
        if(vs>ve)
            cal_ve=vs-Jerk*pow(deltaT,2);
        else
            cal_ve=vs+Jerk*pow(deltaT,2);
        minDis=(vs+cal_ve)*deltaT;
        vmin=qMax(vs,cal_ve);
        if(fabs(cmd_pos)<fabs(minDis) || maxAcc<Jerk*deltaT || Vmax<vmin){//距离太短或最大加速度太小或最大速度太小，退出S型速度规划
            can_do_path=false;
        }
        else{//判断是否需要修正末速度
            double cal_maxAcc=sqrt(fabs(vs-ve)*Jerk);
            if(cal_maxAcc>maxAcc)
                cal_maxAcc=maxAcc;
            cal_maxAcc=((int64_t)((cal_maxAcc+coverAccuracy)/(Jerk*deltaT)))*Jerk*deltaT;
            double t1_tmp=cal_maxAcc/Jerk;
            double t2_tmp=0;
            if(fabs(cal_maxAcc)>coverAccuracy)
                t2_tmp=(fabs(vs-ve)-pow(cal_maxAcc,2)/Jerk)/cal_maxAcc;
            t2_tmp=((int64_t)((t2_tmp+coverAccuracy)/deltaT))*deltaT;
            double fixed_ve=0;
            if(ve>vs)
                fixed_ve=vs+(pow(cal_maxAcc,2)/Jerk)+cal_maxAcc*t2_tmp;
            else
                fixed_ve=vs-(pow(cal_maxAcc,2)/Jerk)-cal_maxAcc*t2_tmp;
            double moveDis=(fixed_ve+vs)*t1_tmp+(fixed_ve+vs)*t2_tmp/2;
            if(fabs(moveDis)>fabs(cmd_pos)){//需要修正末速度
                double veTmp=fixed_ve,vsTmp=vs,vCenter=fixed_ve;
                double minus_dis=0;
                bool fixFinished=false;
                while(!fixFinished){
                    vCenter=(veTmp+vsTmp)/2;
                    cal_maxAcc=sqrt(fabs(vs-vCenter)*Jerk);
                    if(cal_maxAcc>maxAcc)
                        cal_maxAcc=maxAcc;
                    cal_maxAcc=((int64_t)((cal_maxAcc+coverAccuracy)/(Jerk*deltaT)))*Jerk*deltaT;
                    t1_tmp=cal_maxAcc/Jerk;
                    double t2_tmp=0;
                    if(fabs(cal_maxAcc)>coverAccuracy)
                        t2_tmp=(fabs(vs-vCenter)-pow(cal_maxAcc,2)/Jerk)/cal_maxAcc;
                    t2_tmp=((int64_t)((t2_tmp+coverAccuracy)/deltaT))*deltaT;
                    if(vCenter>vs)
                        vCenter=vs+(pow(cal_maxAcc,2)/Jerk)+cal_maxAcc*t2_tmp;
                    else
                        vCenter=vs-(pow(cal_maxAcc,2)/Jerk)-cal_maxAcc*t2_tmp;
                    moveDis=(vCenter+vs)*t1_tmp+(vCenter+vs)*t2_tmp/2;
                    if(fabs(minus_dis-(moveDis-fabs(cmd_pos)))<coverAccuracy)
                        fixFinished=true;
                    else{
                        minus_dis=moveDis-fabs(cmd_pos);
                        if(moveDis>fabs(cmd_pos)) veTmp=vCenter;
                        else vsTmp=vCenter;
                    }
                }
            }
            can_do_path=true;//末速度已满足要求，可以S型速度规划
        }
    }
    else{//初速度和末速度相等的情况下
        ve=v0;
        double cal_minDis=(vs+0.5*Jerk*pow(deltaT,2))*4.0*deltaT;
        double cal_vmin=vs+Jerk*pow(deltaT,2);
        if(cal_minDis>fabs(cmd_pos) || cal_vmin>Vmax || Jerk*deltaT>maxAcc){//距离太短或最大加速度太小或最大速度太小，退出S型速度规划
            can_do_path=false;
        }
        else can_do_path=true;//可以S型速度规划
    }
    return true;
}

bool MotionPlan_S::performPath(bool path_pause)//执行规划
{
    if(!judge_path){
        if(!judge_path_condition())return false;
        get_time_point();
//        qDebug()<<123;
        judge_path=true;
    }
    if(path_busy){
        if(recordTime+deltaT>=motion_time[7]){
            recordTime=motion_time[7];
            cur_move_pos=cmd_pos;
            vs=ve;
            val_Auv=0;
            path_busy=0;
        }
        else {
            recordTime+=deltaT;
            get_dis();
        }
        return true;
    }
    else return false;
}

void MotionPlan_S::get_time_point()
{
    if(can_do_path){
        if(ve!=v0){
            double jerk2=0,acc2=0;
            double jerk_2=Jerk;
            double acc_2=maxAcc;
            if((pow(acc_2,2)/fabs(jerk_2))>fabs(ve-v0))acc_2=sqrt(fabs(ve-v0)*jerk_2);
            if(ve>=v0) { jerk2=jerk_2; acc2=acc_2; }
            else { jerk2=(-1)*jerk_2; acc2=(-1)*acc_2;}
            double s2=(v0+ve)*(acc2/jerk2)+(v0+ve)/2*(ve-v0-pow(acc2,2)/jerk2)/acc2;
            if(fabs(s2)>fabs(cmd_pos)){
                double ve_c=ve;
                double v0_c=v0;
                double ve_f=(ve_c+v0_c)/2;
                acc_2=sqrt(fabs(ve_f-v0)*jerk_2);
                if(ve_f>=v0) { jerk2=jerk_2;acc2=acc_2; }
                else { jerk2=(-1)*jerk_2; acc2=(-1)*acc_2;}
                s2=(v0+ve_f)*(acc2/jerk2)+(v0+ve_f)/2*(ve_f-v0-pow(acc2,2)/jerk2)/acc2;
                while(fabs(s2)>fabs(cmd_pos) || fabs(cmd_pos)-fabs(s2)>binaryOffsetDis){
                    if(fabs(s2)>fabs(cmd_pos)){
                        ve_c=ve_f;
                        ve_f=(ve_c+v0_c)/2;
                        acc_2=sqrt(fabs(ve_f-v0)*jerk_2);
                        if(ve_f>=v0) { jerk2=jerk_2;acc2=acc_2; }
                        else { jerk2=(-1)*jerk_2; acc2=(-1)*acc_2;}
                        s2=(v0+ve_f)*(acc2/jerk2)+(v0+ve_f)/2*(ve_f-v0-pow(acc2,2)/jerk2)/acc2;
                    }
                    else if(fabs(cmd_pos)-fabs(s2)>binaryOffsetDis){
                        v0_c=ve_f;
                        ve_f=(ve_c+v0_c)/2;
                        acc_2=sqrt(fabs(ve_f-v0)*jerk_2);
                        if(ve_f>=v0) { jerk2=jerk_2;acc2=acc_2; }
                        else { jerk2=(-1)*jerk_2; acc2=(-1)*acc_2;}
                        s2=(v0+ve_f)*(acc2/jerk2)+(v0+ve_f)/2*(ve_f-v0-pow(acc2,2)/jerk2)/acc2;
                    }
                }
                ve=ve_f;
                acc1=acc2,jerk1=jerk2;
                t1=t2=t3=t4=0;
                t5=t7=acc1/jerk1;
                t6=(ve-v0-pow(acc1,2)/jerk1)/acc1;
                get_note();
                return;
            }
        }
        double acc_0=maxAcc;
        double acc_1=maxAcc;
        double jerk_0=Jerk;
        double jerk_1=Jerk;
        double vs=0;
        if(Vmax>=v0) { jerk0=jerk_0;acc0=acc_0; }
        else { jerk0=(-1)*jerk_0; acc0=(-1)*acc_0;}
        if((pow(acc_0,2)/fabs(jerk0))>fabs(Vmax-v0))acc_0=sqrt(fabs(Vmax-v0)*jerk_0);
        if(Vmax>=v0) { jerk0=jerk_0;acc0=acc_0; }
        else { jerk0=(-1)*jerk_0; acc0=(-1)*acc_0;}
        double s0=(v0+Vmax)*(acc0/jerk0)+(v0+Vmax)/2*(Vmax-v0-pow(acc0,2)/jerk0)/acc0;
        if(Vmax>=ve){ jerk1=(-1)*jerk_1; acc1=(-1)*acc_1;}
        else{ jerk1=jerk_1; acc1=acc_1; }
        if((pow(acc_1,2)/fabs(jerk1))>fabs(ve-Vmax))acc_1=sqrt(fabs(Vmax-ve)*jerk_1);
        if(Vmax>=ve){ jerk1=(-1)*jerk_1; acc1=(-1)*acc_1;}
        else{ jerk1=jerk_1; acc1=acc_1; }
        double s1=(ve+Vmax)*(acc1/jerk1)+(ve+Vmax)/2*(ve-Vmax-pow(acc1,2)/jerk1)/acc1;
        if(fabs(s0+s1)>fabs(cmd_pos)){
            if(fabs(Vmax-v0)>fabs(Vmax-ve))vs=v0;
            else vs=ve;
            double vmax_f=Vmax;
            double vs_f=vs;
            double v_f=(vs_f+vmax_f)/2;
            acc_0=sqrt(fabs(v_f-v0)*jerk_0);
            if(v_f>=v0) { jerk0=jerk_0;acc0=acc_0; }
            else { jerk0=(-1)*jerk_0; acc0=(-1)*acc_0;}
            s0=(v0+v_f)*(acc0/jerk0)+(v0+v_f)/2*(v_f-v0-pow(acc0,2)/jerk0)/acc0;

            acc_1=sqrt(fabs(v_f-ve)*jerk_1);
            if(v_f>=ve){ jerk1=(-1)*jerk_1; acc1=(-1)*acc_1;}
            else{ jerk1=jerk_1; acc1=acc_1; }
            s1=(ve+v_f)*(acc1/jerk1)+(ve+v_f)/2*(ve-v_f-pow(acc1,2)/jerk1)/acc1;
            while(fabs(s0+s1)>fabs(cmd_pos) || fabs(cmd_pos)-fabs(s0+s1)>binaryOffsetDis){
                if(fabs(s0+s1)>fabs(cmd_pos)){
                    vmax_f=v_f;
                    v_f=(vs_f+vmax_f)/2;
                    acc_0=sqrt(fabs(v_f-v0)*jerk_0);
                    if(v_f>=v0) { jerk0=jerk_0;acc0=acc_0; }
                    else { jerk0=(-1)*jerk_0; acc0=(-1)*acc_0;}
                    s0=(v0+v_f)*(acc0/jerk0)+(v0+v_f)/2*(v_f-v0-pow(acc0,2)/jerk0)/acc0;

                    acc_1=sqrt(fabs(v_f-ve)*jerk_1);
                    if(v_f>=ve){ jerk1=(-1)*jerk_1; acc1=(-1)*acc_1;}
                    else{ jerk1=jerk_1; acc1=acc_1; }
                    s1=(ve+v_f)*(acc1/jerk1)+(ve+v_f)/2*(ve-v_f-pow(acc1,2)/jerk1)/acc1;
                }
                else if(fabs(cmd_pos)-fabs(s0+s1)>binaryOffsetDis){
                    vs_f=v_f;
                    v_f=(vs_f+vmax_f)/2;
                    acc_0=sqrt(fabs(v_f-v0)*jerk_0);//if((pow(acc_0,2)/fabs(jerk0))>fabs(v_f-v0))
                    if(v_f>=v0) { jerk0=jerk_0;acc0=acc_0; }
                    else { jerk0=(-1)*jerk_0; acc0=(-1)*acc_0;}
                    s0=(v0+v_f)*(acc0/jerk0)+(v0+v_f)/2*(v_f-v0-pow(acc0,2)/jerk0)/acc0;

                    acc_1=sqrt(fabs(v_f-ve)*jerk_1);//if((pow(acc_1,2)/fabs(jerk1))>fabs(ve-v_f))
                    if(v_f>=ve){ jerk1=(-1)*jerk_1; acc1=(-1)*acc_1;}
                    else{ jerk1=jerk_1; acc1=acc_1; }
                    s1=(ve+v_f)*(acc1/jerk1)+(ve+v_f)/2*(ve-v_f-pow(acc1,2)/jerk1)/acc1;
                }
            }
            Vmax=v_f;
            t4=(fabs(cmd_pos)-fabs(s0+s1))/Vmax;

        }
        else t4=(fabs(cmd_pos)-fabs(s0+s1))/fabs(Vmax);

        t1=t3=acc0/jerk0;
        t5=t7=acc1/jerk1;
        t2=(Vmax-v0-pow(acc0,2)/jerk0)/acc0;
        t6=(ve-Vmax-pow(acc1,2)/jerk1)/acc1;
        get_note();
    }
    else{
        int moveDir = (cmd_pos >= 0) ? 1 : -1;
        int deltaT_cnt=fabs(cmd_pos)/deltaT/Vmax;
        if(deltaT_cnt==0)deltaT_cnt=1;
        motion_time[7]=deltaT_cnt*deltaT;
        fixDis=(cmd_pos-moveDir*Vmax*motion_time[7])/motion_time[7];
    }
}

void MotionPlan_S::get_note()
{
    motion_time[0]=0;
    motion_time[1]=t1;
    motion_time[2]=t1+t2;
    motion_time[3]=t1+t2+t3;
    motion_time[4]=t1+t2+t3+t4;
    motion_time[5]=t1+t2+t3+t4+t5;
    motion_time[6]=t1+t2+t3+t4+t5+t6;
    motion_time[7]=t1+t2+t3+t4+t5+t6+t7;

    motion_v[0]=v0;
    motion_v[1]=motion_v[0]+0.5*jerk0*pow(t1,2);
    motion_v[2]=motion_v[1]+acc0*t2;
    motion_v[3]=motion_v[2]+acc0*t3-0.5*jerk0*pow(t3,2);
    motion_v[4]=motion_v[3];
    motion_v[5]=motion_v[4]+0.5*jerk1*pow(t5,2);
    motion_v[6]=motion_v[5]+acc1*t6;
    motion_v[7]=motion_v[6]+acc1*t7-0.5*jerk1*pow(t7,2);//motion_v[6]+acc1*t7-0.5*jerk1*pow(t1,2)

    int moveDir= (cmd_pos>=0) ? 1 : -1;
    motion_dis[0]=0;
    motion_dis[1]=motion_dis[0]+moveDir*(motion_v[0]*t1+jerk0*pow(t1,3)*1.0/6.0);
    motion_dis[2]=motion_dis[1]+moveDir*(((motion_v[1]+motion_v[2])/2*t2));
    motion_dis[3]=motion_dis[2]+moveDir*(motion_v[2]*t3+0.5*acc0*pow(t3,2)-jerk0*pow(t3,3)*1.0/6.0);
    motion_dis[4]=motion_dis[3]+moveDir*(motion_v[3]*t4);
    motion_dis[5]=motion_dis[4]+moveDir*(motion_v[4]*t5+1.0/6.0*jerk1*pow(t5,3));
    motion_dis[6]=motion_dis[5]+moveDir*(((motion_v[5]+motion_v[6])/2*t6));
    motion_dis[7]=motion_dis[6]+moveDir*(motion_v[6]*t7+0.5*acc1*pow(t7,2)-jerk1*pow(t7,3)*1.0/6.0);//motion_dis[6]+motion_v[6]*t7+0.5*acc1*pow(t7,2)-1.0/3*jerk1*pow(t7,3)
    int deltaT_cnt=motion_time[7]/deltaT;
    motion_time[7]=deltaT_cnt*deltaT;
    double t=motion_time[7]-motion_time[6];
    double actual_dis=motion_dis[6]+moveDir*(motion_v[6]*t+0.5*acc1*pow(t,2)-jerk1*pow(t,3)*1.0/6.0);
    fixDis=(cmd_pos-actual_dis)/(motion_time[7]);
}

void MotionPlan_S::get_dis()
{
    int moveDir= (cmd_pos>=0) ? 1 : -1;
    if(can_do_path){
        if(recordTime>=motion_time[0] && recordTime<motion_time[1]){
            double t=recordTime-motion_time[0];
            vs=motion_v[0]+0.5*jerk0*pow(t,2);
            val_Auv=jerk0*t;
            cur_move_pos=moveDir*(motion_v[0]*t+1.0/6.0*jerk0*pow(t,3));
        }
        else if(recordTime>=motion_time[1] && recordTime<motion_time[2]){
            double t=recordTime-motion_time[1];
            vs=motion_v[1]+acc0*t;
            val_Auv=acc0;
            cur_move_pos=motion_dis[1]+moveDir*(motion_v[1]*t+0.5*acc0*pow(t,2));
        }
        else if(recordTime>=motion_time[2] && recordTime<motion_time[3]){
            double t=recordTime-motion_time[2];
            vs=motion_v[2]+acc0*t-0.5*jerk0*pow(t,2);
            val_Auv=acc0-jerk0*t;
            cur_move_pos=motion_dis[2]+moveDir*(motion_v[2]*t+0.5*acc0*pow(t,2)-1.0/6.0*jerk0*pow(t,3));

        }
        else if(recordTime>=motion_time[3] && recordTime<motion_time[4]){
            double t=recordTime-motion_time[3];
            vs=motion_v[3];
            val_Auv=0;
            cur_move_pos=motion_dis[3]+moveDir*(motion_v[3]*t);
        }
        else if(recordTime>=motion_time[4] && recordTime<motion_time[5]){
            double t=recordTime-motion_time[4];
            vs=motion_v[4]+0.5*jerk1*pow(t,2);
            val_Auv=jerk1*t;
            cur_move_pos=motion_dis[4]+moveDir*(motion_v[4]*t+1.0/6.0*jerk1*pow(t,3));
        }
        else if(recordTime>=motion_time[5] && recordTime<motion_time[6]){
            double t=recordTime-motion_time[5];
            vs=motion_v[5]+acc1*t;
            val_Auv=acc1;
            cur_move_pos=motion_dis[5]+moveDir*(motion_v[5]*t+0.5*acc1*pow(t,2));
        }
        else if(recordTime>=motion_time[6] && recordTime<=motion_time[7]){
            double t=recordTime-motion_time[6];
            vs=motion_v[6]+acc1*t-0.5*jerk1*pow(t,2);
            val_Auv=acc1-jerk1*t;
            cur_move_pos=motion_dis[6]+moveDir*(motion_v[6]*t+0.5*acc1*pow(t,2)-1.0/6.0*jerk1*pow(t,3));
        }

    }
    else{
        cur_move_pos=moveDir*Vmax*recordTime;
    }
    cur_move_pos+=(fixDis*recordTime);
}

bool MotionPlan_S::dis_cal_time(double &dis,double &cal_time)//根据位移计算所花时间以及实际位移
{
    if(!judge_path_condition()){
        dis=0;cal_time=0;
        return false;
    }
    double cmdPos_M=cmd_pos,v0_M=v0,Vmax_M=Vmax,ve_M=ve,maxAcc_M=maxAcc,Jerk_M=Jerk,deltaT_M=deltaT,maxProportion_M=maxProportion;
    get_time_point();
    double dis_tmp=0,time_tmp=0;
    while(path_busy){
        if(recordTime+deltaT>=motion_time[7]){
            recordTime=motion_time[7];
            cur_move_pos=cmd_pos;
            dis=cur_move_pos;
            cal_time=recordTime;
            ini_path_data(cmdPos_M,v0_M,Vmax_M,ve_M,maxAcc_M,Jerk_M,deltaT_M,maxProportion_M);
            return true;
        }
        else{
            recordTime+=deltaT;
            get_dis();
        }
        if(fabs(cur_move_pos)>=fabs(dis)){
            if(fabs(cur_move_pos-dis)<=fabs(cur_move_pos-dis_tmp)){
                dis=cur_move_pos;
                cal_time=recordTime;
            }
            else{
                dis=dis_tmp;
                cal_time=time_tmp;
            }
            ini_path_data(cmdPos_M,v0_M,Vmax_M,ve_M,maxAcc_M,Jerk_M,deltaT_M,maxProportion_M);
            return true;
        }
        else{
            dis_tmp=cur_move_pos;
            time_tmp=recordTime;
        }
    }
}

bool MotionPlan_S::dis_cal_time(double &dis)//根据位移计算所花时间以及实际位移
{
    if(!judge_path_condition()){
        dis=0;
        return false;
    }
    double cmdPos_M=cmd_pos,v0_M=v0,Vmax_M=Vmax,ve_M=ve,maxAcc_M=maxAcc,Jerk_M=Jerk,deltaT_M=deltaT,maxProportion_M=maxProportion;
    get_time_point();
    double dis_tmp=0;
    while(path_busy){
        if(recordTime+deltaT>=motion_time[7]){
            recordTime=motion_time[7];
            cur_move_pos=cmd_pos;
            dis=cur_move_pos;
            ini_path_data(cmdPos_M,v0_M,Vmax_M,ve_M,maxAcc_M,Jerk_M,deltaT_M,maxProportion_M);
            return true;
        }
        else{
            recordTime+=deltaT;
            get_dis();
        }
        if(fabs(cur_move_pos)>=fabs(dis)){
            if(fabs(cur_move_pos-dis)<=fabs(cur_move_pos-dis_tmp)){
                dis=cur_move_pos;
            }
            else{
                dis=dis_tmp;
            }
            ini_path_data(cmdPos_M,v0_M,Vmax_M,ve_M,maxAcc_M,Jerk_M,deltaT_M,maxProportion_M);
            return true;
        }
        else{
            dis_tmp=cur_move_pos;
        }
    }
}

bool MotionPlan_S::get_all_time(double &allTime)
{
    if(!judge_path_condition()){
        allTime=0;
        return false;
    }
    double cmdPos_M=cmd_pos,v0_M=v0,Vmax_M=Vmax,ve_M=ve,maxAcc_M=maxAcc,Jerk_M=Jerk,deltaT_M=deltaT,maxProportion_M=maxProportion;
    get_time_point();
    allTime=motion_time[7];
    ini_path_data(cmdPos_M,v0_M,Vmax_M,ve_M,maxAcc_M,Jerk_M,deltaT_M,maxProportion_M);
    return true;
}


//                            QString params_FILE_PATH ="C:/Users/llmachine/Desktop/axis_data.txt";
//                            QFile file(params_FILE_PATH);
//                            if(file.open(QIODevice::Append | QIODevice::Text)){
//                                QTextStream out(&file);
//                                out<<QString::number(cur_move_pos)<<","<<QString::number(vs)<<","<<QString::number(val_Auv)<<"\n";
//                                file.flush();
//                                file.close();
//                            }


//                        double current_vs= vs+0.5*(val_Auv + val_Auv-(Jerk*dec_jerk_muti) * deltaT) * deltaT;
//                        double current_acc=val_Auv-(Jerk*dec_jerk_muti) * deltaT;
//                        if(current_vs-0.5*(Jerk*dec_jerk_muti)*pow(current_acc/(Jerk*dec_jerk_muti),2)>ve){
//                            cur_move_pos=moveDir *(vs*deltaT+0.5*val_Auv*pow(deltaT,2)-1.0/6*(Jerk*dec_jerk_muti)*pow(deltaT,3));
//                            vs+=0.5*(val_Auv + val_Auv-(Jerk*dec_jerk_muti) * deltaT) * deltaT;
//                            val_Auv -= (Jerk*dec_jerk_muti) * deltaT;
//                        }
//                        else{
//                            current_vs= vs+val_Auv * deltaT;
//                            current_acc=val_Auv;
//                            if(current_vs-0.5*(Jerk*dec_jerk_muti)*pow(current_acc/(Jerk*dec_jerk_muti),2)>ve){
//                                cur_move_pos=moveDir *(vs*deltaT+0.5*val_Auv*pow(deltaT,2));
//                                vs+=val_Auv * deltaT;
//                            }
//                            else{
//                                cur_move_pos=moveDir *(vs*deltaT+0.5*val_Auv*pow(deltaT,2)+1.0/6*(Jerk*dec_jerk_muti)*pow(deltaT,3));
//                                vs+=0.5*(val_Auv + val_Auv+(Jerk*dec_jerk_muti) * deltaT) * deltaT;
//                                val_Auv += (Jerk*dec_jerk_muti) * deltaT;
//                            }
//                        }
