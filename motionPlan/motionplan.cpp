#include "motionplan.h"

void motionPlan::ini_path_data(const pathInitData &pathInit)
{
    cmd_pos=pathInit.cmd_pos; remaining_pos=pathInit.cmd_pos;;
    v0=fabs(pathInit.v0); Vmax=fabs(pathInit.VMax); ve=fabs(pathInit.ve);
    maxAcc=fabs(pathInit.max_acc); Jerk=fabs(pathInit.Jerk);
    deltaT=fabs(pathInit.deltaT); maxProportion=fabs(pathInit.maxProportion); dec_jerk_muti=fabs(pathInit.dec_jerk_muti);

    path_busy=1;dec_move_pos=0;
    dec_finished=false;path_pause_cmd=false;
    can_do_path=false;judge_path=false;

    extra_pos=0;per_add_pos=0; vs=v0; val_Auv=0;
    preView_phase=0;motion_phase=0;
    acc_t3=0;even_t0=0;dec_t1=0;dec_t2=0;dec_t3=0;
    cur_move_pos=0; recordTime=0;
}

bool motionPlan::pathBusy()//判断是否在规划中
{
    if(path_busy) return true;
    else return false;
}

bool motionPlan::judge_path_condition()//规划前进行初始条件判断
{
    if(path_busy==0)return false;
    if(fabs(Vmax*deltaT/cmd_pos)>maxProportion)Vmax=fabs(maxProportion*cmd_pos/deltaT);
    if(Vmax<qMax(vs,ve))
        Vmax=qMax(vs,ve);
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

bool motionPlan::performPath(bool path_pause)//执行规划
{
    if(!judge_path){//进行S型判断
        if(!judge_path_condition())return false;
        judge_path=true;
    }
    int path_cmd=0;
    if(can_do_path){//S型规划模式
        if(!path_pause_cmd || dec_finished) path_pause_cmd=path_pause;
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
                            cmd_pos=remaining_pos; vs=dec_ve; val_Auv=0;
                            preView_phase=0; motion_phase=0;
                            can_do_path=false; judge_path=false;
                            extra_pos=0; per_add_pos=0;
                            acc_t3=0; even_t0=0; dec_t1=0; dec_t2=0; dec_t3=0;
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

void motionPlan::doMotionPlan(bool doPath)//进入运动模式
{
    if(doPath){
        switch(motion_phase){
            case 0:{
                switch(preView_phase){
                    case 0:{//加加速运动
                        if(preView(positiveAcceleration))
                            movePos(positiveAcceleration);
                        else{
                            if(preView(steadyAcceleration)){
                                movePos(steadyAcceleration);
                                preView_phase=1;
                            }
                            else{
                                preFicure();
                                if(acc_t3==0) cur_move_pos=0;
                                else{
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
                        if(preView(steadyAcceleration))
                            movePos(steadyAcceleration);
                        else{
                            preFicure();
                            if(acc_t3==0) cur_move_pos=0;
                            else{
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
                    movePos(steadyAcceleration);
                    even_t0--;
                }
                if(even_t0==0) motion_phase++;
            }break;
            case 2:{//加减速运动
                if(dec_t1>0){
                    movePos(positiveAcceleration);
                    dec_t1-=1;
                }
                else if(dec_t1<0){
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
                    movePos(steadyAcceleration);
                    dec_t2--;
                }
                if(dec_t2==0) motion_phase++;
            }break;
            case 4:{//减减速运动
                if(dec_t3>0){
                    movePos(negetiveAcceleration);
                    dec_t3-=1;
                }
                else if(dec_t3<0){
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

bool motionPlan::preView(enumType motionType)//前瞻下一时刻
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

void motionPlan::preFicure()//全部前瞻完,结束前瞻
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

void motionPlan::movePos(enumType motionType)//下一时刻运动计算
{
    int moveDir = (remaining_pos >= 0) ? 1 : -1;
    switch (motionType){
    case negetiveAcceleration:{//加速度绝对值减小
        cur_move_pos=moveDir *(vs*deltaT+0.5*val_Auv*pow(deltaT,2)-1.0/6.0*Jerk*pow(deltaT,3));
        vs+=0.5*(val_Auv + val_Auv-Jerk * deltaT) * deltaT;
        val_Auv -= Jerk * deltaT;
    }break;
    case steadyAcceleration:{//加速度不变
        cur_move_pos=moveDir *(vs*deltaT+0.5*val_Auv*pow(deltaT,2));
        vs+=val_Auv * deltaT;
        val_Auv = val_Auv;
    }break;
    case positiveAcceleration:{//加速度绝对值增大
        cur_move_pos=moveDir *(vs*deltaT+0.5*val_Auv*pow(deltaT,2)+1.0/6.0*Jerk*pow(deltaT,3));
        vs+=0.5*(val_Auv + val_Auv+Jerk * deltaT) * deltaT;
        val_Auv += Jerk * deltaT;
    }break;}
    recordTime+=deltaT;
}

void motionPlan::get_move_proportion(double &curProportion)//获得当前所走总位移占全程位移的比例
{
    if(fabs(cmd_pos+dec_move_pos)<=coverAccuracy) curProportion=0;
    else curProportion=curProportion=(cmd_pos-remaining_pos+dec_move_pos)/(cmd_pos+dec_move_pos);

}

void motionPlan::get_move_msg(double &curPos)//获得当前所走总位移
{
    curPos=cmd_pos-remaining_pos+dec_move_pos;
}

void motionPlan::get_move_msg(double &curPos,double &curTime)//获得当前所走总位移
{
    curPos=cmd_pos-remaining_pos+dec_move_pos;
    curTime=recordTime;
}

void motionPlan::get_move_msg(double &curPos_m,double &curVs_m, double &curAcc_m)//获得当前所走总位移、速度、加速度
{
    curPos_m=cur_move_pos;
    curVs_m=vs;
    curAcc_m=val_Auv;
}

void motionPlan::get_move_msg(double &curPos_m,double &curVs_m, double &curAcc_m,double &curTime_m)//获得当前所走总位移、速度、加速度、时间
{
    curPos_m=cmd_pos-remaining_pos+dec_move_pos;
    curVs_m=vs;
    curAcc_m=val_Auv;
    curTime_m=recordTime;
}

bool motionPlan::get_fixed_spd(double &fixedVe,double &fixedVmax)//获得修正的末速度
{
    if(!judge_path_condition()){
        fixedVe=0;fixedVmax=0;
        return false;
    }
    pathInitData pathInitTmp;
    pathInitTmp.cmd_pos=cmd_pos; pathInitTmp.VMax=Vmax;
    fixedVmax=0;
    while(path_busy){
        doMotionPlan(can_do_path);
        if(fixedVmax<vs)fixedVmax=vs;
    }
    fixedVe=vs;
    ini_path_data(pathInitTmp);
    return true;
}

bool motionPlan::get_all_time(double &allTime)//获得走完全程所花时间
{
    if(!judge_path_condition()){
        allTime=0;
        return false;
    }
    pathInitData pathInitTmp;
    pathInitTmp.cmd_pos=cmd_pos; pathInitTmp.VMax=Vmax;
    while(path_busy)
        doMotionPlan(can_do_path);
    allTime=recordTime;
    ini_path_data(pathInitTmp);
    return true;
}

bool motionPlan::dis_cal_time(double &dis,double &cal_time)//根据位移计算所花时间以及实际位移
{
    if(!judge_path_condition()){
        dis=0;cal_time=0;
        return false;//未初始化
    }
    pathInitData pathInitTmp;
    pathInitTmp.cmd_pos=cmd_pos; pathInitTmp.VMax=Vmax;
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
    ini_path_data(pathInitTmp);
    return true;
}

bool motionPlan::dis_cal_time(double &dis)//根据位移计算所花时间以及实际位移
{
    if(!judge_path_condition()){
        dis=0;
        return false;//未初始化
    }
    pathInitData pathInitTmp;
    pathInitTmp.cmd_pos=cmd_pos; pathInitTmp.VMax=Vmax;
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
    ini_path_data(pathInitTmp);
    return true;
}
