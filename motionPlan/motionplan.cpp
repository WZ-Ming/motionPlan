#include "motionplan.h"

motionPlan::motionPlan()
{

}

motionPlan::~motionPlan()
{

}

void motionPlan::ini_path_data(const pathInitData &pathInit)
{
    cmd_pos=pathInit.cmd_pos; remaining_pos=pathInit.cmd_pos;;
    v0=fabs(pathInit.v0); Vmax=fabs(pathInit.VMax); ve=fabs(pathInit.ve);
    maxAcc=fabs(pathInit.max_acc); Jerk=fabs(pathInit.Jerk);
    deltaT=fabs(pathInit.deltaT);

    path_busy=true; can_do_path=false;
    decInit=false; dec_finished=false; path_pause_cmd=false;

    vs=v0; val_Auv=0;
    preView_phase=0; preViewDone=false;

    dec_move_pos=0; cur_move_pos=0; recordTime=0;

    ficureTime[0]=0; acc_t=0;
    moveDir = (cmd_pos >= 0) ? 1 : -1;
    judge_path_condition();
#ifdef motionDebug
    motionStatusMsg.clear();
#endif
}

bool motionPlan::pathBusy()//判断是否在规划中
{
    if(path_busy) return true;
    else return false;
}

void motionPlan::judge_path_condition()//规划前进行初始条件判断
{
    if(fabs(Vmax*deltaT/cmd_pos)>maxProportion)
        Vmax=fabs(maxProportion*cmd_pos/deltaT);
    if(Vmax<qMax(vs,ve))
        Vmax=qMax(vs,ve);
    if(fabs(ve-vs)>coverAccuracy){//初速度和末速度不相等的情况下
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
            double moveDis=0,cal_maxAcc=0,t1_tmp=0,t2_tmp=0,cal_vmax=0;
            get_time_point(ve,cal_vmax,cal_maxAcc,t1_tmp,t2_tmp,moveDis);
            if(fabs(moveDis)>fabs(cmd_pos)){//需要修正末速度
                double veTmp=ve,vsTmp=vs,vCenter=ve,minus_dis=0;
                bool fixFinished=false;
                while(!fixFinished){
                    vCenter=(veTmp+vsTmp)/2;
                    get_time_point(vCenter,cal_vmax,cal_maxAcc,t1_tmp,t2_tmp,moveDis);
                    if(fabs(minus_dis-(moveDis-fabs(cmd_pos)))<coverAccuracy)
                        fixFinished=true;
                    else
                        minus_dis=moveDis-fabs(cmd_pos);
                    if(fixFinished){
                        ve=vCenter;//修正的末速度
                        get_ficure_point(vs,vCenter,t1_tmp,t2_tmp);
                        preViewDone=true;
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
        double cal_minDis=(vs+0.5*Jerk*pow(deltaT,2))*4.0*deltaT;
        double cal_vmin=vs+Jerk*pow(deltaT,2);
        if(cal_minDis>fabs(cmd_pos) || cal_vmin>Vmax || Jerk*deltaT>maxAcc){//距离太短或最大加速度太小或最大速度太小，退出S型速度规划
            can_do_path=false;
        }
        else can_do_path=true;//可以S型速度规划
    }
    if(can_do_path) emit sendMsg(tr("开始速度规划...\n参数满足7段S型速度规划条件!"));
    else  emit sendMsg(tr("开始速度规划...\n参数无法满足7段S型速度规划条件!\n被迫进行匀速运动!"));
}

bool motionPlan::performPath(bool path_pause)//执行规划
{
    if(!path_busy){
        emit sendMsg(tr("没初始化参数,无法进行计算,请先初始化参数!"));
        return false;
    }
    if(can_do_path){//S型规划模式
        if(!path_pause_cmd || dec_finished)
            path_pause_cmd=path_pause;
        if(dec_finished){
            if(path_pause_cmd)
                return false;
            else
                dec_finished=false;
        }
        else{
            if(path_pause_cmd & !decInit){
                if(val_Auv>0){//减速初始化
                    double cal_vmax,cal_maxAcc,t1_tmp,t2_tmp,moveDis;
                    get_time_point(dec_ve,cal_vmax,cal_maxAcc,t1_tmp,t2_tmp,moveDis);
                    get_ficure_point(cal_vmax,dec_ve,t1_tmp,t2_tmp);
                    preViewDone=false;
                    preView_phase=2;
                }
                decInit=true;
            }
        }
    }
    else{//匀速模式
        path_pause_cmd=path_pause;
        if(path_pause_cmd) return false;
    }
    doMotionPlan(can_do_path);
    return true;
}

void motionPlan::doMotionPlan(bool doPath)//进入运动模式
{
#ifdef motionDebug
    static QString motionMsg;
    static bool canSendMsg=false;
    canSendMsg=false;
#endif
    static bool haveMove=false;
    if(doPath){
        if(!preViewDone){
            haveMove=false;
            switch(preView_phase){
                case 0:{//加加速运动
                    if(preView(positiveAcceleration)){
#ifdef motionDebug
                        if(motionStatusMsg!=tr("开始加加速运动阶段:\n")){
                            motionMsg=tr("开始加加速运动阶段:\n");
                            canSendMsg=true;
                        }
#endif
                        movePreViewPos(positiveAcceleration);
                    }
                    else{
                        if(preView(steadyAcceleration)){
#ifdef motionDebug
                            if(motionStatusMsg!=tr("开始匀速运动阶段:\n")){
                                motionMsg=tr("开始匀速运动阶段:\n");
                                canSendMsg=true;
                            }
#endif
                            movePreViewPos(steadyAcceleration);
                            preView_phase=1;
                        }
                        else{
                            preFicure();
                            if(acc_t!=0){
#ifdef motionDebug
                                if(motionStatusMsg!=tr("开始减加速运动阶段:\n")){
                                    motionMsg=tr("开始减加速运动阶段:\n");
                                    canSendMsg=true;
                                }
#endif
                                haveMove=true;
                                movePreViewPos(negetiveAcceleration);
                                acc_t--;
                            }
                            if(acc_t==0) preViewDone=true;
                            else preView_phase=2;
                            if(!haveMove){
                                if(acc_t==0) goto lastMove;
                                else goto decAcc;
                            }
                        }
                    }
                }break;
                case 1:{//匀加速运动
                    if(preView(steadyAcceleration)){
#ifdef motionDebug
                        if(fabs(val_Auv)<coverAccuracy){
                            if(motionStatusMsg!=tr("开始匀速运动阶段:\n")){
                                motionMsg=tr("开始匀加速运动阶段:\n");
                                canSendMsg=true;
                            }
                        }
                        else{
                             if(motionStatusMsg!=tr("开始匀加速运动阶段:\n")){
                                motionMsg=tr("开始匀加速运动阶段:\n");
                                canSendMsg=true;
                             }
                        }
#endif
                        movePreViewPos(steadyAcceleration);
                    }
                    else{
                        preFicure();
                        if(acc_t!=0){
#ifdef motionDebug
                            if(motionStatusMsg!=tr("开始减加速运动阶段:\n")){
                                motionMsg=tr("开始减加速运动阶段:\n");
                                canSendMsg=true;
                            }
#endif
                            haveMove=true;
                            movePreViewPos(negetiveAcceleration);
                            acc_t--;
                        }
                        if(acc_t==0) preViewDone=true;
                        else preView_phase=2;
                        if(!haveMove){
                            if(acc_t==0) goto lastMove;
                            else goto decAcc;
                        }
                    }
                }break;
        decAcc: case 2:{//减加速运动
                    if(acc_t!=0){
#ifdef motionDebug
                        if(motionStatusMsg!=tr("开始减加速运动阶段:\n")){
                            motionMsg=tr("开始减加速运动阶段:\n");
                            canSendMsg=true;
                        }
#endif
                        haveMove=true;
                        movePreViewPos(negetiveAcceleration);
                        acc_t--;
                    }
                    if(acc_t==0) preViewDone=true;
                    if(!haveMove && acc_t==0)
                        goto lastMove;
                }break;
            }
        }
        else{
    lastMove:
            static double timeTmp=0;
            static bool haveData=false;
            recordTime+=deltaT;
            if(recordTime-ficureTimeRecord>ficureTime[3])
                haveData=false;
            else
                haveData=true;
            if(!haveData)
                recordTime=ficureTime[3]+ficureTimeRecord;

            if(recordTime-ficureTimeRecord>ficureTime[2]){
#ifdef motionDebug
                if(ficureMoveDir>0){
                    if(motionStatusMsg!=tr("开始减减运动阶段:\n")){
                        motionMsg=tr("开始减减运动阶段:\n");
                        canSendMsg=true;
                    }
                }
                else{
                    if(motionStatusMsg!=tr("开始减加运动阶段:\n")){
                        motionMsg=tr("开始减加运动阶段:\n");
                        canSendMsg=true;
                    }
                }
#endif
                timeTmp=recordTime-ficureTimeRecord-ficureTime[2];
                moveFicurePos(positiveAcceleration,timeTmp);
            }
            else if(recordTime-ficureTimeRecord>ficureTime[1]){
#ifdef motionDebug
                if(val_Auv>0){
                    if(motionStatusMsg!=tr("开始匀加运动阶段:\n")){
                        motionMsg=tr("开始匀加运动阶段:\n");
                        canSendMsg=true;
                    }
                }
                else{
                    if(motionStatusMsg!=tr("开始匀减运动阶段:\n")){
                        motionMsg=tr("开始匀减运动阶段:\n");
                        canSendMsg=true;
                    }
                }
#endif
                timeTmp=recordTime-ficureTimeRecord-ficureTime[1];
                moveFicurePos(steadyAcceleration,timeTmp);
            }
            else if(recordTime-ficureTimeRecord>ficureTime[0]){
#ifdef motionDebug
                if(ficureMoveDir>0){
                    if(motionStatusMsg!=tr("开始加减运动阶段:\n")){
                        motionMsg=tr("开始加减运动阶段:\n");
                        canSendMsg=true;
                    }
                }
                else{
                    if(motionStatusMsg!=tr("开始加加运动阶段:\n")){
                        motionMsg=tr("开始加加运动阶段:\n");
                        canSendMsg=true;
                    }
                }
#endif
                timeTmp=recordTime-ficureTimeRecord-ficureTime[0];
                moveFicurePos(negetiveAcceleration,timeTmp);
            }
            else{
#ifdef motionDebug
                if(motionStatusMsg!=tr("开始匀速运动阶段:\n")){
                    motionMsg=tr("开始匀速运动阶段:\n");
                    canSendMsg=true;
                }
#endif
                timeTmp=recordTime-ficureTimeRecord;
                cur_move_pos=moveDir *vs*timeTmp;
            }
            cur_move_pos-=recordDis;
            recordDis+=cur_move_pos;
            if(!haveData){
                if(path_pause_cmd && fabs(remaining_pos-cur_move_pos)>coverAccuracy){
                    dec_finished=true;
                    decInit=false;
                    dec_move_pos+=(cmd_pos-remaining_pos+cur_move_pos);
                    cmd_pos=remaining_pos-cur_move_pos;
                    v0=vs;
                    val_Auv=0;
                    preView_phase=0;
                    preViewDone=false;
                    acc_t=0;
                    ficureTime[0]=0;
                    judge_path_condition();
                }
                else
                    path_busy=false;
            }
        }
    }
    else{
        if((moveDir*Vmax*deltaT)/(cmd_pos+dec_move_pos)>maxProportion)
            cur_move_pos=maxProportion*(cmd_pos+dec_move_pos);
        else
            cur_move_pos=moveDir*Vmax*deltaT;
        recordTime+=deltaT;
        if(fabs(remaining_pos)<fabs(cur_move_pos)){
            cur_move_pos=remaining_pos;
            path_busy=false;
        }
    }
    remaining_pos -= cur_move_pos;
#ifdef motionDebug
    if(canSendMsg){
        motionStatusMsg=motionMsg;
        get_move_msg(motionMsg);
        emit sendMsg(motionMsg);
    }
    if(!path_busy){
        if(motionStatusMsg!=tr("规划完成:\n")){
            motionMsg=tr("规划完成:\n");
            motionStatusMsg=motionMsg;
            get_move_msg(motionMsg);
            emit sendMsg(motionMsg);
        }
    }
#endif
}

bool motionPlan::preView(enumType motionType)//前瞻下一时刻
{
    static double current_vs=0,current_acc=0,cal_vmax=0,move_dis=0;
    switch (motionType) {
        case steadyAcceleration:{//加速度不变
            move_dis=moveDir *(vs*deltaT+0.5*val_Auv*pow(deltaT,2));
            current_vs= vs+ val_Auv * deltaT;
            current_acc=val_Auv;
        }break;
        case positiveAcceleration:{//加速度增大
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
        cal_maxAcc=(static_cast<int>((cal_maxAcc+coverAccuracy)/(Jerk*deltaT)))*Jerk*deltaT;
        double t1_tmp=cal_maxAcc/Jerk;
        double t2_tmp=0;
        if(fabs(cal_maxAcc)>coverAccuracy)
            t2_tmp=(fabs(cal_vmax-ve)-pow(cal_maxAcc,2)/Jerk)/cal_maxAcc;
        double moveDis=(ve+cal_vmax)*t1_tmp+(cal_vmax+ve)*t2_tmp/2;
        if(fabs(moveDis)<=fabs(cur_remaining_pos)) return true;
        else return false;
    }
}

void motionPlan::preFicure()//全部前瞻完,结束前瞻
{
    double cal_acc_dis=moveDir*(vs*val_Auv/Jerk+0.5*val_Auv*pow(val_Auv/Jerk,2)-Jerk*pow(val_Auv/Jerk,3)*1.0/6.0);
    double moveDis,cal_vmax,cal_maxAcc,t1_tmp,t2_tmp;
    get_time_point(ve,cal_vmax,cal_maxAcc,t1_tmp,t2_tmp,moveDis);
    ficureTime[0]=fabs(moveDis-fabs(remaining_pos-cal_acc_dis))/cal_vmax;
    get_ficure_point(cal_vmax,ve,t1_tmp,t2_tmp);
}

void motionPlan::get_time_point(const double vEnd,double &cal_vmax,double &cal_maxAcc,double &t1_tmp,double &t2_tmp,double &moveDis)//获取到达最大速度的时间,最大加速度,以及后半程各阶段时间
{
    acc_t=static_cast<int>((val_Auv+coverAccuracy)/Jerk/deltaT);
    cal_vmax=vs+val_Auv*val_Auv/Jerk-0.5*Jerk*pow(val_Auv/Jerk,2);
    cal_maxAcc=sqrt(fabs(cal_vmax-vEnd)*Jerk);
    if(cal_maxAcc>maxAcc)
        cal_maxAcc=maxAcc;
    cal_maxAcc=(static_cast<int>((cal_maxAcc+coverAccuracy)/(Jerk*deltaT)))*Jerk*deltaT;
    t1_tmp=cal_maxAcc/Jerk;
    t2_tmp=0;
    if(fabs(cal_maxAcc)>coverAccuracy)
        t2_tmp=(fabs(cal_vmax-vEnd)-pow(cal_maxAcc,2)/Jerk)/cal_maxAcc;
    moveDis=(cal_vmax+vEnd)*t1_tmp+(cal_vmax+vEnd)*t2_tmp/2;
}

void motionPlan::get_ficure_point(const double vBegin,const double vEnd,const double t1_tmp,const double t2_tmp)//获取后半程时间,位移,速度,加速度节点信息
{
    if(vBegin>vEnd) ficureMoveDir=1;
    else ficureMoveDir=-1;

    recordDis=0;

    ficureTimeRecord=recordTime+acc_t*deltaT;
    ficureTime[1]=t1_tmp+ficureTime[0];
    ficureTime[2]=ficureTime[1]+t2_tmp;
    ficureTime[3]=ficureTime[2]+t1_tmp;

    ficureV[0]=vBegin;
    ficureAcc[0]=0;
    ficureDis[0]=moveDir*ficureV[0]*ficureTime[0];

    ficureV[1]=ficureV[0]+0.5*(ficureAcc[0] + ficureAcc[0]-ficureMoveDir*Jerk * t1_tmp) * t1_tmp;
    ficureAcc[1]=ficureAcc[0]-ficureMoveDir*Jerk*t1_tmp;
    ficureDis[1]=ficureDis[0]+moveDir *(ficureV[0]*t1_tmp+0.5*ficureAcc[0]*pow(t1_tmp,2)-ficureMoveDir*1.0/6.0*Jerk*pow(t1_tmp,3));

    ficureV[2]=ficureV[1]+ficureAcc[1]*t2_tmp;
    ficureAcc[2]=ficureAcc[1];
    ficureDis[2]=ficureDis[1]+moveDir *(ficureV[0]+ve)*t2_tmp/2;
}

void motionPlan::moveFicurePos(enumType motionType,const double timeTmp)
{
    switch (motionType){
    case positiveAcceleration:{
        cur_move_pos=ficureDis[2]+moveDir *(ficureV[2]*timeTmp+0.5*ficureAcc[2]*pow(timeTmp,2)+ficureMoveDir*1.0/6.0*Jerk*pow(timeTmp,3));
        vs=ficureV[2]+0.5*(ficureAcc[2] + ficureAcc[2]+ficureMoveDir*Jerk * timeTmp) * timeTmp;
        val_Auv =ficureAcc[2]+ ficureMoveDir*Jerk * timeTmp;
    }break;

    case steadyAcceleration:{
        cur_move_pos=ficureDis[1]+moveDir *(ficureV[1]*timeTmp+0.5*ficureAcc[1]*pow(timeTmp,2));
        vs=ficureV[1]+ficureAcc[1] * timeTmp;
    }break;
    case negetiveAcceleration:{
        cur_move_pos=ficureDis[0]+moveDir *(ficureV[0]*timeTmp+0.5*ficureAcc[0]*pow(timeTmp,2)-ficureMoveDir*1.0/6.0*Jerk*pow(timeTmp,3));
        vs=ficureV[0]+0.5*(ficureAcc[0] + ficureAcc[0]-ficureMoveDir*Jerk * timeTmp) * timeTmp;
        val_Auv =ficureAcc[0]- ficureMoveDir*Jerk * timeTmp;
    }break;}
}

void motionPlan::movePreViewPos(enumType motionType)//下一时刻运动计算
{
    switch (motionType){
    case positiveAcceleration:{//加速度绝对值增大
        cur_move_pos=moveDir *(vs*deltaT+0.5*val_Auv*pow(deltaT,2)+1.0/6.0*Jerk*pow(deltaT,3));
        vs+=0.5*(val_Auv + val_Auv+Jerk * deltaT) * deltaT;
        val_Auv += Jerk * deltaT;
    }break;
    case steadyAcceleration:{//加速度不变
        cur_move_pos=moveDir *(vs*deltaT+0.5*val_Auv*pow(deltaT,2));
        vs+=val_Auv * deltaT;
        //val_Auv = val_Auv;
    }break;
    case negetiveAcceleration:{//加速度绝对值减小
        cur_move_pos=moveDir *(vs*deltaT+0.5*val_Auv*pow(deltaT,2)-1.0/6.0*Jerk*pow(deltaT,3));
        vs+=0.5*(val_Auv + val_Auv-Jerk * deltaT) * deltaT;
        val_Auv -= Jerk * deltaT;
    }break;}
    recordTime+=deltaT;
}

void motionPlan::get_move_msg(double &curProportion,double &curPos_m,double &curVs_m, double &curAcc_m,double &curTime_m)//获得当前所走位移占比、总位移、速度、加速度、时间
{
    if(fabs(cmd_pos+dec_move_pos)<=coverAccuracy)
        curProportion=0;
    else
        curProportion=(cmd_pos-remaining_pos+dec_move_pos)/(cmd_pos+dec_move_pos);
    curPos_m=cmd_pos-remaining_pos+dec_move_pos;
    curVs_m=vs;
    curAcc_m=val_Auv;
    curTime_m=recordTime;
}

#ifdef motionDebug
void motionPlan::get_move_msg(QString &motionMsg)
{
    motionMsg+=tr("当前位移:")+QString::number(cmd_pos-remaining_pos+dec_move_pos,'f',6)+"\n";
    motionMsg+=tr("当前速度:")+QString::number(vs)+"\n";
    motionMsg+=tr("当前加速度:")+QString::number(val_Auv)+"\n";
    motionMsg+=tr("当前时间:")+QString::number(recordTime)+"\n";
}
#endif
