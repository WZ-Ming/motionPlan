#include "llmotionecat.h"

llmotionECat::llmotionECat()
{

}

void llmotionECat::get_path_status(int &path_busy){
    path_busy=group_busy;
}

void llmotionECat::set_start_path(int start){
    user_start_path=start;
}

void llmotionECat::run(){
    run_var = true;
    group_busy = 0;
    path_pause=false;
    user_start_path=0;
    int cmd_pos[32]={0};
    double time=0;
    while (run_var) {
        if(user_start_path==1){
            if(syncNewPath(path_pause)){
                time+=0.001;
                for(int i=0;i<motion_data_pravete.size();i++){
                    int dis_val_path_Int=momsg.axis_cmd_pos[i]*((momsg.cmd_pos-momsg.remaining_pos)/momsg.cmd_pos);
                    int dis_val_path_Inc=dis_val_path_Int-momsg.dis_val_moved[i];
                    cmd_pos[i]=dis_val_path_Inc;
                    momsg.dis_val_moved[i]+=dis_val_path_Inc;
                    emit send_data(0,i,cmd_pos[i],momsg.dis_val_moved[i],time);
                    QTime timer;
                    timer.start();
                    while(timer.elapsed()<=cal_delay+cal_time){;}
                }
            }
            group_busy=momsg.path_busy;
        }
        else if(user_start_path==2){
            if(differNewPath(path_pause)){
                time+=0.001;
                for(int i=0;i<motion_data_pravete.size();i++){
                    emit send_data(0,i,unmsg[i].int_pos,unmsg[i].dis_val_moved,time);
                    QTime timer;
                    timer.start();
                    while(timer.elapsed()<=cal_delay+cal_time){;}
                }
            }
            for(int i=0;i<motion_data_pravete.size();i++){
                group_busy+=unmsg[i].path_busy;
            }
            if(group_busy>0)group_busy=1;
        }
        else time=0;
        for(int i=0;i<motion_data_pravete.size();i++){
            if(jogMode[i]==1){//------JOG模式开始---------------------------------------------//
                if(user_start_jog[i]==1)
                {
                    if(jogmsg[i].control==0)         //step1:当前未启动状态，则先配置参数
                    {
                        jogmsg[i].moveDir = (motion_data_pravete[i].Vmax >= 0) ? 1 : -1;
                        jogmsg[i].Vmax = fabs(motion_data_pravete[i].Vmax);
                        jogmsg[i].maxAuv_acc = motion_data_pravete[i].maxAuv_acc;
                        jogmsg[i].Jerk = motion_data_pravete[i].Jerk;
                        jogmsg[i].val_Auv = 0;
                        jogmsg[i].vs = 0;
                        jogmsg[i].ve = 0;
                        jogmsg[i].control=1;
                    }
                    else if(jogmsg[i].control==1)
                    {
                        double deltaT = jogmsg[i].period;//jogmsg[i].li_t - jogmsg[i].li_t_last;
                        int jog_move_pos=0;
                        double T1 = jogmsg[i].val_Auv / jogmsg[i].Jerk;//当前加速度减到0需要时间T1
                        if(jogmsg[i].vs + 0.5 * jogmsg[i].Jerk * T1 * T1 > jogmsg[i].Vmax){
                            if(jogmsg[i].val_Auv-jogmsg[i].Jerk * deltaT<0){
                                jog_move_pos=jogmsg[i].moveDir *(jogmsg[i].vs*deltaT+0.5*jogmsg[i].val_Auv*pow(deltaT,2));
                                jogmsg[i].vs+=jogmsg[i].val_Auv * deltaT;
                            }
                            else {
                                jog_move_pos=jogmsg[i].moveDir *(jogmsg[i].vs*deltaT+0.5*jogmsg[i].val_Auv*pow(deltaT,2)-1.0/6*jogmsg[i].Jerk*pow(deltaT,3));
                                jogmsg[i].vs+=0.5*(jogmsg[i].val_Auv + jogmsg[i].val_Auv-jogmsg[i].Jerk * deltaT) * deltaT;
                                jogmsg[i].val_Auv -= jogmsg[i].Jerk * deltaT;
                            }
                        }
                        else if(jogmsg[i].val_Auv < jogmsg[i].maxAuv_acc){
                            jog_move_pos=jogmsg[i].moveDir *(jogmsg[i].vs*deltaT+0.5*jogmsg[i].val_Auv*pow(deltaT,2)+1.0/6*jogmsg[i].Jerk*pow(deltaT,3));
                            jogmsg[i].vs+=0.5*(jogmsg[i].val_Auv + jogmsg[i].val_Auv+jogmsg[i].Jerk * deltaT) * deltaT;
                            jogmsg[i].val_Auv += jogmsg[i].Jerk * deltaT;
                        }
                        else{
                            jog_move_pos=jogmsg[i].moveDir *(jogmsg[i].vs*deltaT+0.5*jogmsg[i].val_Auv*pow(deltaT,2));
                            jogmsg[i].vs+=jogmsg[i].val_Auv * deltaT;
                        }

                        emit send_data(0,i,jog_move_pos,jogmsg[i].val_Auv,jogmsg[i].vs);
                        QTime timer;
                        timer.start();
                        while(timer.elapsed()<=cal_delay+cal_time){;}
                    }
                }
                else if(jogmsg[i].control==1)//------JOG模式退出后的收尾部分---------------------------------------------//
                {
                    double deltaT = jogmsg[i].period;//jogmsg[i].li_t - jogmsg[i].li_t_last;
                    int jog_move_pos=0;
                    double T1 = jogmsg[i].val_Auv / jogmsg[i].Jerk;//当前加速度减到0需要时间T1
                    if(jogmsg[i].val_Auv>=0){
                        jog_move_pos=jogmsg[i].moveDir *(jogmsg[i].vs*deltaT+0.5*jogmsg[i].val_Auv*pow(deltaT,2)-1.0/6*jogmsg[i].Jerk*pow(deltaT,3));
                        jogmsg[i].vs+=0.5*(jogmsg[i].val_Auv + jogmsg[i].val_Auv-jogmsg[i].Jerk * deltaT) * deltaT;
                        jogmsg[i].val_Auv -= jogmsg[i].Jerk * deltaT;
                    }
                    else{
                        if(jogmsg[i].vs - 0.5 * jogmsg[i].Jerk * T1 * T1 < jogmsg[i].ve){
                            jog_move_pos=jogmsg[i].moveDir *(jogmsg[i].vs*deltaT+0.5*jogmsg[i].val_Auv*pow(deltaT,2)+1.0/6*jogmsg[i].Jerk*pow(deltaT,3));
                            jogmsg[i].vs+=0.5*(jogmsg[i].val_Auv + jogmsg[i].val_Auv+jogmsg[i].Jerk * deltaT) * deltaT;
                            jogmsg[i].val_Auv += jogmsg[i].Jerk * deltaT;
                        }
                        else if(jogmsg[i].val_Auv > -jogmsg[i].maxAuv_acc){
                            jog_move_pos=jogmsg[i].moveDir *(jogmsg[i].vs*deltaT+0.5*jogmsg[i].val_Auv*pow(deltaT,2)-1.0/6*jogmsg[i].Jerk*pow(deltaT,3));
                            jogmsg[i].vs+=0.5*(jogmsg[i].val_Auv + jogmsg[i].val_Auv-jogmsg[i].Jerk * deltaT) * deltaT;
                            jogmsg[i].val_Auv -= jogmsg[i].Jerk * deltaT;
                        }
                        else{
                            jog_move_pos=jogmsg[i].moveDir *(jogmsg[i].vs*deltaT+0.5*jogmsg[i].val_Auv*pow(deltaT,2));
                            jogmsg[i].vs+=jogmsg[i].val_Auv * deltaT;
                        }
                    }
                    if(jogmsg[i].vs<0){
                        jogMode[i]=0;
                        jogmsg[i].control=0;
                    }
                    else{
                        emit send_data(0,i,jog_move_pos,jogmsg[i].val_Auv,jogmsg[i].vs);
                    }
                    QTime timer;
                    timer.start();
                    while(timer.elapsed()<=cal_delay+cal_time){;}
                }
            }
        }
        QThread::usleep(100);
    }
    quit();
}

int llmotionECat::set_start_Jog(int axisNo,int start_Bit_In)//90,91 Jog方式启动制定轴
{
    if(axisNo<0 || axisNo>31)return -1;
    user_start_jog[axisNo] = start_Bit_In;
    if(start_Bit_In == 1) jogMode[axisNo] = 1;
}

bool llmotionECat::syncNewPath(bool path_pause)
{
    static double extra_pos=0,per_add_pos=0;
    static double vs=0,ve=0,val_Auv=0;
    static int t1=0,t2=0,t3=0,t4=0;
    static int preView_phase=0,motion_phase=0;
    static bool dec_finished=false,path_pause_cmd=false;
    double cur_move_pos=0;
    const double deltaT=0.001;
    int path_cmd=0;
    if(!path_pause_cmd | dec_finished)path_pause_cmd=path_pause;
    if(momsg.path_busy){
        if(dec_finished){
            if(path_pause_cmd) path_cmd=0;
            else path_cmd=1;
        }
        else{
            if(path_pause_cmd)path_cmd=2;
            else path_cmd=1;
        }
        switch(path_cmd){
            case 1:{
                if(dec_finished){
                    dec_finished=false;
                    momsg.cmd_pos=momsg.remaining_pos;
                    for(int i=0;i<32;i++){
                        momsg.axis_cmd_pos[i]-=momsg.dis_val_moved[i];
                        momsg.dis_val_moved[i]=0;
                    }
                }
                int moveDir = (momsg.remaining_pos >= 0) ? 1 : -1;
                switch(motion_phase){
                    case 0:{
                        switch(preView_phase){
                            case 0:{
                                if(preView(momsg.cmd_pos,momsg.remaining_pos,vs,momsg.Vmax,val_Auv,momsg.maxAuv_acc,momsg.Jerk,deltaT,moveDir,1)){
                                    cur_move_pos=moveDir *(vs*deltaT+0.5*val_Auv*pow(deltaT,2)+1.0/6*momsg.Jerk*pow(deltaT,3));
                                    vs+=0.5*(val_Auv + val_Auv+momsg.Jerk * deltaT) * deltaT;
                                    val_Auv += momsg.Jerk * deltaT;
                                    t1++;
                                }
                                else{
                                    if(preView(momsg.cmd_pos,momsg.remaining_pos,vs,momsg.Vmax,val_Auv,momsg.maxAuv_acc,momsg.Jerk,deltaT,moveDir,0)){
                                        cur_move_pos=moveDir *(vs*deltaT+0.5*val_Auv*pow(deltaT,2));
                                        vs+=val_Auv * deltaT;
                                        t2++;
                                        preView_phase=1;
                                    }
                                    else{
                                        double pre_move_pos=0,pre_cal_vmax=0;
                                        int pre_t3=0;
                                        preFicure(val_Auv,vs,momsg.Jerk,deltaT,moveDir,&pre_move_pos,&pre_cal_vmax,&pre_t3);
                                        t4=(2.0*(momsg.remaining_pos-pre_move_pos)-momsg.cmd_pos)/(moveDir*pre_cal_vmax*deltaT);
                                        extra_pos=2.0*(momsg.remaining_pos-pre_move_pos)-momsg.cmd_pos-t4*moveDir*pre_cal_vmax*deltaT;
                                        per_add_pos=extra_pos/(t1+t2+2*pre_t3+t4);

                                        cur_move_pos=moveDir *(vs*deltaT+0.5*val_Auv*pow(deltaT,2)-1.0/6*momsg.Jerk*pow(deltaT,3));
                                        vs+=0.5*(val_Auv + val_Auv-momsg.Jerk * deltaT) * deltaT;
                                        val_Auv -= momsg.Jerk * deltaT;
                                        t3++;
                                        preView_phase=2;
                                    }
                                }
                            }break;
                            case 1:{
                                if(preView(momsg.cmd_pos,momsg.remaining_pos,vs,momsg.Vmax,val_Auv,momsg.maxAuv_acc,momsg.Jerk,deltaT,moveDir,0)){
                                    cur_move_pos=moveDir *(vs*deltaT+0.5*val_Auv*pow(deltaT,2));
                                    vs+=val_Auv * deltaT;
                                    t2++;
                                    preView_phase=1;
                                }
                                else{
                                    double pre_move_pos=0,pre_cal_vmax=0;
                                    int pre_t3=0;
                                    preFicure(val_Auv,vs,momsg.Jerk,deltaT,moveDir,&pre_move_pos,&pre_cal_vmax,&pre_t3);
                                    t4=(2.0*(momsg.remaining_pos-pre_move_pos)-momsg.cmd_pos)/(moveDir*pre_cal_vmax*deltaT);
                                    extra_pos=2.0*(momsg.remaining_pos-pre_move_pos)-momsg.cmd_pos-t4*moveDir*pre_cal_vmax*deltaT;
                                    per_add_pos=extra_pos/(t1+t2+2*pre_t3+t4);

                                    cur_move_pos=moveDir *(vs*deltaT+0.5*val_Auv*pow(deltaT,2)-1.0/6*momsg.Jerk*pow(deltaT,3));
                                    vs+=0.5*(val_Auv + val_Auv-momsg.Jerk * deltaT) * deltaT;
                                    val_Auv -= momsg.Jerk * deltaT;
                                    t3++;
                                    preView_phase=2;
                                }
                            }break;
                            case 2:{
                                cur_move_pos=moveDir *(vs*deltaT+0.5*val_Auv*pow(deltaT,2)-1.0/6*momsg.Jerk*pow(deltaT,3));
                                vs+=0.5*(val_Auv + val_Auv-momsg.Jerk * deltaT) * deltaT;
                                val_Auv -= momsg.Jerk * deltaT;
                                t3++;
                                if(val_Auv==0){
                                    if(t4>0)motion_phase++;
                                    else motion_phase+=2;
                                }
                            }break;
                        }
                    }break;
                    case 1:{
                        cur_move_pos=moveDir *(vs*deltaT);
                        t4--;
                        if(t4==0)motion_phase++;
                    }break;
                    case 2:{
                        cur_move_pos=moveDir *(vs*deltaT+0.5*val_Auv*pow(deltaT,2)-1.0/6*momsg.Jerk*pow(deltaT,3));
                        vs+=0.5*(val_Auv + val_Auv-momsg.Jerk * deltaT) * deltaT;
                        val_Auv -= momsg.Jerk * deltaT;
                        t3--;
                        if(t3==0){
                            if(t2>0)motion_phase++;
                            else motion_phase+=2;
                        }
                    }break;
                    case 3:{
                        cur_move_pos=moveDir *(vs*deltaT+0.5*val_Auv*pow(deltaT,2));
                        vs+=val_Auv * deltaT;
                        val_Auv = val_Auv;
                        t2--;
                        if(t2==0) motion_phase++;
                    }break;
                    case 4:{
                        cur_move_pos=moveDir *(vs*deltaT+0.5*val_Auv*pow(deltaT,2)+1.0/6*momsg.Jerk*pow(deltaT,3));
                        vs+=0.5*(val_Auv + val_Auv+momsg.Jerk * deltaT) * deltaT;
                        val_Auv += momsg.Jerk * deltaT;
                        t1--;
                        if(t1==0) momsg.path_busy=0;
                    }break;
                }
                if(momsg.path_busy){
                    if(extra_pos!=0){
                        extra_pos-=per_add_pos;
                        cur_move_pos+=per_add_pos;
                    }
                }
                else cur_move_pos=momsg.remaining_pos;
                momsg.remaining_pos -= cur_move_pos;
            }break;
            case 2:{
                if(!dec_finished & (fabs(momsg.remaining_pos)>=1)){
                    double T1 = fabs(val_Auv / (momsg.Jerk*dec_jerk_muti));
                    int moveDir = (momsg.remaining_pos >= 0) ? 1 : -1;
                    if(vs - 0.5 * (momsg.Jerk*dec_jerk_muti) * pow(T1,2) >ve){
                        double current_vs= vs+0.5*(val_Auv + val_Auv-(momsg.Jerk*dec_jerk_muti) * deltaT) * deltaT;
                        double current_acc=val_Auv-(momsg.Jerk*dec_jerk_muti) * deltaT;
                        if(current_vs-0.5*(momsg.Jerk*dec_jerk_muti)*pow(current_acc/(momsg.Jerk*dec_jerk_muti),2)>ve){
                            cur_move_pos=moveDir *(vs*deltaT+0.5*val_Auv*pow(deltaT,2)-1.0/6*(momsg.Jerk*dec_jerk_muti)*pow(deltaT,3));
                            vs+=0.5*(val_Auv + val_Auv-(momsg.Jerk*dec_jerk_muti) * deltaT) * deltaT;
                            val_Auv -= (momsg.Jerk*dec_jerk_muti) * deltaT;
                        }
                        else{
                            current_vs= vs+val_Auv * deltaT;
                            current_acc=val_Auv;
                            if(current_vs-0.5*(momsg.Jerk*dec_jerk_muti)*pow(current_acc/(momsg.Jerk*dec_jerk_muti),2)>ve){
                                cur_move_pos=moveDir *(vs*deltaT+0.5*val_Auv*pow(deltaT,2));
                                vs+=val_Auv * deltaT;
                            }
                            else{
                                cur_move_pos=moveDir *(vs*deltaT+0.5*val_Auv*pow(deltaT,2)+1.0/6*(momsg.Jerk*dec_jerk_muti)*pow(deltaT,3));
                                vs+=0.5*(val_Auv + val_Auv+(momsg.Jerk*dec_jerk_muti) * deltaT) * deltaT;
                                val_Auv += (momsg.Jerk*dec_jerk_muti) * deltaT;
                            }
                        }
                    }
                    else{
                        cur_move_pos=moveDir *(vs*deltaT+0.5*val_Auv*pow(deltaT,2)+1.0/6*(momsg.Jerk*dec_jerk_muti)*pow(deltaT,3));
                        vs+=0.5*(val_Auv + val_Auv+(momsg.Jerk*dec_jerk_muti) * deltaT) * deltaT;
                        val_Auv += (momsg.Jerk*dec_jerk_muti) * deltaT;
                    }
                    momsg.remaining_pos -= cur_move_pos;
                    if(fabs(cur_move_pos)<=1){
                        dec_finished=true;
                        vs=0,ve=0,val_Auv=0;
                        t1=0,t2=0,t3=0,t4=0;
                        preView_phase=0,motion_phase=0;
                        extra_pos=0,per_add_pos=0;
                    }
                }
                else dec_finished=true;
            }break;
            default: break;

        }
    }
    if(momsg.path_busy==0){
        dec_finished=false,path_pause_cmd=false;
        vs=0,ve=0,val_Auv=0;
        t1=0,t2=0,t3=0,t4=0;
        preView_phase=0,motion_phase=0;
        extra_pos=0,per_add_pos=0;
    }
    if(path_cmd==0)return false;
    else return true;
}

bool llmotionECat::differNewPath(bool path_pause)
{
    static bool dec_finished=false,path_pause_cmd=false;
    const double deltaT=0.001;
    int path_cmd=0,free_judge=0,dec_judge=0;
    if(!path_pause_cmd | dec_finished)path_pause_cmd=path_pause;
    for(unsigned short i=0;i<motion_data_pravete.size();i++){
        if(unmsg[i].remaining_pos!=0){
            unmsg[i].path_busy=1;
            free_judge++;
            if(dec_finished){
                if(path_pause_cmd) path_cmd=0;
                else path_cmd=1;
            }
            else{
                if(path_pause_cmd)path_cmd=2;
                else path_cmd=1;
            }
        }
        else{
             unmsg[i].path_busy=0;
             path_cmd=0;
        }
        switch (path_cmd){
            case 1:{
                dec_finished=false;
                int moveDir = (unmsg[i].remaining_pos >= 0) ? 1 : -1;
                switch(unmsg[i].motion_phase){
                    case 0:{
                        unmsg[i].extra_pos=0;
                        switch(unmsg[i].preView_phase){
                            case 0:{
                                if(preView(unmsg[i].cmd_pos,unmsg[i].remaining_pos,unmsg[i].vs,unmsg[i].Vmax,unmsg[i].val_Auv,unmsg[i].maxAuv_acc,unmsg[i].Jerk,deltaT,moveDir,1)){
                                    unmsg[i].int_pos=moveDir *(unmsg[i].vs*deltaT+0.5*unmsg[i].val_Auv*pow(deltaT,2)+1.0/6*unmsg[i].Jerk*pow(deltaT,3));
                                    unmsg[i].vs+=0.5*(unmsg[i].val_Auv + unmsg[i].val_Auv+unmsg[i].Jerk * deltaT) * deltaT;
                                    unmsg[i].val_Auv += unmsg[i].Jerk * deltaT;
                                    unmsg[i].t1++;
                                }
                                else{
                                    if(preView(unmsg[i].cmd_pos,unmsg[i].remaining_pos,unmsg[i].vs,unmsg[i].Vmax,unmsg[i].val_Auv,unmsg[i].maxAuv_acc,unmsg[i].Jerk,deltaT,moveDir,0)){
                                        unmsg[i].int_pos=moveDir *(unmsg[i].vs*deltaT+0.5*unmsg[i].val_Auv*pow(deltaT,2));
                                        unmsg[i].vs+=unmsg[i].val_Auv * deltaT;
                                        unmsg[i].val_Auv = unmsg[i].val_Auv;
                                        unmsg[i].t2++;
                                        unmsg[i].preView_phase=1;
                                    }
                                    else{
                                        double pre_move_pos=0,pre_cal_vmax=0;
                                        int pre_t3=0;
                                        preFicure(unmsg[i].val_Auv,unmsg[i].vs,unmsg[i].Jerk,deltaT,moveDir,&pre_move_pos,&pre_cal_vmax,&pre_t3);
                                        int t4_num=((int)(2*(unmsg[i].remaining_pos-pre_move_pos)-unmsg[i].cmd_pos))/((int)(moveDir*pre_cal_vmax*deltaT));
                                        unmsg[i].t4=t4_num;
                                        unmsg[i].extra_pos=(int)(2*(unmsg[i].remaining_pos-pre_move_pos)-unmsg[i].cmd_pos)-unmsg[i].t4*((int)(moveDir*pre_cal_vmax*deltaT));
                                        unmsg[i].max_per_add_pos=unmsg[i].extra_pos/(unmsg[i].t1+unmsg[i].t2+2*pre_t3+unmsg[i].t4)+moveDir;
                                        unmsg[i].per_add_pos=0;

                                        unmsg[i].int_pos=moveDir *(unmsg[i].vs*deltaT+0.5*unmsg[i].val_Auv*pow(deltaT,2)-1.0/6*unmsg[i].Jerk*pow(deltaT,3));
                                        unmsg[i].vs+=0.5*(unmsg[i].val_Auv + unmsg[i].val_Auv-unmsg[i].Jerk * deltaT) * deltaT;
                                        unmsg[i].val_Auv -= unmsg[i].Jerk * deltaT;
                                        unmsg[i].t3++;
                                        unmsg[i].preView_phase=2;
                                    }
                                }
                            }break;
                            case 1:{
                                if(preView(unmsg[i].cmd_pos,unmsg[i].remaining_pos,unmsg[i].vs,unmsg[i].Vmax,unmsg[i].val_Auv,unmsg[i].maxAuv_acc,unmsg[i].Jerk,deltaT,moveDir,0)){
                                    unmsg[i].int_pos=moveDir *(unmsg[i].vs*deltaT+0.5*unmsg[i].val_Auv*pow(deltaT,2));
                                    unmsg[i].vs+=unmsg[i].val_Auv * deltaT;
                                    unmsg[i].val_Auv = unmsg[i].val_Auv;
                                    unmsg[i].t2++;
                                    unmsg[i].preView_phase=1;
                                }
                                else{
                                    double pre_move_pos=0,pre_cal_vmax=0;
                                    int pre_t3=0;
                                    preFicure(unmsg[i].val_Auv,unmsg[i].vs,unmsg[i].Jerk,deltaT,moveDir,&pre_move_pos,&pre_cal_vmax,&pre_t3);
                                    int t4_num=((int)(2*(unmsg[i].remaining_pos-pre_move_pos)-unmsg[i].cmd_pos))/((int)(moveDir*pre_cal_vmax*deltaT));
                                    unmsg[i].t4=t4_num;
                                    unmsg[i].extra_pos=(int)(2*(unmsg[i].remaining_pos-pre_move_pos)-unmsg[i].cmd_pos)-unmsg[i].t4*((int)(moveDir*pre_cal_vmax*deltaT));
                                    unmsg[i].max_per_add_pos=unmsg[i].extra_pos/(unmsg[i].t1+unmsg[i].t2+2*pre_t3+unmsg[i].t4)+moveDir;
                                    unmsg[i].per_add_pos=0;

                                    unmsg[i].int_pos=moveDir *(unmsg[i].vs*deltaT+0.5*unmsg[i].val_Auv*pow(deltaT,2)-1.0/6*unmsg[i].Jerk*pow(deltaT,3));
                                    unmsg[i].vs+=0.5*(unmsg[i].val_Auv + unmsg[i].val_Auv-unmsg[i].Jerk * deltaT) * deltaT;
                                    unmsg[i].val_Auv -= unmsg[i].Jerk * deltaT;
                                    unmsg[i].t3++;
                                    unmsg[i].preView_phase=2;
                                }
                            }break;
                            case 2:{
                                unmsg[i].int_pos=moveDir *(unmsg[i].vs*deltaT+0.5*unmsg[i].val_Auv*pow(deltaT,2)-1.0/6*unmsg[i].Jerk*pow(deltaT,3));
                                unmsg[i].vs+=0.5*(unmsg[i].val_Auv + unmsg[i].val_Auv-unmsg[i].Jerk * deltaT) * deltaT;
                                unmsg[i].val_Auv -= unmsg[i].Jerk * deltaT;
                                unmsg[i].t3++;
                                if(unmsg[i].val_Auv==0){
                                    unmsg[i].preView_phase=0;
                                    if(unmsg[i].t4>0)unmsg[i].motion_phase++;
                                    else unmsg[i].motion_phase+=2;
                                }
                            }break;
                        }
                    }break;
                    case 1:{
                        unmsg[i].int_pos=moveDir *(unmsg[i].vs*deltaT);
                        unmsg[i].t4--;
                        if(unmsg[i].t4==0)unmsg[i].motion_phase++;
                    }break;
                    case 2:{
                        unmsg[i].int_pos=moveDir *(unmsg[i].vs*deltaT+0.5*unmsg[i].val_Auv*pow(deltaT,2)-1.0/6*unmsg[i].Jerk*pow(deltaT,3));
                        unmsg[i].vs+=0.5*(unmsg[i].val_Auv + unmsg[i].val_Auv-unmsg[i].Jerk * deltaT) * deltaT;
                        unmsg[i].val_Auv -= unmsg[i].Jerk * deltaT;
                        unmsg[i].t3--;
                        if(unmsg[i].t3==0){
                            if(unmsg[i].t2>0)unmsg[i].motion_phase++;
                            else unmsg[i].motion_phase+=2;
                        }
                    }break;
                    case 3:{
                        unmsg[i].int_pos=moveDir *(unmsg[i].vs*deltaT+0.5*unmsg[i].val_Auv*pow(deltaT,2));
                        unmsg[i].vs+=unmsg[i].val_Auv * deltaT;
                        unmsg[i].val_Auv = unmsg[i].val_Auv;
                        unmsg[i].t2--;
                        if(unmsg[i].t2==0) unmsg[i].motion_phase++;
                    }break;
                    case 4:{
                        unmsg[i].int_pos=moveDir *(unmsg[i].vs*deltaT+0.5*unmsg[i].val_Auv*pow(deltaT,2)+1.0/6*unmsg[i].Jerk*pow(deltaT,3));
                        unmsg[i].vs+=0.5*(unmsg[i].val_Auv + unmsg[i].val_Auv+unmsg[i].Jerk * deltaT) * deltaT;
                        unmsg[i].val_Auv += unmsg[i].Jerk * deltaT;
                        unmsg[i].t1--;
                        if(unmsg[i].t1==0) unmsg[i].motion_phase=0;
                    }break;

                }
                if(unmsg[i].extra_pos!=0){
                    if(fabs(unmsg[i].extra_pos)>=fabs(unmsg[i].per_add_pos+moveDir) && fabs(unmsg[i].per_add_pos+moveDir)<=fabs(unmsg[i].max_per_add_pos)){
                        unmsg[i].per_add_pos+=moveDir;
                    }
                    else{
                        while(fabs(unmsg[i].extra_pos)<fabs(unmsg[i].per_add_pos)) unmsg[i].per_add_pos-=moveDir;
                    }
                    unmsg[i].extra_pos-=unmsg[i].per_add_pos;
                    unmsg[i].int_pos+=unmsg[i].per_add_pos;
                }
                unmsg[i].dis_val_moved+=unmsg[i].int_pos;
                unmsg[i].remaining_pos-=unmsg[i].int_pos;
            }break;
            case 2:{
                if(!dec_finished & (unmsg[i].remaining_pos!=0)){
                    double T1 = fabs(unmsg[i].val_Auv / (unmsg[i].Jerk*dec_jerk_muti));
                    int moveDir = (unmsg[i].remaining_pos >= 0) ? 1 : -1;
                    if(unmsg[i].vs - 0.5 * (unmsg[i].Jerk*dec_jerk_muti) * pow(T1,2) >unmsg[i].ve){
                        double current_vs= unmsg[i].vs+0.5*(unmsg[i].val_Auv + unmsg[i].val_Auv-(unmsg[i].Jerk*dec_jerk_muti) * deltaT) * deltaT;
                        double current_acc=unmsg[i].val_Auv-(unmsg[i].Jerk*dec_jerk_muti) * deltaT;
                        if(current_vs-0.5*(unmsg[i].Jerk*dec_jerk_muti)*pow(current_acc/(unmsg[i].Jerk*dec_jerk_muti),2)>unmsg[i].ve){
                            unmsg[i].int_pos=moveDir *(unmsg[i].vs*deltaT+0.5*unmsg[i].val_Auv*pow(deltaT,2)-1.0/6*(unmsg[i].Jerk*dec_jerk_muti)*pow(deltaT,3));
                            unmsg[i].vs+=0.5*(unmsg[i].val_Auv + unmsg[i].val_Auv-(unmsg[i].Jerk*dec_jerk_muti) * deltaT) * deltaT;
                            unmsg[i].val_Auv -= (unmsg[i].Jerk*dec_jerk_muti) * deltaT;
                        }
                        else{
                            current_vs= unmsg[i].vs+unmsg[i].val_Auv * deltaT;
                            current_acc=unmsg[i].val_Auv;
                            if(current_vs-0.5*(unmsg[i].Jerk*dec_jerk_muti)*pow(current_acc/(unmsg[i].Jerk*dec_jerk_muti),2)>unmsg[i].ve){
                                unmsg[i].int_pos=moveDir *(unmsg[i].vs*deltaT+0.5*unmsg[i].val_Auv*pow(deltaT,2));
                                unmsg[i].vs+=unmsg[i].val_Auv * deltaT;
                            }
                            else{
                                unmsg[i].int_pos=moveDir *(unmsg[i].vs*deltaT+0.5*unmsg[i].val_Auv*pow(deltaT,2)+1.0/6*(unmsg[i].Jerk*dec_jerk_muti)*pow(deltaT,3));
                                unmsg[i].vs+=0.5*(unmsg[i].val_Auv + unmsg[i].val_Auv+(unmsg[i].Jerk*dec_jerk_muti) * deltaT) * deltaT;
                                unmsg[i].val_Auv += (unmsg[i].Jerk*dec_jerk_muti) * deltaT;
                            }
                        }
                    }
                    else{
                        unmsg[i].int_pos=moveDir *(unmsg[i].vs*deltaT+0.5*unmsg[i].val_Auv*pow(deltaT,2)+1.0/6*(unmsg[i].Jerk*dec_jerk_muti)*pow(deltaT,3));
                        unmsg[i].vs+=0.5*(unmsg[i].val_Auv + unmsg[i].val_Auv+(unmsg[i].Jerk*dec_jerk_muti) * deltaT) * deltaT;
                        unmsg[i].val_Auv += (unmsg[i].Jerk*dec_jerk_muti) * deltaT;
                    }
                    unmsg[i].dis_val_moved+=unmsg[i].int_pos;
                    unmsg[i].remaining_pos-=unmsg[i].int_pos;
                    if(unmsg[i].int_pos==0){
                        unmsg[i].vs=0;
                        unmsg[i].val_Auv=0;
                        unmsg[i].t1=0;
                        unmsg[i].t2=0;
                        unmsg[i].t3=0;
                        unmsg[i].t4=0;
                        unmsg[i].preView_phase=0;
                        unmsg[i].motion_phase=0;
                        unmsg[i].extra_pos=0;
                        unmsg[i].max_per_add_pos=0;
                        unmsg[i].per_add_pos=0;
                        unmsg[i].cmd_pos=unmsg[i].remaining_pos;
                        unmsg[i].dis_val_moved=0;
                    }
                    else dec_judge++;
                }
                else dec_finished=true;
            }break;
            default:break;
        }
    }
    if(free_judge==0){
        dec_finished=false;
        path_pause_cmd=false;
        return false;
    }
    else{
        if(path_cmd==0)return false;
        else if(dec_judge==0 && path_cmd==2){
            dec_finished=true;
            return false;
        }
        else return true;
    }
}

bool llmotionECat::preView(double all_dis,double remaining_dis,double vs,double vmax,double val_auv,double maxAcc,double jerk,double deltaT,double moveDir,int motionType)
{
    double current_vs=0,current_acc=0,cal_vmax=0;
    double move_dis=0;
    switch (motionType) {
        case 0:{
            move_dis=moveDir *(vs*deltaT+0.5*val_auv*pow(deltaT,2));
            current_vs= vs+ val_auv * deltaT;
            current_acc=val_auv;
        }break;
        case 1:{
           move_dis=moveDir *(vs*deltaT+0.5*val_auv*pow(deltaT,2)+1.0/6*jerk*pow(deltaT,3));
           current_vs= vs+0.5*(val_auv + val_auv+jerk * deltaT) * deltaT;
           current_acc=val_auv+jerk * deltaT;
        }break;
        default:break;
    }
    cal_vmax=current_vs+current_acc*current_acc/jerk-0.5*jerk*pow(current_acc/jerk,2);
    if(cal_vmax>vmax || current_acc>maxAcc)return false;
    else{
        move_dis+=moveDir*(current_vs*current_acc/jerk+0.5*current_acc*pow(current_acc/jerk,2)-jerk*pow(current_acc/jerk,3)*1.0/6);
        if((2*remaining_dis-all_dis-2*move_dis)*(all_dis/(fabs(all_dis)))>=0)return true;
        else return false;
    }
}

void llmotionECat::preFicure(double val_auv,double vs,double Jerk,double deltaT,int moveDir,double *pre_move_pos,double *pre_cal_vmax,int *pre_t3)
{
    *pre_move_pos=0;
    *pre_t3=0;
    do{
        *pre_move_pos+=moveDir*(vs*deltaT+0.5*val_auv*pow(deltaT,2)-1.0/6*Jerk*pow(deltaT,3));
        vs+=0.5*(2*val_auv-Jerk * deltaT) * deltaT;
        val_auv -= Jerk * deltaT;
        *pre_t3+=1;
    }while(val_auv>=Jerk * deltaT);
    *pre_cal_vmax=vs;
}
