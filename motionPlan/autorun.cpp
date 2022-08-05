#include "autorun.h"

autoRun::autoRun()
{
    start();
}

autoRun::~autoRun(){

}

void autoRun::run(){
    run_var = true;
    cmd=0;
    while(run_var){
        switch (cmd){
            case 31:{
                sync_path();
                cmd=0;
            }break;
            case 32:{
                S_path();
                cmd=0;
            }break;
            case 33:{
                exp_path();
                cmd=0;
            }break;
            default:break;
        }
    }
}

int autoRun::S_path()
{
    int     dis_val_moved[7]        =   {0,0,0,0,0,0,0};
    int     dis_val_path_Inc[7]     =   {0,0,0,0,0,0,0};
    int     dis_val_path_Int[7]     =   {0,0,0,0,0,0,0};
    double  dis_val_path_float[7]   =   {0,0,0,0,0,0,0};
    double  dis_val_path[7]         =   {0,0,0,0,0,0,0};

    double S_Dist = 0;
    double S_Vmax = 0;
    double S_Amax = 0;
    double S_Jerk = 0;

    int first_i=0;
    for(int i=0;i<motion_data_pravete.size();i++){
        if(first_i++ == 0){
            S_Vmax = fabs(motion_data_pravete[i].Vmax);
            S_Amax = fabs(motion_data_pravete[i].maxAuv_acc);
            S_Jerk = fabs(motion_data_pravete[i].Jerk);
            S_Dist = motion_data_pravete[i].cmd_pos;
        }
        else{
            if(fabs(S_Dist)<fabs(motion_data_pravete[i].cmd_pos)) S_Dist = motion_data_pravete[i].cmd_pos;
            if(fabs(S_Vmax)>fabs(motion_data_pravete[i].Vmax)) S_Vmax = fabs(motion_data_pravete[i].Vmax);
            if(fabs(S_Amax)>fabs(motion_data_pravete[i].maxAuv_acc)) S_Amax = fabs(motion_data_pravete[i].maxAuv_acc);
            if(fabs(S_Jerk)>fabs(motion_data_pravete[i].Jerk)) S_Jerk = fabs(motion_data_pravete[i].Jerk);
        }
    }

    MotionPlan_S pathObject;
    pathObject.ini_path_data(S_Dist,v0,S_Vmax,ve,S_Amax,S_Jerk);
    while(pathObject.pathBusy()){
        if(pathObject.performPath(path_pause)){
            double t=0,dis=0,time,vs=0,acc=0;
            pathObject.get_move_proportion(t);
            pathObject.get_move_msg(dis,vs,acc,time);
            qDebug()<<t<<","<<dis<<","<<vs<<","<<acc<<","<<time;
            for(int i=0;i<motion_data_pravete.size();i++){
                if(do_float){
                    dis_val_path[i] = t * motion_data_pravete[i].cmd_pos + dis_val_path_float[i];
                    dis_val_path_Int[i] = (int)(dis_val_path[i]);
                    dis_val_path_float[i] = dis_val_path[i] - dis_val_path_Int[i];
                    dis_val_path_Inc[i] = dis_val_path_Int[i] - dis_val_moved[i];
                    dis_val_moved[i] += dis_val_path_Inc[i];
                }
                else{
                    dis_val_path_Int[i] = t * motion_data_pravete[i].cmd_pos;
                    dis_val_path_Inc[i] = dis_val_path_Int[i] - dis_val_moved[i];
                    dis_val_moved[i] += dis_val_path_Inc[i];
                }
                emit send_data(1,i,dis_val_path_Inc[i],dis_val_moved[i],time);
                QTime timer;
                timer.start();
                while(timer.elapsed()<=cal_delay+cal_time){;}
            }
        }
    }

}

int autoRun::sync_path()
{
    int     dis_val_moved[7]        =   {0,0,0,0,0,0,0};
    int     dis_val_path_Inc[7]     =   {0,0,0,0,0,0,0};
    int     dis_val_path_Int[7]     =   {0,0,0,0,0,0,0};
    double  dis_val_path_float[7]   =   {0,0,0,0,0,0,0};
    double  dis_val_path[7]         =   {0,0,0,0,0,0,0};

    double S_Dist = 0;
    double S_Vmax = 0;
    double S_Amax = 0;
    double S_Jerk = 0;

    int first_i=0;
    for(int i=0;i<motion_data_pravete.size();i++){
        if(first_i++ == 0){
            S_Vmax = fabs(motion_data_pravete[i].Vmax);
            S_Amax = fabs(motion_data_pravete[i].maxAuv_acc);
            S_Jerk = fabs(motion_data_pravete[i].Jerk);
            S_Dist = motion_data_pravete[i].cmd_pos;
        }
        else{
            if(fabs(S_Dist)<fabs(motion_data_pravete[i].cmd_pos)) S_Dist = motion_data_pravete[i].cmd_pos;
            if(fabs(S_Vmax)>fabs(motion_data_pravete[i].Vmax)) S_Vmax = fabs(motion_data_pravete[i].Vmax);
            if(fabs(S_Amax)>fabs(motion_data_pravete[i].maxAuv_acc)) S_Amax = fabs(motion_data_pravete[i].maxAuv_acc);
            if(fabs(S_Jerk)>fabs(motion_data_pravete[i].Jerk)) S_Jerk = fabs(motion_data_pravete[i].Jerk);
        }
    }

    MotionPlan pathObject;
    pathObject.ini_path_data(S_Dist,v0,S_Vmax,ve,S_Amax,S_Jerk);
    while(pathObject.pathBusy()){
        if(pathObject.performPath(path_pause)){
            double t=0,dis=0,time;
            pathObject.get_move_proportion(t);
            pathObject.get_move_msg(dis,time);

            for(int i=0;i<motion_data_pravete.size();i++){
                if(do_float){
                    dis_val_path[i] = t * motion_data_pravete[i].cmd_pos + dis_val_path_float[i];
                    dis_val_path_Int[i] = (int)(dis_val_path[i]);
                    dis_val_path_float[i] = dis_val_path[i] - dis_val_path_Int[i];
                    dis_val_path_Inc[i] = dis_val_path_Int[i] - dis_val_moved[i];
                    dis_val_moved[i] += dis_val_path_Inc[i];
                }
                else{
                    dis_val_path_Int[i] = t * motion_data_pravete[i].cmd_pos;
                    dis_val_path_Inc[i] = dis_val_path_Int[i] - dis_val_moved[i];
                    dis_val_moved[i] += dis_val_path_Inc[i];
                }
                emit send_data(0,i,dis_val_path_Inc[i],dis_val_moved[i],time);
                QTime timer;
                timer.start();
                while(timer.elapsed()<=cal_delay+cal_time){;}
            }
        }
    }
}

int autoRun::exp_path()
{
    int     dis_val_moved[7]        =   {0,0,0,0,0,0,0};
    int     dis_val_path_Inc[7]     =   {0,0,0,0,0,0,0};
    int     dis_val_path_Int[7]     =   {0,0,0,0,0,0,0};
    double  dis_val_path_float[7]   =   {0,0,0,0,0,0,0};
    double  dis_val_path[7]         =   {0,0,0,0,0,0,0};

    double S_Dist = 0;
    double S_Vmax = 0;
    double S_Amax = 0;
    double S_Jerk = 0;

    int first_i=0;
    for(int i=0;i<motion_data_pravete.size();i++){
        if(first_i++ == 0){
            S_Vmax = fabs(motion_data_pravete[i].Vmax);
            S_Amax = fabs(motion_data_pravete[i].maxAuv_acc);
            S_Jerk = fabs(motion_data_pravete[i].Jerk);
            S_Dist = motion_data_pravete[i].cmd_pos;
        }
        else{
            if(fabs(S_Dist)<fabs(motion_data_pravete[i].cmd_pos)) S_Dist = motion_data_pravete[i].cmd_pos;
            if(fabs(S_Vmax)>fabs(motion_data_pravete[i].Vmax)) S_Vmax = fabs(motion_data_pravete[i].Vmax);
            if(fabs(S_Amax)>fabs(motion_data_pravete[i].maxAuv_acc)) S_Amax = fabs(motion_data_pravete[i].maxAuv_acc);
            if(fabs(S_Jerk)>fabs(motion_data_pravete[i].Jerk)) S_Jerk = fabs(motion_data_pravete[i].Jerk);
        }
    }

    MotionPlan pathObject;
    pathObject.ini_path_data(S_Dist,v0,S_Vmax,ve,S_Amax,S_Jerk,0.001,1,muti_T);
    while(pathObject.pathBusy()){
        if(pathObject.performPath(path_pause)){
            double t=0,dis=0,time,vs=0,acc=0;
            pathObject.get_move_proportion(t);
            pathObject.get_move_msg(dis,vs,acc,time);

            for(int i=0;i<motion_data_pravete.size();i++){
                if(do_float){
                    dis_val_path[i] = t * motion_data_pravete[i].cmd_pos + dis_val_path_float[i];
                    dis_val_path_Int[i] = (int)(dis_val_path[i]);
                    dis_val_path_float[i] = dis_val_path[i] - dis_val_path_Int[i];
                    dis_val_path_Inc[i] = dis_val_path_Int[i] - dis_val_moved[i];
                    dis_val_moved[i] += dis_val_path_Inc[i];
                }
                else{
                    dis_val_path_Int[i] = t * motion_data_pravete[i].cmd_pos;
                    dis_val_path_Inc[i] = dis_val_path_Int[i] - dis_val_moved[i];
                    dis_val_moved[i] += dis_val_path_Inc[i];
                }
                emit send_data(0,i,dis_val_path_Inc[i],dis_val_moved[i],time);

                QTime timer;
                timer.start();
                while(timer.elapsed()<=cal_delay+cal_time){;}
            }
        }
    }
}
