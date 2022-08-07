#include "autorun.h"

void autoRun::run(){
    double path_moved=0,path_Inc_pos=0;
    double t=0;
    double dis=0,time,vs=0,acc=0;
    bool pause=false;
    QScopedPointer<motionPlan> motion(new motionPlan);
    motion->ini_path_data(pathInit);
    while(motion->pathBusy()){
        pause=path_pause;
        if(motion->performPath(pause)){
            motion->get_move_proportion(t);
            motion->get_move_msg(dis,vs,acc,time);
            path_Inc_pos = t * pathInit.cmd_pos - path_moved;
            path_moved+=path_Inc_pos;
            emit send_data(path_Inc_pos,path_moved,time);
        }
    }
    emit send_cal_done_sig();
}
