#include "autorun.h"

autoRun::autoRun():motion(new motionPlan)
{
#ifdef motionDebug
    connect(motion,&motionPlan::sendMsg,this,&autoRun::sendMotionPlanMsg);
#endif
}

autoRun::~autoRun()
{
    delete motion;
}

void autoRun::run(){
    static double path_moved=0;
    static double proportion=0,dis=0,vs=0,acc=0,time=0;
    static bool pause=false;
    path_moved=0;
    motion->ini_path_data(pathInit);
    while(motion->pathBusy()){
        if(forceQuit) break;
        pause=path_pause;
        if(motion->performPath(pause)){
            motion->get_move_msg(proportion,dis,vs,acc,time);
            emit send_data(dis-path_moved,dis,vs,acc,time);
            path_moved=dis;
        }
    }
    forceQuit=false;
    emit send_cal_done_sig();
}
