#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    autorun=new autoRun();
    autorun->start();
    connect(autorun,SIGNAL(send_data(int,int,int,int,double)),this,SLOT(rec_data(int,int,int,int,double)));
    init_chart();
    centralWidget()->setMouseTracking(true);
    setMouseTracking(true);
    ui->chartView->setAttribute(Qt::WA_TransparentForMouseEvents, true);
}

MainWindow::~MainWindow()
{
    delete ui;
    if(autorun!=nullptr){
        autorun->run_var=false;
        QThread::msleep(100);
        autorun->quit();
        while(autorun->isRunning());
        delete autorun;
    }
}

void MainWindow::rec_data(int path_type,int axis, int inc_pos, int move_pos,double time)
{
    int t1=QTime(0,0,0).msecsTo(QTime::currentTime());
    if(path_type){
        QLineSeries *series=(QLineSeries *)ui->chartView->chart()->series().at(4);
        series->setName("旧插补耗时:"+QString::number(time));
    }
    else{
        QLineSeries *series=(QLineSeries *)ui->chartView->chart()->series().at(5);
        series->setName("新插补耗时:"+QString::number(time));
    }
    autorun->cal_delay=ui->lineEdit_delay->text().toInt();
    axis_time[axis]+=0.001;
    QLineSeries *series=(QLineSeries *)ui->chartView->chart()->series().at(axis);
    series->setName(QString::number(axis+1)+"轴:"+QString::number(move_pos));
    series->append(axis_time[axis],inc_pos);
    int t2=QTime(0,0,0).msecsTo(QTime::currentTime());
    autorun->cal_time=t2-t1;
}

void MainWindow::init_chart()
{
    QChart *chart=new QChart();
    ui->chartView->setChart(chart);
    ui->chartView->setRenderHint(QPainter::Antialiasing);
    QPen pen;
    pen.setStyle(Qt::SolidLine);
    pen.setWidth(2);
    QValueAxis *axisX=new QValueAxis;
    axisX->setLabelFormat("%0.3f");
    axisX->setTickCount(30);
    axisX->setTitleText("时间(秒)");
    QValueAxis *axisY=new QValueAxis;
    axisY->setLabelFormat("%0.3f");
    axisY->setTickCount(31);
    axisY->setTitleText("位移(脉冲数)");
    for(int i=0;i<6;i++){
        if(i==4){
            QLineSeries *seriesTmp=new QLineSeries;
            QString seriesName="旧插补耗时";
            seriesTmp->setName(seriesName);
            pen.setColor(color.at(i));
            seriesTmp->setPen(pen);
            chart->addSeries(seriesTmp);
            chart->setAxisX(axisX,seriesTmp);
            chart->setAxisY(axisY,seriesTmp);
        }
        else if(i==5){
            QLineSeries *seriesTmp=new QLineSeries;
            QString seriesName="新插补耗时";
            seriesTmp->setName(seriesName);
            pen.setColor(color.at(i));
            seriesTmp->setPen(pen);
            chart->addSeries(seriesTmp);
            chart->setAxisX(axisX,seriesTmp);
            chart->setAxisY(axisY,seriesTmp);
        }
        else{
            QLineSeries *seriesTmp=new QLineSeries;
            QString seriesName=QString::number(i+1)+"轴";
            seriesTmp->setName(seriesName);
            pen.setColor(color.at(i));
            seriesTmp->setPen(pen);
            chart->addSeries(seriesTmp);
            chart->setAxisX(axisX,seriesTmp);
            chart->setAxisY(axisY,seriesTmp);
        }
    }
    for(int i=0;i<4;i++) axis_time[i]=0;
}

bool MainWindow::checking_axis_data()
{
    if(ui->checkBox_AxisIdx_0->checkState()==Qt::Checked){
        path_axis_motion axis_data;
        axis_data.cmd_pos=ui->lineEdit_Axis_dis_0->text().toDouble();
        axis_data.Vmax=ui->lineEdit_Axis_spd_0->text().toDouble();
        axis_data.maxAuv_acc=ui->lineEdit_Axis_acc_0->text().toDouble();
        axis_data.Jerk=ui->lineEdit_Jerk_0->text().toDouble();
        autorun->motion_data_pravete.append(axis_data);
    }
    if(ui->checkBox_AxisIdx_1->checkState()==Qt::Checked){
        path_axis_motion axis_data;
        axis_data.cmd_pos=ui->lineEdit_Axis_dis_1->text().toDouble();
        axis_data.Vmax=ui->lineEdit_Axis_spd_1->text().toDouble();
        axis_data.maxAuv_acc=ui->lineEdit_Axis_acc_1->text().toDouble();
        axis_data.Jerk=ui->lineEdit_Jerk_1->text().toDouble();
        autorun->motion_data_pravete.append(axis_data);
    }
    if(ui->checkBox_AxisIdx_2->checkState()==Qt::Checked){
        path_axis_motion axis_data;
        axis_data.cmd_pos=ui->lineEdit_Axis_dis_2->text().toDouble();
        axis_data.Vmax=ui->lineEdit_Axis_spd_2->text().toDouble();
        axis_data.maxAuv_acc=ui->lineEdit_Axis_acc_2->text().toDouble();
        axis_data.Jerk=ui->lineEdit_Jerk_2->text().toDouble();
        autorun->motion_data_pravete.append(axis_data);
    }
    if(ui->checkBox_AxisIdx_3->checkState()==Qt::Checked){
        path_axis_motion axis_data;
        axis_data.cmd_pos=ui->lineEdit_Axis_dis_3->text().toDouble();
        axis_data.Vmax=ui->lineEdit_Axis_spd_3->text().toDouble();
        axis_data.maxAuv_acc=ui->lineEdit_Axis_acc_3->text().toDouble();
        axis_data.Jerk=ui->lineEdit_Jerk_3->text().toDouble();
        autorun->motion_data_pravete.append(axis_data);
    }
    if(autorun->motion_data_pravete.size()>0){
        QValueAxis *axisX=(QValueAxis *)ui->chartView->chart()->axisX();
        QValueAxis *axisY=(QValueAxis *)ui->chartView->chart()->axisY();
        axisX->setRange(ui->lineEdit_minX->text().toDouble(),ui->lineEdit_maxX->text().toDouble());
        axisY->setRange(ui->lineEdit_minY->text().toDouble(),ui->lineEdit_maxY->text().toDouble());
        autorun->cal_delay=ui->lineEdit_delay->text().toInt();
        return true;
    }
    else return false;
}

void MainWindow::on_btn_Axis_StepF_clicked()
{
    autorun->motion_data_pravete.clear();
    if(!checking_axis_data())return;
    if(ui->checkBox_float->checkState()==Qt::Checked)autorun->do_float=true;
    else autorun->do_float=false;
    autorun->v0=ui->v0->text().toDouble();
    autorun->ve=ui->ve->text().toDouble();
    QString params_FILE_PATH ="E:/DeskTop/axis_data.txt";
    QFile file(params_FILE_PATH);
    QString data;
    data.clear();
    if(file.open(QIODevice::WriteOnly | QIODevice::Text)){
        QTextStream out(&file);
        out<<data;
        file.flush();
        file.close();
    }
    autorun->cmd=31;
}

void MainWindow::on_btn_old_path_clicked()
{
    autorun->motion_data_pravete.clear();
    if(!checking_axis_data())return;
    if(ui->checkBox_float->checkState()==Qt::Checked)autorun->do_float=true;
    else autorun->do_float=false;
    autorun->v0=ui->v0->text().toDouble();
    autorun->ve=ui->ve->text().toDouble();
    autorun->cmd=32;
}

void MainWindow::on_exp_path_clicked()
{
    autorun->motion_data_pravete.clear();
    if(!checking_axis_data())return;
    if(ui->checkBox_float->checkState()==Qt::Checked)autorun->do_float=true;
    else autorun->do_float=false;
    autorun->v0=ui->v0->text().toDouble();
    autorun->ve=ui->ve->text().toDouble();
    QString params_FILE_PATH ="E:/DeskTop/axis_data.txt";
    QFile file(params_FILE_PATH);
    QString data;
    data.clear();
    if(file.open(QIODevice::WriteOnly | QIODevice::Text)){
        QTextStream out(&file);
        out<<data;
        file.flush();
        file.close();
    }
    autorun->muti_T=ui->muti_T->text().toDouble();
    autorun->cmd=33;
}

void MainWindow::on_pause_clicked()
{
    autorun->path_pause=true;
}

void MainWindow::on_start_clicked()
{
    autorun->path_pause=false;
}

void MainWindow::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton){
        drag_flag = true;
        m_lastPointF=event->pos();
        setCursor(Qt::OpenHandCursor);
    }
}

void MainWindow::mouseReleaseEvent(QMouseEvent *event)
{
   drag_flag = false;
    setCursor(Qt::ArrowCursor);
}

void MainWindow::mouseMoveEvent(QMouseEvent *event)
{
    if (drag_flag){
        QPoint curPos = event->pos();
        QPoint offset = curPos - m_lastPointF;
        m_lastPointF = curPos;
        ui->chartView->chart()->scroll(-offset.x(), offset.y());
    }
}

void MainWindow::wheelEvent(QWheelEvent *event)
{
    const QPoint curPos = event->pos();
    QPointF curVal = ui->chartView->chart()->mapToValue(QPointF(curPos));
    const double factor = 1.1;//缩放比例
    QValueAxis *axisX=(QValueAxis *)ui->chartView->chart()->axisX();
    QValueAxis *axisY=(QValueAxis *)ui->chartView->chart()->axisY();
    const double xMin = axisX->min();
    const double xMax = axisX->max();
    const double yMin = axisY->min();
    const double yMax = axisY->max();
    const double xCentral = curVal.x();
    const double yCentral = curVal.y();
    double leftOffset=0,rightOffset=0,bottomOffset=0,topOffset=0;
    if (event->delta() > 0){//放大
        bottomOffset = 1.0 / factor * (yCentral - yMin);
        topOffset = 1.0 / factor * (yMax - yCentral);
        leftOffset = 1.0 / factor * (xCentral - xMin);
        rightOffset = 1.0 / factor * (xMax - xCentral);
    }
    else{//缩小
        bottomOffset = 1.0 * factor * (yCentral - yMin);
        topOffset = 1.0 * factor * (yMax - yCentral);
        leftOffset = 1.0 * factor * (xCentral - xMin);
        rightOffset = 1.0 * factor * (xMax - xCentral);
    }
    ui->chartView->chart()->axisX()->setRange(xCentral - leftOffset, xCentral + rightOffset);
    ui->chartView->chart()->axisY()->setRange(yCentral - bottomOffset, yCentral + topOffset);
}
