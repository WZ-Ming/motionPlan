#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    centralWidget()->setMouseTracking(true);
    setMouseTracking(true);
    ui->chartView->setAttribute(Qt::WA_TransparentForMouseEvents, true);
    autorun=new autoRun();
    connect(autorun,SIGNAL(send_data(double,double,double)),this,SLOT(rec_data(double,double,double)),Qt::BlockingQueuedConnection);
    connect(autorun,&autoRun::send_cal_done_sig,this,&MainWindow::rec_cal_done_sig);
    init_chartAndComBox();
    setWindowTitle("静态前瞻式7段S型速度规划");
}

MainWindow::~MainWindow()
{
    delete ui;
    if(autorun!=nullptr){
        autorun->quit();
        autorun->wait();
        delete autorun;
    }
}

void MainWindow::rec_cal_done_sig()
{
    autorun->quit();
    autorun->wait();
}

void MainWindow::rec_data(double inc_pos, double move_pos, double time)
{
    series->setName("耗时:"+QString::number(time)+"秒   总位移:"+QString::number(move_pos));
    series->append(time,inc_pos);
    static bool outRange=false;
    if(time<=rangeX0){
        rangeX0=time;
        outRange=true;
    }
    else if(time>=rangeX1){
        rangeX1=time;
        outRange=true;
    }
    if(inc_pos<=rangeY0){
        rangeY0=inc_pos;
        outRange=true;
    }
    else if(inc_pos>=rangeY1){
        rangeY1=inc_pos;
        outRange=true;
    }
    if(outRange){
        ui->chartView->chart()->axisX()->setRange(rangeX0-autorun->pathInit.deltaT,rangeX1+autorun->pathInit.deltaT);
        ui->chartView->chart()->axisY()->setRange(rangeY0-1,rangeY1+1);
        outRange=false;
    }
}

void MainWindow::init_chartAndComBox()
{
    QChart *chart=new QChart();
    ui->chartView->setChart(chart);
    ui->chartView->setRenderHint(QPainter::Antialiasing);
    QPen pen;
    pen.setStyle(Qt::SolidLine);
    pen.setWidth(2);
    QValueAxis *axisX=new QValueAxis;
    axisX->setLabelFormat("%0.3f");
    axisX->setTickCount(0);
    axisX->setTitleText("时间(秒)");
    QValueAxis *axisY=new QValueAxis;
    axisY->setLabelFormat("%0.3f");
    axisY->setTickCount(0);
    axisY->setTitleText("位移");
    series=new QLineSeries;
    series->setName("耗时");
    pen.setColor(Qt::black);
    series->setPen(pen);
    chart->addSeries(series);
    chart->setAxisX(axisX,series);
    chart->setAxisY(axisY,series);
    axisX->setRange(rangeX0,rangeX1);
    axisY->setRange(rangeY0,rangeY1);
}

void MainWindow::on_btn_start_clicked()
{
    if(!autorun->isRunning()){
        autorun->pathInit.v0=ui->lineEdit_V0->text().toDouble();
        autorun->pathInit.ve=ui->lineEdit_Ve->text().toDouble();
        autorun->pathInit.Jerk=ui->lineEdit_Jerk->text().toDouble();
        autorun->pathInit.VMax=ui->lineEdit_VMax->text().toDouble();
        autorun->pathInit.cmd_pos=ui->lineEdit_distance->text().toDouble();
        autorun->pathInit.max_acc=ui->lineEdit_acc->text().toDouble();
        autorun->pathInit.dec_jerk_muti=ui->doubleSpinBox_decMuti->value();
        //autorun->pathInit.maxProportion=1;
        autorun->pathInit.deltaT=0.001;
        ui->chartView->chart()->update();
        autorun->start();
    }
    else{
        autorun->path_pause=false;
        ui->btn_start->setText("启动");
    }
}

void MainWindow::on_btn_pause_clicked()
{
    autorun->path_pause=true;
    ui->btn_start->setText("继续");
}

void MainWindow::on_btn_clear_clicked()
{
    if(series!=nullptr)
        series->clear();
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
        ui->chartView->chart()->update();
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
    rangeX0=xCentral - leftOffset;
    rangeX1=xCentral + rightOffset;
    rangeY0=yCentral - bottomOffset;
    rangeY1=yCentral + topOffset;

    ui->chartView->chart()->axisX()->setRange(rangeX0,rangeX1);
    ui->chartView->chart()->axisY()->setRange(rangeY0,rangeY1);
    ui->chartView->chart()->update();
}

void MainWindow::on_btn_showPoints_clicked()
{
    static bool showPoint=false;
    if(series!=nullptr){
        if(showPoint){
            series->setPointsVisible(false);
            series->setPointLabelsVisible(false);
            ui->btn_showPoints->setText("显示数据");
        }
        else{
            series->setPointsVisible(true);
            series->setPointLabelsVisible(true);
            ui->btn_showPoints->setText("隐藏数据");
        }
        if(showPoint) showPoint=false;
        else showPoint=true;
    }
}
