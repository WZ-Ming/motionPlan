#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    disChart(new myChart(0,this)),
    spdChart(new myChart(1,this)),
    accChart(new myChart(2,this)),
    dataTableWidget(new QTableWidget(this)),
    autorun(new autoRun())
{
    ui->setupUi(this);
    setWindowTitle("静态前瞻式7段S型速度规划");
#ifdef motionDebug
    connect(autorun,&autoRun::sendMotionPlanMsg,this,&MainWindow::recMotionPlanMsg);
#endif
    connect(autorun,&autoRun::send_data,this,&MainWindow::rec_data,Qt::BlockingQueuedConnection);
    connect(this,&MainWindow::send_data,disChart,&myChart::rec_data);//,Qt::BlockingQueuedConnection
    connect(this,&MainWindow::send_data,spdChart,&myChart::rec_data);
    connect(this,&MainWindow::send_data,accChart,&myChart::rec_data);
    connect(autorun,&autoRun::send_cal_done_sig,this,&MainWindow::rec_cal_done_sig);

    ui->lineEdit_V0->setValidator(new QDoubleValidator(this));
    ui->lineEdit_Ve->setValidator(new QDoubleValidator(this));
    ui->lineEdit_Jerk->setValidator(new QDoubleValidator(this));
    ui->lineEdit_VMax->setValidator(new QDoubleValidator(this));
    ui->lineEdit_distance->setValidator(new QDoubleValidator(this));
    ui->lineEdit_acc->setValidator(new QDoubleValidator(this));
    ui->lineEdit_planSingleTime->setValidator(new QDoubleValidator(this));

    ui->mdiArea->setMouseTracking(true);
    ui->mdiArea->setContextMenuPolicy(Qt::CustomContextMenu);
    ui->mdiArea->addSubWindow(dataTableWidget,Qt::WindowMinMaxButtonsHint | Qt::WindowTitleHint);
    ui->mdiArea->addSubWindow(accChart,Qt::WindowMinMaxButtonsHint | Qt::WindowTitleHint);
    ui->mdiArea->addSubWindow(spdChart,Qt::WindowMinMaxButtonsHint | Qt::WindowTitleHint);
    ui->mdiArea->addSubWindow(disChart,Qt::WindowMinMaxButtonsHint | Qt::WindowTitleHint);
    ui->mdiArea->tileSubWindows();
    ui->mdiArea->setOption(QMdiArea::DontMaximizeSubWindowOnActivation,true);

    ui->dockWidget->setWindowTitle(tr("参数设置"));
    dataTableWidget->setWindowTitle("规划数据");
    initTableWidget();
}

MainWindow::~MainWindow()
{
    delete ui;
    if(autorun!=nullptr){
        if(autorun->isRunning()){
            autorun->forceQuit=true;
            autorun->quit();
            autorun->wait();
        }
        delete autorun;
    }
    delete disChart;
    delete spdChart;
    delete accChart;
    delete dataTableWidget;
}

void MainWindow::initTableWidget()
{
   dataTableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
   dataTableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
   QTableWidgetItem *headerItem;
   QStringList widgethead;
   widgethead<<"总时间"<<"总位移"<<"当前时刻位移"<<"当前时刻速度"<<"当前时刻加速度";
   dataTableWidget->setHorizontalHeaderLabels(widgethead);
   dataTableWidget->setColumnCount(widgethead.size());
   for(int i=0;i<dataTableWidget->columnCount();i++)
   {
       headerItem=new QTableWidgetItem(widgethead.at(i));
       dataTableWidget->setHorizontalHeaderItem(i,headerItem);
   }
}

void MainWindow::rec_data(double realData, double allData, double vs, double acc,double time)
{
    static int row=0;
    dataTableWidget->setRowCount(dataTableWidget->rowCount()+1);
    row=dataTableWidget->rowCount()-1;
    dataTableWidget->setItem(row,0,new QTableWidgetItem(QString::number(time)));
    dataTableWidget->item (row,0)->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
    dataTableWidget->setItem(row,1,new QTableWidgetItem(QString::number(allData)));
    dataTableWidget->item (row,1)->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
    dataTableWidget->setItem(row,2,new QTableWidgetItem(QString::number(realData)));
    dataTableWidget->item (row,2)->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
    dataTableWidget->setItem(row,3,new QTableWidgetItem(QString::number(vs)));
    dataTableWidget->item (row,3)->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
    dataTableWidget->setItem(row,4,new QTableWidgetItem(QString::number(acc)));
    dataTableWidget->item (row,4)->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
    dataTableWidget->scrollToBottom();

    emit send_data(realData,allData,vs,acc,time);
}

void MainWindow::recMotionPlanMsg(const QString &arg)
{
    ui->textEdit->append(arg);
}

void MainWindow::rec_cal_done_sig()
{
    autorun->quit();
    autorun->wait();
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
        //autorun->pathInit.dec_jerk_muti=ui->doubleSpinBox_decMuti->value();
        autorun->pathInit.deltaT=ui->lineEdit_planSingleTime->text().toDouble();
        disChart->clearData();
        spdChart->clearData();
        accChart->clearData();
        while(dataTableWidget->rowCount()>0)
            dataTableWidget->removeRow(0);
        ui->textEdit->clear();
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

void MainWindow::on_btn_cancelPlan_clicked()
{
    if(autorun->isRunning()){
        autorun->forceQuit=true;
        ui->btn_start->setText("启动");
    }
}

void MainWindow::on_btn_showPoints_clicked()
{
    static bool showPoint=false;
    if(showPoint)
        ui->btn_showPoints->setText("显示数据");
    else
        ui->btn_showPoints->setText("隐藏数据");
    disChart->showData(showPoint);
    spdChart->showData(showPoint);
    accChart->showData(showPoint);
    if(showPoint) showPoint=false;
    else showPoint=true;
}

void MainWindow::on_mdiArea_customContextMenuRequested(const QPoint &pos)
{
    QAction act_menu("参数菜单显示", this);
    QAction act_cascadeShow("级联模式",this);
    QAction act_tileShow("平铺模式",this);
    connect(&act_menu, &QAction::triggered, this, &MainWindow::showMenu);
    connect(&act_cascadeShow, &QAction::triggered, this, &MainWindow::cascadeShow);
    connect(&act_tileShow, &QAction::triggered, this, &MainWindow::tileShow);
    QMenu menu,mdiShow("图表显示模式");
    menu.addAction(&act_menu);
    menu.addMenu(&mdiShow);
    mdiShow.addAction(&act_cascadeShow);
    mdiShow.addAction(&act_tileShow);
    menu.exec(QCursor::pos());
}

void MainWindow::cascadeShow()
{
    ui->mdiArea->cascadeSubWindows();
}

void MainWindow::tileShow()
{
    ui->mdiArea->tileSubWindows();
}

void MainWindow::showMenu()
{
    if(ui->dockWidget->isHidden())
        ui->dockWidget->show();
}
