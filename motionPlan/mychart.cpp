#include "mychart.h"
#include "ui_mychart.h"

myChart::myChart(uchar type, QWidget *parent) :
    QWidget(parent),
    chartType(static_cast<chartShowType>(type)),
    ui(new Ui::myChart)
{
    ui->setupUi(this);
    setMouseTracking(true);
    ui->chartView->setAttribute(Qt::WA_TransparentForMouseEvents, true);
    switch (chartType) {
    case disChartType:setWindowTitle("位移时间曲线");break;
    case spdChartType:setWindowTitle("速度时间曲线");break;
    case accChartType:setWindowTitle("加速度时间曲线");break;
    }
    init_chart();
}

myChart::~myChart()
{
    delete ui;
}

void myChart::rec_data(double realData, double allData, double vs, double acc,double time)
{
    QLineSeries *series=qobject_cast<QLineSeries *>(ui->chartView->chart()->series().at(0));
    static double chartData=0;
    static bool outRange=false;

    switch (chartType) {
    case disChartType:{
        chartData=allData;
        series->setName("耗时:"+QString::number(time)+"秒   总位移:"+QString::number(allData));
    }break;
    case spdChartType:{
        chartData=vs;
        series->setName("耗时:"+QString::number(time)+"秒   当前速度:"+QString::number(vs));
    }break;
    case accChartType:{
        chartData=acc;
        series->setName("耗时:"+QString::number(time)+"秒   当前加速度:"+QString::number(acc));
    }break;}

    series->append(time,chartData);
    if(time<=rangeX0){
        rangeX0=time;
        outRange=true;
    }
    else if(time>=rangeX1){
        rangeX1=time;
        outRange=true;
    }
    if(chartData<=rangeY0){
        rangeY0=chartData;
        outRange=true;
    }
    else if(chartData>=rangeY1){
        rangeY1=chartData;
        outRange=true;
    }
    if(outRange){
        ui->chartView->chart()->axisX()->setRange(rangeX0,rangeX1);
        ui->chartView->chart()->axisY()->setRange(rangeY0,rangeY1);
        outRange=false;
    }
}

void myChart::init_chart()
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
    switch (chartType) {
    case disChartType:{
        axisY->setTitleText("位移");
        pen.setColor(Qt::red);
    }break;
    case spdChartType:{
        axisY->setTitleText("速度");
        pen.setColor(Qt::green);
    }break;
    case accChartType:{
        axisY->setTitleText("加速度");
        pen.setColor(Qt::blue);
    }break;}
    QLineSeries *series=new QLineSeries;
    series->setName("耗时");
    series->setPen(pen);
    chart->addSeries(series);
    chart->setAxisX(axisX,series);
    chart->setAxisY(axisY,series);
    axisX->setRange(rangeX0,rangeX1);
    axisY->setRange(rangeY0,rangeY1);
}

void myChart::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton){
        drag_flag = true;
        m_lastPointF=event->pos();
        setCursor(Qt::OpenHandCursor);
    }
}

void myChart::mouseReleaseEvent(QMouseEvent *event)
{
    drag_flag = false;
    setCursor(Qt::ArrowCursor);
}

void myChart::mouseMoveEvent(QMouseEvent *event)
{
    if (drag_flag){
        QPoint curPos = event->pos();
        QPoint offset = curPos - m_lastPointF;
        m_lastPointF = curPos;
        ui->chartView->chart()->scroll(-offset.x(), offset.y());
        ui->chartView->chart()->update();
    }
}

void myChart::wheelEvent(QWheelEvent *event)
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

void myChart::clearData()
{
    QLineSeries *series=qobject_cast<QLineSeries *>(ui->chartView->chart()->series().at(0));
    series->clear();
}

void myChart::showData(bool showPoint)
{
    QLineSeries *series=qobject_cast<QLineSeries *>(ui->chartView->chart()->series().at(0));
    if(showPoint){
        series->setPointsVisible(false);
        series->setPointLabelsVisible(false);
    }
    else{
        series->setPointsVisible(true);
        series->setPointLabelsVisible(true);
    }
}


