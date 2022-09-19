#ifndef MYCHART_H
#define MYCHART_H

#include <QWidget>
#include<QtCharts>
#include <QWheelEvent>
#include <QMouseEvent>

using namespace QtCharts;

namespace Ui {
class myChart;
}

class myChart : public QWidget
{
    Q_OBJECT

public:
    explicit myChart(uchar type,QWidget *parent = nullptr);
    ~myChart();
    void clearData();
    void showData(bool showPoint);
    enum chartShowType{disChartType=0,spdChartType,accChartType};

protected:
    void wheelEvent(QWheelEvent* event);

    void mousePressEvent(QMouseEvent *event);

    void mouseMoveEvent(QMouseEvent *event);

    void mouseReleaseEvent(QMouseEvent *event);

public slots:
    void rec_data(double, double, double , double , double);

private:
    void init_chart();

    chartShowType chartType;
    bool drag_flag=false;
    QPoint m_lastPointF;
    double rangeX0=0,rangeX1=1,rangeY0=0,rangeY1=1;

    Ui::myChart *ui;
};

#endif // MYCHART_H
