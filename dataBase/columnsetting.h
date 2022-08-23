#ifndef COLUMNSETTING_H
#define COLUMNSETTING_H

#include <QWidget>

namespace Ui {
class ColumnSetting;
}

class ColumnSetting : public QWidget
{
    Q_OBJECT

public:
    explicit ColumnSetting(QWidget *parent = nullptr);
    ~ColumnSetting();
    
    Ui::ColumnSetting *ui;
private slots:
    void on_btn_setDone_clicked();
    
signals:
    void sendColumnSetMsg(const QStringList &);

};

#endif // COLUMNSETTING_H
