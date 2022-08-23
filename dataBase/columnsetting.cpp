#include "columnsetting.h"
#include "ui_columnsetting.h"

ColumnSetting::ColumnSetting(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ColumnSetting)
{
    ui->setupUi(this);
    ui->comboBox_dataType->addItem("NULL");
    ui->comboBox_dataType->addItem("INTEGER");
    ui->comboBox_dataType->addItem("REAL");
    ui->comboBox_dataType->addItem("TEXT");
    ui->comboBox_dataType->addItem("BLOB");
}

ColumnSetting::~ColumnSetting()
{
    delete ui;
}

void ColumnSetting::on_btn_setDone_clicked()
{
    if(ui->lineEdit_columnName->text().isEmpty()){
        return;
    } 
    QStringList strList;
    strList.append(ui->lineEdit_columnName->text());
    strList.append(ui->comboBox_dataType->currentText());
    strList.append(ui->lineEdit_defaultValue->text());
    if(ui->checkBox_NULL->isChecked())
        strList.append("NOT NULL");
    else
        strList.append(QString());
    if(ui->checkBox_UNIQUE->isChecked())
        strList.append("UNIQUE");
    else
        strList.append(QString());
    emit sendColumnSetMsg(strList);
}
