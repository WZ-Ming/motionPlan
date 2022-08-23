#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSqlDatabase>
#include <QSqlTableModel>
#include <QSqlRecord>
#include <QItemSelectionModel>
#include <QFileDialog>
#include <QDebug>
#include <QTime>
#include <QTimer>
#include <QLabel>
#include <QLineEdit>
#include <QVector>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include<QSpacerItem>
#include<QMessageBox>
#include<QSqlQuery>
#include<QSqlError>
#include<QSqlField>
#include<QComboBox>
#include<QString>
#include<QListWidget>
#include<QTime>
#include<QDate>
#include<QInputDialog>
#include<QPointer>
#include<QVariant>
#include<QValidator>
#include"columnsetting.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

protected:
    void closeEvent(QCloseEvent *event);

private slots:
    void getColumnSetMsg(const QStringList &arg);

    void updateTabView();
    
    void on_btn_alter_clicked();

    //void on_tableView_customContextMenuRequested(const QPoint &pos);

    void on_tableView_clicked(const QModelIndex &index);

    void on_btn_revoke_clicked();

    void on_btn_insertRow_clicked();

    void on_btn_deleteRow_clicked();

    void on_btn_execute_clicked();

    void on_act_open_triggered();

    void on_act_save_triggered();

    void on_toolBtn_oldInstruction_clicked();

    void on_act_new_triggered();

    void on_act_delete_triggered();

    void on_btn_insertColumn_clicked();
        
    void on_btn_hideColumn_clicked();

    void on_btn_find_clicked();

    void on_Tbtn_sort_clicked(bool checked);

private:
    void setControlEnable(bool enable);

private:
    const QString InstrTable="INSTRUCTION";
    QSqlDatabase DB;
    struct editControl{
        QLabel* labelTmp=nullptr;
        QLineEdit* lineEditTmp=nullptr;
        QHBoxLayout* HBoxTmp=nullptr;
        QVariant::Type dataType=QVariant::Type::Invalid;
    };
    QVector<editControl>editVector;
    QSqlTableModel *sqlModel=nullptr;
    //QItemSelectionModel *selectModel=nullptr;

    Ui::MainWindow *ui;
    QLabel* labelHint=nullptr;
    QLabel* labelComBox=nullptr;
    QComboBox* tableComBox=nullptr;
    QVBoxLayout *Vbox=nullptr;
    QListWidget* instrListWidget=nullptr;
    QPointer<ColumnSetting> columnSet;
};

#endif // MAINWINDOW_H
