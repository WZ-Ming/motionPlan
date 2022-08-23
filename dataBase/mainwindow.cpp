#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    labelHint(new QLabel),
    labelComBox(new QLabel),
    tableComBox(new QComboBox),
    Vbox(new QVBoxLayout),
    instrListWidget(new QListWidget)
{
    ui->setupUi(this);
    labelHint->setText("提示:");
    ui->statusBar->addWidget(labelHint);
    instrListWidget->setWindowTitle("指令");
    instrListWidget->setWindowFlags(instrListWidget->windowFlags() &~ Qt::WindowMaximizeButtonHint &~ Qt::WindowMinimizeButtonHint);//& Qt::WindowStaysOnTopHint
    connect(instrListWidget,&QListWidget::clicked,[this](const QModelIndex &index){
        ui->lineEdit_command->setText(instrListWidget->item(index.row())->text().split("> ").at(1));
    });
    DB=QSqlDatabase::addDatabase("QSQLITE");
    ui->frame_edit->setLayout(Vbox);
    labelComBox->setText("数据库表格:");
    tableComBox->setSizeAdjustPolicy(QComboBox::SizeAdjustPolicy::AdjustToContents);
    ui->toolBar->addSeparator();
    ui->toolBar->addWidget(labelComBox);
    ui->toolBar->addWidget(tableComBox);
    setControlEnable(false);
}

MainWindow::~MainWindow()
{
    if(sqlModel!=nullptr){
        bool InstrTableExist=true;
        if(!DB.tables().contains(InstrTable)){
            QSqlQuery query(sqlModel->database());
            query.exec("CREATE TABLE "+InstrTable+"(Time TEXT PRIMARY KEY NOT NULL,Instruction TEXT NOT NULL)");
            if(!query.isActive()){
                InstrTableExist=false;
                ui->statusBar->showMessage("ERROR: "+query.lastError().text(),5000);
            }
        }
        if(InstrTableExist){
            QSqlQuery query(sqlModel->database());
            QString str;
            for(int i=0;i<instrListWidget->count();i++){
                QStringList instrList=instrListWidget->item(i)->text().split("> ");
                str="INSERT INTO "+InstrTable+" VALUES ('"+instrList.at(0)+"','"+instrList.at(1)+"')";
                query.exec(str);
                if(!query.isActive())
                    ui->statusBar->showMessage("ERROR: "+query.lastError().text(),5000);
            }
        }
    }

    delete ui;
    if(sqlModel!=nullptr) delete sqlModel;
    if(labelComBox!=nullptr) delete labelComBox;
    if(tableComBox!=nullptr) delete tableComBox;
    if(Vbox!=nullptr) delete Vbox;
    if(labelHint!=nullptr) delete labelHint;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    if(columnSet)
        columnSet->close();
    instrListWidget->close();
}

void MainWindow::setControlEnable(bool enable)
{
    centralWidget()->setEnabled(enable);
    ui->act_save->setEnabled(enable);
    ui->act_new->setEnabled(enable);
    ui->act_delete->setEnabled(enable);
}

void MainWindow::updateTabView()
{
    sqlModel->setTable(tableComBox->currentText());
    if(tableComBox->currentText()==InstrTable)
        sqlModel->setSort(0,Qt::DescendingOrder);
    else
        sqlModel->setSort(0,Qt::AscendingOrder);
    sqlModel->setEditStrategy(QSqlTableModel::OnManualSubmit);
    if(sqlModel->select()){
        QSqlRecord rec=sqlModel->record();
        int i=0;
        for(;i<rec.count();i++){
            if(editVector.size()-1<i){
                QHBoxLayout *Hbox=new QHBoxLayout;
                editControl editControlTmp;
                editControlTmp.labelTmp=new QLabel;
                editControlTmp.lineEditTmp=new QLineEdit;
                editControlTmp.HBoxTmp=Hbox;
                editControlTmp.dataType=rec.value(i).type();
                editVector.append(editControlTmp);
                editControlTmp.labelTmp->setText(rec.fieldName(i));
                editControlTmp.labelTmp->setSizePolicy(QSizePolicy::Preferred,QSizePolicy::Preferred);
                editControlTmp.lineEditTmp->setSizePolicy(QSizePolicy::Preferred,QSizePolicy::Preferred);
                Hbox->addWidget(editControlTmp.labelTmp);
                Hbox->addWidget(editControlTmp.lineEditTmp);
                Vbox->addLayout(Hbox);
            }
            else{
                editVector[i].labelTmp->setText(rec.fieldName(i));
                editVector[i].dataType=rec.value(i).type();
                editVector[i].lineEditTmp->clear();
            }
            if(editVector[i].dataType==QVariant::Int){
                editVector[i].lineEditTmp->setPlaceholderText("请输入整型");
                editVector[i].lineEditTmp->setValidator(new QIntValidator(this));
            }
            else if(editVector[i].dataType==QVariant::Bool){
                editVector[i].lineEditTmp->setPlaceholderText("请输入bool型(0,1)");
                editVector[i].lineEditTmp->setValidator(new QIntValidator(0,1,this));
            }
            else if(editVector[i].dataType==QVariant::Double){
                editVector[i].lineEditTmp->setPlaceholderText("请输入浮点型");
                editVector[i].lineEditTmp->setValidator(new QDoubleValidator(this));
            }
            else{
                editVector[i].lineEditTmp->setPlaceholderText("字符串类型");
                editVector[i].lineEditTmp->setValidator(nullptr);
            }
        }
        int i_tmp=0;
        if(editVector.size()>i){
            for(;i<editVector.size();i++){
                delete editVector[i].HBoxTmp;
                delete editVector[i].labelTmp;
                delete editVector[i].lineEditTmp;
                i_tmp++;
            }
        }
        for(int i=0;i<i_tmp;i++)
            editVector.removeLast();

        if(sqlModel->rowCount()>0){
            ui->tableView->selectRow(0);
            rec=sqlModel->record(0);
            for(int i=0;i<rec.count();i++)
                editVector[i].lineEditTmp->setText(rec.value(i).toString());
        }
    }
    ui->tableView->resizeColumnsToContents();
}

void MainWindow::getColumnSetMsg(const QStringList &arg)
{
   QString str="ALTER TABLE "+tableComBox->currentText()+" ADD COLUMN "+arg.at(0)+" "+arg.at(1);
   if(!arg.at(2).isEmpty())
       str+=" DEFAULT "+arg.at(2);
   if(!arg.at(3).isEmpty())
       str+=" "+arg.at(3);
   if(!arg.at(4).isEmpty())
       str+=" "+arg.at(4);
   QSqlQuery query(sqlModel->database());
   query.exec(str);
   if(!query.isActive())
       ui->statusBar->showMessage("ERROR: "+query.lastError().text(),5000);
   else{
       ui->statusBar->showMessage("添加列成功!",5000);
       updateTabView();
   }
}

void MainWindow::on_act_new_triggered()
{
    QString text=QInputDialog::getText(this,"创建数据表","请输入数据表名");
    if(!text.isEmpty()){
        QSqlQuery query(sqlModel->database());
        query.exec("CREATE TABLE "+text+"(ID INT PRIMARY KEY NOT NULL)");
        if(!query.isActive())
            ui->statusBar->showMessage("ERROR: "+query.lastError().text(),5000);
        else{
            tableComBox->addItem(text);
            tableComBox->setCurrentText(text);
        }
    }
}

void MainWindow::on_act_open_triggered()
{
    QString file_select_name = QFileDialog::getOpenFileName(this, "打开数据库", "", tr("DB files(*.db)"));
    if(file_select_name.isEmpty()) return;
    DB.setDatabaseName(file_select_name);
    if(DB.open()){
        tableComBox->disconnect();
        tableComBox->clear();
        if(sqlModel!=nullptr) delete sqlModel;
        sqlModel=new QSqlTableModel(this,DB);
        ui->tableView->verticalHeader()->setVisible(false);
        ui->tableView->setModel(sqlModel);
        ui->tableView->setEditTriggers(QAbstractItemView::NoEditTriggers);

        if(DB.tables().contains(InstrTable)){
            sqlModel->setTable(InstrTable);
            sqlModel->setSort(0,Qt::DescendingOrder);
            if(sqlModel->select()){
                QSqlRecord rec;
                for(int i=0;i<sqlModel->rowCount();i++){
                    rec=sqlModel->record(i);
                    QString str;
                    for(int j=0;j<rec.count();j++){
                        str+=rec.value(j).toString();
                        str+="> ";
                    }
                    str.remove(str.size()-2,2);
                    instrListWidget->addItem(str);
                }
            }
        }

        connect(tableComBox,&QComboBox::currentTextChanged,[this](){//QString text
            updateTabView();
        });
        QStringList tableList=DB.tables();
        foreach(QString str,tableList)
            tableComBox->addItem(str);

        setControlEnable(true);
    }
}

void MainWindow::on_act_save_triggered()
{
    if(sqlModel->submitAll())
        ui->statusBar->showMessage("save successful!",5000);
    else
        ui->statusBar->showMessage("ERROR: "+sqlModel->lastError().text(),5000);
}

void MainWindow::on_act_delete_triggered()
{
    if(!tableComBox->currentText().isEmpty()){
        QSqlQuery query(sqlModel->database());
        query.exec("DROP TABLE "+tableComBox->currentText());
        if(!query.isActive())
            ui->statusBar->showMessage("ERROR: "+query.lastError().text(),5000);
        else
            tableComBox->removeItem(tableComBox->currentIndex());
    }
}

void MainWindow::on_btn_insertRow_clicked()
{
    QSqlRecord rec=sqlModel->record();
    for(int i=0;i<rec.count();i++)
        rec.setValue(i,editVector[i].lineEditTmp->text());
    sqlModel->insertRecord(ui->tableView->currentIndex().row()+1,rec);
    ui->tableView->selectRow(ui->tableView->currentIndex().row()+1);
}

void MainWindow::on_btn_deleteRow_clicked()
{
    QModelIndexList IndexList = ui->tableView->selectionModel()->selectedIndexes();
    QVector<int> vector;
    foreach(QModelIndex index,IndexList)
        vector.push_back(index.row());
    if(vector.count()==0) return;
    QVector<int>::iterator it=std::unique(vector.begin(),vector.end());
    for(QVector<int>::iterator itTmp=it-1;itTmp>=vector.begin();itTmp--){
        if(tableComBox->currentText()==InstrTable){
            QListWidgetItem* itemTmp=instrListWidget->takeItem(*itTmp);
            delete itemTmp;
        }
        sqlModel->removeRow(*itTmp);
    }
    if(!sqlModel->submitAll())
        ui->statusBar->showMessage("ERROR: "+sqlModel->lastError().text(),5000);
    //updateTabView();
    ui->tableView->selectRow(*(it-1));
}

void MainWindow::on_btn_insertColumn_clicked()
{
    if(!columnSet){
        columnSet=new ColumnSetting;
        connect(columnSet,&ColumnSetting::sendColumnSetMsg,this,&MainWindow::getColumnSetMsg);
        columnSet->setAttribute(Qt::WA_DeleteOnClose, true);
    }
    else
        columnSet->setWindowFlags(Qt::WindowStaysOnTopHint);
    columnSet->setWindowFlags(instrListWidget->windowFlags() &~ Qt::WindowMaximizeButtonHint &~ Qt::WindowMinimizeButtonHint);
    columnSet->show();
}

void MainWindow::on_btn_hideColumn_clicked()
{
    QModelIndexList IndexList = ui->tableView->selectionModel()->selectedIndexes();
    QVector<int> vector;
    foreach(QModelIndex index,IndexList)
        vector.push_back(index.column());
    if(vector.count()==0) return;
    QVector<int>::iterator it=std::unique(vector.begin(),vector.end());
    for(QVector<int>::iterator itTmp=vector.begin();itTmp<it;itTmp++)
        ui->tableView->setColumnHidden(*itTmp,true);
}

void MainWindow::on_btn_alter_clicked()
{
    int row=ui->tableView->currentIndex().row();
    if(row<0){
        ui->statusBar->showMessage("未选中行!",5000);
        return;
    }
    QSqlRecord rec=sqlModel->record(row);
    for(int i=0;i<editVector.size();i++){
        if(editVector[i].dataType==QVariant::Int || editVector[i].dataType==QVariant::Bool)
            rec.setValue(i,editVector[i].lineEditTmp->text().toInt());
        else if(editVector[i].dataType==QVariant::Double)
            rec.setValue(i,editVector[i].lineEditTmp->text().toDouble());
        else
            rec.setValue(i,editVector[i].lineEditTmp->text());
    }
    sqlModel->setRecord(row,rec);
}

void MainWindow::on_tableView_clicked(const QModelIndex &index)
{
    QSqlRecord rec=sqlModel->record(index.row());
    for(int i=0;i<rec.count();i++)
        editVector[i].lineEditTmp->setText(rec.value(i).toString());
}

void MainWindow::on_btn_revoke_clicked()
{
    sqlModel->revertRow(ui->tableView->currentIndex().row());
}

void MainWindow::on_btn_execute_clicked()
{
    QString instr=QDate::currentDate().toString("yyyy/MM/dd")+" "+QTime::currentTime().toString("hh:mm:ss.zzz")+"> ";
    if(!ui->lineEdit_command->text().isEmpty()){
        instrListWidget->insertItem(0,instr+ui->lineEdit_command->text());
        QSqlQuery query(sqlModel->database());
        query.exec(ui->lineEdit_command->text());
        if(query.isActive()){
            if(!sqlModel->submitAll())
                ui->statusBar->showMessage("ERROR: "+sqlModel->lastError().text(),5000);
            updateTabView();
        }
        else
            ui->statusBar->showMessage("ERROR: "+query.lastError().text(),5000);
    }
}

void MainWindow::on_btn_find_clicked()
{
    sqlModel->setFilter(ui->lineEdit_filter->text().trimmed());
}

void MainWindow::on_Tbtn_sort_clicked(bool checked)
{
    QItemSelectionModel *selectModelTmp=ui->tableView->selectionModel();
    QModelIndexList IndexList = selectModelTmp->selectedIndexes();
    if(IndexList.size()>0){
        IndexList.at(0).column();
        Qt::SortOrder order;
        if(checked){
            order=Qt::AscendingOrder;
            ui->Tbtn_sort->setText("升序");
        }
        else{
            order=Qt::DescendingOrder;
            ui->Tbtn_sort->setText("降序");
        }
        sqlModel->sort(IndexList.at(0).column(),order);
        ui->tableView->selectColumn(IndexList.at(0).column());
    }
}

void MainWindow::on_toolBtn_oldInstruction_clicked()
{
    QRect rect=ui->lineEdit_command->geometry();
    instrListWidget->setGeometry(rect.x()+this->geometry().x(),this->geometry().y()+rect.y()-rect.height()*10+ui->toolBar->height(),rect.width(),rect.height()*10);
    instrListWidget->show();
    instrListWidget->setWindowState(Qt::WindowActive);
    instrListWidget->activateWindow();
}
