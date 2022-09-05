#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    initStatusLabel();
    sendthread=new sendThread();
    sendthread->moveToThread(sendthread);
    connect(this,&MainWindow::handleCmd,sendthread,&sendThread::handleCmd);
    connect(sendthread,&sendThread::onSocketStateChange,this,&MainWindow::onSocketStateChange);
    connect(sendthread,&sendThread::onServerErrorOccurs,this,&MainWindow::onServerErrorOccurs);
    connect(sendthread,&sendThread::onClientErrorOccurs,this,&MainWindow::onClientErrorOccurs);
    connect(sendthread,&sendThread::onNewClientconnected,this,&MainWindow::onNewClientconnected);
    connect(sendthread,&sendThread::onClientDisconnected,this,&MainWindow::onClientDisconnected);
    connect(sendthread,&sendThread::sendActionSig,this,&MainWindow::recActionSig);
    connect(sendthread,&sendThread::sendRecState,this,&MainWindow::getRecState);
    connect(sendthread,&sendThread::sendTransState,this,&MainWindow::getTransState);
    sendthread->start();
    getIp();
    loadSettings();
    QFont font=this->font();
    font.setPointSize(11);
    this->setFont(font);
    connect(&errorClearTimer,&QTimer::timeout,[this](){
        errorClearTimer.stop();
        serverErrorLabel->setPalette(ui->statusbar->palette());
        serverErrorLabel->clear();
        clientErrorLabel->setPalette(ui->statusbar->palette());
        clientErrorLabel->clear();
    });
    sendthread->bufferSize=ui->spinBox_bufferSize->value();
}

MainWindow::~MainWindow()
{
    saveSettings();
    sendthread->quit();
    sendthread->wait();
    delete sendthread;
    delete ui;
    delete serverErrorLabel;
    delete listenStateLabel;
    delete socketStateLabel;
    delete clientErrorLabel;
    delete sendProgress;
    delete recProgress;
}

void MainWindow::loadSettings()
{
    QFile afile(IniFilePath);
    if (afile.exists()){
        QSettings readIniFile(IniFilePath, QSettings::IniFormat);

        if (readIniFile.contains("saveFilePath"))
            ui->dirShowLineEdit->setText(readIniFile.value("saveFilePath").toString());
        else
            ui->dirShowLineEdit->setText(QCoreApplication::applicationDirPath());
        if(readIniFile.contains("socketType")){
            if(readIniFile.value("socketType").toString()=="server"){
                if(ui->stackedWidget->currentIndex()==0)
                    on_stackedWidget_currentChanged(0);
                else
                    ui->stackedWidget->setCurrentIndex(0);
                if(ui->localIpComBox->count()>0)
                    ui->act_startListen->trigger();
            }
            else{
                if(ui->stackedWidget->currentIndex()==1)
                    on_stackedWidget_currentChanged(1);
                else
                    ui->stackedWidget->setCurrentIndex(1);
            }
        }
        if(readIniFile.contains("serverAddress"))
            ui->serverAddressLineEdit->setText(readIniFile.value("serverAddress").toString());
        else
            ui->serverAddressLineEdit->setText("192.168.0.0");
    }
    else{
        if(ui->stackedWidget->currentIndex()==0)
            on_stackedWidget_currentChanged(0);
        else
            ui->stackedWidget->setCurrentIndex(0);
        if(ui->localIpComBox->count()>0)
            ui->act_startListen->trigger();
        ui->dirShowLineEdit->setText(QCoreApplication::applicationDirPath());
        ui->serverAddressLineEdit->setText("192.168.0.0");
    }
}

void MainWindow::saveSettings()
{
    QSettings writeIniFile(IniFilePath, QSettings::IniFormat);

    writeIniFile.setValue("saveFilePath",ui->dirShowLineEdit->text());
    if(ui->stackedWidget->currentIndex()==0)
        writeIniFile.setValue("socketType","server");
    else
        writeIniFile.setValue("socketType","client");
    writeIniFile.setValue("serverAddress",ui->serverAddressLineEdit->text());
}

void MainWindow::getIp()
{
    ui->localIpComBox->clear();
#ifdef Q_OS_LINUX
    foreach (QHostAddress hostAddress, QNetworkInterface::allAddresses())  {
        if(hostAddress.protocol() == QAbstractSocket::IPv4Protocol){
            if(hostAddress.toString()!="127.0.0.1")
                ui->localIpComBox->addItem(hostAddress.toString());
        }
     }
#endif
#ifdef Q_OS_WIN
    QString hostName=QHostInfo::localHostName();
    QHostInfo hostInfo=QHostInfo::fromName(hostName);
    foreach(QHostAddress hostAddress,hostInfo.addresses()){
       if(hostAddress.protocol()==QAbstractSocket::IPv4Protocol)
           ui->localIpComBox->addItem(hostAddress.toString());
    }
#endif
}

void MainWindow::initStatusLabel()
{
    listenStateLabel=new QLabel(this);
    serverErrorLabel=new QLabel(this);
    listenStateLabel->setAutoFillBackground(true);
    serverErrorLabel->setAutoFillBackground(true);
    ui->statusbar->addWidget(listenStateLabel);
    ui->statusbar->addPermanentWidget(serverErrorLabel);

    socketStateLabel=new QLabel(this);
    clientErrorLabel=new QLabel(this);
    socketStateLabel->setAutoFillBackground(true);
    clientErrorLabel->setAutoFillBackground(true);
    ui->statusbar->addWidget(socketStateLabel);
    ui->statusbar->addPermanentWidget(clientErrorLabel);
    sendProgress=new QProgressBar(this);
    sendProgress->setFormat("发送:%p%");
    ui->statusbar->addWidget(sendProgress);
    sendProgress->setVisible(false);
    recProgress=new QProgressBar(this);
    recProgress->setFormat("接收:%p%");
    ui->statusbar->addWidget(recProgress);
    recProgress->setVisible(false);
}

void MainWindow::setControlEnable(bool enable, transType trans_type)
{
    if(sendthread->sendSocket_type==sendThread::server){
        ui->act_stopListen->setEnabled(enable);
        //ui->act_startListen->setEnabled(enable);
    }
    else if(sendthread->sendSocket_type==sendThread::client){
        //ui->act_connect->setEnabled(enable);
        ui->act_disconnect->setEnabled(enable);
    }
    if(sendthread->recSocket_type==sendThread::server){
        ui->act_stopListen->setEnabled(enable);
        //ui->act_startListen->setEnabled(enable);
    }
    else if(sendthread->recSocket_type==sendThread::client){
        //ui->act_connect->setEnabled(enable);
        ui->act_disconnect->setEnabled(enable);
    }
    if(trans_type==send){
        ui->testBtn->setEnabled(enable);
        ui->fileTransmitBtn->setEnabled(enable);
    }
    else if(trans_type==receive){
        ui->dirShowLineEdit->setEnabled(enable);
    }
    if(enable && trans_type==send)
        sendthread->sendSocket_type=sendThread::noSocket;
    if(enable && trans_type==receive)
        sendthread->recSocket_type=sendThread::noSocket;
}

void MainWindow::onSocketStateChange(QAbstractSocket::SocketState state)
{
    switch(state){
        case QAbstractSocket::HostLookupState:socketStateLabel->setText(tr("正在查找主机"));break;
        case QAbstractSocket::BoundState:socketStateLabel->setText(tr("已绑定到地址和端口"));break;
        case QAbstractSocket::ClosingState:socketStateLabel->setText(tr("即将关闭"));break;
        case QAbstractSocket::ListeningState:socketStateLabel->setText(tr("正在监听中"));break;
        case QAbstractSocket::ConnectingState:{
            socketStateLabel->setText(tr("正在连接..."));
            socketStateLabel->setPalette(QPalette(QPalette::Background, Qt::yellow));
        }break;
        case QAbstractSocket::ConnectedState:{
            socketStateLabel->setText(tr("已建立连接"));
            socketStateLabel->setPalette(QPalette(QPalette::Background, Qt::green));
            ui->textEdit->append("已和服务器建立连接\n");
            ui->act_connect->setEnabled(false);
        }break;
        case QAbstractSocket::UnconnectedState:{
            socketStateLabel->setText(tr("已断开连接"));
            socketStateLabel->setPalette(QPalette(QPalette::Background, Qt::red));
            ui->textEdit->append("已和服务器断开连接\n");
            ui->act_connect->setEnabled(true);
        }break;
    }
}

void MainWindow::onServerErrorOccurs(QTcpServer* server)
{
    serverErrorLabel->setText(server->errorString());
    serverErrorLabel->setPalette(QPalette(QPalette::Background, Qt::red));
    errorClearTimer.start(5000);
}

void MainWindow::onClientErrorOccurs(QTcpSocket* client)
{
    clientErrorLabel->setText(client->errorString());
    clientErrorLabel->setPalette(QPalette(QPalette::Background, Qt::red));
    errorClearTimer.start(5000);
}

void MainWindow::onNewClientconnected(QTcpSocket* client)
{
    ui->textEdit->append("已和客户端建立连接");
    ui->textEdit->append("client address:"+client->peerAddress().toString());
    ui->textEdit->append("client port:"+QString::number(client->peerPort())+"\n");

    QString str=client->peerAddress().toString();
    QString socketMsg;
    if(str.contains("ffff:"))
        socketMsg=str.split("ffff:").at(1);
    else
        socketMsg=str;
    if(ui->clientcomBox->findText(socketMsg)==-1){
        clientVector.append(client);
        ui->clientcomBox->addItem(socketMsg);
    }
}

void MainWindow::onClientDisconnected()
{
    for(int i=0;i<clientVector.size();i++){
        if(clientVector[i]->state()==QAbstractSocket::UnconnectedState){
            ui->textEdit->append("已和客户端断开连接");
            ui->textEdit->append("client address:"+ui->clientcomBox->itemText(i)+"\n");
            ui->clientcomBox->removeItem(i);
            clientVector[i]->deleteLater();
            clientVector.remove(i);
        }
    }
}

void MainWindow::recActionSig(int type)
{
    switch (type) {
    case sendThread::startListenAct:{
        listenStateLabel->setText(tr("监听中..."));
        listenStateLabel->setPalette(QPalette(QPalette::Background, Qt::green));
        ui->act_startListen->setEnabled(false);
    }break;
    case sendThread::stopListenAct:{
        listenStateLabel->setText(tr("已停止监听"));
        listenStateLabel->setPalette(QPalette(QPalette::Background, Qt::yellow));
        ui->act_startListen->setEnabled(true);
    }break;
    case sendThread::testBtn:{
        ui->textEdit->append("未建立连接，无法发送!\n");
    }break;
    default:break;}
}

void MainWindow::getRecState(int state)
{
    static QTime recRecodeTime;
    switch (state) {
    case receiveThread::recDone:{
        ui->textEdit->append("接收完成!");
        ui->textEdit->append("文件接收大小:"+QString::number(receiveThread::fileRecSize));
        ui->textEdit->append("总接收Byte大小:"+QString::number(receiveThread::allRecSize));
        ui->textEdit->append("接收耗时:"+QString::number(recRecodeTime.elapsed()*0.001,'f',3)+"s\n");
        setControlEnable(true,receive);
        recProgress->setVisible(false);
    }break;
    case receiveThread::recBegin:{
        recRecodeTime.start();
        ui->textEdit->append("\n开始接收...");
        recProgress->setVisible(true);
        setControlEnable(false,receive);
        recProgress->setValue((double)receiveThread::fileRecSize/receiveThread::targetRecFileSize*100);
    }break;
    case receiveThread::recProcessing:{
        recProgress->setValue((double)receiveThread::fileRecSize/receiveThread::targetRecFileSize*100);
    }break;
    case receiveThread::recInterrupt:{
        ui->textEdit->append("接收超时，已强制中断!\n");
        setControlEnable(true,receive);
        recProgress->setVisible(false);
    }break;
    case receiveThread::recFileFail:ui->textEdit->append("创建文件失败,接收失败!\n");break;
    case receiveThread::recTest:ui->textEdit->append("[in] "+receiveThread::recByte);break;
    default:break;
    }
}

void MainWindow::on_act_startListen_triggered()
{
    emit handleCmd(sendThread::startListenAct);
}

void MainWindow::on_act_stopListen_triggered()
{
    emit handleCmd(sendThread::stopListenAct);
}

void MainWindow::on_act_connect_triggered()
{
    sendthread->serverIp=ui->serverAddressLineEdit->text().trimmed();
    emit handleCmd(sendThread::connectAct);
}

void MainWindow::on_act_disconnect_triggered()
{
    emit handleCmd(sendThread::disconnectAct);
}

void MainWindow::on_stackedWidget_currentChanged(int arg1)
{
    if(arg1==0){
        ui->act_server->setVisible(false);
        ui->act_connect->setVisible(false);
        ui->act_disconnect->setVisible(false);
        socketStateLabel->setVisible(false);
        clientErrorLabel->setVisible(false);
        listenStateLabel->setVisible(true);
        serverErrorLabel->setVisible(true);
        ui->act_client->setVisible(true);
        ui->act_stopListen->setVisible(true);
        ui->act_startListen->setVisible(true);
        this->setWindowTitle("服务端"+exeVersion);
    }
    else if(arg1==1){
        ui->act_server->setVisible(true);
        ui->act_connect->setVisible(true);
        ui->act_disconnect->setVisible(true);
        socketStateLabel->setVisible(true);
        clientErrorLabel->setVisible(true);
        listenStateLabel->setVisible(false);
        serverErrorLabel->setVisible(false);
        ui->act_client->setVisible(false);
        ui->act_stopListen->setVisible(false);
        ui->act_startListen->setVisible(false);
        this->setWindowTitle("客户端"+exeVersion);
    }
}

void MainWindow::on_act_server_triggered()
{
    ui->stackedWidget->setCurrentIndex(0);
}

void MainWindow::on_act_client_triggered()
{
    ui->stackedWidget->setCurrentIndex(1);
}

void MainWindow::on_testBtn_clicked()
{
    if(ui->stackedWidget->currentIndex()==0){
        if(ui->clientcomBox->count()==0) return;
        sendthread->send_socket=clientVector.at(ui->clientcomBox->currentIndex());
    }
    else
        sendthread->send_socket=sendthread->tcpClient;
    if(sendthread->send_socket->state()!=QAbstractSocket::ConnectedState){
        ui->textEdit->append("未建立连接，无法发送!");
        return;
    }
    emit handleCmd(sendThread::testBtn);
}

void MainWindow::on_clearTextBtn_clicked()
{
    ui->textEdit->clear();
}

void MainWindow::on_chooseDirBtn_clicked()
{
    QFileDialog fileDialog;
    QString selectedDir=fileDialog.getExistingDirectory(this,"选择文件夹",QCoreApplication::applicationDirPath(),QFileDialog::ShowDirsOnly);
    if(!selectedDir.isEmpty())
        ui->dirShowLineEdit->setText(selectedDir);
}

void MainWindow::on_fileTransmitBtn_clicked()
{
    if(ui->stackedWidget->currentIndex()==0){
        if(ui->clientcomBox->count()==0) return;
        sendthread->send_socket=clientVector.at(ui->clientcomBox->currentIndex());
        sendthread->sendSocket_type=sendThread::server;
    }
    else{
        sendthread->send_socket=sendthread->tcpClient;
        sendthread->sendSocket_type=sendThread::client;
    }
    if(sendthread->send_socket->state()!=QAbstractSocket::ConnectedState){
        ui->textEdit->append("未建立连接，传送失败!");
        return;
    }
    else{
        QFileDialog fileDialog;
        sendthread->sendFileSelected = fileDialog.getOpenFileName(this, "选择文件", QCoreApplication::applicationDirPath(), "");
        emit handleCmd(sendThread::fileSendBtn);
    }
}

void MainWindow::getTransState(int state)
{
    static QTime recRecodeTime;
    switch (state) {
    case sendThread::sendBegin:{
        recRecodeTime.start();
        ui->textEdit->append("开始传送中...");
        setControlEnable(false,send);
        sendProgress->setVisible(true);
    }break;
    case sendThread::sendInterrupt:{
        sendProgress->setVisible(false);
        setControlEnable(true,send);
        ui->textEdit->append("传送已被强制中断!\n");
    }break;
    case sendThread::sendConnectClose:{
        ui->textEdit->append("连接已断开,传送中断!");
        sendProgress->setVisible(false);
        setControlEnable(true,send);
    }break;
    case sendThread::sendDone:{
        sendProgress->setVisible(false);
        setControlEnable(true,send);
        ui->textEdit->append("传送完成!");
        ui->textEdit->append("发送文件大小:"+QString::number(sendthread->fileSendSize));
        ui->textEdit->append("总发送字节大小:"+QString::number(sendthread->allSendSize));
        ui->textEdit->append("发送耗时:"+QString::number(recRecodeTime.elapsed()*0.001,'f',3)+"s\n");
    }break;
    case sendThread::sendProcessing:{
        sendProgress->setValue((double)sendthread->fileSendSize/sendthread->fileMaxSize*100);
    }break;
    case sendThread::sendFileOpenFailed:{
        ui->textEdit->append("打开文件失败，传送中断\n");
        sendProgress->setVisible(false);
        setControlEnable(true,send);
    }break;}
}

void MainWindow::on_dirShowLineEdit_textChanged(const QString &arg1)
{
    receiveThread::recFileDir=arg1;
}

void MainWindow::on_stopSendBtn_clicked()
{
    sendthread->stopSend=true;
}

void MainWindow::on_spinBox_bufferSize_valueChanged(int arg1)
{
    sendthread->bufferSize=arg1;
}
