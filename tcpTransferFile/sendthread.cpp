#include "sendthread.h"

sendThread::sendThread()
{
    qRegisterMetaType<QAbstractSocket::SocketState>("QAbstractSocket::SocketState");
    recThread=new receiveThread;
    connect(recThread,&receiveThread::sendRecState,this,&sendThread::sendRecState);
    recThread->start();
    initServer();
    initClient();

}

sendThread::~sendThread()
{
    delete TcpServer;
    delete tcpClient;
}

void sendThread::initServer()
{
    if(TcpServer)
        delete  TcpServer;
    TcpServer=new QTcpServer(this);
    TcpServer->setProxy(QNetworkProxy::NoProxy);
    connect(TcpServer,SIGNAL(newConnection()),this,SLOT(onNewConnection()));
    connect(TcpServer,&QTcpServer::acceptError,[this](){
        emit onServerErrorOccurs(TcpServer);
    });
}

void sendThread::initClient()
{
    if(tcpClient)
        delete tcpClient;
    tcpClient=new QTcpSocket(this);
    tcpClient->setReadBufferSize(bufferSize);
    tcpClient->setProxy(QNetworkProxy::NoProxy);
    connect(tcpClient,SIGNAL(readyRead()),this,SLOT(onSocketReadyRead()));
    connect(tcpClient,SIGNAL(error(QAbstractSocket::SocketError)),this,SLOT(onErrorOccurs(QAbstractSocket::SocketError)));
    connect(tcpClient,&QTcpSocket::stateChanged,this,&sendThread::onSocketStateChange);
}

void sendThread::onNewConnection()
{
    QTcpSocket *tcpSocket=TcpServer->nextPendingConnection();
    tcpSocket->setReadBufferSize(bufferSize);
    emit onNewClientconnected(tcpSocket);
    connect(tcpSocket,SIGNAL(readyRead()),this,SLOT(onServerReadyRead()));
    connect(tcpSocket,&QTcpSocket::disconnected,this,&sendThread::onClientDisconnected);
}

void sendThread::onErrorOccurs(QAbstractSocket::SocketError)
{
    emit onClientErrorOccurs(tcpClient);
}

void sendThread::run()
{
    exec();
}

void sendThread::handleCmd(int cmd_type)
{
    switch (cmd_type) {
    case startListenAct:{
        if(TcpServer->isListening()) return;
        if(TcpServer->listen(QHostAddress::Any,8080)){
            emit sendActionSig(startListenAct);
        }
    }break;
    case stopListenAct:{
        if(TcpServer->isListening()){
            TcpServer->close();
            emit sendActionSig(stopListenAct);
        }
    }break;
    case connectAct:{
        if(tcpClient->state()==QAbstractSocket::UnconnectedState){
            tcpClient->connectToHost(serverIp,8080);
        }
    }break;
    case disconnectAct:{
        if(tcpClient){
            if(tcpClient->state()==QAbstractSocket::ConnectedState)
                tcpClient->disconnectFromHost();
            else
                initClient();
        }
    }break;
    case testBtn:{
        QString msg="hello";
        QByteArray str=msg.toUtf8();
        send_socket->write(str);
    }break;
    case fileSendBtn:{
        QByteArray str;
        QTime time;
        QFile file(sendFileSelected);
        fileMaxSize=file.size();
        sendFileSelected.remove(0,sendFileSelected.lastIndexOf('/')+1);
        if(file.open(QIODevice::ReadOnly)){
            emit sendTransState(sendBegin);
            stopSend=false;
            str=recThread->beginFlag.toUtf8();
            str+=sendFileSelected.toUtf8();
            str+=recThread->fileSizeFlag.toUtf8();
            str+=QString::number(fileMaxSize).toUtf8();
            str+=recThread->fileSiezEndFlag.toUtf8();
            fileSendSize=0;
            allSendSize=0;
            bool sendFlag=true;
            while(sendFlag){
                if(str.size()==0){
                    str=recThread->endFlag.toUtf8();
                    sendFlag=false;
                }
                if(send_socket->state()==QAbstractSocket::ConnectedState){
                    send_socket->write(str);
                    allSendSize+=str.size();
                }
                else {
                    file.close();
                    emit sendTransState(sendConnectClose);
                    return;
                }
                if(stopSend){
                    file.close();
                    emit sendTransState(sendInterrupt);
                    return;
                }
                time.start();
                while(time.elapsed()<1);
                str=file.read(bufferSize);
                fileSendSize+=str.size();
                emit sendTransState(sendProcessing);
                QApplication::processEvents();
            }
            file.close();
            emit sendTransState(sendDone);
        }
        else
            emit sendTransState(sendFileOpenFailed);
    }break;
    default:break;}
}

void sendThread::onServerReadyRead()
{
    QTcpSocket *tcpSocket = static_cast<QTcpSocket *>(QObject::sender());
    QByteArray str=tcpSocket->readAll();
    recThread->mutex.lock();
    while(str.size()>bufferSize){
        recThread->byteQueue->enqueue(str.mid(0,bufferSize));
        str=str.mid(bufferSize,str.size()-bufferSize);
    }
    recThread->byteQueue->enqueue(str);
    recThread->mutex.unlock();
    recSocket_type=server;
}

void sendThread::onSocketReadyRead()
{
    QByteArray str=tcpClient->readAll();
    recThread->mutex.lock();
    while(str.size()>bufferSize){
        recThread->byteQueue->enqueue(str.mid(0,bufferSize));
        str=str.mid(bufferSize,str.size()-bufferSize);
    }
    recThread->byteQueue->enqueue(str);
    recThread->mutex.unlock();
    recSocket_type=client;
}

