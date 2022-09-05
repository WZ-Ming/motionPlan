#ifndef SOCKETTHREAD_H
#define SOCKETTHREAD_H

#include"receivethread.h"

class sendThread : public QThread
{
    Q_OBJECT
public:
    sendThread();
    ~sendThread();
    enum socketType{server=20,client,noSocket}sendSocket_type,recSocket_type;
    enum cmdType{startListenAct=0,stopListenAct,connectAct,disconnectAct,testBtn,fileSendBtn};
    QTcpSocket *send_socket;
    QString serverIp;
    QString sendFileSelected;
    QTcpSocket *tcpClient=nullptr;

    bool stopSend=false;
    quint64 fileMaxSize=0,fileSendSize=0,allSendSize=0;
    enum sendFlag{sendBegin=10,sendDone,sendProcessing,sendFileFail,sendInterrupt,sendConnectClose,sendFileOpenFailed};
    int bufferSize=12000;

protected:
    void run();

private:
    void initServer();
    void initClient();

    QTcpServer *TcpServer=nullptr;
    receiveThread *recThread=nullptr;

signals:
    void sendTransState(int);

    void onSocketStateChange(QAbstractSocket::SocketState);

    void onServerErrorOccurs(QTcpServer*);

    void onClientErrorOccurs(QTcpSocket*);

    void onNewClientconnected(QTcpSocket*);

    void onClientDisconnected();

    void sendRecState(int);

    void sendActionSig(int);

public slots:
    void handleCmd(int cmd_type);

private slots:
    void onErrorOccurs(QAbstractSocket::SocketError);

    void onNewConnection();

    void onServerReadyRead();

    void onSocketReadyRead();
};

#endif // SOCKETTHREAD_H
