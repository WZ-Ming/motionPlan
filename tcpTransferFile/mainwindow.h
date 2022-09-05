#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "sendthread.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    const QString exeVersion="(1.0)";
    const QString IniFilePath = QCoreApplication::applicationDirPath() + "/settings.ini";
    QTimer errorClearTimer;

    sendThread* sendthread=nullptr;
    QVector<QTcpSocket*> clientVector;
    Ui::MainWindow *ui;
    QLabel *socketStateLabel,*listenStateLabel;
    QLabel *clientErrorLabel,*serverErrorLabel;
    QProgressBar *sendProgress,*recProgress;

    enum transType{receive,send};

signals:
    void handleCmd(int);

private slots:
    void onSocketStateChange(QAbstractSocket::SocketState state);

    void onServerErrorOccurs(QTcpServer* server);

    void onClientErrorOccurs(QTcpSocket* client);

    void onNewClientconnected(QTcpSocket* client);

    void onClientDisconnected();

    void recActionSig(int type);

    void getRecState(int state);

    void getTransState(int state);

    void on_act_startListen_triggered();

    void on_act_stopListen_triggered();

    void on_act_connect_triggered();

    void on_act_disconnect_triggered();

    void on_act_server_triggered();

    void on_act_client_triggered();

    void on_stackedWidget_currentChanged(int arg1);

    void on_testBtn_clicked();

    void on_clearTextBtn_clicked();

    void on_chooseDirBtn_clicked();

    void on_fileTransmitBtn_clicked();

    void on_dirShowLineEdit_textChanged(const QString &arg1);

    void on_stopSendBtn_clicked();

    void on_spinBox_bufferSize_valueChanged(int arg1);

private:
    void receiveFile();

    void getIp();

    void initStatusLabel();

    void initServer();

    void initClient();

    void onClientConnected(QTcpSocket *tcpSocket);

    void loadSettings();

    void saveSettings();

    void setControlEnable(bool enable,transType trans_type);
};
#endif // MAINWINDOW_H
