#ifndef RECEIVETHREAD_H
#define RECEIVETHREAD_H

#include <QObject>
#include<QApplication>
#include <QHostInfo>
#include <QNetworkInterface>
#include <QTcpServer>
#include <QLabel>
#include <QTcpSocket>
#include <QVector>
#include <QtGlobal>
#include <QTimer>
#include <QComboBox>
#include <QCoreApplication>
#include <QFile>
#include <QFileDialog>
#include <QByteArray>
#include <QThread>
#include <QTime>
#include <QMutex>
#include <QQueue>
#include <QProgressBar>
#include <QSettings>
#include<QThread>
#include <QNetworkProxy>
#include<QScopedPointer>
#include <QMetaType>
#include<QTextStream>
#include <QTextBlock>

#ifdef Q_OS_WIN
    #pragma execution_character_set("utf-8")
#endif

class receiveThread : public QThread
{
    Q_OBJECT
public:
    receiveThread();
    QScopedPointer<QQueue<QByteArray>> byteQueue;

    static quint64 targetRecFileSize,fileRecSize,allRecSize;
    static QByteArray recByte;
    static QString recFileDir;

    enum recFlag{recDone=0,recBegin,recProcessing,recFileFail,recInterrupt,recTest};
    QMutex mutex;

    const QString beginFlag="开始传送___:::";
    const QString endFlag="结束传送___:::";
    const QString fileSizeFlag="文件大小___:::";
    const QString fileSiezEndFlag="文件大小传送结束___:::";

    void stopThread();

protected:
    void run();

private:
    bool runFlag=false;

signals:
    void sendRecState(int);
};



#endif // RECEIVETHREAD_H
