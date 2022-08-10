#include "receivethread.h"

quint64 receiveThread::targetRecFileSize=0;
quint64 receiveThread::fileRecSize=0;
quint64 receiveThread::allRecSize=0;
QByteArray receiveThread::recByte=QByteArray();
QString receiveThread::recFileDir=QString();

receiveThread::receiveThread()
{
    byteQueue.reset(new QQueue<QByteArray>());
}

void receiveThread::stopThread()
{
    runFlag=false;
}

void receiveThread::run()
{
    bool readFile=false;
    QFile file;
    QTime time;
    runFlag=true;
    QByteArray str;
    while(runFlag){
        if(readFile){
            if(time.elapsed()>1000){
                readFile=false;
                while(!file.flush());
                file.close();
                emit sendRecState(recInterrupt);
            }
        }
        if(byteQueue->size()>0){
            if(mutex.tryLock()){
                str=byteQueue->dequeue();
                mutex.unlock();
            }
            else continue;
            if(str.size()==0) continue;
            if(readFile){
                if(str.contains(endFlag.toUtf8())){
                    QByteArray lastStr=str.mid(0,str.indexOf(endFlag));
                    file.write(lastStr);
                    fileRecSize+=lastStr.size();
                    allRecSize+=str.size();
                    readFile=false;
                    while(!file.flush());
                    file.close();
                    emit sendRecState(recDone);
                }
                else{
                    file.write(str);
                    fileRecSize+=str.size();
                    allRecSize+=str.size();
                    emit sendRecState(recProcessing);
                    time.start();
                }
            }
            else{
                if(str.contains(beginFlag.toUtf8()) && str.contains(fileSizeFlag.toUtf8())){
                    QByteArray fileName=str.mid(str.indexOf(beginFlag)+beginFlag.toUtf8().size(),str.indexOf(fileSizeFlag)-str.indexOf(beginFlag)-beginFlag.toUtf8().size());
                    QByteArray fileSize=str.mid(str.indexOf(fileSizeFlag)+fileSizeFlag.toUtf8().size(),str.indexOf(fileSiezEndFlag)-str.indexOf(fileSizeFlag)-fileSizeFlag.toUtf8().size());
                    QByteArray beginStr=str.mid(str.indexOf(fileSiezEndFlag)+fileSiezEndFlag.toUtf8().size(),str.size()-fileSiezEndFlag.toUtf8().size()-str.indexOf(fileSiezEndFlag));
                    file.setFileName(recFileDir+"/"+fileName);
                    if(file.open(QIODevice::WriteOnly)){
                        file.write(beginStr);
                        allRecSize=str.size();
                        fileRecSize=beginStr.size();
                        targetRecFileSize=fileSize.toULongLong();
                        time.start();
                        readFile=true;
                        emit sendRecState(recBegin);
                    }
                    else
                        emit sendRecState(recFileFail);
                }
                else{
                    recByte=str;
                    emit sendRecState(recTest);
                }
            }
        }
        QApplication::processEvents();
    }
}
