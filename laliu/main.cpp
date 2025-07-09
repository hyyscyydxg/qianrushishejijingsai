#include "RTCTest.h"
#include <QtWidgets/QApplication>
#include <QFile>
#include <QTextStream>
#include <QDateTime>
#include <QDebug>

// �Զ�����־������
void customMessageHandler(QtMsgType type, const QMessageLogContext& context, const QString& msg) {
    static QFile logFile("rtc_log.txt");
    static QTextStream logStream(&logFile);

    if (!logFile.isOpen()) {
        logFile.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
        logStream.setDevice(&logFile);
    }

    QString timeStamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    QString logLevel;

    switch (type) {
    case QtDebugMsg:    logLevel = "DEBUG"; break;
    case QtInfoMsg:     logLevel = "INFO"; break;
    case QtWarningMsg:  logLevel = "WARNING"; break;
    case QtCriticalMsg: logLevel = "CRITICAL"; break;
    case QtFatalMsg:    logLevel = "FATAL"; break;
    }

    logStream << QString("[%1] [%2] %3\n").arg(timeStamp, logLevel, msg);
    logStream.flush();

    if (type == QtFatalMsg)
        abort(); // ���� fatal error ��ֹ����
}

int main(int argc, char* argv[])
{
    qInstallMessageHandler(customMessageHandler);  // ��װ��־����

    QApplication app(argc, argv);
    RTCTest window;
    window.show();

    return app.exec();
}

