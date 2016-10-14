#include "camcalib.h"
#include <QApplication>
#include <QProcess>

int main(int argc, char *argv[])
{
    //BashScript um Autofocus abzuschalten
    //QProcess autofoc;
    //autofoc.startDetached("/bin/sh", QStringList()<< "./autofocus");

    QApplication a(argc, argv);
    CamCalib w;
    w.show();

    return a.exec();
}
