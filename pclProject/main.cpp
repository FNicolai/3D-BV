#include "pclproject.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    pclProject w;
    w.show();

    return a.exec();
}
