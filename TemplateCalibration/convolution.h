#ifndef CONVOLUTION_H
#define CONVOLUTION_H

#include <QMainWindow>
//#include <opencv2/objdetect/objdetect.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv/cxmisc.h>
//#include <boost/filesystem.hpp>

#include <iostream>
#include <stdio.h>

//#include "opencv2/calib3d/calib3d.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/highgui/highgui.hpp"

//#include "opencv2/imgcodecs.hpp"
//#include "opencv2/core/utility.hpp"

namespace Ui {
class Convolution;
}

class Convolution : public QMainWindow
{
    Q_OBJECT

public:
    explicit Convolution(QWidget *parent = 0);
    ~Convolution();

    void WriteBoardSize(const QString width, const QString height, const QString size);

private slots:
    void on_startvideoButton_clicked();

    void on_start2DButton_clicked();

    void on_start3DButton_clicked();
private:
    Ui::Convolution *ui;
    QString m_boardWidth;
    QString m_boardHeight;
    QString m_boardSize;
};

#endif // CONVOLUTION_H
