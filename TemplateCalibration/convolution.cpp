#include "convolution.h"
#include "ui_convolution.h"

#include "camera_calibration.h"
#include "stereo_calib.h"
#include "video_stream.h"

using namespace std;

Convolution::Convolution(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Convolution)
{
    ui->setupUi(this);
}

Convolution::~Convolution()
{
    delete ui;
    //string filename = "../build-camera_calibration-Desktop_Qt_5_5_1_GCC_64bit-Default/camera_calibration";
    //string s = "pkill -9 -f " + filename;
    //system(s.c_str());
}

void Convolution::on_startvideoButton_clicked()
{
    //Hier finden Sie hilfreiche Befehle:
    //https://asciich.ch/wordpress/usb-kamera-mit-opencv-auslesen/

    //Tipps:
    //Eine oder zwei Kameras einlesen
    //Ausgabe der Videos in 2 Fenstern
    //Falls Kamerastream unterbrochen, abbrechen Warnung ausgeben
    //Konvertieren in IplImage damit cvShowImage funktioniert
    //Falls konvertieren nicht möglich, Fehler melden
    //Ausgabe der Bilder
    //Fenster schließen
    Video_Stream video_stream;
    video_stream.start();
}

void Convolution::on_start2DButton_clicked()
{
    //Hier finden Sie hilfreiche Befehle:
    //http://docs.opencv.org/doc/tutorials/calib3d/camera_calibration/camera_calibration.html

    //Tipps:
    //Initialisieren von Schachbrett Höhe & Breite etc
    //Video einlesen
    //Image und Object Points für die Kalibrierung definieren und initialisieren
    //Schachbrettecken finden und merken
    //Wenn gefunden
    //Pixel um die Ecken definieren
    //Ecken im Video anzeigen durch die benachbarten Pixel
    //add image points
    //add object points
    //Video Anzeigen
    //Kalibrierung mit gefundenen Punkten

    Camera_Calibration camera_calibration;
    camera_calibration.start();
}

void Convolution::on_start3DButton_clicked()
{
    //Hier finden Sie hilfreiche Befehle
    //http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html

    //Tipps
    //Ähnlich zu 2D Kalibrierung

    //Stereo_Calibration stereo_calibration;
    //stereo_calibration.start();

    Convolution w;
    w.show();



}
