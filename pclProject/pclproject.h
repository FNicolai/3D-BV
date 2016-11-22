#ifndef PCLPROJECT_H
#define PCLPROJECT_H

#include <QMainWindow>

namespace Ui {
class pclProject;
}

class pclProject : public QMainWindow
{
    Q_OBJECT

public:
    explicit pclProject(QWidget *parent = 0);
    ~pclProject();

private:
    Ui::pclProject *ui;
};

#endif // PCLPROJECT_H
