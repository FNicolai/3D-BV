#include "pclproject.h"
#include "ui_pclproject.h"

#include "import_and_clean.h"

pclProject::pclProject(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::pclProject)
{
    ui->setupUi(this);
}

pclProject::~pclProject()
{
    delete ui;
}

void pclProject::on_pushButton_import_and_clean_clicked()
{
    Import_And_Clean import_and_clean;
    import_and_clean.start();
}
