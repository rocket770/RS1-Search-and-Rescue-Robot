#include "pracbase.h"
#include "./ui_pracbase.h"

pracbase::pracbase(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::pracbase)
{
    ui->setupUi(this);
}

pracbase::~pracbase()
{
    delete ui;
}

