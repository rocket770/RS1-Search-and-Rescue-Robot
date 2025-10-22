#ifndef PRACBASE_H
#define PRACBASE_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class pracbase; }
QT_END_NAMESPACE

class pracbase : public QMainWindow
{
    Q_OBJECT

public:
    pracbase(QWidget *parent = nullptr);
    ~pracbase();

private:
    Ui::pracbase *ui;
};
#endif // PRACBASE_H
