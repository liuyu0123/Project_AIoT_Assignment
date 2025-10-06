#include <QApplication>
#include <QLabel>
#include <QString>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QLabel *label = new QLabel("Hello, World QLabel!");
    QString message = QString::fromStdString("Hello, World QString!");
    label->setText(message);
    label->show();
    return app.exec();
}