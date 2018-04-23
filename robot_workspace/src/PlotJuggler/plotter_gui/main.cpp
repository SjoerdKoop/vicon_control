#include "mainwindow.h"
#include <QApplication>
#include <QCommandLineParser>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    app.setApplicationName("PlotJuggler");

    qApp->setStyleSheet(QString("QToolTip {\n"
                                "   border: 1px solid black;\n"
                                "   border-radius: 4px;\n"
                                "   background: white;\n"
                                "   color: black; }" ));

    QString VERSION_STRING = QString::number(PJ_MAJOR_VERSION) + QString(".") +
            QString::number(PJ_MINOR_VERSION) + QString(".") +
            QString::number(PJ_PATCH_VERSION);

    app.setApplicationVersion(VERSION_STRING);

    QCommandLineParser parser;
    parser.setApplicationDescription("PlotJuggler: the time series visualization tool that you deserve ");
    parser.addVersionOption();
    parser.addHelpOption();

    QCommandLineOption nosplash_option(QStringList() << "n" << "nosplash",
                                       QCoreApplication::translate("main", "Don't display the splashscreen"));
    parser.addOption(nosplash_option);

    QCommandLineOption test_option(QStringList() << "t" << "test",
                                   QCoreApplication::translate("main", "Generate test curves at startup"));
    parser.addOption(test_option);

    QCommandLineOption loadfile_option(QStringList() << "d" << "datafile",
                                       QCoreApplication::translate("main", "Load a file containing data"),
                                       QCoreApplication::translate("main", "file") );
    parser.addOption(loadfile_option);

    QCommandLineOption config_option(QStringList() << "l" << "layout",
                                     QCoreApplication::translate("main", "Load a file containing the layout configuration"),
                                     QCoreApplication::translate("main", "file") );
    parser.addOption(config_option);

    QCommandLineOption buffersize_option(QStringList() << "buffer_size",
                                     QCoreApplication::translate("main", "Change the maximum size of the streaming buffer (minimum: 10 default: 60)"),
                                     QCoreApplication::translate("main", "seconds") );
    parser.addOption(buffersize_option);

    parser.process( *qApp );

	MainWindow w( parser );
	w.show();
	return app.exec();

    return -1;
}
