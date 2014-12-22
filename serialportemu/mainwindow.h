#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "dialog_serialport.h"

typedef struct SerialPortSettings_s
{
    QSerialPortInfo serialPortInfo;
    QSerialPort::BaudRate baudRate; // Baud1200, Baud2400, Baud4800, Baud9600, Baud19200,
                                    // Baud38400, Baud57600, Baud115200, UnknownBaud
    QSerialPort::DataBits dataBits; // Data5, Data6, Data7, Data8, UnknownDataBits
    QSerialPort::FlowControl flowControl; // NoFlowControl, HardwareControl, SoftwareControl, UnknownFlowControl
    QSerialPort::Parity parity; // NoParity, EvenParity, OddParity, SpaceParity, MarkParity, UnknownParity
    QSerialPort::StopBits stopBits; // OneStop, OneAndHalfStop, TwoStop, UnknownStopBits
} SerialPortSettings_t;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    SerialPortSettings_t m_portSettings;
    QMap<QString, QSerialPortInfo> m_serialPortInfoMap;
    Dialog_SerialPort * m_pDialogSerialPort;
    void printMsg(QString msg);

private slots:
    void handleSerialPort();
    void handleClear();
    void handleSetSerialPort(QString portName,
                             QSerialPort::BaudRate baudRate,
                             QSerialPort::DataBits dataBits,
                             QSerialPort::FlowControl flowControl,
                             QSerialPort::Parity parity,
                             QSerialPort::StopBits stopBits);
};

#endif // MAINWINDOW_H
