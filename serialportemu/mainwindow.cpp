#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    m_pDialogSerialPort = 0;
    connect(ui->actionExit, SIGNAL(triggered()), this, SLOT(close()));
    connect(ui->actionSerialPort, SIGNAL(triggered()), this, SLOT(handleSerialPort()));
    ui->actionProtocol->setEnabled(false);
    connect(ui->pushButton_Clear, SIGNAL(clicked()), this, SLOT(handleClear()));
}

MainWindow::~MainWindow()
{
    if (m_pDialogSerialPort)
    {
        disconnect(m_pDialogSerialPort, SIGNAL(setSerialPort(QString, QSerialPort::BaudRate, QSerialPort::DataBits,
                                                           QSerialPort::FlowControl, QSerialPort::Parity,
                                                           QSerialPort::StopBits)),
                this, SLOT(handleSetSerialPort(QString, QSerialPort::BaudRate, QSerialPort::DataBits,
                                               QSerialPort::FlowControl, QSerialPort::Parity,
                                               QSerialPort::StopBits)));
        delete m_pDialogSerialPort;
    }
    delete ui;
}

void MainWindow::printMsg(QString msg)
{
    ui->plainTextEdit_RxMsgDisplay->appendPlainText(msg);
}

void MainWindow::handleClear()
{
    ui->plainTextEdit_RxMsgDisplay->clear();
}

void MainWindow::handleSerialPort()
{
    foreach (const QSerialPortInfo &portInfo, QSerialPortInfo::availablePorts())
    {
        QString portNameAndDesc = portInfo.portName() + "_" + portInfo.description();
        m_serialPortInfoMap.insert(portNameAndDesc, portInfo);
    }

    if (!m_pDialogSerialPort)
    {
        m_pDialogSerialPort = new Dialog_SerialPort;
        connect(m_pDialogSerialPort, SIGNAL(setSerialPort(QString, QSerialPort::BaudRate, QSerialPort::DataBits,
                                                          QSerialPort::FlowControl, QSerialPort::Parity,
                                                          QSerialPort::StopBits)),
                this, SLOT(handleSetSerialPort(QString, QSerialPort::BaudRate, QSerialPort::DataBits,
                                               QSerialPort::FlowControl, QSerialPort::Parity,
                                               QSerialPort::StopBits)), Qt::DirectConnection);
        connect(this, SIGNAL(destroyed()), m_pDialogSerialPort, SLOT(close()));
    }
    m_pDialogSerialPort->setPortList(m_serialPortInfoMap.keys());
    m_pDialogSerialPort->exec();
}

void MainWindow::handleSetSerialPort(QString portName,
                                     QSerialPort::BaudRate baudRate,
                                     QSerialPort::DataBits dataBits,
                                     QSerialPort::FlowControl flowControl,
                                     QSerialPort::Parity parity,
                                     QSerialPort::StopBits stopBits)
{
    if (m_serialPortInfoMap.contains(portName))
    {
        m_portSettings.serialPortInfo = m_serialPortInfoMap.value(portName);
        m_portSettings.baudRate = baudRate;
        m_portSettings.dataBits = dataBits;
        m_portSettings.flowControl = flowControl;
        m_portSettings.parity = parity;
        m_portSettings.stopBits = stopBits;
        QString portSettingsStr = QString("%1: %2 %3%4%5")
                .arg(m_portSettings.serialPortInfo.portName()).arg(m_portSettings.baudRate)
                .arg(m_portSettings.dataBits).arg(Dialog_SerialPort::getParityChar(m_portSettings.parity)).arg(m_portSettings.stopBits);
        ui->label_PortInfo->setText(portSettingsStr);
    }
    else
    {
        m_portSettings.baudRate = baudRate;
        ui->label_PortInfo->setText("No serial port selected.");
    }
}
