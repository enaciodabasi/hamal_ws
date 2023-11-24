/**
 * @file emg_stop_gui.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "hamal_utils/emg_stop_gui.hpp"

MainWindow::MainWindow(ros::NodeHandle& nh, QApplication* app, QWidget* parent)
    : QMainWindow(parent)
{

    m_CentralWidget = new CentralWidget(nh, this);

    this->setCentralWidget(m_CentralWidget);
}

MainWindow::~MainWindow()
{

}

CentralWidget::CentralWidget(ros::NodeHandle& nh, QWidget* parent)
    : QWidget(parent)
{
    m_EmgRosClient = std::make_unique<EmgClientBase>(nh);

    m_EmgButton = new QPushButton(this);
    m_EmgButtonPM = QPixmap(
        "/home/naci/hamal_ws/src/hamal_utils/gui_resource/emg_button.png"
    );
    m_EmgButton->setIcon(QIcon(m_EmgButtonPM));
    m_EmgButton->setIconSize(m_EmgButtonPM.rect().size());

    connect(
        m_EmgButton, &QPushButton::clicked,
        this, [&](){
            this->m_EmgRosClient->sendEmg();
        });
    
    QHBoxLayout* mainLayout = new QHBoxLayout(this);
    mainLayout->addWidget(m_EmgButton);    

    this->setLayout(mainLayout);

}

CentralWidget::~CentralWidget()
{

}

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    ros::init(argc, argv, "emg_stop_gui_node");

    ros::NodeHandle nh;

    MainWindow mw(nh, &app);    

    mw.show();

    while(ros::ok())
    {

        app.exec();

        ros::spinOnce();
    }

    return 0;

}