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

    m_ResetServiceClient = nh.serviceClient<hamal_custom_interfaces::EmergencyStop>(
        "deactivate_quick_stop_server"
    );

    m_EmgButton = new QPushButton(this);
    m_EmgButtonPM = QPixmap(
        "/home/naci/hamal_ws/src/hamal_utils/gui_resource/emg_button2.png"
    );
    m_EmgButton->setIcon(QIcon(m_EmgButtonPM));
    m_EmgButton->setIconSize(m_EmgButtonPM.rect().size());

    connect(
        m_EmgButton, &QPushButton::clicked,
        this, [&]() -> bool {
            if(this->m_SetPress){
                std::cout << "Setting emg\n";
            this->m_EmgRosClient->sendEmg();
            this->m_SetPress = false;
            }
            else{
                if(!this->m_ResetServiceClient.exists()){
                    return false;
                }
                std::cout << "desetting emg\n";
                hamal_custom_interfaces::EmergencyStop srv;
                srv.request.trigger = std_msgs::Empty();
                if(this->m_ResetServiceClient.call(srv)){
                    this->m_SetPress = true;
                    return (bool)srv.response.stopped;
                }
                return true;
            }
        });
    
    QHBoxLayout* mainLayout = new QHBoxLayout(this);
    mainLayout->addWidget(m_EmgButton);    

    QVBoxLayout* profileCommandLayout = new QVBoxLayout();


    m_TargetValueLEdit = new QLineEdit(this);
    m_MaxVelocityLEdit = new QLineEdit(this);
    m_MaxAccLEdit = new QLineEdit(this);

    m_TargetValueLEdit->setValidator(new QDoubleValidator(-10.0, 10.0, 3, this));
    m_MaxVelocityLEdit->setValidator(new QDoubleValidator(0.0, 10.0, 3, this));
    m_MaxAccLEdit->setValidator(new QDoubleValidator(0.0, 10.0, 3, this));
    
    m_LinearMovementTypeSelect = new QRadioButton("Linear", this);
    m_AngularMovementTypeSelect = new QRadioButton("Angular", this);
    
    m_SendProfileGoalButton = new QPushButton("Send", this);

    QHBoxLayout* radioButtonsLayout = new QHBoxLayout();
    radioButtonsLayout->addWidget(m_LinearMovementTypeSelect);
    radioButtonsLayout->addWidget(m_AngularMovementTypeSelect);

    profileCommandLayout->addLayout(radioButtonsLayout);

    profileCommandLayout->addWidget(m_TargetValueLEdit);
    profileCommandLayout->addWidget(m_MaxVelocityLEdit);
    profileCommandLayout->addWidget(m_MaxAccLEdit);
    
    profileCommandLayout->addWidget(m_SendProfileGoalButton);

    m_ProfileGoalPublisher = nh.advertise<hamal_custom_interfaces::ProfileCommand>(
        "/profile_command/trapezoidal",
        10,
        false
    );

    connect(
        m_SendProfileGoalButton, &QPushButton::clicked,
        this, [&]() -> bool {
            
            hamal_custom_interfaces::ProfileCommand pCmd;
            
            double target, maxVel, maxAcc = 0.0;

            if(this->m_LinearMovementTypeSelect->isChecked()){
                pCmd.linear = true;
            }
            else if(this->m_AngularMovementTypeSelect->isChecked()){
                pCmd.linear = false;
            }
            else{
                return false;
            }

            pCmd.target = this->m_TargetValueLEdit->text().toDouble();
            pCmd.max_vel = this->m_MaxVelocityLEdit->text().toDouble();
            pCmd.max_acc = this->m_MaxAccLEdit->text().toDouble();

            this->m_ProfileGoalPublisher.publish(pCmd);

            return true;
        });

    mainLayout->addLayout(profileCommandLayout);
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