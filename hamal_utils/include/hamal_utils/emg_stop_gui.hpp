/**
 * @file emg_stop_gui.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef EMG_STOP_GUI_HPP_
#define EMG_STOP_GUI_HPP_

#include <QApplication>
#include <QMainWindow>
#include <QWidget>
#include <QPushButton>
#include <QPixmap>
#include <QHBoxLayout>

#include <ros/ros.h>

#include "hamal_utils/emg_stop_client.hpp"

class CentralWidget;

class MainWindow : public QMainWindow
{
    public:

    MainWindow(ros::NodeHandle& nh, QApplication* app = nullptr, QWidget* parent = nullptr);
    ~MainWindow();

    private:

    CentralWidget* m_CentralWidget;

};

class CentralWidget : public QWidget
{
    public:

    CentralWidget(ros::NodeHandle& nh, QWidget* parent = nullptr);

    ~CentralWidget();

    private:

    QPushButton* m_EmgButton;

    QPixmap m_EmgButtonPM; 

    std::unique_ptr<EmgClientBase> m_EmgRosClient;

    bool m_SetPress = true;

    ros::ServiceClient m_ResetServiceClient;


};


#endif // EMG_STOP_GUI_HPP_