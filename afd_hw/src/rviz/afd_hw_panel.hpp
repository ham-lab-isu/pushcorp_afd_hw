#ifndef AFD_HW_PANEL_HPP
#define AFD_HW_PANEL_HPP

#include <rviz_common/panel.hpp>
#include <QVBoxLayout>
#include <QTimer>
#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <thread>
#include <atomic>

namespace afd_rviz_plugin
{
class AFDHWPanel : public rviz_common::Panel
{
    Q_OBJECT
public:
    explicit AFDHWPanel(QWidget *parent = nullptr);
    ~AFDHWPanel();

private:
    void setupChart(QVBoxLayout *layout);
    void setupButtons(QVBoxLayout *layout);

private Q_SLOTS:
    void updateGraph();
    void onButtonClicked(int button_index);

private:
    void afdDataCallback(const std_msgs::msg::Float32::SharedPtr msg); // Declare only

    QtCharts::QChart *chart;
    QtCharts::QLineSeries *series;
    QTimer *timer;
    double x_value;
    double current_afd_value; // Ensure this member is declared here

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
    std::thread ros_thread_;
    std::atomic<bool> spinning_;
};
} // namespace afd_rviz_plugin

#endif // AFD_HW_PANEL_HPP
