#include "afd_hw_panel.hpp"

#include <QVBoxLayout>
#include <QPushButton>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QTimer>
#include <iostream>
#include <pluginlib/class_list_macros.hpp>

namespace afd_rviz_plugin
{
AFDHWPanel::AFDHWPanel(QWidget *parent)
    : rviz_common::Panel(parent),
      chart(new QtCharts::QChart),
      series(new QtCharts::QLineSeries),
      timer(new QTimer(this)),
      x_value(0.0),
      current_afd_value(0.0),
      spinning_(true)
{
    std::cout << "[AFDHWPanel] Initializing..." << std::endl;

    // Layout Setup
    QVBoxLayout *layout = new QVBoxLayout();

    // Chart Setup
    setupChart(layout);

    // Buttons Setup
    setupButtons(layout);

    setLayout(layout);

    // Timer Setup
    connect(timer, &QTimer::timeout, this, &AFDHWPanel::updateGraph);
    timer->start(100); // 0.1 seconds

    // ROS2 Node Setup
    setupRos2Node();
}

AFDHWPanel::~AFDHWPanel()
{
    std::cout << "[AFDHWPanel] Shutting down..." << std::endl;

    spinning_ = false;
    if (ros_thread_.joinable())
    {
        ros_thread_.join();
    }
}

void AFDHWPanel::setupChart(QVBoxLayout *layout)
{
    chart->addSeries(series);
    chart->createDefaultAxes();
    chart->setTitle("AFD Data Visualization");

    auto *chart_view = new QtCharts::QChartView(chart);
    chart_view->setRenderHint(QPainter::Antialiasing);

    layout->addWidget(chart_view);
}

void AFDHWPanel::setupButtons(QVBoxLayout *layout)
{
    for (int i = 0; i < 4; ++i)
    {
        QPushButton *button = new QPushButton("Button " + QString::number(i + 1));
        connect(button, &QPushButton::clicked, this, [this, i]() { onButtonClicked(i); });
        layout->addWidget(button);
    }
}

void AFDHWPanel::setupRos2Node()
{
    node_ = rclcpp::Node::make_shared("afd_hw_panel_node");

    subscription_ = node_->create_subscription<std_msgs::msg::Float32>(
        "afd_data_topic", 10, std::bind(&AFDHWPanel::afdDataCallback, this, std::placeholders::_1));

    ros_thread_ = std::thread([this]() {
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node_);
        while (spinning_)
        {
            executor.spin_some(); // Non-blocking for graceful shutdown
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });
}

void AFDHWPanel::updateGraph()
{
    series->append(x_value, current_afd_value);
    x_value += 0.1;

    if (series->count() > 100)
    {
        series->removePoints(0, series->count() - 100);
    }

    chart->axes(Qt::Horizontal).first()->setRange(x_value - 10, x_value);
    chart->axes(Qt::Vertical).first()->setRange(-10, 10); // Adjust based on expected value range
}

void AFDHWPanel::onButtonClicked(int button_index)
{
    std::cout << "[AFDHWPanel] Button " << button_index + 1 << " clicked!" << std::endl;
}

void AFDHWPanel::afdDataCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    current_afd_value = msg->data;
    std::cout << "[AFDHWPanel] AFD Data Received: " << current_afd_value << std::endl;
}

} // namespace afd_rviz_plugin

// Register the plugin
PLUGINLIB_EXPORT_CLASS(afd_rviz_plugin::AFDHWPanel, rviz_common::Panel)
