#include "afd_hw_panel.hpp"
#include <QPushButton>
#include <QtCharts/QChartView>
#include <iostream>

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
    std::cout << "Creating AFDHWPanel with Line Graph and Buttons..." << std::endl;

    QVBoxLayout *layout = new QVBoxLayout();
    setupChart(layout);  // Fix: Call method with correct argument
    setupButtons(layout); // Fix: Call method with correct argument
    setLayout(layout);

    connect(timer, &QTimer::timeout, this, &AFDHWPanel::updateGraph);
    timer->start(100);

    node_ = rclcpp::Node::make_shared("afd_hw_panel_node");
    subscription_ = node_->create_subscription<std_msgs::msg::Float32>(
        "afd_data_topic", 10, std::bind(&AFDHWPanel::afdDataCallback, this, std::placeholders::_1));

    ros_thread_ = std::thread([this]() {
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node_);
        while (spinning_)
        {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });
}

AFDHWPanel::~AFDHWPanel()
{
    spinning_ = false;
    if (ros_thread_.joinable())
    {
        ros_thread_.join();
    }
}

void AFDHWPanel::setupChart(QVBoxLayout* layout)  // Fix: Match declaration
{
    chart->addSeries(series);
    chart->createDefaultAxes();
    chart->setTitle("AFD Data Visualization");

    auto *chart_view = new QtCharts::QChartView(chart);
    chart_view->setRenderHint(QPainter::Antialiasing);
    layout->addWidget(chart_view);
}

void AFDHWPanel::setupButtons(QVBoxLayout* layout)  // Fix: Match declaration
{
    for (int i = 0; i < 4; ++i)
    {
        QPushButton *button = new QPushButton("Button " + QString::number(i + 1));
        connect(button, &QPushButton::clicked, this, [this, i]() { onButtonClicked(i); });
        layout->addWidget(button);
    }
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
    chart->axes(Qt::Vertical).first()->setRange(-10, 10);
}

void AFDHWPanel::onButtonClicked(int button_index)
{
    std::cout << "Button " << button_index + 1 << " clicked!" << std::endl;
}

void AFDHWPanel::afdDataCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    current_afd_value = msg->data;
}

} // namespace afd_rviz_plugin
