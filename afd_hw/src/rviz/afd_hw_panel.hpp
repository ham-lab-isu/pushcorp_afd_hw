#ifndef AFD_HW_PANEL_HPP
#define AFD_HW_PANEL_HPP

#include <rviz_common/panel.hpp>
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
    /**
     * @brief Constructor for AFDHWPanel.
     * @param parent The parent QWidget (optional).
     */
    explicit AFDHWPanel(QWidget *parent = nullptr);

    /**
     * @brief Destructor for AFDHWPanel.
     */
    ~AFDHWPanel() override;

private Q_SLOTS:
    /**
     * @brief Updates the graph with new data.
     */
    void updateGraph();

    /**
     * @brief Handles button click events.
     * @param button_index The index of the clicked button.
     */
    void onButtonClicked(int button_index);

private:
    /**
     * @brief Callback for receiving AFD data.
     * @param msg Shared pointer to the Float32 message containing AFD data.
     */
    void afdDataCallback(const std_msgs::msg::Float32::SharedPtr msg);

    /**
     * @brief Initializes and configures the chart for displaying AFD data.
     */
    void setupChart();

    /**
     * @brief Initializes and configures buttons for interaction.
     */
    void setupButtons();

    /**
     * @brief Initializes the ROS2 node and subscription.
     */
    void setupRos2Node();

    QtCharts::QChart *chart; ///< Chart for visualizing AFD data.
    QtCharts::QLineSeries *series; ///< Line series representing the data points.
    QTimer *timer; ///< Timer for periodic graph updates.
    double x_value; ///< Current x-axis value for the graph.
    double current_afd_value; ///< Latest AFD data value.

    rclcpp::Node::SharedPtr node_; ///< ROS2 node for subscribing to AFD data.
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_; ///< ROS2 subscription.
    std::thread ros_thread_; ///< Thread for running the ROS2 executor.
    std::atomic<bool> spinning_; ///< Flag to control the ROS2 spinning thread.
};
} // namespace afd_rviz_plugin

#endif // AFD_HW_PANEL_HPP
