#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "std_srvs/srv/set_bool.hpp"
#include "g15_control/spi.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

extern "C" {
#include "spi_data.h"
}


#include <array>
#include <chrono>

class DifferentialDriveController : public rclcpp::Node
{
public:
    DifferentialDriveController()
    : Node("Controller"), x_(0.0), y_(0.0), theta_(0.0), 
        emergency_stop_(false),  // 非常停止の初期状態はfalse
        last_cmd_time_(this->now()),  // 初期化
        spi_("/dev/spidev0.0",
            100000,
            [this]() {
                Spi::Option spi_option = {};
                spi_option.bits_per_word = this->declare_parameter<int>("spi.bits_per_word", 8);
                spi_option.mode = this->declare_parameter<int>("spi.mode", 0);
                return spi_option;
            }())
    {
        this->declare_parameter<double>("wheel_base", 0.38);  // 車輪間の距離 (m)
        this->declare_parameter<double>("wheel_radius", 0.254);  // 車輪の半径 (m)

        this->get_parameter("wheel_base", wheel_base_);
        this->get_parameter("wheel_radius", wheel_radius_);

        // Twistメッセージを購読
        twist_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", rclcpp::SensorDataQoS(), std::bind(&DifferentialDriveController::twist_callback, this, std::placeholders::_1)
        );

        // Odomメッセージを配信
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS());

        // 非常停止サービスを提供
        emergency_stop_service_ = this->create_service<std_srvs::srv::SetBool>(
            "emergency_stop", 
            std::bind(&DifferentialDriveController::handle_emergency_stop, this, std::placeholders::_1, std::placeholders::_2)
        );

        battery_state_publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", rclcpp::SensorDataQoS());

        // 角速度更新タイマー
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&DifferentialDriveController::update_odometry, this)
        );

        // Transform Broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Twistメッセージを受信した時刻を更新
        last_cmd_time_ = this->now();

        // Linear and Angular velocities from the Twist message
        double linear_x = msg->linear.x;  // 前進速度 (m/s)
        double angular_z = msg->angular.z;  // 角速度 (rad/s)

        // Calculate wheel angular velocities
        command_.vel_l = (linear_x - (wheel_base_ / 2.0) * angular_z) / wheel_radius_;
        command_.vel_r = (linear_x + (wheel_base_ / 2.0) * angular_z) / wheel_radius_;
        
        command_.power_command.motor_output = true;
        command_.power_command.power_off = false;
    }

    void update_odometry()
    {
        // 現在の時刻と最後にcmd_velを受信した時刻の差を計算
        auto current_time = this->now();
        auto time_diff = current_time - last_cmd_time_;

        // cmd_velが1秒以上送られてこなかった場合、非常停止がアクティブでない場合はSPI送信データを初期化
        if (time_diff.seconds() > 1.0 && !emergency_stop_) {
            command_.vel_l = 0.0;
            command_.vel_r = 0.0;
            command_.power_command.motor_output = false;
            command_.power_command.power_off = true;
        }

        // 非常停止中はSPI通信を一時停止
        if (emergency_stop_) {
            RCLCPP_WARN(this->get_logger(), "Emergency stop is active. SPI communication paused.");
            return;
        }

        //SPI送受信を行う
        std::array<std::uint8_t,SPI_BUFFER_SIZE> tx_buffer;
        std::array<std::uint8_t,SPI_BUFFER_SIZE> rx_buffer;
        commandSerialize(&command_, tx_buffer.data());
        try{
            spi_.write(tx_buffer.data(),rx_buffer.data(),tx_buffer.size());
        }catch(Spi::Error::Kind error){
            RCLCPP_ERROR(this->get_logger(),"%d",error);
        }
        if(resultDeserialize(&result_,rx_buffer.data()) != true){
            //RCLCPP_INFO(this->get_logger(),"ERROR!!");
        }
        // RCLCPP_INFO(this->get_logger(),"%2x%2x,%2x%2x%2x%2x,%2x%2x%2x%2x,%2x%2x,%2x%2x,%2x",
        // rx_buffer.at(0),
        // rx_buffer.at(1),
        // rx_buffer.at(2),
        // rx_buffer.at(3),
        // rx_buffer.at(4),
        // rx_buffer.at(5),
        // rx_buffer.at(6),
        // rx_buffer.at(7),
        // rx_buffer.at(8),
        // rx_buffer.at(9),
        // rx_buffer.at(10),
        // rx_buffer.at(11),
        // rx_buffer.at(12),
        // rx_buffer.at(13),
        // rx_buffer.at(14)
        // );
        RCLCPP_INFO(this->get_logger(), "pwr:(%x,%x) vel:(%5f,%5f) cnt:(%x,%x),sum:%x",
        result_.power_result.i_bat,result_.power_result.v_bat,
        result_.vel_l,result_.vel_r,
        result_.cnt_l,result_.cnt_r,
        result_.check_sum
        );


        // 仮定として、車輪の回転角速度から移動量を計算
        double delta_left = result_.vel_l * wheel_radius_;
        double delta_right = result_.vel_r * wheel_radius_;

        // ロボットの進行距離と回転角度を計算
        double delta_s = (delta_left + delta_right) / 2.0;  // 平均移動距離
        double delta_theta = (delta_right - delta_left) / wheel_base_;  // 回転角度

        // 現在の姿勢を更新
        theta_ += delta_theta;
        x_ += delta_s * cos(theta_);
        y_ += delta_s * sin(theta_);

        // Odomメッセージを作成して配信
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = this->get_clock()->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        // Set the position
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        // Set the velocity
        odom_msg.twist.twist.linear.x = (delta_left + delta_right) / 2.0;
        odom_msg.twist.twist.angular.z = delta_theta / 0.1;  // Assuming the update rate is 10 Hz

        odom_publisher_->publish(odom_msg);

        // Transformの作成と配信
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = odom_msg.header.stamp;
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base_link";

        odom_tf.transform.translation.x = x_;
        odom_tf.transform.translation.y = y_;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.rotation.x = q.x();
        odom_tf.transform.rotation.y = q.y();
        odom_tf.transform.rotation.z = q.z();
        odom_tf.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(odom_tf);

        auto battery_msg = sensor_msgs::msg::BatteryState();
        battery_msg.voltage = result_.power_result.v_bat;  // バッテリー電圧 (V)
        battery_msg.current = result_.power_result.i_bat;  // 放電電流 (A)、負の場合は放電中
        battery_msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;  // 健康状態
        battery_msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;  // リチウムイオン
        battery_msg.present = true;  // バッテリーが接続されているかどうか
        battery_state_publisher_->publish(battery_msg);

        //RCLCPP_INFO(this->get_logger(), "Odometry updated: x = %f, y = %f, theta = %f", x_, y_, theta_);
    }

    // 非常停止のハンドラ
    void handle_emergency_stop(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                               std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        emergency_stop_ = request->data;
        if (emergency_stop_) {
            RCLCPP_WARN(this->get_logger(), "Emergency stop activated.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Emergency stop deactivated.");
        }
        response->success = true;
        response->message = emergency_stop_ ? "Emergency stop is now active." : "Emergency stop is now deactivated.";
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr emergency_stop_service_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double wheel_base_;
    double wheel_radius_;

    double x_, y_, theta_;  // ロボットの現在の位置と向き
    float v_left_, v_right_;  // 左右の車輪の角速度
    
    Command command_;
    Result result_;
    Spi spi_;
    bool emergency_stop_;  // 非常停止フラグ
    rclcpp::Time last_cmd_time_;  // cmd_velを受信した最後の時刻
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DifferentialDriveController>());
    rclcpp::shutdown();
    return 0;
}
