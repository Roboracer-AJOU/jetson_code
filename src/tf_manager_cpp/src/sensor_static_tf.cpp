#include <cmath>
#include <memory>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

class SensorStaticTF : public rclcpp::Node
{
public:
  SensorStaticTF() : Node("sensor_static_tf_node")
  {
    broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    publish_lidar_tf();
    publish_imu_tf();

    // /tf_static 을 주기적으로 다시 낸다.
    //
    // TRANSIENT_LOCAL 로 한 번만 내는 게 정석이고, ROS 노드들은 다 그렇게 받는다
    // (tf2 리스너가 TRANSIENT_LOCAL 로 붙는다). 문제는 foxglove_bridge 가
    // **VOLATILE** 로 붙는다는 것이다 — 브릿지가 이 노드보다 늦게 뜨면 이미
    // 나간 latch 를 못 받고, base_link->laser 가 영영 안 온다.
    //
    // 그러면 Foxglove 에서 `laser` 프레임 토픽이 통째로 안 그려진다
    // ("Missing transform from frame <laser> to frame <map>"). /scan 도, FGM
    // 갭 마커도 안 보이고, `map` 프레임인 /local_path 만 보인다. 배터리 갈고
    // 노드를 다시 켤 때마다 됐다 안 됐다 한 게 이 시작 순서 문제였다.
    //
    // 값은 매번 같으니 tf2 쪽은 덮어쓰기라 무해하다. 늦게 붙는 구독자를
    // 위한 것이라 1 Hz 면 충분하다.
    republish_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        publish_lidar_tf();
        publish_imu_tf();
      });
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr republish_timer_;
  bool logged_once_ = false;

  // 재발행 때마다 declare_parameter 를 부르면 두 번째에 예외가 난다.
  double param(const std::string & name, double fallback)
  {
    if (!has_parameter(name)) {
      return declare_parameter(name, fallback);
    }
    return get_parameter(name).as_double();
  }

  void publish_lidar_tf()
  {
    // lidar_yaw 기본값 0.0 = 스캔 0deg 가 차 정면을 가리킨다는 전제.
    //
    // 2026-08-15 실측 기록 (TF·Cartographer·휠오도를 안 쓰는 측정. 물체를 차에
    // 붙인 채 주위를 한 바퀴 돌리며 /scan 원본의 최근접점 각도를 기록):
    //   차 정면 -> 스캔 -177deg,  차 왼쪽 -> -90deg,
    //   차 뒤   -> 0deg 부근(차체에 가려 95deg 폭이 무효),
    //   차 오른쪽 -> +90deg,  누적 회전 +2.3deg (닫힘, 미러링 없음)
    // 즉 측정 당시에는 base_link 각도 = 스캔 각도 + 180deg 였다.
    //
    // 그 상태 그대로면 이 값은 π 여야 하고, 0.0 은 스캔 0deg 가 차 정면을
    // 향하도록 (라이다를 돌려 달거나 드라이버에서 각도를 회전시켜) 맞춰
    // 놓은 경우에만 맞다. 스캔 0deg 방향이 바뀌면 이 값도 같이 바꿔야 한다.
    const double roll = param("lidar_roll", 0.0);
    const double pitch = param("lidar_pitch", 0.0);
    const double yaw = param("lidar_yaw", 0.0);

    geometry_msgs::msg::TransformStamped tf;

    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "laser";

    tf.transform.translation.x = 0.31;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.20;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    if (!logged_once_) {
      RCLCPP_INFO(
        get_logger(),
        "LiDAR TF base_link->laser xyz=(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f)",
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z,
        roll, pitch, yaw);
    }

    broadcaster_->sendTransform(tf);
  }

  void publish_imu_tf()
  {
    // ebimu_driver가 칩 축(+X 왼쪽, +Y 앞, +Z 아래)을 ROS 축으로 바꿔 발행함.
    // (IMU를 다시 달면서 수평면 180° 돌아간 상태 — ebimu_driver 상단 주석 참고)
    // imu_link = base_link과 같은 방향(+X 앞, +Y 왼쪽, +Z 위). 위치만 IMU 장착점.
    // 축 변환은 드라이버가 하므로 여기 rpy는 0이 맞다. 장착점이 바뀌면
    // imu_x/y/z만 고친다.
    // Cartographer tracking_frame Z가 아래면 맵이 뒤집히고 스캔이 그리드에 안 붙음.
    const double roll = param("imu_roll", 0.0);
    const double pitch = param("imu_pitch", 0.0);
    const double yaw = param("imu_yaw", 0.0);

    geometry_msgs::msg::TransformStamped tf;

    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "imu_link";

    // 뒷바퀴 축(base_link 원점) 기준 실측. 예전 값 (0.21, 0.035, 0.035).
    // +x 앞, +y 왼쪽. z는 재측정 안 했으므로 그대로 둔다.
    tf.transform.translation.x = param("imu_x", 0.22);
    tf.transform.translation.y = param("imu_y", 0.025);
    tf.transform.translation.z = param("imu_z", 0.035);

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    if (!logged_once_) {
      RCLCPP_INFO(
        get_logger(),
        "IMU TF base_link->imu_link xyz=(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f)",
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z,
        roll, pitch, yaw);
      RCLCPP_INFO(
        get_logger(),
        "/tf_static 1Hz 재발행 — foxglove_bridge 가 VOLATILE 로 붙어서, "
        "늦게 떠도 laser 프레임을 받게 하려는 것");
      logged_once_ = true;
    }

    broadcaster_->sendTransform(tf);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorStaticTF>());
  rclcpp::shutdown();
  return 0;
}
