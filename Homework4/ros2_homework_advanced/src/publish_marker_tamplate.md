# RViz Marker 发布模板

下面是一个可直接套用的 C++ 模板，用于在 ROS2 中向 rviz 发布 Marker。你可以把其中的坐标系、类型、颜色、尺寸和位置按需替换。

## C++ 模板

```cpp
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

class MarkerPublisher : public rclcpp::Node
{
public:
	MarkerPublisher() : Node("marker_publisher")
	{
		// 创建 Marker 发布器，话题名可以根据需要修改
		marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

		// 定时发布，避免 rviz 里只显示一瞬间
		timer_ = this->create_wall_timer(500ms, std::bind(&MarkerPublisher::publishMarker, this));
	}

private:
	void publishMarker()
	{
		visualization_msgs::msg::Marker marker;

		// 1. 设置坐标系，必须和 rviz 的 Fixed Frame 对应
		marker.header.frame_id = "map";
		marker.header.stamp = this->now();

		// 2. 设置命名空间和 ID，用于区分不同 Marker
		marker.ns = "basic_shapes";
		marker.id = 0;

		// 3. 设置类型：CUBE / SPHERE / ARROW / TEXT_VIEW_FACING / LINE_STRIP / POINTS 等
		marker.type = visualization_msgs::msg::Marker::SPHERE;

		// 4. 设置动作：ADD 表示添加或更新，DELETE 表示删除
		marker.action = visualization_msgs::msg::Marker::ADD;

		// 5. 设置位姿
		marker.pose.position.x = 0.0;
		marker.pose.position.y = 0.0;
		marker.pose.position.z = 0.5;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;

		// 6. 设置尺寸，注意不同类型对 scale 的含义可能不同
		marker.scale.x = 0.5;
		marker.scale.y = 0.5;
		marker.scale.z = 0.5;

		// 7. 设置颜色，alpha 必须大于 0，否则无法显示
		marker.color.r = 0.1f;
		marker.color.g = 0.8f;
		marker.color.b = 0.2f;
		marker.color.a = 1.0f;

		// 8. 生命周期，0 表示一直显示
		marker.lifetime = rclcpp::Duration::from_seconds(0.0);

		// 9. 发布 Marker
		marker_pub_->publish(marker);
	}

	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MarkerPublisher>());
	rclcpp::shutdown();
	return 0;
}
```
