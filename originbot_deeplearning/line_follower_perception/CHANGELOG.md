# Changelog for package line_follower_perception

tros_2.1.0 (2024-03-28)
------------------
1. 新增适配ros2 humble零拷贝。
2. 新增中英双语Readme。
3. 适配重构dnn_node。
4. 零拷贝通信使用的qos的Reliability由RMW_QOS_POLICY_RELIABILITY_RELIABLE（rclcpp::QoS(10)）变更为RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT（rclcpp::SensorDataQoS()）。

tros_2.0.0 (2023-05-11)
------------------
1. 更新package.xml，支持应用独立打包

tros_1.1.2rc1 (2022-10-09)
------------------
1. 头文件中删除不必要的类型引用。
