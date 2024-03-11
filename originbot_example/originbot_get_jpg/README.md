originbot_get_jpg 支持获取指定帧数图片；

`ros2 run originbot_get_jpg take_pictures` 默认读取image_raw数据

`ros2 run originbot_get_jpg take_pictures  --ros-args -p sub_img_topic:=/hbmem_img  --ros-args -p take_nums:=10` 参数化至读取nv12格式图片，默认先读取10张图片；


