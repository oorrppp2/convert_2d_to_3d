# convert_2d_to_3d

darknet 패키지로부터 /darknet_ros/bounding_boxes 메시지 토픽을 받고, 
realsense 패키지로부터 카메라 포인트 클라우드 정보인 /camera/depth_registered/points 메시지 토픽을 받아
2D bounding box의 center point를 3D camera coordinate 값을 변환하여 /detected_object 메시지 토픽을 발행하는 패키지 입니다.
