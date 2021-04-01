* color_velocity.cpp<br/>
차량의 권장 주행 속력을 색상으로 표시해 놓은 (RGB) 지도를 받아 단색 지도로 변환하는 프로그램<br/>
(ex. {0, 0, 0} → 85(기본값), {0, 0, 255} → 110(교차로, 3m/s), {255, 255, 0} → 127(교
차로, 2.6m/s))

* sector_publisher.cpp<br/>
차량의 위치(/filtered_data)를 받아서 현재 차량이 위치한 곳의 섹터(/sector_info)를 전송하는 노드<br/>
(ex. {0, 0, 0} → Sector A(기본값, 직진), {0, 255, 140} → Sector F(교차로 중 하나),
...)

* velocity_publisher.cpp<br/>
차량의 위치(/filtered_data)를 받아서 현재 차량이 위치한 곳의 권장 주행 속력(/recommend_vel)을 전송하는 노드<br/>
(권장 속력 = v , 지도의 픽셀 값 = z 라 할 때, v = 340 / z)
