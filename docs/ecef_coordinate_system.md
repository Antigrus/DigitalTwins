# GEO / NED / ECEF 坐标体系与姿态变换
## 坐标定义
GEO：纬度北正、经度东正、WGS-84椭球高。ECEF：X指纬度0经度0，Y指纬度0经度90E，Z北极。NED为North/East/Down。BODY右手系 +X前向、+Y右、+Z下；ARRAY +X法向、+Y水平阵列轴、+Z上。
## GEO→ECEF
WGS-84 a=6378137 m，f=1/298.257223563，e²=f(2−f)。N=a/sqrt(1−e²sin²φ)：X=(N+h)cosφcosλ，Y=(N+h)cosφsinλ，Z=(N(1−e²)+h)sinφ。
## NED→ECEF
R列为North/East/Down基轴：[-sinφcosλ,-sinλ,-cosφcosλ; -sinφsinλ,cosλ,-cosφsinλ; cosφ,0,-sinφ]。v_ECEF=R v_NED，逆为转置。
## 姿态
BODY→NED=Rz(yaw)Ry(pitch)Rx(roll)。yaw从真北向东为正；pitch正机头抬起；roll按Rx右手旋转。BODY→ECEF=R_NED_ECEF R_BODY_NED。示例阵面roll=180°，将阵面 +Z 向上。阵面扫描使用Rz(az)Ry(-el)，正el指向阵面+Z。
## 阵面/信道
P_array=P_node+R_body_ecef offset_body；R_array_ecef=R_body_ecef R_mount R_scan。u=(P_rx−P_tx)/d；接收方向为−u。u_array=R_array_ecef^T u。距离、时延、径向速度、多普勒均在ECEF计算。
## 精度
ECEF几何及矩阵double；方向图角、功率dB、波束表、复信道float。位置不使用float。
## 轨迹
CSV：time_s,latitude_deg,longitude_deg,altitude_m,yaw_deg,pitch_deg,roll_deg。位置ECEF线性插值；姿态角最短角差插值；椭球高单位m。轨迹至少两个点、时间严格递增。
