# GEO / NED / ECEF 坐标体系与姿态变换

## 坐标定义
- GEO：纬度北正、经度东正；高度为 WGS-84 椭球高。
- ECEF：X 指向 lat=0 lon=0，Y 指向 lat=0 lon=90E，Z 指向北极；单位 m。
- NED：当地 North/East/Down 右手系。
- BODY：节点本体，默认 +X 前向、+Y 右、+Z 上。Euler yaw/pitch/roll，deg。

## GEO→ECEF
WGS-84 a=6378137 m，f=1/298.257223563，e²=f(2−f)。φ纬度、λ经度、h椭球高，N=a/sqrt(1−e²sin²φ)：X=(N+h)cosφcosλ，Y=(N+h)cosφsinλ，Z=(N(1−e²)+h)sinφ。

## NED→ECEF
矩阵列为 N/E/D 基轴：R=[-sinφcosλ,-sinλ,-cosφcosλ; -sinφsinλ,cosλ,-cosφsinλ; cosφ,0,-sinφ]。v_ECEF=R v_NED，反向为转置。

## 姿态
BODY→NED：Rz(yaw) Ry(-pitch) Rx(roll)。yaw 从真北向东为正，pitch 正为机头抬起，roll 按 Rx 右手旋转。BODY→ECEF：R_NED_ECEF R_BODY_NED。阵面安装姿态 BODY→ARRAY，波束扫描在阵面局部坐标叠加。

## 阵面与信道
P_array=P_node+R_BODY_ECEF offset_BODY。R_array_ECEF=R_BODY_ECEF R_mount R_scan。传播方向 u=(P_rx−P_tx)/d；接收方向用 −u。方向向量转 ARRAY 局部使用 R_array_ECEF^T。距离、时延、速度投影和多普勒全部在 ECEF 计算。

## 数值精度
ECEF 位置、速度、距离、时延、多普勒、矩阵用 double；波束角、方向图、功率 dB 和复信道系数用 float。地球尺度坐标不使用 float。

## 轨迹 CSV
格式：time_s,latitude_deg,longitude_deg,altitude_m,yaw_deg,pitch_deg,roll_deg。时间严格递增，至少两个点。位置转换到 ECEF 后插值，速度为 ECEF 线段差分。姿态角最短角差插值。端点外位置钳制，速度使用邻接线段值。高度须为椭球高；若来源为正高需先改正。

## 验证
make test 检查赤道、北极、GEO/ECEF 往返、NED基轴和矩阵正交性。
