# DigitalTwins 工程说明（ECEF 迁移版）
## 1. 功能与目录
C11 多节点、多阵面动态 LOS 信道模拟。配置、轨迹、地理坐标、三维数学、波束缓存、信道和应用流程分模块。
- geodesy：WGS-84 GEO/ECEF、NED/ECEF
- config：INI/CSV 解析、对象查找
- trajectory：GEO/NED轨迹加载、ECEF插值和速度
- math3d：向量、矩阵、姿态
- beam：阵列方向图和缓存
- channel：阵面位置/姿态、增益、LOS信道
- main：链路枚举和输出

## 2. 精度策略
ECEF 位置、轨迹时间、速度、距离、时延、多普勒和矩阵用 double。姿态角、方向图角度、波束表、dB、复信道系数用 float。地球尺度位置不使用 float。

## 3. 坐标约定
轨迹输入 GEO：纬度北正、经度东正、WGS-84椭球高。ECEF X指(0°,0°)，Y指(0°,90°E)，Z指北极。NED为North/East/Down。BODY采用右手NED对齐约定：+X前向、+Y右侧、+Z下。BODY→NED = Rz(yaw) Ry(pitch) Rx(roll)，yaw从真北向东为正，pitch正机头抬起，roll按Rx右手旋转。BODY→ECEF = R_NED_ECEF R_BODY_NED。ARRAY局部 +X为法向、+Y为水平阵列轴、+Z为竖直向上。示例阵面安装 roll=180°，使阵面 +Z 与 BODY向下轴反向。扫描角在 ARRAY 局部，正 elevation 指向 +Z。
阵面位置 P_array=P_node+R_body_ecef offset_body。阵面旋转 R_array_ecef=R_body_ecef R_mount R_scan。传播方向转换到阵面局部使用矩阵转置。

## 4. 轨迹
CSV格式：time_s,latitude_deg,longitude_deg,altitude_m,yaw_deg,pitch_deg,roll_deg。至少两个点且时间递增。先将位置采样转换到 ECEF，再做线性插值，速度由当前线段 ECEF差分计算。姿态角最短角差插值。超出轨迹时间范围位置钳制端点，速度使用邻接线段。若输入高度为正高，需先转换椭球高。

## 5. 配置
simulation.ini 指定 start/end/step 以及 nodes/arrays CSV。nodes.csv 包含节点状态、轨迹路径、TX阵面选择和扫描角。arrays.csv 包含阵面所属节点、安装偏移（BODY局部m）、安装姿态（deg）、阵元数、间距cm、频率MHz和单元功率dBm。路径相对 INI 所在目录。

## 6. 信道公式
d=|P_rx−P_tx|；tau=d/c；vr=(v_rx−v_tx)·u；fd=−vr fc/c；FSPL=20log10(4πd fc/c)。功率估算按单元功率+阵列规模+Tx/Rx方向增益−FSPL。复信道包含相干Tx阵列因子、Rx方向响应和自由空间传播。
当前仅单径 LOS；无多径、遮挡、阴影、极化、互耦、阵元方向图或硬件误差。

## 7. 编译运行
make clean && make；make test；./channel_sim config/simulation.ini；./channel_sim config/simulation.ini --time 2.5。

## 8. API生命周期
config_load完成配置和轨迹读取；BeamTable_InitAllNodes生成/复用方向图；update_state按时刻更新状态；channel计算一条TX→RX阵面链路；TriNodesCalResult_Update保存结果；config_free释放轨迹。全局缓存适用于单实例，配置变更需在计算任务安全同步后重建。
