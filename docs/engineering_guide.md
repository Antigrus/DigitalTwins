# DigitalTwins 工程说明（ECEF 迁移版）

## 1. 工程功能
本工程以节点轨迹和阵面配置为输入，模拟节点半双工 TX/RX 状态下的有向 LOS 链路，计算阵面方向图增益、距离、时延、径向速度、多普勒、自由空间路径损耗、接收功率和复信道系数。配置解析、轨迹、三维数学、波束表和信道计算模块分离，便于将配置输入替换为上位机协议或嵌入式接口。

## 2. 目录
- `include/sim_types.h`：公共结构体和容量定义
- `include/geodesy.h` / `src/geodesy.c`：WGS-84 GEO/ECEF、NED/ECEF 转换
- `include/config.h` / `src/config.c`：INI/CSV 解析与对象查找
- `include/trajectory.h` / `src/trajectory.c`：轨迹加载、插值、ECEF 速度和姿态更新
- `include/math3d.h` / `src/math3d.c`：向量、矩阵、Euler 旋转
- `include/beam.h` / `src/beam.c`：方向图生成、共享缓存、结果缓存
- `include/channel.h` / `src/channel.c`：阵面 ECEF 位置/姿态、方向增益、LOS 信道
- `src/main.c`：应用流程和输出
- `config/`：示例配置
- `docs/`：完整说明

## 3. 数值精度
ECEF 位置、轨迹时间、速度、距离、时延、多普勒和旋转矩阵使用 double。节点姿态和方向图角度、波束表、阵列因子、功率 dB、复信道系数采用 float；这是精度和运算成本之间的合理折衷。地球半径约 6.4e6 m，ECEF 位置不使用 float，以避免米级分辨率损失。

## 4. 坐标和姿态
轨迹输入 GEO（纬度、经度、椭球高）与 NED 姿态角。位置通过 WGS-84 转换到 ECEF。NED 到 ECEF 矩阵的列是当地 North/East/Down 轴在 ECEF 中的表达。BODY-to-NED 使用 `Rz(yaw)*Ry(-pitch)*Rx(roll)`，BODY-to-ECEF 为 `R_ned_to_ecef * R_body_to_ned`。阵面安装偏移在 BODY 中，安装姿态为 BODY→ARRAY，波束扫描为 ARRAY 局部旋转。

## 5. 运行流程
1. `config_load()` 读取 INI、节点 CSV、阵面 CSV 和 GEO/NED 轨迹。
2. `BeamTable_InitAllNodes()` 生成/复用阵列方向图缓存。
3. 每个仿真时刻 `update_state()` 将轨迹插值到 ECEF，计算 ECEF 速度，并更新姿态。
4. 根据节点 TR 状态形成有向链路。
5. `channel()` 在 ECEF 下计算阵面位置、传播方向、距离和径向速度，再将方向变换到阵面局部坐标查询波束增益。
6. 更新 `TriNodesCalResult` 并输出。

## 6. 配置格式
`simulation.ini` 的时间字段单位 s，CSV 路径相对 INI 目录。`nodes.csv` 选择轨迹文件、TX 阵面和节点扫描角。`arrays.csv` 的安装偏移 x/y/z 是 BODY 局部 m，安装 yaw/pitch/roll 是 BODY 局部角度（deg）；阵元间距 cm，频率 MHz，单元功率 dBm。

轨迹格式为：
```csv
time_s,latitude_deg,longitude_deg,altitude_m,yaw_deg,pitch_deg,roll_deg
```
纬度北正、经度东正。高度使用 WGS-84 椭球高，若输入来自正高/海拔高度，应先进行大地水准面改正。姿态为 NED 参考：yaw 从真北向东为正，pitch 正表示前向抬起，roll 正方向遵循 `Rx(roll)` 右手旋转。轨迹位置插值采用 ECEF 线性插值；跨日期变更线不会产生经度数值跳变。姿态角采用最短角差插值。

## 7. 信道公式
阵面位置 `P_array = P_node + R_body_ecef * offset_body`。方向 `u=(P_rx-P_tx)/d`，时延 `tau=d/c`，径向速度 `vr=(v_rx-v_tx)·u`，多普勒 `fd=-vr*fc/c`，FSPL `20log10(4*pi*d*fc/c)`。阵面方向图角度从 ARRAY 局部方向计算。功率模型使用单阵元功率加阵列规模和 Tx/Rx 增益；复信道包含相干 Tx 阵列合成、方向响应和自由空间传播项。

## 8. Build 与验证
```sh
make clean
make
make test
./channel_sim config/simulation.ini
./channel_sim config/simulation.ini --time 2.5
```
单元测试覆盖 GEO/ECEF 往返、赤道和极点、NED 基轴、姿态组合基本性质。若目标平台数学库/编译器不同，需在平台上重新验证。

## 9. 限制
当前仅 LOS 自由空间单径；没有多径、遮挡、阴影衰落、极化、互耦、阵元方向图或硬件误差。轨迹插值为 ECEF 直线，适用于短段轨迹；长距离/高速轨迹建议输入更密集采样或采用曲线/动力学模型。节点、阵面和缓存仍有编译期容量限制。全局缓存使当前程序适用于单仿真实例。
