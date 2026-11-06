# 三节点多阵面信道模拟工程（波束表缓存版）

> **详细工程说明：** 结构体设计、接口函数、波束表缓存复用、节点/阵面数量扩展、DSP 移植注意事项请阅读 [`docs/engineering_guide.md`](docs/engineering_guide.md)。

## 1. 本版本改动

- 节点增加 `tr_state`，支持 `TX` / `RX`（也支持 CSV 中使用 `1` / `0`）。节点是半双工状态，节点下所有阵面自动继承该状态；阵面不再单独配置 `can_tx/can_rx`。
- 阵面配置包含横向/纵向阵元数量、横向/纵向阵元间距（cm）、工作频率（MHz）、单阵元发射功率（dBm）。
- 波束增益表不再从 TXT 文件读取，而是在初始化阶段按阵列因子公式计算。
- 所有节点和阵面共享一个全局波束缓存。缓存键为 `(num_h, num_v, spacing_h_cm, spacing_v_cm, frequency_mhz)`；命中返回已有索引，未命中才生成新表。
- 保留节点轨迹、速度导出的姿态、阵面相对节点的位置/安装姿态、自由空间路径损耗、多普勒和复信道输出。
- 结果只输出到控制台，不导出 CSV。

## 2. 波束增益公式

波长由阵面频率计算：`lambda = c / (frequency_mhz * 1e6)`。

阵面局部坐标约定：局部 `+X` 为阵面法向（波束正前方），`+Y` 为水平阵列轴，`+Z` 为垂直阵列轴。角度定义为 `theta` 仰角、`phi` 方位角，方向余弦为：

- 水平阵列轴：`sin(phi) * cos(theta)`
- 垂直阵列轴：`sin(theta)`

配置中的 `spacing_h_cm`、`spacing_v_cm` 使用 cm；计算相位前先乘以 `0.01f` 转换为 m。相邻阵元相位差：

`phase_h = 2*pi*((spacing_h_cm*0.01)/lambda)*sin(phi)*cos(theta)`

`phase_v = 2*pi*((spacing_v_cm*0.01)/lambda)*sin(theta)`

本工程生成的表值与给定 `cal_beamform()` 一致，采用：

`gain = abs(sum(exp(j*phase))) / sqrt(M*N)`

其中 `M=num_h`，`N=num_v`。代码利用矩形均匀阵列的可分离性质，将二维复数求和改写为两个一维 Dirichlet 因子，数学结果等价，但避免每个角度点都遍历全部阵元。给定公式中的 `/2` 对应 `spacing_h=spacing_v=lambda/2`；本版本将其推广为独立的横、纵阵元间距。

表维度为 `181 x 181`，索引 `theta_index=theta_deg+90`、`phi_index=phi_deg+90`，角度范围均为 `[-90,90]`。表中保存线性场幅度，不是 dB；信道计算时转换为 `20*log10(gain)`。

## 3. 缓存与内存

缓存全局唯一，跨节点、跨阵面共享。顺序查找即可，配置完全匹配时返回已有索引。`MAX_BEAM_TABLES` 默认 8，可在编译时覆盖，例如：

`make CFLAGS='-std=c11 -O2 -Wall -Wextra -Wpedantic -DMAX_BEAM_TABLES=4'`

一张 `181x181 float` 表占 `131044` bytes；默认 8 张约 `1.0 MiB`。DSP RAM 不足时应下调最大缓存数，或把缓存放到可访问的外部 DDR/专用存储段。

## 4. 配置文件

`config/nodes.csv`：

`id,name,enabled,tr_state,trajectory_file,tx_array_id,beam_azimuth_deg,beam_elevation_deg`

`tr_state` 为 `TX` 或 `RX`，也可写 `1` 或 `0`。`tx_array_id` 是指定发射阵面的 **array_id**；`-1` 表示自动遍历该节点的所有启用阵面。指定 ID 必须存在且启用。`beam_azimuth_deg/beam_elevation_deg` 是波束扫描角，定义在节点各阵面的局部坐标系中；当前同一节点的阵面共用这组扫描角，既用于发射波束，也用于接收波束。默认 `0,0` 表示不扫描。方位角绕阵面局部 +Z 轴旋转，俯仰角使局部 +X 波束轴朝局部 +Z 方向抬起。

`config/arrays.csv`：

`array_id,node_id,name,enabled,x,y,z,yaw,pitch,roll,num_h,num_v,spacing_h_cm,spacing_v_cm,frequency_mhz,element_tx_power_dbm`

其中 `x,y,z` 是阵面相对节点的位置偏移；`yaw,pitch,roll` 是阵面相对节点的安装姿态。频率单位 MHz，单阵元发射功率单位 dBm，阵元间距单位 cm。

本工程配置文件用于模拟上位机下发并已解析完成的数据；不再读取任何波束方向图文件。

`Node` 保存 `tx_array_id` 和 `beam_direction`；`ArrayMount.initial_attitude` 仍只表示阵面相对节点的固定安装姿态。最终方向变换采用 `R_node * R_mount * R_scan`，扫描角不会覆盖安装姿态。

## 5. 收发状态与链路

程序遍历节点对。若两个节点状态相反，则 TX 节点向 RX 节点建立有向链路；状态相同则不建立链路。节点状态决定其所有阵面的收发角色。

每条链路要求 Tx/Rx 阵面工作频率一致（误差不超过 `1e-6 MHz`），否则该阵面对链路不可用。

每个节点指定 `tx_array_id>=0` 时，只使用该阵面发射；接收节点仍遍历其所有启用阵面。设为 `-1` 时，发射端也遍历全部启用阵面。

全局 `g_tri_nodes_cal_result`（类型 `TriNodesCalResult`）保存每条有向节点/阵面链路的最新 `ChannelResult`。链路键为 `(txid, rxid, txaid, rxaid)`：首次计算追加记录，后续帧只覆盖同键记录，不会覆盖其他链路。`count` 表示当前已登记链路数，容量由 `MAX_LINK_RESULTS` 限制（默认最大配置上限为 15360 条记录，约 1 MiB）。可通过编译宏 `-DMAX_LINK_RESULTS=256` 等方式降低 RAM 占用，但必须确保容量覆盖实际链路数。多任务系统中若计算任务和上传任务并发访问，应在外层加临界区或采用双缓冲。

输出的接收功率估算采用单阵元发射功率作为输入：

`Pr(dBm) = P_element(dBm) + 10log10(M_tx*N_tx) + Gtx(dB) + Grx(dB) - FSPL(dB)`

这里 `Gtx/Grx=20log10(table_amplitude)`。该式按所有发射阵元同频同相馈电、阵列因子归一化方式与给定公式一致的理想模型计算；未加入馈线损耗、阵元方向图、互耦、极化失配和硬件幅相误差。

## 6. 编译运行

```sh
make
make run
./channel_sim config/simulation.ini --time 2.5
make clean
```

Windows 可使用 MSYS2/MinGW-w64 的 gcc 和 make。
