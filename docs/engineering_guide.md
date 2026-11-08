# 三节点多阵面动态信道模拟工程说明文档

> 版本：v3（共享波束表缓存版）  
> 适用对象：工程维护者、后续功能开发者，以及将算法移植到 DSP/嵌入式平台的开发者。

## 1. 工程定位与设计原则

本工程用于模拟多个运动节点之间的自由空间视距（LOS）无线信道。每个节点可以安装一个或多个阵面；节点采用半双工收发状态，节点状态决定该节点全部阵面的角色。程序根据节点轨迹计算位置、速度和姿态，再根据节点收发状态形成有向链路，逐个计算发射阵面与接收阵面之间的信道结果。

工程刻意将**节点、节点运动状态、阵面、阵列方向图、链路结果**分开管理，主要为了支持以下可复用场景：

1. 多个阵面使用相同阵列参数时，只计算一次波束增益表，其他阵面共享该表。
2. 同一节点可以挂载不同数量的阵面；节点数量和每节点阵面数量通过配置数据决定，代码中的宏定义提供容量上限。
3. 阵面安装位置、安装姿态和发射功率可以不同，而不必重复计算相同阵列几何参数对应的方向图。
4. 上位机配置解析完成后，可以把解析结果填入本工程的数据结构，并调用初始化接口；当前 CSV 仅用于模拟“上位机已下发并解析”的输入，不包含 UDP 通信功能。
5. 算法核心与配置文件解析、控制台输出相对独立，便于后续替换为 DSP 工程中的数据接收接口、任务调度和输出接口。

### 1.1 目录结构

```text
three_node_channel_sim_c_v3/
├── include/
│   └── sim.h                 # 全部公共结构体、宏定义和接口声明
├── src/
│   ├── main.c                # 示例主流程、链路枚举和控制台输出
│   ├── config.c              # INI/CSV 配置解析和对象初始化
│   ├── trajectory.c          # 轨迹加载、位置/速度/姿态更新
│   ├── math3d.c              # 三维向量和旋转矩阵运算
│   ├── beam.c                # 阵列方向图计算、缓存、查表
│   └── channel.c              # 阵面位置、方向增益和 LOS 信道计算
├── config/
│   ├── simulation.ini
│   ├── nodes.csv
│   ├── arrays.csv
│   └── trajectories/          # 各节点轨迹文件
├── docs/
│   ├── design_notes.md
│   └── engineering_guide.md   # 本说明文档
├── Makefile
└── README.md
```

### 1.2 初始化与运行的总体顺序

```text
配置输入（当前示例为 INI/CSV）
        │
        ▼
config_load()
  ├─ 创建 Node 对象
  ├─ 创建 NodePosition 对象并加载轨迹
  └─ 将阵面参数写入 Node.arrays[]
        │
        ▼
BeamTable_InitAllNodes()
  ├─ 遍历所有节点
  ├─ 遍历每个节点的所有阵面
  ├─ 用阵列参数查询共享缓存
  └─ 未命中时计算并保存 181×181 方向图
        │
        ▼
按仿真时刻循环
  ├─ update_state() 更新位置、速度、姿态
  ├─ 根据节点 TX/RX 状态枚举有向链路
  └─ 对每组 Tx 阵面 / Rx 阵面调用 channel()
        │
        ▼
ChannelResult（距离、时延、多普勒、增益、功率、复信道）
```

在真实上位机/嵌入式系统中，配置解析层可以替换为网络接收和协议解析层；关键要求是，在调用波束缓存初始化之前，所有节点及阵面的配置字段必须已经完整、有效地写入 `Node` 对象。

---

## 2. 核心对象与结构体

所有公共结构体定义在 `include/sim.h`。以下按对象之间的关系说明。

### 2.1 `Vec3`：三维向量

```c
typedef struct { float x, y, z; } Vec3;
```

用于统一表示全局位置、速度、阵面相对节点的偏移量，以及方向向量。坐标单位由使用场景决定：位置/偏移量通常为 m，速度为 m/s，方向向量通常为单位向量。

相关接口：

- `v3(x, y, z)`：创建向量。
- `add(a, b)`、`sub(a, b)`：向量加减。
- `scale(a, s)`：向量数乘。
- `dot(a, b)`：点积。
- `norm(a)`：向量模长。

注意：`norm()` 不会自动归一化向量。需要单位方向向量时，应显式使用 `scale(v, 1.0 / norm(v))`，并先判断模长是否接近零。

### 2.2 `Attitude`：姿态角

```c
typedef struct { float heading, pitch, roll; } Attitude;
typedef struct { float azimuth, elevation; } BeamDirection;
```

三个字段均以**度**为单位：

- `heading`：绕全局/局部 Z 轴的航向角。
- `pitch`：俯仰角。
- `roll`：滚转角。

`math3d.c` 中的欧拉旋转顺序为：

\[
R=R_z(heading)R_y(-pitch)R_x(roll)
\]

矩阵内部计算时才转换为弧度。调用接口和填充结构体时不要传弧度。当前轨迹更新策略根据速度方向计算 heading 和 pitch，并将 roll 置零；若设备有独立姿态传感器，应由外部姿态数据覆盖该策略。

### 2.3 `BeamDirection`：波束扫描方向

```c
typedef struct { float azimuth, elevation; } BeamDirection;
```

单位为度，表示相对于阵面局部坐标系的扫描偏移，而不是世界坐标系角度，也不是阵面安装姿态。方位角绕局部 +Z 轴旋转；俯仰角使局部 +X 波束轴朝局部 +Z 方向抬起。当前波束扫描参数放在 `Node` 中，因此该节点的各阵面共用扫描方向；发射和接收计算都会应用该方向。如果未来需要每个阵面独立扫描，可将该字段下沉到 `ArrayMount`。

### 2.4 `Complex`：复数

```c
typedef struct { float re, im; } Complex;
```

用于保存复信道系数的实部和虚部。当前没有封装复数运算接口；`channel()` 直接计算并写入 `ChannelResult.h`。

### 2.5 `TrajectoryPoint`：轨迹采样点

```c
typedef struct { float t; Vec3 p; } TrajectoryPoint;
```

- `t`：采样时刻，单位 s。
- `p`：该时刻的全局位置，单位 m。

轨迹文件按时间递增排列。`load_trajectory()` 要求至少两个有效点，且相邻时间严格递增。轨迹点之间使用线性插值；速度取当前所在轨迹线段的差分速度，因此在轨迹折点处速度可能发生跳变。

### 2.5 `BeamTableConfig`：方向图生成参数，也是缓存键

```c
typedef struct {
    uint16_t num_h;
    uint16_t num_v;
    float spacing_h_cm;
    float spacing_v_cm;
    float frequency_mhz;
} BeamTableConfig;
```

字段含义：

| 字段 | 含义 | 单位 |
|---|---|---|
| `num_h` | 水平方向阵元数 M | 个 |
| `num_v` | 垂直方向阵元数 N | 个 |
| `spacing_h_cm` | 水平相邻阵元间距 | cm |
| `spacing_v_cm` | 垂直相邻阵元间距 | cm |
| `frequency_mhz` | 阵列工作频率 | MHz |

**这五个字段共同决定本工程的理想均匀矩形阵列方向图，因此也构成缓存键。**

字段变更影响判断：

- 阵元数变化：方向图变化，必须使用另一张表。
- 横向或纵向间距变化：方向图变化，必须使用另一张表。
- 工作频率变化：波长变化，方向图变化，必须使用另一张表。
- 阵面安装位置/姿态变化：不改变阵面局部方向图，不需要新表。
- 单阵元发射功率变化：只影响链路预算，不改变方向图，不需要新表。
- 阵面名称、节点 ID、阵面 ID 变化：不改变方向图，不需要新表。

当前缓存浮点比较容差为：水平/垂直间距 `1e-6 cm`，频率 `1e-6 MHz`（约 1 Hz）。这些容差适配单精度 `float`，不要沿用双精度时期的极小容差。如果上位机配置本身存在较大数值量化误差，应该在配置归一化阶段统一参数精度，而不是随意扩大缓存比较容差。

### 2.6 `BeamTableEntry`：一张已计算的方向图

```c
typedef struct {
    BeamTableConfig config;
    float gain[NTH][NPH];
} BeamTableEntry;
```

每个缓存条目包含生成参数和一张 `181×181` 的二维 `float` 表：

```c
float gain[NTH][NPH];
```

`gain[theta_idx][phi_idx]` 保存对应角度组合的**线性场幅度**，不是 dB，也不是功率增益。第一维对应 theta（俯仰/仰角），第二维对应 phi（方位角）。两个角度的范围均为 `[-90°, +90°]`，网格间隔为 1°。

角度与数组下标的对应关系：

```c
int theta_idx = (int)(theta_deg + 90.0);
int phi_idx   = (int)(phi_deg + 90.0);
float g = entry->gain[theta_idx][phi_idx];
```

例如 `gain[0][0]` 对应 `theta=-90°、phi=-90°`；`gain[90][90]` 对应 `theta=0°、phi=0°`；`gain[180][180]` 对应 `theta=+90°、phi=+90°`。实际查询前必须保证角度位于范围内，并注意浮点角度不能直接作为数组下标。信道计算中，方向图线性幅度通过 `20*log10(gain)` 转换为 dB。

### 2.7 `BeamTableCache`：全局共享缓存

```c
typedef struct {
    uint16_t count;
    BeamTableEntry entries[MAX_BEAM_TABLES];
} BeamTableCache;
```

- `count`：当前已经成功生成的不同方向图数量。
- `entries[]`：方向图条目数组。
- `MAX_BEAM_TABLES`：缓存最大条目数，默认 8，可在编译时覆盖。

工程定义了全局对象：

```c
extern BeamTableCache g_beam_cache;
```

它由 `src/beam.c` 实际定义。所有节点、所有阵面共用这一份缓存，不应在每个节点或每个阵面中各自创建独立缓存，否则会失去跨节点复用效果。

内存估算：

- 一张表：`181 × 181 × sizeof(float) = 131044 bytes`。
- 默认 8 张表：约 1.0 MiB（不含少量配置字段）。
- 16 张表：约 2.0 MiB。

在 DSP 上应根据可用 RAM 设置 `MAX_BEAM_TABLES`，也可以将缓存放到外部 DDR 或专用内存段。注意：增加缓存条目上限会直接增加静态内存占用。

### 2.8 `ArrayMount`：节点上的一个阵面

```c
typedef struct {
    int id;
    int enabled;
    char name[MAX_NAME];
    Vec3 offset;
    Attitude initial_attitude;
    BeamTableConfig beam_config;
    float element_tx_power_dbm;
    int16_t beam_table_index;
} ArrayMount;
```

| 字段 | 说明 |
|---|---|
| `id` | 阵面 ID，应在系统中保持可区分 |
| `enabled` | 阵面使能标志，0 表示不参与信道计算 |
| `name` | 阵面名称 |
| `offset` | 阵面相对节点参考点的位置偏移，单位 m |
| `initial_attitude` | 阵面相对节点的安装姿态，单位度 |
| `beam_config` | 阵列几何和频率参数 |
| `element_tx_power_dbm` | 单阵元发射功率，单位 dBm |
| `beam_table_index` | 该阵面对应的共享缓存索引，由缓存初始化接口写入 |

`beam_table_index` 是运行期关联字段，不应由上位机直接指定。加载新配置后必须重新调用 `BeamTable_InitAllNodes()`，由程序根据 `beam_config` 重新分配索引。

阵面位置和安装姿态属于阵面实例属性；方向图表属于阵列参数属性。多个阵面可以有不同的 `offset`、`initial_attitude`、`element_tx_power_dbm`，同时共享同一个 `beam_table_index`。

### 2.9 `Node`：节点对象

```c
typedef struct {
    int id;
    int enabled;
    char name[MAX_NAME];
    NodeTRState tr_state;
    Vec3 velocity;
    Attitude attitude;
    ArrayMount arrays[MAX_ARRAYS_PER_NODE];
    size_t array_count;
} Node;
```

节点负责保存设备级属性：

- `tr_state`：节点半双工状态，`NODE_TX=1`、`NODE_RX=0`。
- `velocity`：当前速度，单位 m/s。
- `attitude`：当前节点姿态，单位度。
- `arrays[]`：该节点挂载的阵面实例。
- `array_count`：当前实际挂载的阵面数。

**收发状态属于节点，不属于阵面。** 一个节点为 TX 时，其全部使能阵面都作为发射阵面；一个节点为 RX 时，其全部使能阵面都作为接收阵面。若业务上需要同一节点的不同阵面同时具有不同收发状态，则当前半双工节点模型需要调整，不能只修改 CSV。

### 2.10 `NodePosition`：节点运动数据

```c
typedef struct {
    int id;
    int enabled;
    char name[MAX_NAME];
    char trajectory_file[MAX_PATH];
    TrajectoryPoint *trajectory;
    size_t trajectory_count;
    Vec3 position;
} NodePosition;
```

将运动数据从 `Node` 中分离，使节点静态属性和运动轨迹互不耦合。

- `trajectory`：动态分配的轨迹点数组。
- `trajectory_count`：有效轨迹点数量。
- `position`：最近一次 `update_state()` 更新得到的全局位置。

`trajectory` 的内存由 `load_trajectory()` 分配，必须由 `config_free()` 释放。不要手动释放后再次调用 `update_state()`。

### 2.11 `Node` 的指定发射阵面与波束方向

`Node` 新增字段：

```c
int tx_array_id;                 /* -1 自动选择；否则使用对应 ArrayMount.id */
BeamDirection beam_direction;   /* 相对阵面局部坐标系，单位度 */
```

`tx_array_id` 保存的是指定发射阵面的 `ArrayMount.id`，不是 `arrays[]` 下标。指定值必须对应本节点一个启用阵面；配置加载时会检查。主循环枚举 TX 阵面时按该 ID 筛选，`channel()` 也会再次校验传入阵面 ID，保证直接调用接口时同样遵守节点选择。值为 `-1` 时发射端遍历全部启用阵面。接收端仍遍历全部启用阵面。当前示例为了简化测试，将节点 ID 与指定阵面 ID 设为相同的 1、2、3；真实部署时阵面 ID 可以独立编号。

### 2.12 `ChannelResult`：一次阵面到阵面的信道结果

```c
typedef struct {
    float t, d, tau, vr, fd, fspl;
    float gtx_db, grx_db;
    float element_tx_power_dbm, pr_dbm;
    int txid, rxid, txaid, rxaid; /* original IDs, retained for reporting */
    uint16_t index;               /* direct-cache key; array fields encode local slots */
    uint8_t valid;
    Complex h;
} ChannelResult;
```

主要字段：

| 字段 | 含义 | 单位/说明 |
|---|---|---|
| `t` | 计算时刻 | s |
| `d` | Tx/Rx 阵面参考点距离 | m |
| `tau` | 自由空间传播时延 | s |
| `vr` | 相对径向速度 | m/s |
| `fd` | 多普勒频移 | Hz |
| `fspl` | 自由空间路径损耗 | dB |
| `gtx_db` | 发射阵面方向增益项 | dB，按场幅度 `20log10()` |
| `grx_db` | 接收阵面方向增益项 | dB，按场幅度 `20log10()` |
| `element_tx_power_dbm` | 单个发射阵元输入功率 | dBm |
| `pr_dbm` | 接收功率估算 | dBm |
| `txid/rxid` | 发射/接收节点 ID | - |

| `txaid/rxaid` | 发射/接收阵面 ID | - |
| `valid` | 结果是否有效 | 1 有效，0 无效 |
| `h` | 复信道系数 | 当前模型定义见第 7 节 |

### 2.13 `TriNodesCalResult`：全局最新链路结果集合

```c
#define LINK_INDEX_SPACE       (1U << 12)
#define LINK_INDEX_BITMAP_SIZE (LINK_INDEX_SPACE / 8U)

typedef struct {
    ChannelResult links[LINK_INDEX_SPACE];
    uint8_t occupied[LINK_INDEX_BITMAP_SIZE];
    size_t count;
} TriNodesCalResult;

extern TriNodesCalResult g_tri_nodes_cal_result;
```

全局对象由工程提供，`count` 表示已登记的索引槽位数。`uint16_t index` 的位布局为 `[11:10]=txid`、`[9:8]=rxid`、`[7:4]=tx_array_slot`、`[3:0]=rx_array_slot`，因此索引空间为 `2^12=4096`。**阵面字段编码的是阵面在所属节点 `arrays[]` 中的槽位（0～15），不是 `ArrayMount.id`**：工程配置中的阵面 ID 11、21、31 等不能直接放入 4 bit，计算函数通过阵面指针相对 `Node.arrays` 的偏移取得槽位。节点 ID 必须在 0～3 范围内且在配置中唯一；工程当前限制最多 3 个节点。`TriNodesCalResult_Update()` 直接写入 `links[index]`，同一索引自然覆盖，不同索引互不影响，查找复杂度为 O(1)。`occupied` 位图区分未登记槽位和已登记但 `valid=0` 的结果；`TriNodesCalResult_Get()` 检查占用状态后返回对应记录。`TriNodesCalResult_Init()` 清空结果数组、位图和计数。索引空间固定为 4096 项；在最多 3 个节点、每节点最多 16 个阵面的约束下，实际最大有向链路数为 `3*2*16*16=1536`。该方案以较高的固定 RAM 占用换取 O(1) 查询，当前常见 32-bit `int`/`float` ABI 下 `sizeof(TriNodesCalResult)=279048` bytes（约 272.5 KiB）；移植到目标 MCU/DSP 时应使用目标编译器重新计算结构体大小。

该对象是全局共享数据。若计算和结果上传在不同任务/中断上下文并发执行，必须由调用方提供临界区保护或双缓冲。

调用方应先检查 `valid`，再使用其余物理结果。无效结果可能由节点/阵面未使能、收发状态不符、频率不一致、距离过近或方向增益无效等原因导致。

### 2.14 `SimConfig`：仿真配置总对象

```c
typedef struct {
    float start, end, step;
    Node nodes[MAX_NODES];
    size_t nn;
    NodePosition positions[MAX_NODES];
    size_t np;
} SimConfig;
```

它是示例程序的配置根对象，包含仿真时间范围、节点数组和节点运动数据数组。`nn` 与 `np` 分别为实际节点数和运动对象数，正常情况下二者相等。

当前实现使用固定容量数组：`MAX_NODES=3`，每节点 `MAX_ARRAYS_PER_NODE=16`。节点 ID 为 0～3 的 2 bit 编码值且必须唯一；实际节点数由 `nn`、`np` 决定。阵面索引编码使用 `arrays[]` 槽位，因此每个节点的阵面数不得超过 16。

---

## 3. 波束增益的计算与复用机制

本节是工程的核心复用设计。理解这一部分后，才能正确扩展节点数、阵面数和阵列参数组合数。

### 3.1 为什么可以复用波束表

波束方向图描述的是**阵列自身在局部坐标系中的方向响应**。在当前理想均匀矩形阵列模型下，它只由以下参数决定：

```text
(num_h, num_v, spacing_h_cm, spacing_v_cm, frequency_mhz)
```

方向图不由阵面安装在节点的什么位置、阵面朝向哪里、节点 ID 是多少、阵面发射功率是多少决定。

例如：

- Node1.Array1：16×8，水平/垂直间距均为 7.5 cm，频率 2000 MHz。
- Node2.Array1：同样是 16×8、7.5 cm、2000 MHz，但安装位置不同。
- Node3.Array2：同样的阵列参数，但安装姿态和发射功率不同。

三个阵面在各自局部坐标系中的方向图完全相同。因此只需要生成一张表，三个阵面都保存相同的 `beam_table_index`。

### 3.2 生成公式和数据含义

波长：

方向图配置中的阵元间距以 cm 保存，代入物理公式前乘以 `0.01f` 转换为 m；节点位置、阵面偏移和轨迹坐标仍以 m 为单位。

\[
\lambda=\frac{c}{f},\qquad f=frequency\_mhz\times10^6
\]

横向和纵向相邻阵元相位差：

\[
\psi_H=2\pi\frac{d_H}{\lambda}\sin\phi\cos\theta
\]

\[
\psi_V=2\pi\frac{d_V}{\lambda}\sin\theta
\]

对于均匀矩形阵列，阵列因子可分解为两个一维几何级数因子：

\[
A_H=\left|\frac{\sin(M\psi_H/2)}{\sin(\psi_H/2)}\right|,\quad
A_V=\left|\frac{\sin(N\psi_V/2)}{\sin(\psi_V/2)}\right|
\]

当分母接近 0 时，代码使用极限值 M 或 N，避免数值除零。最终保存：

\[
G(\theta,\phi)=\frac{A_HA_V}{\sqrt{MN}}
\]

该结果是线性场幅度。方向图按 1° 步进生成 `181×181` 表，查询时进行双线性插值。

代码采用闭式表达式，而不是在每个角度点上遍历所有阵元做复数求和。对于理想、均匀、矩形、同幅同相阵列，两者数学等价；闭式计算显著减少初始化时的运算量。

### 3.3 缓存查询和创建流程

核心接口：

```c
int16_t BeamTable_GetOrCreate(BeamTableCache *cache, const BeamTableConfig *config);
```

内部步骤：

1. 检查输入指针。
2. 从 `entries[0]` 到 `entries[count-1]` 逐项比较缓存键。
3. 如果找到相同配置，直接返回已有索引，不重新计算方向图。
4. 如果未找到，检查 `count` 是否达到 `MAX_BEAM_TABLES`。
5. 在 `entries[count]` 中生成方向图。
6. 生成成功后保存配置、增加 `count`，并返回新索引。
7. 参数非法或缓存满时返回 `-1`。

索引只在当前缓存生命周期内有效。调用 `BeamTable_Init()` 或 `BeamTable_InitAllNodes()` 清空缓存后，旧索引不再有效，所有阵面的 `beam_table_index` 必须重新建立。

### 3.4 所有节点和阵面统一初始化

推荐在所有节点/阵面配置加载完成后，统一调用：

```c
if (!BeamTable_InitAllNodes(&g_beam_cache, config.nodes, (uint16_t)config.nn)) {
    /* 缓存容量不足或某个阵列参数非法 */
}
```

该函数会先清空缓存，再遍历每个节点的 `array_count` 个阵面，逐个调用 `BeamTable_GetOrCreate()`，并把返回索引写入对应的 `ArrayMount.beam_table_index`。

**不要在每个节点初始化时分别清空缓存。** 例如先初始化 Node1，再初始化 Node2 时再次调用 `BeamTable_Init()`，会丢失 Node1 的缓存条目和索引关联。正确方式是：先完成全部配置，再对全部节点统一初始化一次。

### 3.5 缓存复用示例

假设四个阵面配置如下：

| 阵面 | 阵列配置 | 安装姿态/位置 | 预期结果 |
|---|---|---|---|
| A1 | 16×8，7.5/7.5 cm，2000 MHz | 位置 0，姿态 0 | 新建表，索引 0 |
| A2 | 16×8，7.5/7.5 cm，2000 MHz | 位置不同，姿态不同 | 命中索引 0 |
| B1 | 8×8，7.5/7.5 cm，2000 MHz | 任意 | 新建表，索引 1 |
| C1 | 16×8，7.5/7.5 cm，2100 MHz | 任意 | 新建表，索引 2 |

最终是 4 个阵面实例、3 张唯一方向图。阵面数量增加并不意味着方向图表数量同比增加；只有出现新的缓存键组合时才会增加表数量。

### 3.6 缓存复用的边界和注意事项

1. **缓存键必须覆盖所有影响方向图的物理参数。** 如果后续加入阵元幅度加权、相位加权、阵元位置误差、阵元方向图或极化模型，这些参数也可能影响方向图，必须纳入缓存键，或者建立新的方向图类型/缓存层。
2. **阵面安装姿态不能混入局部方向图缓存键。** 安装姿态用于把全局传播方向变换到阵面局部坐标；局部方向图本身可以复用。
3. **单阵元功率不能混入方向图缓存键。** 功率在链路预算阶段使用。
4. **缓存容量按唯一配置数计算，而不是阵面数计算。** 但最坏情况下每个阵面配置都不同，此时唯一表数可能等于阵面总数。
5. **缓存满时初始化失败。** 当前实现不会自动淘汰旧表，因为各阵面保存索引，淘汰会使其他对象的索引失效。需要动态运行时更新配置时，应采用引用计数/句柄机制，或在暂停仿真后统一重建缓存。
6. **配置修改后必须重建关联。** 不要直接改动某个阵面的 `beam_config` 后继续使用原 `beam_table_index`。
7. **缓存不是线程安全的。** 当前设计假设在初始化阶段单线程建立缓存，运行阶段只读。如果多个任务可能同时创建缓存条目，需要互斥保护；如果运行阶段只查表，则可以采用只读共享。

### 3.7 方向图查表接口

```c
float beam_gain(const BeamTableCache *cache, int16_t index, float theta_deg, float phi_deg);
```

输入角度单位为度，范围均为 `[-90, 90]`。超出范围时当前实现会钳位到边界，而不是报错。函数根据角度在 1° 网格中的位置，取相邻四个采样点进行双线性插值。

若缓存指针或索引无效，返回 0.0。调用方应避免把 0.0 直接送入 `log10()`；当前 `channel()` 会在转换 dB 前检查增益是否大于 0。

---

## 4. 节点数量和阵面数量的可变扩展

### 4.1 当前工程已经支持的“可变数量”

当前对象使用固定容量数组，但实际对象数量由计数器控制：

- 节点数量：`SimConfig.nn`，上限 `MAX_NODES`。
- 运动对象数量：`SimConfig.np`，上限 `MAX_NODES`。
- 每节点阵面数量：`Node.array_count`，上限 `MAX_ARRAYS_PER_NODE`。
- 唯一方向图数量：`BeamTableCache.count`，上限 `MAX_BEAM_TABLES`。

因此，运行时可以配置 1～3 个节点；每个节点也可以配置 0～16 个阵面，只要不超过编译期容量。代码不要求每个节点阵面数相同。

### 4.2 增加节点数量

当前 `MAX_NODES` 定义为 3，链路索引中的节点 ID 字段为 2 bit（0～3）。若系统需要更多节点，必须重新设计 `uint16_t index` 的位分配，不能只提高该宏。

1. 在 `sim.h` 中提高 `MAX_NODES`。
2. 检查所有依赖该宏的静态数组，包括 `SimConfig.nodes[]`、`SimConfig.positions[]`、`g_node_positions[]`。
3. 检查 `config_load()` 中节点数量边界检查。
4. 检查主循环和链路枚举是否适合更大规模。
5. 检查计算复杂度：当前节点对枚举为 O(N²)，阵面组合计算还会乘上 Tx/Rx 两侧阵面数。

节点规模较大时，建议将 `SimConfig` 的节点和位置数组改成动态分配，而不是无限提高静态数组宏。

### 4.3 增加每节点阵面数量

当前 `MAX_ARRAYS_PER_NODE` 为 16。由于链路索引为每个端点预留 4 bit 阵面槽位，不得超过 16；若需要更多阵面，必须重新设计索引位分配。随后重新编译。配置解析器会根据 `node_id` 找到对应节点，将每条阵面记录追加到该节点的 `arrays[]`，并递增 `array_count`。主流程通过 `array_count` 遍历，不需要为“12 个阵面”专门增加 12 组代码。

注意：提高该宏会增大每个 `Node` 对象的内存占用。由于 `Node` 数组按 `MAX_NODES` 静态分配，节点上限和阵面上限同时提高时，内存增长近似为二者乘积。

### 4.4 运行时动态阵面数与编译期上限的区别

当前工程属于“运行时实际数量可变、编译期最大容量固定”的设计。它适合资源受限的 DSP：内存需求可预测，不需要频繁 malloc/free。

如果后续要求阵面数完全不受宏限制，可以把 `Node` 改为指针加容量字段，例如：

```c
typedef struct {
    int id;
    int enabled;
    char name[MAX_NAME];
    NodeTRState tr_state;
    Vec3 velocity;
    Attitude attitude;
    ArrayMount *arrays;
    size_t array_count;
    size_t array_capacity;
} Node;
```

配置解析时按需 `malloc/realloc` 扩容，释放时逐节点释放 `arrays`。改为动态分配后必须同步修改：

- `config_load()`：为阵面数组分配/扩容，并处理内存分配失败。
- `config_free()`：释放每个节点的 `arrays`。
- `BeamTable_InitAllNodes()`：继续使用 `array_count` 遍历，不再依赖固定数组容量。
- 上位机配置更新流程：避免在其他任务仍持有阵面指针时移动/释放数组。

如果部署目标是 DSP，建议优先采用固定容量方案；只有确实需要运行期任意扩容且内存管理条件允许时，再改为动态分配。

### 4.5 阵面总数与波束缓存容量不是同一个概念

例如 3 个节点，每个节点 8 个阵面，总阵面数为 24；如果 24 个阵面只使用 3 种阵列参数组合，则只需要 3 张方向图表。反过来，若 9 个阵面各自使用不同参数，而 `MAX_BEAM_TABLES=8`，初始化就会失败。

所以容量规划时应分别估算：

- `MAX_NODES`：节点对象数量。
- `MAX_ARRAYS_PER_NODE`：每个节点可挂载的阵面数量。
- `MAX_BEAM_TABLES`：全系统不同阵列配置组合数量。

这三个上限相互独立，不要简单地把 `MAX_BEAM_TABLES` 设置成阵面总数；应根据可能出现的唯一配置组合数量设置。

---

## 5. 公共接口函数使用说明

### 5.1 配置与对象查找接口

#### `int config_load(SimConfig *c, const char *ini)`

用途：读取 INI、节点 CSV、阵面 CSV 和各节点轨迹文件，完成 `SimConfig` 的初始化。

典型调用：

```c
SimConfig *cfg = calloc(1, sizeof(*cfg));
if (!cfg) return -1;
if (!config_load(cfg, "config/simulation.ini")) {
    free(cfg);
    return -1;
}
```

返回 1 表示成功，0 表示失败。当前函数会清零传入对象，因此不要把已经加载且仍需保留的 `SimConfig` 再次传入。

注意事项：

- CSV 当前按逗号分隔并用 `sscanf()` 解析，不支持带逗号的引号字段。
- 节点轨迹路径相对 INI 所在目录解析。
- `nodes.csv` 中的 `tr_state` 支持 `TX/RX` 或 `1/0`。
- `nodes.csv` 还必须提供 `tx_array_id,beam_azimuth_deg,beam_elevation_deg`；`tx_array_id=-1` 表示自动选择，否则必须匹配本节点已启用阵面的 ID。
- 每个阵面记录的 `node_id` 必须对应已存在节点。
- 当前配置解析采用固定容量上限，超过上限会失败。
- `enabled` 字段会被加载；禁用节点/阵面在信道计算阶段被跳过。
- 真实上位机接入时，可以不调用 `config_load()`，而是直接填充结构体，再执行后续初始化。

#### `void config_free(SimConfig *c)`

用途：释放各节点轨迹数组，并清零 `SimConfig`。

典型调用：

```c
config_free(cfg);
free(cfg);
```

注意：释放后不能继续访问 `cfg->positions[i].trajectory`。当前 `g_node_positions[]` 是 `c->positions[]` 的浅拷贝，其中轨迹指针仍指向 `c` 所持有的内存；`config_free()` 后全局副本中的轨迹指针也会失效。当前主程序只在 `config_free()` 后退出，因此没有问题。若后续需要在释放 `SimConfig` 后继续使用全局位置对象，必须改成深拷贝或取消该全局浅拷贝。

#### `Node *find_node(SimConfig *c, int id)` / `NodePosition *find_pos(SimConfig *c, int id)`

按 ID 在线性数组中查找对象，找到返回内部对象指针，未找到返回 `NULL`。返回的是 `SimConfig` 内部地址，不需要调用方释放；只要 `SimConfig` 未释放或重新初始化，该指针才有效。

### 5.2 轨迹接口

#### `int load_trajectory(NodePosition *p, const char *path)`

读取轨迹文件并动态分配 `p->trajectory`。文件每行格式为：

```csv
time_s,x_m,y_m,z_m
```

要求至少两个有效采样点，且时间严格递增。成功返回 1，失败返回 0。

#### `int update_state(Node *n, NodePosition *p, float t)`

根据时刻 `t` 对轨迹线性插值，更新：

- `p->position`：节点全局位置。
- `n->velocity`：当前轨迹段速度。
- `n->attitude.heading/pitch`：根据速度方向计算。
- `n->attitude.roll`：当前固定为 0。

如果请求时刻早于首个轨迹点或晚于最后一个轨迹点，位置会被钳制到轨迹端点；速度仍采用首段或末段的线段速度。若节点静止（速度模长很小），函数不会重置姿态，而是保留此前姿态值。

### 5.3 三维数学接口

#### `Mat3 node_rot(Attitude a)`

返回节点姿态对应的旋转矩阵。

#### `Mat3 array_rot(Attitude node, Attitude mount)`

返回阵面安装坐标系相对世界坐标系的旋转矩阵，计算关系为：

```text
R_array = R_node * R_mount
```

#### `Mat3 compose_rot(Mat3 a, Mat3 b)`

矩阵乘法接口，返回 `a*b`。用于按坐标系层级组合旋转。当前波束最终姿态为：

```text
R_beam = R_node * R_mount * R_scan
```

其中 `R_scan = Rz(azimuth) * Ry(-elevation)`，使阵面局部 +X 波束轴指向 `(cos(el)cos(az), cos(el)sin(az), sin(el))`。传播方向转换到波束局部坐标时使用 `R_beam` 的转置。`array_gain_amplitude()` 中已按此顺序执行。

#### `Vec3 apply(Mat3 m, Vec3 v)`

将矩阵作用于向量。`channel.c` 中使用旋转矩阵转置，将全局传播方向变换到阵面局部坐标。

### 5.4 波束缓存接口

#### `void BeamTable_Init(BeamTableCache *cache)`

将缓存计数清零。它不逐项清除大数组内容，因为 `count=0` 后旧条目已不再被视为有效。初始化缓存前应确保没有对象继续使用旧索引。

#### `bool BeamTable_ConfigEqual(const BeamTableConfig *a, const BeamTableConfig *b)`

比较两组方向图参数是否属于同一缓存键。输入为空指针返回 false。

#### `bool BeamTable_Generate(const BeamTableConfig *config, float gain[NTH][NPH])`

根据配置生成一张 `181×181` 二维方向图。调用时直接传入二维数组，例如 `BeamTable_Generate(&cfg, entry.gain)`。函数参数在 C 语言中会调整为“指向含 `NPH` 个 `float` 的数组的指针”，因此编译器知道每行的长度，可以直接使用 `gain[theta_idx][phi_idx]`。该函数只负责计算表，不负责缓存管理。

#### `int16_t BeamTable_GetOrCreate(BeamTableCache *cache, const BeamTableConfig *config)`

先查缓存，命中返回旧索引；未命中则生成并添加新表。失败返回 -1。该接口适合增量注册单个阵面，但若要初始化全系统，推荐使用 `BeamTable_InitAllNodes()`。

#### `const float (*BeamTable_Get(const BeamTableCache *cache, int16_t index))[NPH]`

根据索引返回二维表。返回值是“指向一行（包含 `NPH` 个 float）的指针”，因此可以这样访问：

```c
const float (*table)[NPH] = BeamTable_Get(&g_beam_cache, array->beam_table_index);
if (table != NULL) {
    float g = table[theta_idx][phi_idx];
}
```

返回指针指向缓存内部存储，不得释放，也不要在缓存重建后继续使用。

#### `bool BeamTable_InitAllNodes(BeamTableCache *cache, Node *nodes, uint16_t node_count)`

清空缓存并遍历所有节点的所有阵面，完成方向图生成/复用，同时写入每个阵面的 `beam_table_index`。它是当前工程推荐的统一初始化入口。

#### `float beam_gain(const BeamTableCache *cache, int16_t index, float theta_deg, float phi_deg)`

根据缓存索引和局部角度查询方向图线性场幅度。内部使用双线性插值，分别读取 `table[t0][p0]`、`table[t0][p1]`、`table[t1][p0]`、`table[t1][p1]` 四个相邻角度点。角度单位是度，超范围钳位，缓存或索引无效返回 0。

### 5.5 信道接口

#### `ChannelResult channel(...)`

```c
ChannelResult channel(const SimConfig *c,
                      const Node *tx, const NodePosition *tp,
                      const Node *rx, const NodePosition *rp,
                      const ArrayMount *ta, const ArrayMount *ra,
                      float t);
```

输入为一条有向链路的 Tx 节点、Rx 节点、各自位置对象、Tx/Rx 阵面和计算时刻。函数返回 `ChannelResult`，不直接打印。Tx 节点设置 `tx_array_id>=0` 时，传入的发射阵面 ID 必须与其一致；Tx/Rx 的 `beam_direction` 都会作为各自阵面的局部扫描偏移应用。

调用前应确保：

1. `update_state()` 已对 Tx/Rx 节点在时刻 `t` 更新完成。
2. Tx/Rx 阵面已完成 `beam_table_index` 关联。
3. `g_beam_cache` 中的表仍然有效。
4. 节点状态分别为 TX 和 RX。
5. 两阵面频率一致（当前容差 `1e-6 MHz`）。

无效情况返回 `valid=0`。函数当前使用全局 `g_beam_cache`，因此参数中没有显式传入缓存指针。若未来需要同时运行多个仿真实例，建议将缓存放进仿真实例上下文，并把缓存指针作为参数传入，避免多个实例争用全局状态。

#### `void TriNodesCalResult_Init(TriNodesCalResult *result)` / `int TriNodesCalResult_Update(TriNodesCalResult *result, const ChannelResult *channel_result)`

初始化结果集合，或将单条链路的最新结果插入/覆盖到集合中。工程主程序将每次 `channel()` 返回值写入全局 `g_tri_nodes_cal_result`。如果硬件平台需要由通信任务读取该对象，请在复制/读取过程中保护共享访问。

#### `void print_channel(ChannelResult r)`

控制台格式化输出接口。算法层不依赖它；嵌入式移植时可替换为串口日志、消息队列或二进制结果上报。

---

## 6. 配置文件与上位机数据映射

### 6.1 `simulation.ini`

```ini
start_time_s=0
end_time_s=5
step_s=0.5
nodes_csv=nodes.csv
arrays_csv=arrays.csv
```

时间单位均为秒。CSV 路径相对 INI 文件所在目录。

### 6.2 `nodes.csv`

```csv
id,name,enabled,tr_state,trajectory_file,tx_array_id,beam_azimuth_deg,beam_elevation_deg
1,Node1,1,TX,trajectories/node1.csv,1,0,0
2,Node2,1,RX,trajectories/node2.csv,2,0,0
3,Node3,1,RX,trajectories/node3.csv,3,0,0
```

每行对应一个节点。`tr_state` 是节点级半双工状态；轨迹文件路径对应 `NodePosition`。`tx_array_id` 指定发射阵面 ID，`-1` 为自动选择；扫描方位角/俯仰角定义在阵面局部坐标系，单位为度。

### 6.3 `arrays.csv`

```csv
array_id,node_id,name,enabled,x,y,z,yaw,pitch,roll,num_h,num_v,spacing_h_cm,spacing_v_cm,frequency_mhz,element_tx_power_dbm
1,1,A1,1,0,0,0,0,0,0,16,8,7.5,7.5,2000,20
2,2,A2,1,0,0,0,0,0,0,16,8,7.5,7.5,2000,20
3,3,A3,1,0,0,0,0,0,0,8,8,7.5,7.5,2000,18
```

每条记录通过 `node_id` 归属到一个节点。`x/y/z` 是阵面相对节点参考点的偏移，`yaw/pitch/roll` 是安装姿态。阵面参数映射到 `ArrayMount`：阵列参数写入 `beam_config`，单阵元功率写入 `element_tx_power_dbm`。

在真实上位机协议中，字段名称可以不同，但建议保留相同的语义边界：节点 ID 和 TR 状态属于节点配置；阵面 ID、安装位姿、阵列参数和单阵元功率属于阵面配置。

---

## 7. 信道模型、坐标变换与结果解释

### 7.1 坐标约定

当前全局坐标系 Z 轴向上。节点局部 +X 轴沿节点运动方向。阵面局部坐标约定：

- +X：阵面法向/正前方。
- +Y：水平阵列轴。
- +Z：垂直阵列轴。

阵面全局位置：

\[
P_{array}=P_{node}+R_{node}P_{offset}
\]

阵面旋转矩阵：

\[
R_{array}=R_{node}R_{mount}
\]

全局传播方向通过 `R_array` 的转置变换到阵面局部坐标，再计算 `theta` 和 `phi`。如果硬件波束表使用的角度零点、方位正方向或阵面法向定义不同，应统一修改 `array_gain_amplitude()` 中的坐标映射，不能只修改 CSV 角度值。

### 7.2 距离、时延、多普勒和路径损耗

\[
d=\|P_{rx,array}-P_{tx,array}\|
\]

\[
\tau=\frac{d}{c}
\]

相对径向速度：

\[
v_r=(v_{rx}-v_{tx})\cdot\hat{u}
\]

多普勒：

\[
f_d=-\frac{v_rf_c}{c}
\]

自由空间路径损耗：

\[
FSPL=20\log_{10}\left(\frac{4\pi d f_c}{c}\right)
\]

### 7.3 接收功率估算

当前模型采用单阵元发射功率作为输入：

\[
P_r=P_{element}+10\log_{10}(M_{tx}N_{tx})+G_{tx}+G_{rx}-FSPL
\]

其中 `Gtx` 和 `Grx` 是方向图表线性场幅度转换得到的 dB 值：`20log10(amplitude)`。该功率模型假设发射阵元同频同相、均匀馈电，未考虑馈线损耗、阵元方向图、互耦、极化失配和硬件幅相误差。

### 7.4 复信道系数

当前代码计算：

\[
|h|=\sqrt{M_{tx}N_{tx}}\,A_{tx}A_{rx}\frac{\lambda}{4\pi d}
\]

\[
\angle h=-2\pi f_c\tau
\]

并写入 `h.re` 和 `h.im`。这是以单个 Tx 阵元输入信号为参考的简化复信道系数，包含理想相干发射阵列合成、接收阵面方向响应和自由空间传播项。使用时必须与链路预算中的功率参考保持一致；不要把 `h` 的幅度再次乘上发射阵元数而造成重复计入。

---

## 8. 扩展到 DSP/嵌入式系统的建议

### 8.1 用上位机解析结果替代 CSV

建议将工程分为三层：

1. **协议层**：接收 UDP/TCP 数据，完成帧校验、字段解析。
2. **配置层**：把解析字段填入 `SimConfig`、`Node`、`ArrayMount` 等对象，并做范围校验。
3. **算法层**：调用 `BeamTable_InitAllNodes()`，再调用轨迹更新和 `channel()`。

这样算法层不需要知道数据来自 CSV、UDP 还是共享内存。

### 8.2 缓存内存放置

默认缓存约 1 MiB。DSP RAM 不足时：

- 降低 `MAX_BEAM_TABLES`；
- 将 `g_beam_cache` 放到外部 DDR 或链接脚本指定的存储段；
- 根据芯片内存布局检查 DMA/Cache 一致性要求；
- 不要把大缓存放到线程栈或任务栈中，应使用静态存储或专用内存池。

### 8.3 初始化时机

方向图生成包含大量三角函数计算，应尽量放在配置加载/任务启动阶段，而不是每个 100 ms 或 1 s 信道更新周期内重复执行。动态运行阶段只进行索引查找和插值。

如果设备运行中允许更换阵列参数，建议采用“构建新缓存—验证成功—切换配置句柄”的方式，避免计算过程中修改正在使用的缓存。

### 8.4 浮点和数学库

本工程所有参与算法计算的实数数据、结构体字段、函数参数和返回值统一采用 C `float` 单精度类型，不使用 `double`。数学运算统一调用单精度接口，例如 `sinf`、`cosf`、`sqrtf`、`atan2f`、`hypotf`、`log10f`、`fabsf` 和 `floorf`。配置文件中的浮点字段使用 `strtof()` 和 `sscanf(..., "%f", ...)` 解析。

`printf()` 输出时，`float` 会按 C 语言默认参数提升规则转换为 `double`，因此格式化输出仍使用 `%f`/`%e`；这只是可变参数接口的语言规则，不代表算法内部使用了双精度。

单精度的有效位数和动态范围低于双精度。缓存键比较容差也已按 `float` 调整：间距 `1e-6 cm`，频率 `1e-6 MHz`（约 1 Hz）。如果上位机传入的参数精度高于单精度可表达范围，进入算法前应统一量化/归一化，不能依赖 `float` 保留额外小数位。移植到 DSP 时，应确认数学库提供上述单精度函数，并使用目标编译器验证数值误差和运算性能。

### 8.5 多任务并发

建议将缓存构建和信道计算分离：

- 初始化任务独占写缓存。
- 信道计算任务只读缓存。
- 配置更新时暂停/同步相关计算任务，完成新缓存构建后再切换。

若多个任务会同时修改节点状态、位置或阵面配置，应使用双缓冲、互斥锁或消息队列传递完整快照，避免 `channel()` 读取到一半更新的数据。

### 8.6 多仿真实例

当前 `g_beam_cache` 和 `g_node_positions` 是全局对象，适合单实例程序。若一个进程中要并行运行多个独立场景，建议新增 `SimContext`，将缓存、节点、位置和计数器封装到上下文中，并将接口改为显式接收上下文指针。否则不同场景会共享缓存状态，难以隔离配置生命周期。

---

## 9. 编译、运行和验证

```sh
make clean
make
./channel_sim config/simulation.ini
./channel_sim config/simulation.ini --time 2.5
make clean
```

修改缓存容量示例：

```sh
make clean
make CFLAGS='-std=c11 -O2 -Wall -Wextra -Wpedantic -DMAX_BEAM_TABLES=4'
```

建议至少进行以下验证：

1. 两个阵面使用完全相同的 `BeamTableConfig`，确认得到相同 `beam_table_index`。
2. 只改变安装姿态或偏移量，确认仍然复用同一张表，但链路方向增益可能变化。
3. 改变阵元数、间距或频率中的任意一项，确认产生新的缓存索引。
4. 让唯一配置数超过 `MAX_BEAM_TABLES`，确认初始化明确失败，而不是访问越界。
5. 增加每节点阵面记录数，确认 `array_count` 正确递增，并检查是否超过 `MAX_ARRAYS_PER_NODE`。
6. 将节点状态从 TX/RX 互换，确认有向链路方向随之改变。
7. 将 Tx/Rx 阵面频率设置为不同值，确认对应链路无效。
8. 对比方向图公式的直接复数求和实现和闭式实现，验证指定阵列配置下的误差满足预期。

---

## 10. 当前版本的限制与后续改进清单

1. 节点数、每节点阵面数和缓存表数目前均采用固定容量数组；实际数量可变，但有编译期上限。
2. 当前缓存使用线性遍历查找，唯一方向图数量较少时足够；若配置组合达到数百/数千，可改用哈希表或排序索引。
3. 缓存初始化不支持运行中安全淘汰；运行时更新配置应引入句柄、引用计数或双缓冲机制。
4. `g_beam_cache` 是全局对象，当前适用于单仿真实例。
5. `g_node_positions` 对轨迹指针采用浅拷贝；`config_free()` 后不得继续使用其中的轨迹指针。
6. 轨迹使用线性插值，姿态由速度方向推导且 roll 固定为零；如需真实平台姿态，应接入姿态传感器/姿态轨迹。
7. 方向图模型是理想均匀矩形阵列模型，尚未包含阵元方向图、互耦、极化、幅相误差和扫描相位控制。
8. 当前方向图角度映射依赖本工程的局部坐标约定，必须与硬件波束表坐标定义核对。
9. 信道仅考虑单径自由空间 LOS，不包含反射、多径、遮挡、阴影衰落和噪声。
10. 当前配置解析器是轻量级示例，不是完整 CSV/INI 标准解析器；正式产品中建议由上位机协议解析模块提供经过校验的数据结构。

