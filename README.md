# DigitalTwins：多节点、多阵面动态信道仿真

本工程使用 C11 实现节点运动、姿态、阵面波束增益与自由空间 LOS 信道仿真。节点轨迹采用 WGS-84 GEO 输入，位置/速度和信道几何统一在 ECEF 计算。

## 工程文档
- [模块架构](docs/module_architecture.html)
- [完整工程说明与配置指南](docs/engineering_guide.md)
- [ECEF、NED、姿态与信道坐标说明](docs/ecef_coordinate_system.md)
- [计算流程说明](docs/channel_calculation_and_attitude_guide.html)

## 编译和运行
```sh
make clean && make
make test
./channel_sim config/simulation.ini
./channel_sim config/simulation.ini --time 2.5
```

## 轨迹输入
CSV 列：`time_s,latitude_deg,longitude_deg,altitude_m,yaw_deg,pitch_deg,roll_deg`。纬度北正、经度东正；高度为 WGS-84 椭球高；姿态为 NED 参考角，单位 deg。阵面偏移是 BODY 局部 m，阵面安装角是 BODY 局部 deg。详见工程说明。

## 模型边界
当前为理想单径 LOS 自由空间模型，不包含多径、遮挡、阴影衰落、极化、互耦或阵元方向图。
