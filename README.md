# NAO 跑道视觉闭环行走控制器

这是 Team TJArk NAO 跑道演示项目的公开、脱敏重构版，由项目负责人陈炫彰整理。仓库将原先集成在 B-Human 中的跑道感知与纠偏逻辑提取为独立 Python 模块，使算法可以通过合成线段和确定性仿真直接审查、运行和测试。

## 赛事与指导背景

本项目为 2026 年[中国机器人及人工智能大赛（CRAIC）](https://www.caairobot.com/)创新赛项目，由陈炫彰担任主要负责人；指导教师包括[陈启军教授](https://see.tongji.edu.cn/info/1376/10292.htm)等。陈启军是同济大学机器人与人工智能实验室主导师、教授、博士生导师，曾任电子与信息工程学院院长和控制科学与工程系主任，并担任[中国自动化学会理事](https://www.caa.org.cn/article/6/12.html)等职务。赛事奖项尚未公布，本仓库不作获奖声明。

## 准确定位

- `RUNNING` 是演示状态名，底层调用的是相对速度行走接口，不代表独立的跑步步态。
- 控制器使用几何规则、低通滤波和有界比例反馈，不是强化学习。
- 原项目的步态引擎来自 B-Human Code Release 2024；本仓库不复制或冒充该上游实现。
- 当前 `forward_speed_ratio = 0.80` 是无量纲的相对速度请求，不是 `0.8 m/s`。

## 自研控制链路

```text
胸口按钮上升沿
   -> STAND / RUNNING 状态切换

地面白线检测结果
   -> 长度、方向、前向距离筛选
   -> 左右边界最优候选
   -> 双边界虚拟中心线 / 单边界宽度推算
   -> 航向与横向误差低通
   -> 有界比例纠偏
   -> 相对前进、横移、旋转命令

视觉短时丢失 <= 1 s -> 使用最近可靠结果
视觉持续丢失 > 1 s  -> 回退到里程计航向保持
```

默认参数对应原演示使用的 1,000 mm 跑道宽度、1,100 mm 前视距离、3° 视觉死区和 4° 里程计死区。所有物理量的单位都在数据类型和配置名称中显式标注。

## 快速运行

无需机器人框架或第三方依赖：

```bash
PYTHONPATH=src python3 -m runway_controller --steps 160
PYTHONPATH=src python3 -m unittest discover -s tests -v
```

也可以安装为本地命令：

```bash
python3 -m pip install -e .
nao-runway-demo --steps 160
```

演示会生成两条合成跑道边界，在误差空间中检查控制方向、限幅、视觉短时丢失和收敛行为。输出结果仅用于算法回归，不是实机速度、成功率或比赛成绩。

## 核心接口

```python
from runway_controller import LineSegment, RunwayController

controller = RunwayController()
result = controller.update(
    lines=[
        LineSegment(300, 650, 2300, 650),
        LineSegment(300, -350, 2300, -350),
    ],
    odometry_heading_rad=0.0,
    timestamp_ms=0,
)

print(result.source)
print(result.command)
```

`LineSegment` 使用机器人地面坐标系，长度单位为 mm；控制输出是 `[-1, 1]` 语义的相对请求。接入其他机器人时，需要由适配层完成传感器坐标、速度语义和安全约束映射。

## 与真实项目的关系

真实版本集成于 Team TJArk 的 B-Human 2024 工程中：

- 读取 B-Human `FieldLines` 感知结果；
- 通过胸口按钮切换站立/行走状态并联动胸灯；
- 将纠偏量交给 B-Human 的相对速度行走接口；
- 在视觉不可用时使用里程计保持初始方向。

公开版只重构上述团队新增逻辑，不包含 B-Human/TJArk 完整比赛代码、二进制、机器人配置、网络地址、密钥或构建产物。上游项目及其许可证见 [B-Human Code Release](https://github.com/bhuman/BHumanCodeRelease/tree/coderelease2024)。

## 测试覆盖

自动化测试验证：

- 双边界宽度与虚拟中心线重建；
- 单边界缺失时的跑道宽度推算；
- 1 秒视觉记忆与里程计回退；
- 短线、横线等无效候选过滤；
- 输出限幅和状态按钮去抖；
- 含短时丢线的合成闭环误差收敛。

## License

本公开重构采用 Apache License 2.0。B-Human、NAO 软件栈及其他第三方组件遵循各自许可证，不属于本仓库授权范围。
