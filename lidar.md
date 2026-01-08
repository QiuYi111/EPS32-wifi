Unitree LiDAR 点云数据处理技术文档
本文档详细描述了 
script.js
 中从接收原始 WebSocket 二进制流到在浏览器中渲染 3D 点云的完整计算流程。

1. 数据流与协议层 (Data Stream & Protocol)
1.1 传输层
数据通过 WebSocket 以二进制格式 (arraybuffer) 接收。

服务端: ESP32 发送封装后的数据包。
数据封装: 每个包包含一个自定义头部和 MAVLink 负载。
1.2 报文结构
字段	偏置 (Byte)	长度 (Byte)	说明
Magic	0	4	校验位 0xDEADBEEF
Timestamp	4	4	传感器时间戳
Type	8	1	负载类型 (1 为 LIDAR_RAW)
Length	9	2	负载长度
Payload	11	Variable	MAVLink 原始帧
2. MAVLink 解析与数据同步 (Parser & Sync)
LiDAR 的原始数据被拆分为两种类型的 MAVLink 消息，必须成对匹配才能还原三维坐标。

2.1 距离包 (Distance Packet - MSG ID 16)
包含 120 个探测点的原始距离读数（uint16, Little Endian）。

存储: 缓存到 this.distCache，以 packetId 为键。
2.2 辅助包 (Auxiliary Packet - MSG ID 17)
提供计算坐标所需的元数据和校准参数。

核心字段:
packet_id: 同步 ID。
sysVerticalAngleStart/Span: 垂直扫描角度。
comHorizontalAngleStart/Step: 水平扫描角度。
thetaAngle/ksiAngle: 核心校准偏角。
reflectData: 120 个点的反射强度。
2.3 缓冲区匹配
由于网络延迟，距离包和辅助包到达顺序可能颠倒。
MAVLinkParser
 使用两个 Map 对象进行缓存，当同一个 packetId 的两个包都到齐时，触发 
_tryCombine
 进行数学计算。

3. 数学运算模型 (Mathematical Model)
这是处理流程中最复杂的部分，旨在将球坐标（角度和距离）叠加机械偏差后，转换到直角坐标系。

3.1 单位归一化
距离 (Range): $L = \text{range} \times 0.001$ (单位：米)。
角度 (Radian): 所有角度 $\alpha$ (垂直) 和 $\beta$ (水平) 均从角度制转换为弧度制。
3.2 机械参数 (Mechanical Bias)
bAxisDist: 激光器离轴距离。
zBias: 垂直中心偏移量 (44.5mm)。
Theta/Ksi: 内部光学镜片组的初始修正角。
3.3 核心变换公式
对于每一个点 $j \in [0, 119]$：

计算当前点的插值角度:

$\alpha_j = \text{VerticalStart} + j \times \text{VerticalStep}$
$\beta_j = \text{HorizontalStart} + j \times \text{HorizontalStep}$
计算中间分量 A 和 B: $$A = (-\cos\theta \cdot \sin\xi + \sin\theta \cdot \sin\alpha \cdot \cos\xi) \cdot L + \frac{\text{bAxisDist}}{1000}$$ $$B = \cos\alpha \cdot \cos\xi \cdot L$$

投影到传感器坐标系 (Local Frame):

$x = \cos\beta \cdot A - \sin\beta \cdot B$
$y = \sin\beta \cdot A + \cos\beta \cdot B$
$z = (\sin\theta \cdot \sin\xi + \cos\theta \cdot \sin\alpha \cdot \cos\xi) \cdot L + \text{zBias}$
4. 可视化映射 (Visualization Mapping)
4.1 坐标轴转换 (Frame Alignment)
LiDAR 坐标系通常是 Z 轴向上，而 WebGL (Three.js) 标准坐标系通常是 Y 轴向上。因此在传输到显存前进行了置换：

Three_X = x
Three_Y = z (将传感器的垂直轴映射为世界坐标的高度)
Three_Z = y (将传感器的平面轴映射为世界坐标的深度)
4.2 性能优化
预分配 (Pre-allocation): 使用 Float32Array 初始化顶点缓冲区，避免内存频繁分配。
动态渲染: 通过 setDrawRange 只渲染当前有效的点，提高帧率。
伪彩色映射:
$Color.R = Intensity \times 0.5$
$Color.G = Intensity \times 0.8$
$Color.B = 0.5 + Intensity \times 0.5$ (基础蓝色调，越高越亮)
5. 异常处理
无效点过滤: 当 range == 0 时，忽略该点（不入队）。
缓存清理: 当缓存超过阈值时，自动删除最旧的记录，防止内存溢出。