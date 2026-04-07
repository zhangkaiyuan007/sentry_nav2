---

## 各层实现方法

**地图处理**
将已有的pgm和yaml文件直接喂给 Nav2 的 map_server。

**定位**
直接利用super lio的relocation.py进行准确定位

**Costmap**
直接用nav2的costmap 2d

**Smac Planner**
Nav2 内置，开箱即用

**MPPI Controller**
关键参数：`vx_max: 2.0`，`iteration_count` 调到 2000 以上（工控机扛得住），`lambda` 控制保守程度，动态场景建议 `0.3~0.5`。对向场景靠 local costmap 的膨胀自然规避。

## 车子
- 差速构型
- 速度平均2m/s
- footprint 0.6x0.6
- 搭载mid360