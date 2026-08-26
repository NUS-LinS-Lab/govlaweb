# HANDOFF — 在 HTML 里复刻 viser 3D 场景的样子

> 目标：网页里加载导出的 PLY，渲染出和我们内部 viser viewer（`/tmp/viz_realgoal.py`，端口 8090）**一模一样**的画面。
> PLY 里的 3D 坐标已经是最终坐标（正确的相机内参、正确的 4:3 比例、正确的深度极性都已烘焙进点的 xyz 里），所以你**不需要**再碰任何相机内参 / intrinsics。你只要正确渲染点云 + 摆对相机 + 用对参数即可。

---

## 1. 参考实现（源文件路径）

| 文件 | 作用 |
|---|---|
| `/tmp/viz_realgoal.py` | viser 参考实现（你要复刻的就是它跑出来的样子）。所有参数以这个文件为准。 |
| `/tmp/export_ply.py` | 把 viser 的 3 个默认图层导出成 PLY 的脚本（反投影逻辑与 viz 完全一致）。 |

如果任何参数有歧义，**以 `/tmp/viz_realgoal.py` 的实际代码为准**。

---

## 2. 数据文件（已导出，就位）

路径：`goalvlaweb/static/data/real_world/<scene>/`
场景文件夹：`tomato/`、`sweep/`、`duck/`、`bottle/`
（注意：web 的 `bottle/` = 内部 pipeline 里的 `bottle2`，即"扶正瓶子"任务。）

每个场景 3 个 PLY（= viser 的 3 个默认图层）：

| 文件 | 图层 | 类型 | 渲染方式 |
|---|---|---|---|
| `<scene>_init_da.ply` | Init (Depth-Anything) 初始场景 | 点云，160000 点，含顶点色 | `THREE.Points` |
| `<scene>_goal.ply` | Goal 目标场景 | 点云，160000 点，含顶点色 | `THREE.Points` |
| `<scene>_matching.ply` | Feature matching 特征匹配连线 | 顶点 + edge 元素 | `THREE.LineSegments` |

格式：全部是 **binary_little_endian PLY**，坐标单位 **米**，顶点色 `red/green/blue`（uchar 0–255）。
three.js 的 `PLYLoader` 可直接加载这三种（点云和带 edge 的线段都支持）。

`_matching.ply` 结构：顶点按 pair 排列（v0-v1 是第一条线，v2-v3 第二条…），并带 `element edge`（`vertex1`/`vertex2`）。`PLYLoader` 读出后是带 index 的 geometry，用 `THREE.LineSegments` 渲染即可，每条线两端颜色取自 init 图对应像素的真实 RGB。

---

## 3. ⚠️ 比例（最重要，之前反复强调过）

**画面必须是 4:3，不是 16:9。** 点云本身的 xyz 已经是正确的 4:3 几何（内参在导出时已修正过——原始 viser 的 `get_matrix` 有个 bug 会把场景拉成 1.729 的宽高比，我们用修正后的针孔模型重新反投影，得到真实的 1.333=4:3）。

对你（HTML 端）意味着：
- **不要**给点云做任何各向异性缩放 / 拉伸。原样加载 xyz。
- **canvas / 渲染视口用 4:3**（例如 800×600、1000×750、1200×900）。相机 `aspect = width/height` 必须跟 canvas 实际宽高比一致，否则又会看起来被拉伸。
- 换句话说：几何是对的，你只要别在渲染层再引入非 1.333 的宽高比就行。

---

## 4. 相机 / 坐标系（复刻 viser 的视角）

viser 里的设置（`/tmp/viz_realgoal.py` 末尾）：

```python
server.scene.set_up_direction("+z")          # 场景 up 轴 = +z
# 客户端连接时的初始相机：
client.camera.position = (0.0, 0.0, 0.0)     # 相机在原点
client.camera.look_at  = (0.0, 0.0, 1.0)     # 看向 +z（深度方向）
client.camera.up       = (1.0, 0.0, 0.0)     # 相机 up = +x
```

坐标系约定（PLY 里就是这套）：
- **+z = 相机深度方向（朝前）**，值越大越远。场景背景在 z 大的一端，最近处 z≈0.73m，最远≈1.6m。
- x 向左右展开（范围约 -0.94 ~ +0.70m），y 上下。
- 相机放原点朝 +z 看，`up` 用 +x。

three.js 里等价做法：
```js
camera = new THREE.PerspectiveCamera(fov, 4/3, 0.01, 100);
camera.position.set(0, 0, 0);
camera.up.set(1, 0, 0);
camera.lookAt(0, 0, 1);
// OrbitControls.target.set(0, 0, 1) 让轨道中心落在场景上
```
FOV 取一个中等值（viser 默认约 45–50° 垂直 FOV），配合 4:3 aspect。相机在原点、看向 +z，用户可以自由 orbit。

---

## 5. 点 / 线的大小和密度（viser 保存的配置）

来自 `/tmp/viz_realgoal.py` 的 `SZ` 和密度默认值：

| 项 | viser 值 | 说明 |
|---|---|---|
| 场景点云 point size | **0.003**（世界单位，米） | `init_da` 和 `goal` 都用这个 |
| 匹配线 line width | **1.0** | |
| 场景点密度 | **100%** | 全部 160000 点都显示 |
| 匹配线密度 | **50%** | 只显示一半的匹配线（随机抽样，见下） |

three.js 端：
- `THREE.Points` 用 `PointsMaterial({ size: 0.003, sizeAttenuation: true, vertexColors: true })`。`sizeAttenuation: true` 让点随距离缩放（和 viser 的世界单位一致）。如果点看起来太小/太大，微调 size，但先从 0.003 起。
- 线：`LineBasicMaterial({ vertexColors: true, linewidth: 1 })`（注意 WebGL 大多忽略 linewidth>1，1.0 没问题）。
- **匹配线密度 50%**：viser 里是随机抽 50% 的线显示。你可以在加载 `_matching.ply` 后随机保留一半 edge，或简单起见全画（244 条以内，量不大，全画也可以——但要 100% 复刻就抽 50%）。

背景：viser 默认深色背景（近黑）。用深色 canvas 背景最接近（例如 `#0a0a0a` 或纯黑）。

---

## 6. 默认可见图层

viser 的 `DEF_ON` —— 默认**三层全开**：
- ✅ Init (Depth-Anything) 场景点云 (`_init_da.ply`)
- ✅ Goal 场景点云 (`_goal.ply`)
- ✅ Feature matching 连线 (`_matching.ply`)

（viser 里还有 predicted-goal / source-object 高亮等图层，但默认关闭、也没导出。只需这三层。）

如果做图层开关 UI，就这三个 checkbox，默认全勾。

---

## 7. 每个场景的规模（心里有数）

| 场景 | init_da 点 | goal 点 | 匹配线 |
|---|---|---|---|
| tomato | 160000 | 160000 | 153 |
| sweep  | 160000 | 160000 | 177 |
| duck   | 160000 | 160000 | 244 |
| bottle | 160000 | 160000 | 91 |

单个点云 PLY 约 2.4 MB。4 场景 × 2 点云 ≈ 19 MB，按需懒加载即可。

---

## 8. 验收标准（怎么算复刻对了）

1. 画面是 **4:3**，点云没有被横向拉伸（不是 16:9 的"变宽"）。
2. 背景（深度大的点）在**远端**，前景物体在近端——深度前后关系正确（duck 曾经反过，已修，PLY 是对的）。
3. 初始相机在原点朝 +z 看，能看到和 viser 一样的正视角；可自由 orbit。
4. 三层默认全显示，颜色来自真实 RGB。
5. 点大小 0.003、线宽 1、匹配线约一半。

有疑问直接对照跑起来的 viser（`http://localhost:8090`）逐项比对。
