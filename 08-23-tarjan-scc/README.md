# 08-23 Strongly Connected Components (Tarjan's Algorithm)

## 主题
有向图的强连通分量（SCC）—— Tarjan 算法（一次 DFS，O(V+E)），并与 Kosaraju 算法、Floyd-Warshall 可达性基准交叉验证。

## 核心技术
- **Tarjan 算法**：`dfn`（DFS 序）+ `low`（可达最低祖先）+ 栈维护，`low[u]==dfn[u]` 时弹栈得到一个 SCC。
- **Kosaraju 算法**：两次 DFS（原图 + 反图），作为独立参照实现。
- **Floyd-Warshall 可达性闭包**：O(V³) 精确基准，用于严格验证 SCC 定义（`u,v` 同 SCC ⟺ 互相可达）。
- **缩点 DAG**：将每个 SCC 压缩为超节点，验证缩点图无环。

## 量化验证（7/7 通过）
| 测试 | 内容 | 结果 |
|------|------|------|
| Test1 | 随机图上 Tarjan 与 Kosaraju 的 SCC 划分完全一致 | ✅ |
| Test2 | SCC 划分 == 相互可达关系（Floyd-Warshall 基准） | ✅ |
| Test3 | 缩点后 DAG 无环 | ✅ |
| Test4 | 孤立节点独立成一个 SCC | ✅ |
| Test5 | 100 节点单环 → 全部合并为 1 个 SCC | ✅ |
| Test6 | 80 节点 DAG → 每个节点自成一个 SCC | ✅ |
| 图像 | 640x400 彩色 SCC 划分图，均值 157 / 标准差 82.6 | ✅ |

## 文件
- `main.cpp` — 算法 + 验证 + PPM 可视化
- `output` — 编译产物
- `tarjan_scc_output.ppm` — SCC 划分可视化（每列一个节点，颜色 = 所属 SCC）

## 输出示例
```
[Test1 随机图] Tarjan SCC=5  Kosaraju SCC=5  划分一致=✅
[Test2 可达性基准] SCC划分 == 相互可达关系=✅
[Test3 缩点无环] 缩点图节点=5  无环=✅
[Test4 孤立节点] 孤立点独立成SCC=✅
[Test5 单环图] 100节点单环 SCC=1 应为1=✅
[Test6 DAG图] 80节点DAG SCC=80 应为80=✅
[图像量化] 像素统计=✅
=== 结果汇总: 7/7 通过 ===
```
