# 思路

## 全局路径规划
先根据桥墩位置和边界位置，尽可能靠右生成一条从起点到终点的路径点

## 运动规划 
根据船舶控制约束，将生成的路径转化为控制命令

## KeyWord
全局路径计算， MPC控制， 避碰算法

## 优化约束
1. 减少调整舵角次数（减少调整方向的次数）
2. 降低路径的长度
3. 在可控范围内增加螺旋浆的转速以减少任务耗时
4. 限速12km/h = 3.33333...m/s
5. 总长度100.0米，水线宽17.2米
6. 不能越过中线
7. 一定范围内的避碰 （500m）



## 船舶控制模型
Todo...
| 变量名           | 变量定义                 | 数据类型 |
|------------------|--------------------------|----------|
| ps_delta_1       | 左舵舵角指令（°）         | Float    |
| ps_rps_1         | 左螺旋桨转速指令（RPM）   | Float    |
| ss_delta_1       | 右舵舵角指令（°）         | Float    |
| ss_rps_1         | 右螺旋桨转速指令（RPM）   | Float    |


实际可以控制的就是舵角度和转速

| 字段名           | 含义                     | 数据类型 |
|------------------|--------------------------|----------|
| t                | 当前场景运行时间（s）     | Int      |
| ps_delta_2       | 左舵舵角反馈（°）         | Float    |
| ps_rps_2         | 左螺旋桨转速反馈（RPM）   | Float    |
| ss_delta_2       | 右舵舵角反馈（°）         | Float    |
| ss_rps_2         | 右螺旋桨转速反馈（RPM）   | Float    |
| motion_lon       | 本船经度（°）             | Float    |
| motion_lat       | 本船纬度（°）             | Float    |
| motion_heading   | 本船真航向（°；0-360°）   | Float    |
| motion_u         | 本船纵荡速度（m/s）       | Float    |
| motion_v         | 本船横荡速度（m/s）       | Float    |
| motion_r         | 本船转艏角速度（°/s）     | Float    |
| ts_ship_list     | 避碰船列表               | List     |

其中“ts_ship_list”为二维数组，矩阵信息表示为：n行，4列（n为未知数，即避碰船数量未知），每行代表一艘避碰船信息，每列分别代表避碰船的经度、纬度、真航向和对地速度。

## 运动学模型
$$
\begin{aligned}
x_{k+1} &= x_k + \Delta t \cdot (u_k \cos\psi_k - v_k \sin\psi_k) \\
y_{k+1} &= y_k + \Delta t \cdot (u_k \sin\psi_k + v_k \cos\psi_k) \\
\psi_{k+1} &= \psi_k + \Delta t \cdot r_k \\
u_{k+1} &= u_k + \Delta t \cdot (-X_u u_k + X_{u|u|} |u_k|u_k + X_n n_k^2) \\
v_{k+1} &= v_k + \Delta t \cdot (-Y_v v_k - Y_r r_k + Y_\delta \delta_k) \\
r_{k+1} &= r_k + \Delta t \cdot (-N_v v_k - N_r r_k + N_\delta \delta_k)
\end{aligned}
$$

各项物理含义:
		$X_u, Y_v, N_r$：线性阻尼项（水动力阻力）
		$X_{u|u|}$：非线性阻力系数（例如随速度平方增长）
		$X_n$：推进器输出推力对应转速的增益系数
		$Y_\delta, N_\delta$：舵角对侧向力/转矩的影响


## 状态变量
$$
\mathbf{x}_k =
\begin{bmatrix}
x_k \\
y_k \\
\psi_k \\
u_k \\
v_k \\
r_k
\end{bmatrix}, \quad
\mathbf{u}_k =
\begin{bmatrix}
\delta_k \\
n_k
\end{bmatrix}
$$

## 状态转移矩阵
$$
A = \frac{\partial f}{\partial \mathbf{x}} =
\begin{bmatrix}
1 & 0 & -\Delta t(u s + v c) & \Delta t c & -\Delta t s & 0 \\
0 & 1 & \Delta t(u c - v s) & \Delta t s & \Delta t c & 0 \\
0 & 0 & 1 & 0 & 0 & \Delta t \\
0 & 0 & 0 & 1 + \Delta t(-X_u + 2X_{u|u|}|u|) & 0 & 0 \\
0 & 0 & 0 & 0 & 1 - \Delta t Y_v & -\Delta t Y_r \\
0 & 0 & 0 & 0 & -\Delta t N_v & 1 - \Delta t N_r
\end{bmatrix}
$$

## 控制输入矩阵
$$
B = \frac{\partial f}{\partial \mathbf{u}} =
\begin{bmatrix}
0 & 0 \\
0 & 0 \\
0 & 0 \\
0 & 2 \Delta t X_n n \\
\Delta t Y_\delta & 0 \\
\Delta t N_\delta & 0
\end{bmatrix}
$$