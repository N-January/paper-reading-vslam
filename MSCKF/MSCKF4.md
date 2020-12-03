# OC-KF(Observability-constrained-KF)

---

[toc]

&nbsp;

----

## Reference

1. Robust Stereo Visual Inertial Odometry for Fast Autonomous Flight. 开源S-MSCKF的论文；
2. Observability-constrained Vision-aided Inertial Navigation. OC-EKF的论文；
3. https://zhuanlan.zhihu.com/p/304889273. 关于FEJ的总结；
4. High-Precision, Consistent EKF-based Visual-Inertial Odometry. MSCKF2.0 论文；
5. https://blog.csdn.net/wubaobao1993/article/details/109299097. 关于MSCKF1.0预测部分的总结；

&nbsp;

---

## Notation

依旧先来说清楚文中的Notation是如何表示的：

1. $l$ 节点的理想值表示为$\mathrm{x}_{l}$；在 k 时刻的估计值表示为$\mathrm{\hat{x}}_{l}^{(k)}$；其误差状态表示为$\mathrm{\tilde{x}}_{l}$；
2. $l$ 节点的估计值在不同时刻的关系为：$\mathbf{\hat{x}}^{(k+n)}_{l}={}^{(k+n)}\mathbf{\tilde{x}}^{(k)}_{l}+\mathbf{\hat{x}}^{(k)}_{l}$，特别的，对于旋转有：${}^{l}_{G}\mathbf{\hat{q}}^{(k+n)}\approx (\mathbf{I}-\lfloor {}^{(k+n)}\theta^{(k)}_{l}\rfloor_{\times}){}^{l}_{G}\mathbf{\hat{q}}^{(k)}$；

&nbsp;

----

## IMU状态传递方程

INS系统的重中之重，还是先来推导IMU的状态传递方程。为了和参考【3】的推导保持一致，这里的状态变量也定为$\mathbf{X}=\left[{}^{I}_{G}\mathrm{q}\quad {}^{G}p_{I}\quad {}^{G}v_{I} \right]$，忽略零偏的部分。

&nbsp;

### MSCKF2.0的传递方程

在参考【3】中，作者从理论意义上推导了IMU的传递方程，推得的状态转移矩阵如下：
$$
\begin{bmatrix}
{}^{I_{l+1}}_{G}\tilde{\theta}^{(l)} \\
{}^{G}\tilde{p}_{I_{l+1}}^{(l)} \\
{}^{G}\tilde{v}_{I_{l+1}}^{(l)}
\end{bmatrix} = 
\begin{bmatrix} 
{}^{I_{l+1}}_{I_{l}}R^{(l)} & \mathbf{0} & \mathbf{0} \\
-({}^{I_l}_{G}R^{(l)})^{T}\left[\hat{\mathrm{y}}^{(l)}_{l}\right]_{\times} & \mathbf{I} & \mathbf{I}\Delta{t} \\
-({}^{I_l}_{G}R^{(l)})^{T}\left[\hat{\mathrm{s}}^{(l)}_{l}\right]_{\times} & \mathbf{0} & \mathbf{I}
\end{bmatrix}
\begin{bmatrix}
{}^{I_{l}}_{G}\tilde{\theta}^{(l)} \\
{}^{G}\tilde{p}_{I_{l}}^{(l)} \\
{}^{G}\tilde{v}_{I_{l}}^{(l)}
\end{bmatrix}
+
\begin{bmatrix}
{}^{I_{l+1}}_{I_{l}}\tilde{\theta}^{(l)} \\
({}^{I_l}_{G}R^{(l)})^{T}\left[\tilde{\mathrm{y}}^{(l)}_{l}\right] \\
({}^{I_l}_{G}R^{(l)})^{T}\left[\tilde{\mathrm{s}}^{(l)}_{l}\right]
\end{bmatrix} \tag{1}
$$
其中$-({}^{I_l}_{G}R^{(l)})^{T}\left[\hat{\mathrm{y}}^{(l)}_{l}\right]_{\times}$和$-({}^{I_l}_{G}R^{(l)})^{T}\left[\hat{\mathrm{s}}^{(l)}_{l}\right]_{\times}$可以理解为是旋转误差作用于位移和速度的杆臂，具有一定的物理意义；

> 其实在实际的MSCKF2.0中，传递方程并不是公式（1）所示的形式，而是将旋转部分从矩阵中去掉了，具体可见参考【4】

&nbsp;

### MSCKF1.0的传递方程

在MSCKF1.0的理论中，IMU的误差状态传递方程主要由运动方程求得，如下：
$$
\dot{\tilde{\mathbf{X}}}_{\mathrm{IMU}}=\mathbf{F} \tilde{\mathbf{X}}_{\mathrm{IMU}}+\mathbf{G} \mathbf{n}_{\mathrm{IMU}}  \tag{2}
$$

其中：
$$
\mathbf{F}=\left[\begin{array}{ccc}
-\lfloor\hat{\boldsymbol{\omega}} \times\rfloor & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} \\
\mathbf{0}_{3 \times 3} & \mathbf{0}_{3} & \mathbf{I}_{3 \times 3} \\
-({}_{G}^{I_l}R)^{T}\lfloor\hat{\mathbf{a_m}} \times\rfloor & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} \\
\end{array}\right]
$$
由于此处的G对于能观性的分析没有实质性的用处，这里不作分析；

将微分方程转为离散的形式：
$$
\boldsymbol{\tilde{X}}\left(t_{l+1}\right)=\boldsymbol{\Phi}\left(t_{l+1}, t_{l}\right) \boldsymbol{\tilde{X}}\left(t_{l}\right)+\int_{t_{l}}^{t_{l+1}} \boldsymbol{\Phi}\left(t_{l+1}, \tau\right) \boldsymbol{G}(\tau) \boldsymbol{n}(\tau) \mathrm{d} \tau \tag{3}
$$
其中：
$$
\begin{cases}
\dot{\boldsymbol{\Phi}}\left(t_{l+1}, t_{l}\right) = \boldsymbol{F}(t)\boldsymbol{\Phi}\left(t_{l+1}, t_{l}\right) \\
\boldsymbol{\Phi}\left(t_{l+1}, t_{l}\right)=\exp(\int_{t_{l}}^{t_{l+1}} \boldsymbol{F}(t) \mathrm{d} t)  
\end{cases} \tag{4}
$$
&nbsp;

### OC-KF的系统状态传递方程

在OC-KF中，IMU部分的传递方程使用的是MSCKF1.0中的微分方程的形式，这里先来推导状态转移矩阵的闭式解。请读者耐心看完理想情况下的推导过程，因为整个OC-KF的核心思路就是由该推导启发的。

> Notation：
>
> 以下均表示理想情况下的状态传递



由公式（4）的第一行可以列出如下公式，这里把时间跨度直接认为是 t，初始的时间为 t0：
$$
\begin{aligned}
\begin{bmatrix}
\dot{\Phi}_{11}(t) & \dot{\Phi}_{12}(t) & \dot{\Phi}_{13}(t) \\
\dot{\Phi}_{21}(t) & \dot{\Phi}_{22}(t) & \dot{\Phi}_{23}(t) \\
\dot{\Phi}_{31}(t) & \dot{\Phi}_{32}(t) & \dot{\Phi}_{33}(t) \\
\end{bmatrix}
&=\left[\begin{array}{ccc}
-\lfloor{\boldsymbol{\omega}(t)}\rfloor_{\times} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} \\
\mathbf{0}_{3 \times 3} & \mathbf{0}_{3} & \mathbf{I}_{3 \times 3} \\
-({}^{G}_{t}R)^{T}\lfloor{\mathbf{a_m}(t)}\rfloor_{\times} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} \\
\end{array}\right]
\begin{bmatrix}
{\Phi}_{11}(t) & {\Phi}_{12}(t) & {\Phi}_{13}(t) \\
{\Phi}_{21}(t) & {\Phi}_{22}(t) & {\Phi}_{23}(t) \\
{\Phi}_{31}(t) & {\Phi}_{32}(t) & {\Phi}_{33}(t) \\
\end{bmatrix} \\
\begin{bmatrix}
{\Phi}_{11}(t_0) & {\Phi}_{12}(t_0) & {\Phi}_{13}(t_0) \\
{\Phi}_{21}(t_0) & {\Phi}_{22}(t_0) & {\Phi}_{23}(t_0) \\
{\Phi}_{31}(t_0) & {\Phi}_{32}(t_0) & {\Phi}_{33}(t_0) \\
\end{bmatrix} 
&= 
\begin{bmatrix}
\mathbf{I} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{I} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{I} \\
\end{bmatrix}
\end{aligned} \tag{5}
$$
公式（5）可以引出如下的几组公式：
$$
\begin{cases}
\dot{\Phi}_{11}(t)=-\lfloor \hat{\mathcal{w}} \rfloor_{\times}\Phi_{11}(t) \\
\dot{\Phi}_{12}(t)=-\lfloor \hat{\mathcal{w}} \rfloor_{\times}\Phi_{12}(t) \\ 
\dot{\Phi}_{13}(t)=-\lfloor \hat{\mathcal{w}} \rfloor_{\times}\Phi_{13}(t) \\ 
\end{cases}  \quad
\begin{cases}
\dot{\Phi}_{21}(t)=\Phi_{31}(t) \\
\dot{\Phi}_{22}(t)=\Phi_{32}(t) \\ 
\dot{\Phi}_{23}(t)=\Phi_{33}(t) \\ 
\end{cases}  \quad
\begin{cases}
\dot{\Phi}_{31}(t)=-({}^{G}_{I_t}R)^{T}\lfloor{\mathbf{a_m}}\rfloor_{\times} \Phi_{11}(t) \\
\dot{\Phi}_{32}(t)=-({}^{G}_{I_t}R)^{T}\lfloor{\mathbf{a_m}}\rfloor_{\times} \Phi_{12}(t) \\ 
\dot{\Phi}_{33}(t)=-({}^{G}_{I_t}R)^{T}\lfloor{\mathbf{a_m}}\rfloor_{\times} \Phi_{13}(t) \\ 
\end{cases} \tag{6}
$$
下面就是一组一组的展开一下：

&nbsp;

#### 第一组

容易看出，第一组的闭式解其实都是exp函数，所以：
$$
\begin{cases}
\Phi_{11}(t)=exp(-\lfloor {\mathcal{w}} \rfloor_{\times}(t-t_0))\Phi_{11}(t_0) \\
\Phi_{12}(t)=exp(-\lfloor {\mathcal{w}} \rfloor_{\times}(t-t_0))\Phi_{12}(t_0) \\ 
\Phi_{13}(t)=exp(-\lfloor {\mathcal{w}} \rfloor_{\times}(t-t_0))\Phi_{13}(t_0) \\ 
\end{cases}
$$
由于初始的状态转移矩阵为单位矩阵，所以$\Phi_{12}(t_0)$和$\Phi_{13}(t_0)$均为0，$\Phi_{11}(t_0)=\mathbf{I}$，于是：
$$
\begin{cases}
\Phi_{11}(t, t_{0})=exp(\int_{t_0}^{t}(-\lfloor {\omega}(t)\rfloor_{\times})dt)={}_{t_0}^{t}R \\
\Phi_{12}(t, t_{0})=\mathbf{0} \\
\Phi_{13}(t, t_{0})=\mathbf{0} \\
\end{cases} \tag{7}
$$

&nbsp;

#### 第三组

由于第二组中的$\dot{\Phi}_{21}$与$\Phi_{31}$相关，所以这里先推导第三组的情况：
$$
\begin{cases}
\Phi_{31}(t)=\int_{t_0}^{t} -({}^{G}_{t}R)^{T}\lfloor {\mathbf{a_m}}(t)\rfloor_{\times} {}_{t_0}^{t}R dt \\
\Phi_{32}(t)= \mathbf{I} \times \Phi_{32}(t_0) = \mathbf{0} \\ 
\Phi_{33}(t)= \mathbf{I}\times \Phi_{33}(t_0) = \mathbf{I} \\ 
\end{cases} \tag{8A}
$$
重点分析首行元素：
$$
\begin{aligned}
\Phi_{31}(t)&=\int_{t_0}^{t} -({}^{G}_{t}R)^{T}\lfloor {\mathbf{a_m}}(t)\rfloor_{\times} {}_{t_0}^{t}R dt \\
&=\int_{t_0}^{t} -({}^{G}_{t}R)^{T}\lfloor {}_{G}^{t}R  ({}^{G} {\mathbf{a}}(t)+{}^{G}\mathbf{g})\rfloor_{\times} {}_{t_0}^{t}R dt \\
&=-\int_{t_0}^{t}\lfloor {}^{G}{\mathbf{a}}(t)+{}^{G}\mathbf{g}\rfloor_{\times} {}_{t}^{G}R {}_{t_0}^{t}R dt \\
&=-\int_{t_0}^{t}\lfloor {}^{G}{\mathbf{a}}(t)+{}^{G}\mathbf{g}\rfloor_{\times} dt ({}^{t_0}_{G}R)^{T} \\
&=-\lfloor {}^{G}v_{t}-{}^{G}v_{t_0}+{}^{G}\mathbf{g}(t-t_0) \rfloor_{\times}({}^{t_0}_{G}R)^{T}
\end{aligned} \tag{8B}
$$
其中：

1. $\mathbf{a}_{m}$表示在机体坐标系中的测量值，夹杂重力；
2. ${}^{G}\mathbf{a}$表示在世界坐标系下的加速度，不夹杂重力；
3. 最后一行化简引入了${}^{G}v_{t}={}^{G}v_{t_0}+\int{}^{G}\mathbf{a}dt$，其中的加速度不夹杂重力；

所以公式（8A）可以重新写作：
$$
\begin{cases}
\Phi_{31}(t)= -\lfloor {}^{G}v_{t}-{}^{G}v_{t_0}+{}^{G}\mathbf{g}(t-t_0) \rfloor_{\times}({}^{t_0}_{G}R)^{T}\\
\Phi_{32}(t)= \mathbf{0} \\ 
\Phi_{33}(t)= \mathbf{I} \\ 
\end{cases} \tag{8}
$$
&nbsp;

#### 第二组

将第三组的结果带入到第二组中
$$
\begin{cases}
\Phi_{21}(t)= -\int_{t_0}^{t} \int_{t_0}^{t}\lfloor {}^{G}{\mathbf{a}}(t)+{}^{G}\mathbf{g}\rfloor_{\times} dt d\tau({}^{t_0}_{G}R)^{T}\\
\Phi_{22}(t)= \mathbf{I} \times \Phi_{22}(t_0) = \mathbf{I} \\ 
\Phi_{23}(t)= \Delta{t}\times \Phi_{22}(t_0) = \mathbf{I}\Delta{t} \\ 
\end{cases} \tag{9A}
$$
重点分析首行元素：
$$
\begin{aligned}
\Phi_{21}(t) &= -\int_{t_0}^{t} \int_{t_0}^{t}\lfloor {}^{G}\hat{\mathbf{a}}(t)+{}^{G}\mathbf{g}\rfloor_{\times} dt d\tau({}^{t_0}_{G}R)^{T} \\
&=-\lfloor {}^{G}p_{t}-{}^{G}p_{t_0}-{}^{G}v_{t_0}(t-t_0)+\frac{1}{2}{}^{G}\mathbf{g}(t-t_0)^2 \rfloor_{\times}({}^{t_0}_{G}R)^{T}
\end{aligned} \tag{9B}
$$
所以公式（9A）可以重新写作：
$$
\begin{cases}
\Phi_{21}(t)= -\lfloor {}^{G}p_{t}-{}^{G}p_{t_0}-{}^{G}v_{t_0}(t-t_0)+\frac{1}{2}{}^{G}\mathbf{g}(t-t_0)^2 \rfloor_{\times}({}^{t_0}_{G}R)^{T} \\
\Phi_{22}(t)= \mathbf{I} \\ 
\Phi_{23}(t)= \mathbf{I}\Delta{t} \\ 
\end{cases} \tag{9}
$$
&nbsp;

##### 小结

综合公式（7）（8）（9）可得：
$$
\boldsymbol{\Phi}\left(t, t_{0}\right)=
\begin{bmatrix}
{}_{t_0}^{t}R & 0 & 0 \\
-\lfloor \mathbf{{y}}^{(t)} \rfloor_{\times}({}^{t_0}_{G}R)^{T} & \mathbf{I} & \mathbf{I}\Delta{t} \\
-\lfloor \mathbf{{s}}^{(t)} \rfloor_{\times}({}^{t_0}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \tag{10}
$$

&nbsp;

需要注意的是这里的转移矩阵是从 t0 时刻开始进行递推的。

&nbsp;

----

### 相邻时刻的状态转移矩阵

如上一章节可知，t1 时刻和 t2 时刻的状态转移矩阵分别如下：

#### t1 时刻的状态转移矩阵

$$
\boldsymbol{\Phi}\left(t_1, t_{0}\right)=
\begin{bmatrix}
{}_{t_0}^{t_1}R & 0 & 0 \\
-\lfloor \mathbf{{y}}^{(t_1)} \rfloor_{\times}({}^{t_0}_{G}R)^{T} & \mathbf{I} & \mathbf{I}\Delta{t}_1 \\
-\lfloor \mathbf{{s}}^{(t_1)} \rfloor_{\times}({}^{t_0}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \tag{11}
$$

&nbsp;

#### t2 时刻的状态转移矩阵

$$
\boldsymbol{\Phi}\left(t_2, t_{0}\right)=
\begin{bmatrix}
{}_{t_0}^{t_2}R & 0 & 0 \\
-\lfloor \mathbf{{y}}^{(t_2)} \rfloor_{\times}({}^{t_0}_{G}R)^{T} & \mathbf{I} & \mathbf{I}\Delta{t}_2 \\
-\lfloor \mathbf{{s}}^{(t_2)} \rfloor_{\times}({}^{t_0}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \tag{12}
$$

&nbsp;

#### t1 到 t2 时刻的状态转移矩阵

结合公式（11）（12）易得：
$$
\begin{aligned}
\Phi(t_2, t_1)&=\Phi(t_2, t_0) (\Phi(t_1, t_0))^{-1} \\
&=\begin{bmatrix}
{}_{t_0}^{t_2}R & 0 & 0 \\
-\lfloor \mathbf{{y}}^{(t_2)} \rfloor_{\times}({}^{t_0}_{G}R)^{T} & \mathbf{I} & \mathbf{I}\Delta{t}_2 \\
-\lfloor \mathbf{{s}}^{(t_2)} \rfloor_{\times}({}^{t_0}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix}
\begin{bmatrix}
{}_{t_0}^{t_1}R^{T} & 0 & 0 \\
(\lfloor \mathbf{{y}}^{(t_1)} - \mathbf{{s}}^{(t_1)}\Delta{t}_1 \rfloor_{\times})({}^{t_1}_{G}R)^{T} & \mathbf{I} & -\mathbf{I}\Delta{t}_1 \\
\lfloor \mathbf{{s}}^{(t_1)} \rfloor_{\times}({}^{t_1}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \\
&=\begin{bmatrix}
{}_{t_1}^{t_2}R & 0 & 0 \\
-\lfloor \mathbf{{y}}^{(t_2)}-\mathbf{{y}}^{(t_1)}+\mathbf{\hat{s}}^{(t_1)}\Delta{t}_1- \mathbf{\hat{s}}^{(t_1)}\Delta{t}_2 \rfloor_{\times}({}^{t_1}_{G}R)^{T} & \mathbf{I} & \mathbf{I}(\Delta{t}_2-\Delta{t}_1） \\
-\lfloor \mathbf{{s}}^{(t_2)}-\mathbf{{s}}^{(t_1)} \rfloor_{\times}({}^{t_1}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \\
&=\begin{bmatrix}
{}_{t_1}^{t_2}R & 0 & 0 \\
-\lfloor {}^{G}p_{t_2}-{}^{G}p_{t_1}-{}^{G}v_{t_1}\Delta{t}_{1}^{2}+\frac{1}{2}\mathbf{g}(\Delta{t}_{1}^{2})^2 \rfloor_{\times}({}^{t_1}_{G}R)^{T} & \mathbf{I} & \mathbf{I}(\Delta{t}_{1}^{2}) \\
-\lfloor {}^{G}v_{t_2}-{}^{G}v_{t_1}+\mathbf{g}\Delta{t}_{1}^{2} \rfloor_{\times}({}^{t_1}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \\
&=\begin{bmatrix}
{}_{t_1}^{t_2}R & 0 & 0 \\
-\lfloor {}^{G}p_{t_2}-{}^{G}p_{t_1}-{}^{G}v_{t_1}\Delta{t}_{1}^{2}+\frac{1}{2}\mathbf{g}(\Delta{t}_{1}^{2})^2 \rfloor_{\times}({}^{t_1}_{G}R)^{T} & \mathbf{I} & \mathbf{I}(\Delta{t}_{1}^{2}) \\
-\lfloor {}^{G}v_{t_2}-{}^{G}v_{t_1}+\mathbf{g}\Delta{t}_{1}^{2} \rfloor_{\times}({}^{t_1}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix}
\end{aligned} \tag{13}
$$

可以看到由微分方程推得的状态转移矩阵的闭式解和公式（1）基本一致（感觉推了个寂寞），不过对于噪声项的部分不太能保证。

&nbsp;

-----

## 视觉部分的观测方程

下面的所有的下角标 $l$ 表示id为 $l$ 的相机，不表示时间，这里暂时不涉及时间，可以认为是理想的观测模型。

为了分析能观性，这里还需要一个步骤就是观测模型，以单个观测点$P_{f_j}$为例，其观测模型为：
$$
\begin{aligned}
z_l&=\pi({}^{C_l}\mathrm{p}_{f_j})+n_{l} \\
{}^{C_l}\mathrm{p}_{f_j}&={}^{C}_{I}R {}^{G}_{l}R({}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_l})+{}^{C}\mathrm{p}_I 
\end{aligned} \tag{14}
$$

所以观测模型为：
$$
\begin{aligned}
H_{(I_l|l)}&=J_{(f_j|l)} \quad {}^{C}_{I}R \quad {}_{G}^{I_l}R\begin{bmatrix} \underbrace{\left[{}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_i}\right]_{\times}({}_{G}^{I_l}R)^{T}}_{{\partial e}/{\partial \theta}} & \underbrace{ -\mathbf{I}_{3\times3}}_{{\partial e}/{\partial \mathrm{p}}} & \underbrace{ \mathbf{0}_{3\times3}}_{{\partial e}/{\partial \mathrm{v}}}\end{bmatrix} \\ 
H_{(f_j|l)}&=J_{(f_j|l)} \quad {}^{C}_{I}R \quad {}_{G}^{I_l}R
\end{aligned} \tag{15}
$$
其中：
$$
J_{(f_j|l)}=\frac{1}{Z}\begin{bmatrix}1 & 0 & -\frac{X}{Z} \\ 0 & 1 & -\frac{Y}{Z} \end{bmatrix}
$$
&nbsp;

-----

## 理想情况下能观性的分析

OC-KF在做能观性分析的时候，不像参考【3】中所示的是在能观性矩阵中抽出一部分进行通用的分析，该方法采用递推的方式证明了在时刻 t，零空间应该是什么样的，且理想状态下是如何传播的（propagation），下面就两个部分进行分析：

### 在 t 时刻，系统零空间是如何的

假设系统从 t0 时刻开始，那么该时刻的零空间为：
$$
\mathbf{N}_{t_0}=\begin{bmatrix}
\begin{array}{c|c}
\mathbf{0} & {}^{t_0}_{G}R\mathbf{g} \\
\mathbf{I} & -\lfloor {}^{G}p_{t_0} \rfloor_{\times}\mathbf{g} \\
\mathbf{0} & -\lfloor {}^{G}v_{t_0} \rfloor_{\times}\mathbf{g} \\ \hline
\mathbf{I} & -\lfloor {}^{G}p_{f_j} \rfloor_{\times}\mathbf{g}
\end{array}
\end{bmatrix} \tag{16}
$$
根据能观性矩阵的定义，在 t 时刻，对 $f_j$ 特征点的能观性矩阵的对应行：
$$
\mathcal{O}_{t}=\mathbf{H}_{f_j}\Phi(t, t_0) \tag{17}
$$
所以在 t 时刻，系统的零空间满足：
$$
\mathcal{O}_{t}\mathbf{N}_t=\mathbf{H}_{f_j}\underbrace{\Phi(t, t_0)\mathbf{N}_{t_0}}_{part1} \tag{18}
$$
将公式（10）和公式（16）带入到公式（18）的part1中，可以得到（在系统的传播或者说预测阶段不涉及到观测部分，所以观测部分的零空间照抄上去就好了）：
$$
\begin{aligned}
\Phi(t,t_0)\mathbf{N}_{t_0}&=\begin{bmatrix}
{}_{t_0}^{t}R & 0 & 0 \\
-\lfloor \mathbf{{y}}^{(t)} \rfloor_{\times}({}^{t_0}_{G}R)^{T} & \mathbf{I} & \mathbf{I}\Delta{t} \\
-\lfloor \mathbf{{s}}^{(t)} \rfloor_{\times}({}^{t_0}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix}
\begin{bmatrix}
\mathbf{0} & {}^{t_0}_{G}R\mathbf{g} \\
\mathbf{I} & -\lfloor {}^{G}p_{t_0} \rfloor_{\times}\mathbf{g} \\
\mathbf{0} & -\lfloor {}^{G}v_{t_0} \rfloor_{\times}\mathbf{g} \\ \hline
\mathbf{I} & -\lfloor {}^{G}p_{f_j} \rfloor_{\times}\mathbf{g}
\end{bmatrix} \\
&=\begin{bmatrix}
\mathbf{0} & {}^{t}_{G}R\mathbf{g} \\
\mathbf{I} & -\lfloor {}^{G}p_{t} \rfloor_{\times}\mathbf{g} \\
\mathbf{0} & -\lfloor {}^{G}v_{t} \rfloor_{\times}\mathbf{g} \\ \hline
\mathbf{I} & -\lfloor {}^{G}p_{f_j} \rfloor_{\times}\mathbf{g}
\end{bmatrix}
\end{aligned} \tag{19}
$$
可以看到在理想情况下，系统的零空间从在 t 时刻和初始时刻 t0 的零空间还是颇为相似的。需要注意的是在实际情况下，公式（19）中的变量均是由 t-1 时刻的值预测出来的。

&nbsp;

### 相邻时刻间系统的零空间是如何传播的

另一方面 ，t1 到 t2 时刻的系统零空间满足：
$$
\begin{aligned}
\Phi(t_2, t_1)\mathbf{N}_{t_1}&=
\begin{bmatrix}
{}_{t_1}^{t_2}R & 0 & 0 \\
-\lfloor {}^{G}p_{t_2}-{}^{G}p_{t_1}-{}^{G}v_{t_1}\Delta{t}_{1}^{2}+\frac{1}{2}\mathbf{g}(\Delta{t}_{1}^{2})^2 \rfloor_{\times}({}^{t_1}_{G}R)^{T} & \mathbf{I} & \mathbf{I}(\Delta{t}_{1}^{2}) \\
-\lfloor {}^{G}v_{t_2}-{}^{G}v_{t_1}+\mathbf{g}\Delta{t}_{1}^{2} \rfloor_{\times}({}^{t_1}_{G}R)^{T} & \mathbf{0} & \mathbf{I}
\end{bmatrix} 
\begin{bmatrix}
\mathbf{0} & {}^{t_1}_{G}R\mathbf{g} \\
\mathbf{I} & -\lfloor {}^{G}p_{t_1} \rfloor_{\times}\mathbf{g} \\
\mathbf{0} & -\lfloor {}^{G}v_{t_1} \rfloor_{\times}\mathbf{g}
\end{bmatrix} \\
&= \begin{bmatrix}
\mathbf{0} & {}^{t_2}_{G}R\mathbf{g} \\
\mathbf{I} & -\lfloor {}^{G}p_{t_2} \rfloor_{\times}\mathbf{g} \\
\mathbf{0} & -\lfloor {}^{G}v_{t_2} \rfloor_{\times}\mathbf{g} \\
\end{bmatrix}
\end{aligned}  \tag{20}
$$

&nbsp;

### t 时刻传播过来的零空间是否是观测矩阵的零空间

在 t 时刻，观测矩阵为：
$$
\begin{aligned}
\mathbf{H}_{t}&=J_{f_j}{}^{C}_{I}R{}_{G}^{t}R\begin{bmatrix} 
\begin{array}{ccc|c}
\underbrace{\left[{}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{t}\right]_{\times}({}_{G}^{t}R)^{T}}_{{\partial e}/{\partial \theta}} & \underbrace{ -\mathbf{I}_{3\times3}}_{{\partial e}/{\partial \mathrm{p}}} & \underbrace{ \mathbf{0}_{3\times3}}_{{\partial e}/{\partial \mathrm{v}}} & \underbrace{\mathbf{I}_{3\times3}}_{\partial e/\partial p_{f_j}}
\end{array}
\end{bmatrix} 
\end{aligned} \tag{21}
$$
结合公式（19）易得：
$$
\begin{aligned}
\mathbf{H}_{t}\mathbf{N}_{t}&=
\begin{bmatrix} 
\left[{}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{t}\right]_{\times}({}_{G}^{t}R)^{T} &  -\mathbf{I}_{3\times3} & \mathbf{0}_{3\times3} & \mathbf{I}_{3\times3}
\end{bmatrix}
\begin{bmatrix}
\mathbf{0} & {}^{t}_{G}R\mathbf{g} \\
\mathbf{I} & -\lfloor {}^{G}p_{t} \rfloor_{\times}\mathbf{g} \\
\mathbf{0} & -\lfloor {}^{G}v_{t} \rfloor_{\times}\mathbf{g} \\
\mathbf{I} & -\lfloor {}^{G}p_{f_j} \rfloor_{\times}\mathbf{g}
\end{bmatrix} \\
&= \mathbf{0}
\end{aligned}
$$
&nbsp;

### 小结

由上述分析可知，在理想情况下：

1. 性质一：系统的零空间可以通过状态转移矩阵传播到当前时刻 t，且形式与初始零空间相似，由 t 时刻状态的预测值有关；
2. 性质二：相邻时刻的零空间可以通过状态转移矩阵进行传播，且零空间的形式也满足通用形式；
3. 性质三：通过状态传递矩阵传播到当前时刻 t 的零空间与观测矩阵正交，亦即优化问题的优化方向与零空间正交，因此优化量不会破坏系统的零空间；

&nbsp;

----

## OC-KF对于零空间的处理

> Notation:
>
> 一下均是从实际情况出发了。
>
> 状态变量中仅有IMU的位姿以及空间中特征点的位置，不像MSCKF有一个滑动窗口记录所有的相机位姿，所以某个时刻的位姿一旦被优化了，**那么该位姿的观测方程的线性化点就确定了**，后面不会改变，这是和MSCKF的很大的不同；

### OC-KF想做什么？

通过上面对理想情况的分析，OC-KF建立的一个比较理想化的方法，该方法把上述的三个性质使用的淋漓尽致：

1. 在时刻 t 构成的能观性矩阵中的项目为：
   $$
   \hat{\mathcal{O}}_{t}=\hat{\mathbf{H}}_{f_j}\hat{\Phi}(t,t-1)\hat{\Phi}(t-1,t-2)\dots\hat{\Phi}(t_1, t_0) \tag{22}
   $$
   
2. OC-KF认为，在 t 时刻，之前的状态都已经通过最优化的方法求得了最优解，那么使用性质一，t0 时刻的零空间可以通过$\hat{\Phi}(t-1, t_0)$传播到 t-1 时刻，写作：
   $$
   \mathbf{N}_{t-1}=
   \begin{bmatrix}
   \mathbf{0} & {}^{t-1}_{G}R^{(t-2)}\mathbf{g} \\
   \mathbf{I} & -\lfloor {}^{G}p_{t-1}^{(t-2)} \rfloor_{\times}\mathbf{g} \\
   \mathbf{0} & -\lfloor {}^{G}v_{t-1}^{(t-2)} \rfloor_{\times}\mathbf{g} \\ \hline
   \mathbf{I} & -\lfloor {}^{G}p_{f_j} \rfloor_{\times}\mathbf{g}
   \end{bmatrix}
   =\underbrace{\Phi(t-1,t_0)}_{optimal^{*}}\mathbf{N}_{t_0} \tag{23}
   $$

   其中$optimal^{*}$表示算法认为之前的处理已经趋于理想情况了；其中的 t-1 时刻的零空间是由状态转移矩阵传播过来的，所以零空间中的变量均是使用 t-2 时刻的值**预测出来**的；

3. 对于 t 时刻的状态转移矩阵$\hat{\Phi}(t, t-1)$而言，根据性质二，该状态转移矩阵必然可以将零空间$\mathbf{N}_{t-1}$传播为$\mathbf{N}_t$，且形式上满足通用形式：
   $$
   \mathbf{N}_{t}=
   \begin{bmatrix}
   \mathbf{0} & {}^{t}_{G}R^{(t-1)}\mathbf{g} \\
   \mathbf{I} & -\lfloor {}^{G}p_{t}^{(t-1)} \rfloor_{\times}\mathbf{g} \\
   \mathbf{0} & -\lfloor {}^{G}v_{t}^{(t-1)} \rfloor_{\times}\mathbf{g} \\
   \mathbf{I} & -\lfloor {}^{G}p_{f_j} \rfloor_{\times}\mathbf{g}
   \end{bmatrix}
   =\check{\Phi}(t,t-1)\mathbf{N}_{t-1} =
   \check{\Phi}(t, t-1)
   \begin{bmatrix}
   \mathbf{0} & {}^{t-1}_{G}R^{(t-2)}\mathbf{g} \\
   \mathbf{I} & -\lfloor {}^{G}p_{t-1}^{(t-2)} \rfloor_{\times}\mathbf{g} \\
   \mathbf{0} & -\lfloor {}^{G}v_{t-1}^{(t-2)} \rfloor_{\times}\mathbf{g} \\
   \mathbf{I} & -\lfloor {}^{G}p_{f_j} \rfloor_{\times}\mathbf{g}
   \end{bmatrix}\tag{24}
   $$
   
   
   其中 t 时刻的零空间是由 t-1 时刻的零空间经由相邻时间的状态转移矩阵传播过来的，因此也是使用预测值；公式中使用$\check{\Phi}$来表示期望的传递矩阵。

4. 对于 t 时刻的观测矩阵$\hat{\mathbf{H}}(t)$，根据性质三，该观测矩阵要与零空间$\mathbf{N}_t$正交：

$$
\check{\mathbf{H}}_{f_j}\mathbf{N}_{t} =\mathbf{0} \tag{25}
$$

分析到这里其实就比较明确了，OC-KF的**核心就是期望找到合适的状态转移矩阵和合适的观测矩阵，使得零空间的传播和优化方向与理想情况下的相同**。

下面其实就可以针对公式（24）（25）进行分析了。

&nbsp;

### 修改状态转移矩阵$\hat{\Phi}(t, t-1)$

实际情况下，t-1 时刻到 t 时刻的状态转移矩阵为：
$$
\hat{\Phi}(t, t-1)=exp(\mathbf{F}(t, t-1)\Delta{t})=
\begin{bmatrix}
\hat{\Phi}_{11} & \mathbf{0} & \mathbf{0} \\
\hat{\Phi}_{21} & \mathbf{I} & \mathbf{I}\Delta{t} \\
\hat{\Phi}_{31} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \tag{26}
$$
根据公式（24）有：
$$
\mathbf{N}_{t}=
\begin{bmatrix}
\mathbf{0} & {}^{t}_{G}R^{(t-1)}\mathbf{g} \\
\mathbf{I} & -\lfloor {}^{G}p_{t}^{(t-1)} \rfloor_{\times}\mathbf{g} \\
\mathbf{0} & -\lfloor {}^{G}v_{t}^{(t-1)} \rfloor_{\times}\mathbf{g} \\ \hline
\mathbf{I} & -\lfloor {}^{G}p_{f_j} \rfloor_{\times}\mathbf{g}
\end{bmatrix}=
\begin{bmatrix}
\check{\Phi}_{11} & \mathbf{0} & \mathbf{0} \\
\check{\Phi}_{21} & \mathbf{I} & \mathbf{I}\Delta{t} \\
\check{\Phi}_{31} & \mathbf{0} & \mathbf{I}
\end{bmatrix}
\begin{bmatrix}
\mathbf{0} & {}^{t-1}_{G}R^{(t-2)}\mathbf{g} \\
\mathbf{I} & -\lfloor {}^{G}p_{t-1}^{(t-2)} \rfloor_{\times}\mathbf{g} \\
\mathbf{0} & -\lfloor {}^{G}v_{t-1}^{(t-2)} \rfloor_{\times}\mathbf{g} \\ \hline
\mathbf{I} & -\lfloor {}^{G}p_{f_j} \rfloor_{\times}\mathbf{g}
\end{bmatrix} \tag{27}
$$
这里依旧只考虑IMU状态量，也就是矩阵横线下面特征点部分不考虑。

把公式（27）各项展开有：
$$
\begin{aligned}
\begin{cases}
{}^{t}_{G}R^{(t-1)}\mathbf{g} 
&= \check{\Phi}_{11} {}^{t-1}_{G}R^{(t-2)}\mathbf{g} \\
-\lfloor {}^{G}p_{t}^{(t-1)} \rfloor_{\times}\mathbf{g} 
&= \check{\Phi}_{21}{}^{t-1}_{G}R^{(t-2)}\mathbf{g}-\lfloor {}^{G}p_{t-1}^{(t-2)} \rfloor_{\times}\mathbf{g} - \lfloor {}^{G}v_{t-1}^{(t-2)} \Delta{t} \rfloor_{\times}\mathbf{g}\\
-\lfloor {}^{G}v_{t}^{(t-1)} \rfloor_{\times}\mathbf{g} 
&= \check{\Phi}_{31}{}^{t-1}_{G}R^{(t-2)}\mathbf{g}-\lfloor {}^{G}v_{t-1}^{(t-2)} \rfloor_{\times}\mathbf{g}
\end{cases}
\end{aligned} \tag{28}
$$
公式（28）中第一行很容易求解：
$$
\check{\Phi}_{11}={}^{t}_{G}R^{(t-1)}({}^{t-1}_{G}R^{(t-2)})^{T} \tag{29}
$$
第二行和第三行在求解的时候，作者构建了一个最优化问题，该问题满足如下公式：
$$
\begin{aligned}
\mathop{min}_{\check{\Phi}} &\quad \left\| \check{\Phi}-\hat{\Phi} \right\|_{\mathcal{F}}^{2} \\
s.t. &\quad \check{\Phi}\mathrm{u}=\mathrm{w}
\end{aligned} \tag{30}
$$
其中：

1. 对于第二行和第三行，$\mathrm{u}={}^{t-1}_{G}R^{(t-2)}$；
2. 对于第二行，$\mathrm{w}=\lfloor {}^{G}p_{t-1}^{(t-2)}+{}^{G}v_{t-1}^{(t-2)}\Delta{t}-{}^{G}p_{t}^{(t-1)} \rfloor_{\times}$；
3. 对于第三行，$\mathrm{w}=\lfloor {}^{G}v_{t-1}^{(t-2)}-{}^{G}v_{t}^{(t-1)} \rfloor_{\times}$；

对于公式（30）所示的优化问题而言，采用拉格朗日乘子法和KKT条件求解对偶问题，详细步骤如下：

1. 构建拉格朗日函数$L(\check{\Phi}, \alpha)=\left\| \check{\Phi}-\hat{\Phi} \right\|_{\mathcal{F}}^{2}+\alpha(\check{\Phi}\mathrm{u}-\mathrm{w})$；则原始问题为：
   $$
   \mathop{min}_{\check{\Phi}} \mathop{max}_{\alpha} \underbrace{ \left\| \check{\Phi}-\hat{\Phi} \right\|_{\mathcal{F}}^{2}+\alpha(\check{\Phi}\mathrm{u}-\mathrm{w})}_{ L(\check{\Phi}, \alpha)} \tag{31}
   $$

2. 在满足KKT条件下（因为只涉及到等式约束，所以KKT条件只满足等式约束和梯度约束就可以了）求解原始问题的对偶问题：
   $$
   \begin{aligned}
   \mathop{max}_{\alpha} \mathop{min}_{\check{\Phi}} &\quad \underbrace{ \left\| \check{\Phi}-\hat{\Phi} \right\|_{\mathcal{F}}^{2}}_{f(\check{\Phi})}+\underbrace{\alpha(\check{\Phi}\mathrm{u}-\mathrm{w})}_{g(\check{\Phi})} \\
   s.t. &\quad \frac{\partial f(\check{\Phi})}{\partial \check{\Phi}}+\alpha \frac{\partial g(\check{\Phi})}{\partial \check{\Phi}}=2(\check{\Phi}-\hat{\Phi})+\alpha\mathrm{u}= 0 \quad \text{约束1} \\  
   &\quad \check{\Phi}\mathrm{u}-\mathrm{w} = 0 \quad \text{约束2}
   \end{aligned} \tag{32}
   $$
   
3. 将约束 1 推得的$\check{\Phi}=\hat{\Phi}-\frac{1}{2}\alpha\mathrm{u}$带入到对偶问题中得到：
   $$
   \begin{aligned}
   \mathop{max}_{\alpha} &\quad 
   -\frac{1}{4}\alpha^{2}\mathrm{u}^T\mathrm{u}+\alpha(\hat{\Phi}\mathrm{u}-\mathrm{w})
   \end{aligned} \tag{33}
   $$
   对$\alpha$求导为0可得$\alpha=2(\hat{\Phi}\mathrm{u}-\mathrm{w})(\mathrm{u}^{T}\mathrm{u})^{-1}$，将结果带入约束 1 的推论中可得：
   $$
   \check{\Phi}=\hat{\Phi}-(\hat{\Phi}\mathrm{u}-\mathrm{w})(\mathrm{u}^{T}\mathrm{u})^{-1}\mathrm{u}^{T} \tag{34}
   $$
   最后的 u 取转置是为了维度的适配，该最优解带入KKT条件的约束 2 可得：
   $$
   \begin{aligned}
   \check{\Phi}\mathrm{u}-\mathrm{w}&=(\hat{\Phi}-(\hat{\Phi}\mathrm{u}-\mathrm{w})(\mathrm{u}^{T}\mathrm{u})^{-1}\mathrm{u}^T)\mathrm{u}-\mathrm{w} \\
   &=\hat{\Phi}\mathrm{u}-\mathrm{w}-(\hat{\Phi}\mathrm{u}-\mathrm{w}) \\
   &=\mathbf{0}
   \end{aligned} \tag{35}
   $$
   
   于是公式（34）满足KKT条件，对偶问题的最优解就是原始问题的最优解；

将公式（29）和公式（34）所得到的部分带入到公式（26）中就可以得到修改之后的状态转移矩阵，该矩阵为：
$$
\check{\Phi}(t, t-1)=
\begin{bmatrix}
{}^{t}_{G}R^{(t-1)}({}^{t-1}_{G}R^{(t-2)})^{T} & \mathbf{0} & \mathbf{0} \\
\check{\Phi}_{21} & \mathbf{I} & \mathbf{I}\Delta{t} \\
\check{\Phi}_{31} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \tag{36}
$$
&nbsp;

### 修改观测矩阵$\hat{\mathbf{H}}(t)$

修改过状态转移矩阵之后，t 时刻的零空间如公式（27）左边部分所示，所以我们需要找到最优的观测矩阵$\check{\mathbf{H}}(t)$，使之满足公式（25）：
$$
\begin{aligned}
\check{\mathbf{H}}_{f_j}\mathbf{N}_{t} = 
\begin{bmatrix}\begin{array}{ccc|c}
\mathbf{H}_{\tilde{\theta}} & \mathbf{H}_{\tilde{p}} & \mathbf{H}_{\tilde{v}} & \mathbf{H}_{f_j}
\end{array}\end{bmatrix}
\begin{bmatrix}
\mathbf{0} & {}^{t}_{G}R^{(t-1)}\mathbf{g} \\
\mathbf{I} & -\lfloor {}^{G}p_{t}^{(t-1)} \rfloor_{\times}\mathbf{g} \\
\mathbf{0} & -\lfloor {}^{G}v_{t}^{(t-1)} \rfloor_{\times}\mathbf{g} \\ \hline
\mathbf{I} & -\lfloor {}^{G}p_{f_j} \rfloor_{\times}\mathbf{g}
\end{bmatrix}
\end{aligned} \tag{37}
$$

由零空间的第一列可以得到$\mathbf{H}_{f_j}=-\mathbf{H}_{\tilde{\theta}}$，该结论很重要，它指示了当算法修改了前面的观测矩阵元素时，对应的点的元素也要修改！不过这里也提供了一些方便，我们可以仅仅关注前三个元素，当求出位置的元素之后可以直接替换掉点的元素部分。

于是重点其实还是在第二行，作者和修改状态转移矩阵一样，也是构建了一个最优化问题进行求解：
$$
\begin{aligned}
\mathop{min}_{\check{\mathbf{H}}} &\quad \left\| \check{\mathbf{H}} - \hat{\mathbf{H}} \right\|^{2}_{\mathcal{F}} \\
s.t. &\quad \check{\mathbf{H}}\mathbf{u}=\mathbf{0}
\end{aligned} \tag{38}
$$
其中：

- $\check{\mathbf{H}}$表示要求解的最优值；

- $\hat{\mathbf{H}}$表示实际的观测矩阵，如下：
  $$
  H_{t}=J_{f_j}^{(t)} \quad {}^{C}_{I}R \quad {}_{G}^{I_t}R^{(t-1)}\begin{bmatrix} \underbrace{\left[{}^{G}\hat{\mathrm{p}}_{f_j}-{}^{G}\hat{\mathrm{p}}_{I_t}^{(t-1)}\right]_{\times}({}_{G}^{I_t}R^{(t-1)})^{T}}_{\mathbf{H}_{\theta}} & \underbrace{ -\mathbf{I}_{3\times3}}_{\mathbf{H}_{p}} & \underbrace{ \mathbf{0}_{3\times3}}_{\mathbf{H}_{v}}\end{bmatrix} \tag{39}
  $$

- $\mathbf{u}$表示零空间中的元素，如下：
  $$
  \mathbf{u}=\begin{bmatrix}
  ({}^{t}_{G}R^{(t-1)}\mathbf{g})^{T} & (-\lfloor {}^{G}p_{t}^{(t-1)} \rfloor_{\times}\mathbf{g})^{T} & (-\lfloor {}^{G}v_{t}^{(t-1)} \rfloor_{\times}\mathbf{g})^{T}
  \end{bmatrix}^{T} \tag{40}
  $$
  

依旧使用拉格朗日乘子法和KKT求得最优解为：
$$
\check{\mathbf{H}}(t)=\hat{\mathbf{H}}(t)-\hat{\mathbf{H}}(t)\mathrm{u}(\mathrm{u}^{T}\mathrm{u})^{-1}\mathrm{u}^{T} \tag{41}
$$
&nbsp;

### 小结

从本节的分析可以得知：

1. OC-KF建立了一个比较理想的模型：算法认为 t 时刻之前的状态接近理想状态，因此需要维护的零空间可以按照理想情况从初始时刻传播过来，满足公式（23）；
2. 在 t 时刻的预测阶段，OC-KF通过修改 t-1 到 t 的状态传递矩阵强制使 t 时刻的零空间满足理想情况下的形式，如公式（27）所示，这一步也使得后面再次进行步骤 1 时能满足必要的条件；
3. 在 t 时刻的更新阶段，OC-KF通过修改观测矩阵强制试该优化方向与 t 时刻的零空间正交，一方面保证了能观性，**另一方面保证了优化出来的参数变量不会影响 t 时刻的零空间，即还是步骤 2 中的零空间**，使得后面再次进行步骤 1 是能满足必要的条件；

那么随之而来一个问题：这么强制的优化方向如果不好，使之偏离了理想值，那么步骤 1 所依赖的理想条件是不是就有点儿自相矛盾？

> 零空间：有两个年轻人，一个$\check{\Phi}(t, t-1)$，一个$\check{\mathbf{H}}(t)$，上来就是一个拉格朗日乘子法，一个KKT条件，一个矩阵替换，很快啊！他们说他们是乱打的，可不是乱打的，高等数学，线性代数，最优化，训练有素，有备而来。这两个年轻人不讲武德，来骗，来偷袭我这个t0时刻的老零空间，这好吗？这不好！我劝这两位年轻人耗子尾汁，系统内部要讲武德，不要窝里斗。

&nbsp;

----

## OC-KF方法在MSCKF中的应用

参考【1】所述的开源的S-MSCKF采用了上述的OC-KF对于零空间的维护方法，不同的是MSCKF维护了一个相机姿态的窗口，窗口内的相机位姿都还会被继续更新。

所以这里跟FEJ的分析方法相似，也以 $\alpha_{i+1}$ 时刻对于节点 $l$ 的能观性矩阵项目分析，假设节点 $l$ 是在 t 时刻预测和更新，则对应的项目如下：
$$
\hat{\mathcal{O}}_{l}^{(\alpha_{i+1})} = \hat{\mathbf{H}}_{f_j|l}^{(\alpha_{i+1})}\underbrace{ \check{\Phi}(t,t-1)\check{\Phi}(t-1,t-2)\dots\check{\Phi}(t_1,t_0)}_{\Phi(t, t_0)} \tag{42}
$$
和参考【1】分析一样，因为在 t 时刻之后，之前的状态矩阵已经确定了，所以在 $\alpha_{i+1}$ 时刻，系统状态矩阵部分的连乘已经确定（注意是经过修改之后的），不再变化，所以此时的系统零空间依旧为 t 时刻的系统零空间：
$$
\mathbf{N}_{t}=\begin{bmatrix}
\mathbf{0} & {}^{t}_{G}R^{(t-1)}\mathbf{g} \\
\mathbf{I} & -\lfloor {}^{G}p_{t}^{(t-1)} \rfloor_{\times}\mathbf{g} \\ \hline
\mathbf{I} & -\lfloor {}^{G}p_{f_j} \rfloor_{\times}\mathbf{g}
\end{bmatrix} \tag{43}
$$
这里因为窗口中记录的变量仅为位姿，没有速度，所以这里不再考虑速度进来。所以问题又回到上一章节的修改观测矩阵部分，只不过当时的观测矩阵使用 t 时刻的状态变量，而此时使用 $\alpha_{i}$ 时刻更新后的变量值，但是整个思路没有变。

所以在MSCKF中，仅仅需要记录在 t 时刻的零空间，之后在 $\alpha_{i+1}$ 时刻使用以下公式（44）来迫使优化方向与当时的零空间正交（来骗！来偷袭....）：
$$
\check{\mathbf{H}}(\alpha_{i+1})=\hat{\mathbf{H}}(\alpha_{i+1})-\hat{\mathbf{H}}(\alpha_{i+1})\mathrm{u}(\mathrm{u}^{T}\mathrm{u})^{-1}\mathrm{u}^{T} \tag{44}
$$
&nbsp;

-----

## 总结

本文比较详细的说明了：

1. 理想情况下零空间是可以通过状态传递矩阵进行传递的，该结论同样适用于FEJ的推理方法中；
2. OC-KF是如何维护系统的零空间的；
3. OC-KF的方法如何使用在MSCKF中的；

可以看到，该方法对于零空间的维护相比于FEJ来说：

- 优点是对于初始状态的依赖性不是很强，后续的优化方向虽然和FEJ方法的零空间一样正交，但是结合了当前的状态值；
- 缺点笔者个人觉得就是比较理想，FEJ从理论上可以不通过修改状态传递矩阵$\hat{\Phi}(t,t-1)$就可以达到OC-KF修改状态矩阵的效果，同时如果使用MSCKF2.0中的方法，将旋转从状态转移矩阵中去除，那么误差应该会更小才对；

