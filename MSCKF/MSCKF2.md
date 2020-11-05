# MSCKF（三）——更新部分

----

## Reference

1. A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation. MSCKF1.0的论文；
2. Quaternion Kinematics for Error-State KF. 关于四元数以及ESKF的论文；
3. Robust Stereo Visual Inertial Odometry for Fast Autonomous Flight. S-MSCKF对应的论文；
4. https://github.com/KumarRobotics/msckf_vio S-MSCKF的工程；
5. https://zhuanlan.zhihu.com/p/76341809 知乎上大佬对MSCKF的总结，本文没有过多的程序方面的讲解，都是从理论上推导的，也是个人的一些解惑过程；
6. https://blog.csdn.net/wubaobao1993/article/details/109299097  笔者关于预测部分的总结；

&nbsp;

-----

## 参数的表示

首先重新回顾一下MSCKF的要估计的参数：
$$
\tilde{\mathrm{X}}_{IMU}=\begin{bmatrix} ^{I}_{G}\delta{\theta^T} & \tilde{b}_g^T & ^{G}\tilde{v}_{I}^T & \tilde{b}_{a}^T & ^{G}\tilde{p}_{I}^{T} & _{I}^{C}\delta{\theta}^T & ^{I}\tilde{p}_{C}^T \end{bmatrix}^{T}   \tag{1}
$$
整个过程中的参数向量为：
$$
\mathrm{\tilde{X}_{k}}=\begin{bmatrix}\mathrm{\tilde{X}_{IMU}} & \delta{\theta}_{C_1} & ^{G}\tilde{p}_{C_1}^T & ... & \delta{\theta}_{C_N} & ^{G}\tilde{p}_{C_N}^T	\end{bmatrix}^T  \tag{2}
$$

其次整个框架需要求解的变量其实还是机体的位姿信息：
$$
\mathrm{\widehat{X}_{k}}=\begin{bmatrix}\mathrm{\widehat{X}_{IMU}} & ^{C_1}_{G}\widehat{\overline{q}}^T & ^{G}\widehat{p}_{C_1}^T & ... & ^{C_N}_{G}\widehat{\overline{q}}^T & ^{G}\widehat{p}_{C_N}^T	\end{bmatrix}^T  \tag{3}
$$
&nbsp;

----

## MSCKF的预测部分

这里简单回顾MSCKF的预测部分，详细可以阅读参考[6]；

### IMU误差状态的传导

对于每一帧IMU数据，都可以根据误差状态的微分方程求得状态转移部分的递推方程的闭式解，如下：
$$
\begin{aligned}
\begin{cases}
\mathrm{\tilde{x}_{k+1}^{IMU}} &= \Phi(\mathrm{k+1},\mathrm{k})\mathrm{\tilde{x}_k^{IMU}} \\
\mathrm{P_{k+1|k}^{IMU}} &=\Phi(\mathrm{k+1},\mathrm{k})\mathrm{P_{k|k}^{IMU}}\Phi(\mathrm{k+1},\mathrm{k})^T + \Phi(\mathrm{k+1},\mathrm{k})\mathrm{G}\mathrm{N}\mathrm{G}^T\Phi(\mathrm{k+1},\mathrm{k})^T
\end{cases}
\end{aligned}  \tag{4}
$$
&nbsp;

### 新帧到达前的协方差矩阵更新

新帧未到来之前，由于没有任何新的观测可以影响相机位姿，因此整个协方差的更新过程如下：
$$
\mathrm{P}_{k+1|k}=\begin{bmatrix} \mathrm{P}^{IMU}_{k+1|k} & \Phi(k+1, k)\mathrm{P}^{IC}_{k|k} \\ (\Phi(k+1, k)\mathrm{P}^{IC}_{k|k})^T & \mathrm{P}^{CAM}_{k|k} \end{bmatrix}  \tag{5}
$$
&nbsp;

### 新帧到来时的协方差矩阵的扩展

新帧到来的时候，算法直接使用外参矩阵讲IMU的最新位姿作为其位姿，因此两者的误差状态满足：
$$
\mathrm{\tilde{X}^{CAM}_{k+1|k}}=\begin{bmatrix} {}_{G}^{C}\delta{\theta} \\ ^{G}\tilde{p}_{C} \end{bmatrix}=\begin{bmatrix}\mathrm{\hat{R}_{I}^{C}} & \mathbf{0}_{3x3} & \mathbf{0}_{3x3} & \mathbf{0}_{3x3} & \mathbf{0}_{3x3} & \mathbf{I}_{3x3} & \mathbf{0}_{3x3} \\ 
-\mathrm{\hat{R}_{G}^{I}}^{T}([{}^{I}\mathrm{\hat{p}_{C}}]_{\times}) & \mathbf{0}_{3x3} & \mathbf{0}_{3x3} & \mathbf{0}_{3x3} & \mathbf{I}_{3x3} & \mathbf{0}_{3x3} & \mathrm{\hat{R}_{G}^{I}}^{T} \end{bmatrix} \begin{bmatrix} ^{I}_{G}\delta{\theta} \\ \tilde{b}_g \\ ^{G}\tilde{v}_{I} \\ \tilde{b}_{a} \\ ^{G}\tilde{p}_{I} \\ _{I}^{C}\delta{\theta} \\ ^{I}p_{C} \end{bmatrix} = \mathbf{J}^{CAM}_{IMU} \mathrm{\tilde{X}^{IMU}_{k+1|k}} \tag{6}
$$
所以整个的协方差矩阵的扩展过程如下：
$$
\mathbf{P}_{k+1 \mid k} \leftarrow\left[\begin{array}{c}
\mathbf{I}_{6 N+21} \\
\mathbf{J}
\end{array}\right] \mathbf{P}_{k+1 \mid k}\left[\begin{array}{c}
\mathbf{I}_{6 N+21} \\
\mathbf{J}
\end{array}\right]^{T} \tag{7}
$$

&nbsp;

----

## MSCKF更新部分

该部分主要探讨新帧的观测是如何影响整个滤波的。

### 何时进行更新

根据参考1中III-E节，MSCKF在两个事件上进行更新操作：

1. 当一个特征点被跟踪丢失的时候，此时特征点的信息不再变化，且此时进行三角化的时候，相机位姿的信息也均是优化多次的位姿，较为准确，**我们称之为情况1**；
2. 当窗口中的相机位姿达到上限的时候，此时势必要把一帧旧的相机位姿滑动出去，此时算法会将所有与该位姿相关的特征点用于更新，最大程度的再次利用这个位姿，**我们称之为情况2**；

在S-MSCKF中，上面的两个部分对应两个函数：

1. removeLostFeatures();
2. pruneCamStateBuffer();

后面会简单看一下实际工程中作者是如何做的。

&nbsp;

### 单个特征点的观测方程

在MSCKF中，如果一个点在k时刻被跟丢了，那么该特征点会用来更新整个看到它的相机位姿。

假设特征点$f_j$在世界坐标系下的位置为${}^{G}P_{f_j}$，于是对于相机$i$，可以通过位姿将其转换到相机坐标系下：
$$
{ }^{C_{i}} \mathbf{p}_{f_{j}}=\left[\begin{array}{c}
{ }^{C_{i}} X_{j} \\
{ }^{C_{i}} Y_{j} \\
{ }^{C_{i}} Z_{j}
\end{array}\right]=\mathbf{C}\left({ }_{G}^{C_{i}} \bar{q}\right)\left({ }^{G} \mathbf{p}_{f_{j}}-{ }^{G} \mathbf{p}_{C_{i}}\right) \tag{8}
$$
于是对应的观测就是：
$$
\mathbf{{z}}_{i}^{(j)}=\frac{1}{{}^{C_{i}}Z_{j}}\left[\begin{array}{c}
{}^{C_{i}} X_{j} \\
{}^{C_{i}} Y_{j}
\end{array}\right]+\mathbf{n}_{i}^{(j)}, \quad i \in \mathcal{S}_{j} \tag{9}
$$
但是到这个地方其实还并不是MSCKF的观测方程，因为MSCKF的估计变量为error-state，所以进一步定义误差：
$$
\begin{aligned}
r^{(j)}_{i}&=z^{(j)}_{i}-\hat{z}^{(j)}_{i}(\mathrm{{x}_i}) \\
&=z^{(j)}_{i}-\hat{z}^{(j)}_{i}(\mathrm{\hat{x}_i}+\mathrm{\tilde{x}_i}) \\
&=z^{(j)}_{i}-(\hat{z}^{(j)}_{i}(\mathrm{\hat{x}_i})+\mathrm{J^{(j)}_{\hat{x}_i}}\mathrm{\tilde{x}_i}+\mathrm{J^{(j)}_{{}^{G}\hat{p}_{f_j}}}\mathrm{{}^{G}\tilde{p}_{f_j}})  \\
&=\mathrm{H}^{(j)}_{\mathrm{\hat{x}}_i}\mathrm{\tilde{x}_i}+\mathrm{H}^{(j)}_{\hat{p}_{f_j}}\mathrm{{}^{G}\tilde{p}_{f_j}}+\mathbf{n}_{i}^{(j)}
\end{aligned}   \tag{10}
$$
上面的公式（10）就是我们需要的对于error-state的观测公式了；

&nbsp;

### 对于观测方程的化简

这一步主要是对观测方程进行变换，主要有两个目的：

1. 将特征点的优化从状态空间中去掉，这样整个状态空间中估计的就只有相机的位姿，大大减少待估计的变量数；
2. 由于MSCKF对特征点采用的是延迟初始化的策略，因此当该特征点被三角化的时候，其值已经足够精准，且如果是情况1的话，这个优化显得费力不讨好；

对于特征点$f_j$而言，假设所有观测到该特征点的相机为$\mathrm{x}_i, i \in [1, N]$，根据公式（10），那么一共有N个residual可以叠加在一起，于是有：
$$
\mathbf{r}^{(j)} \simeq \mathbf{H}_{\mathbf{X}}^{(j)} \widetilde{\mathbf{X}}+\mathbf{H}_{f}^{(j) G} \widetilde{\mathbf{p}}_{f_{j}}+\mathbf{n}^{(j)} \tag{11}
$$
其中：

1. residual的维度为2N x 1；
2. $\mathbf{H}^{(j)}_{\mathbf{X}}$维度为2N x 5M，其中M为滑动窗口中所有的相机数；
3. $\mathbf{H}^{(j)}_{\mathbf{f}}$维度为2N x 3；

容易看到，当N大于2时，$\mathbf{H}^{(j)}_{f}$的秩为3（这里不用过分纠结有没有可能是秩<=3，简单说，只要有重投影误差，那么秩就一定>=3），于是其左零空间的维度为（2N-3），假设为矩阵$\mathbf{A^T}$（左零一定加转置符号哈），其维度为（2N-3 x 2N）所以公式（11）两端均乘以矩阵$\mathbf{A^{T}}$得到：
$$
\begin{aligned}
\mathbf{r}^{(j)}_{o} = \mathbf{A^{T}}\mathbf{r}^{(j)} &\simeq \mathbf{A^{T}}\mathbf{H}_{\mathbf{X}}^{(j)} \widetilde{\mathbf{X}}+\mathbf{A^{T}}\mathbf{H}_{f}^{(j) G} \widetilde{\mathbf{p}}_{f_{j}}+\mathbf{A^{T}}\mathbf{n}^{(j)} \\
&= \mathbf{H}_{o}^{(j)} \widetilde{\mathbf{X}} + \mathbf{n}_{o}^{(j)}
\end{aligned}   \tag{12}
$$
其中：

1. 上式中所有矩阵或者向量的行数均为（2N-3）；

2. 噪声矩阵的协方差满足：
   $$
   E\left\{\mathbf{n}_{o}^{(j)} \mathbf{n}_{o}^{(j) T}\right\}=\sigma_{\mathrm{im}}^{2} \mathbf{A}^{T} \mathbf{A}=\sigma_{\mathrm{im}}^{2} \mathbf{I}_{2 M_{j}-3} \tag{13}
   $$
   

&nbsp;

### 对于观测方程的进一步简化

如公式（12）所示，对于一个特征点而言，该公式的计算维度为$（2\mathrm{N}^{(j)}-3）$，那么对于所有的点而言，该问题的维度为$d=\sum (2\mathrm{N}^{(j)}-3)$。

**特别的，如果该维度大于估计变量（error-state）的维度时**，多出来的维度的优化显得没有必要，所以，当$\mathbf{H_o}$的行数大于列数的时候，作者引入了QR分解来帮助进行优化问题的再简化：
$$
\mathbf{H_o}=\left[Q_1, Q_2\right]\begin{bmatrix}\mathbf{T_H} \\ 0\end{bmatrix} \tag{14}
$$
因为Q矩阵是酉矩阵，所以对于公式（12），可以变为：
$$
\begin{aligned}
\begin{bmatrix}Q_1^T \mathbf{r_o^{(j)}} \\Q_2^T \mathbf{r_o^{(j)}} \end{bmatrix} &= \begin{bmatrix}\mathbf{T_H^{(j)}} \\ 0\end{bmatrix}\widetilde{\mathbf{X}}+\begin{bmatrix}Q_1^T \mathbf{n_o^{(j)}} \\Q_2^T \mathbf{n_o^{(j)}} \end{bmatrix} \\
\Rightarrow \mathbf{r_n^{(j)}} &= \mathbf{T_H^{(j)}}\widetilde{\mathbf{X}}+\mathbf{n_n^{(j)}}
\end{aligned}
\tag{15}
$$
其中：

1. $\mathbf{T_H}$的维度为$(\mathrm{6M+21}) \times (\mathrm{6M+21})$，M为滑动窗口中的相机位姿数目；

2. 对于噪声部分，依旧引用公式（13），依旧可以看到：
   $$
   E\left\{\mathbf{n}_{n}^{(j)} \mathbf{n}_{n}^{(j) T}\right\}=\sigma_{\mathrm{im}}^{2} \mathbf{Q}^{T} \mathbf{Q}=\sigma_{\mathrm{im}}^{2} \mathbf{I}_{6 M+21} \tag{16}
   $$
   

&nbsp;

### 更新方程

有了公式（15）所示的观测方程之后，其实就可以套入KF的更新方程了：
$$
\begin{cases}
\mathbf{K}=\mathbf{PT_{H}^{T}(T_{H}PT_{H}^{T}+R_{n})}^{-1} \\
\mathbf{\Delta{X}}=\mathbf{K}\mathbf{r_n} \\
\mathbf{P}_{k+1|k+1}=(\mathbf{I-KT_H})\mathbf{P}_{k+1|k}(\mathbf{I-KT_H})^{T}+\mathbf{KR_nK^T}
\end{cases}  \tag{17}
$$
可以看到，基本上符合整个KF的更新过程，但是第二个公式和传统的KF的方程似乎不太一样，其实本质上两者是一样的，首先看传统下的KF的计算方程：
$$
\mathbf{X}_{k+1|k+1}=\mathbf{X}_{k+1|k}+\mathbf{K}(\mathrm{z}-\mathrm{z}(\mathbf{X}_{k+1|k})) \tag{18}
$$
对于MSCKF：

1. 由于估计的变量为error-state，所以此时的$\mathbf{X}_{k+1|k}=\mathbf{0}$；

2. 再看观测误差项，这里回溯到公式（10）就可以看出，在$\mathbf{\tilde{X}}_{k+1|k}=\mathbf{0}$的前提下：
   $$
   r^{(j)}_{i}=z^{(j)}_{i}-(\hat{z}^{(j)}_{i}(\mathrm{\hat{x}_i})+\mathrm{J^{(j)}_{\hat{x}_i}}\mathrm{\tilde{x}_i}+\mathrm{J^{(j)}_{{}^{G}\hat{p}_{f_j}}}\mathrm{{}^{G}\tilde{p}_{f_j}})=z^{(j)}_{i}-(\hat{z}^{(j)}_{i}(\mathrm{\hat{x}_i})+\mathrm{J^{(j)}_{\hat{x}_i}}\mathbf{0}+\mathrm{J^{(j)}_{{}^{G}\hat{p}_{f_j}}}\mathbf{0})=z^{(j)}_{i}-\hat{z}^{(j)}_{i}(\mathrm{\hat{x}_i}) \tag{19}
   $$

所以我们看到公式（17）是符合整个KF更新的；

&nbsp;

### 更新方程之后

根据公式（17）计算完整个更新量之后，就可以使用更新方程对normal-state进行更新了。同时注意要reset掉k时刻的估计值，这个地方一般情况下不必对协方差矩阵进行再次的计算，但是实际上，在参考2的第6章中的介绍中，在reset的时候也是需要进行协方差矩阵进行稍微的更新的，只不过一般情况下，更新矩阵近似为单位阵，所以不需要更新。

&nbsp;

----

## 疑惑——观测如何影响IMU的参数的

至此其实整个MSCKF的整个过程都已经整理完毕了，但是笔者一直有一个疑惑：视觉的观测是如何影响IMU参数的？

1. 当删除跟踪丢失的特征点时，因为当前帧也就是k时刻的帧已经没有了观测了，所以观测矩阵中对应项为0，那么观测是如何传导到IMU的参数的？
2. 当删除旧的关键帧的时候，会把与其关联的所有的特征点拿出来进行更新，这其中一定有些特征点会被当前帧观测到（当前帧观测不到的会被直接删掉了），进而产生约束；

综上，笔者开始的时候认为只有当删除旧的关键帧的时候，IMU的参数才会被更新。

直到...笔者将S-MSCKF中的对应的H矩阵以及error-state的更新量打印出来才发现，事情并没有那么简单：其实在删除跟踪丢失的关键点的时候，IMU的参数部分也会变化。但是H矩阵符合预期，在最新帧的部分全为0。

实际上对于MSCKF（或者说对于KF这样的方法）而言，真正链接IMU和之前的状态的因素除了观测的约束，还有一个部分就是协方差。

> 这里简单的记录一下个人的理解：为了方便理解，这里直接用Kalman filter的bayes推导方法
>
> 假设在k时刻有了新的输入值$\mathrm{v}_k$和新的观测值$\mathrm{y}_k$；
>
> 于是k时刻的状态变量的预测值为：
> $$
> p\left(\mathbf{x}_{k} \mid \check{\mathbf{x}}_{0}, \mathbf{v}_{1: k}, \mathbf{y}_{0: k-1}\right)=\mathcal{N}\left(\check{\mathbf{x}}_{k}, \check{\mathbf{P}}_{k}\right) \tag{20}
> $$
> 其中：
> $$
> \begin{cases}
> \begin{aligned}
> \check{\mathbf{P}}_{k} &=\mathbf{A}_{k-1} \hat{\mathbf{P}}_{k-1} \mathbf{A}_{k-1}^{T}+\mathbf{Q}_{k} \\
> \check{\mathbf{x}}_{k} &=\mathbf{A}_{k-1} \hat{\mathbf{x}}_{k-1}+\mathbf{v}_{k}
> \end{aligned}
> \end{cases} \tag{21}
> $$
> 对于k时刻的观测值$\mathrm{y}_k$，给出其与状态变量的联合概率分布：
> $$
> \begin{aligned}
> p\left(\mathbf{x}_{k}, \mathbf{y}_{k} \mid \check{\mathbf{x}}_{0}, \mathbf{v}_{1: k}, \mathbf{y}_{0: k-1}\right) &=\mathcal{N}\left(\left[\begin{array}{c}
> \boldsymbol{\mu}_{x} \\
> \boldsymbol{\mu}_{y}
> \end{array}\right],\left[\begin{array}{cc}
> \boldsymbol{\Sigma}_{x x} & \boldsymbol{\Sigma}_{x y} \\
> \boldsymbol{\Sigma}_{y x} & \boldsymbol{\Sigma}_{y y}
> \end{array}\right]\right) \\
> &=\mathcal{N}\left(\left[\begin{array}{c}
> \check{\mathbf{x}}_{k} \\
> \mathbf{C}_{k} \check{\mathbf{x}}_{k}
> \end{array}\right],\left[\begin{array}{cc}
> \tilde{\mathbf{P}}_{k} & \check{\mathbf{P}}_{k} \mathbf{C}_{k}^{T} \\
> \mathbf{C}_{k} \mathbf{P}_{k} & \mathbf{C}_{k} \mathbf{P}_{k} \mathbf{C}_{k}^{T}+\mathbf{R}_{k}
> \end{array}\right]\right)
> \end{aligned}  \tag{22}
> $$
> 对公式（22）所表示的概率对观测进行边缘化得到：
> $$
> p(\mathrm{x}_k|\check{\mathrm{x}}_0,\mathbf{v}_{1: k}, \mathbf{y}_{0: k})=\mathcal{N}(\underbrace{\boldsymbol{\mu}_{x}+\mathbf{\Sigma}_{x y} \boldsymbol{\Sigma}_{y y}^{-1}\left(\mathbf{y}_{k}-\boldsymbol{\mu}_{y}\right)}_{\hat{\mathbf{x}}_{k}},  \underbrace{\boldsymbol{\Sigma}_{x x}-\boldsymbol{\Sigma}_{x y} \boldsymbol{\Sigma}_{y y}^{-1} \boldsymbol{\Sigma}_{y x}}_{\hat{\mathbf{P}}_{k}})  \tag{23}
> $$
> 公式（23）中的$\Sigma_{xy}\Sigma_{yy}^{-1}$对应的就是KF中的增益矩阵K。
>
> 可以看到，联合概率分布中，观测与状态变量的协方差$\Sigma_{xy}$既与观测矩阵C有关，同时也与协方差矩阵P有关。
>
> **对于MSCKF框架而言，在滑动窗口中的相机位姿与IMU参数的协方差其实一直都不为0，理论上来说，每一帧相机位姿其实都来源于IMU，所以两者之间必然是联系起来的**。

&nbsp;

-----

## 总结

本文总结了MSCKF的更新过程，整体上算法还是在按照KF的思路（更确切的说在按照ESKF的思路）在走，整个过程也比较好理解。

下一个专题主要想深入的分析一下整个优化过程可观性的分析，算是更深入的对优化问题中的细节部分进行理解。