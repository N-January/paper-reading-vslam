# The consistency of Visual-Inertial Odometry

---
[toc]
&nbsp;

---
## 写在前面
讲实话，笔者之前就听说过很多关于SLAM系统的能观性、或者说一致性的分析，最开始的切入点当然是大名鼎鼎的First-Estimate-Jacobian（FEJ）技术，且当时更多接触的是Graph-Base的方法，当时就直观感受而言，仅仅觉得FEJ固定了求解的方向，导致整个优化问题的信息矩阵在求解的时候是固定的，由此来解决由于信息矩阵的变化导致过度估计的问题。

但是上述仅仅是直观的一些想法，并没有理论做依据，之前也零星的看过黄老师关于FEJ的文章，但是公式太多，个人水平也很有限，所以不能很好的理解其中的精髓。

这次借着阅读整理MSCKF的机会，更深一步的理解一下如何分析系统的能观性以及FEJ到底在解决什么，参考更多的其实是李明扬大佬的一些文章和MSCKF2.0的工作。

&nbsp;

---

## Reference

1. Consistency of EKF-Based Visual-Inertial Odometry. 关于MSCKF一致性的分析，也是本文主要参考的论文；
2. Analysis and Improvement of the Consistency of Extended Kalman Filter based SLAM. 黄老师关于EKF一致性的分析；
3. Generalized Analysis and Improvement of the Consistency of EKF-based SLAM. 黄老师同年发表的一篇更长的关于一致性的文章，可以认为是参考2的详细版；

&nbsp;

---

## Notation

本文中公式比较多，所以Notation也比较多，先对Notation进行一些前置说明：

1. $l$ 节点的理想值表示为$\mathrm{x}_{l}$；在 k 时刻的估计值表示为$\mathrm{\hat{x}}_{l}^{(k)}$；其误差状态表示为$\mathrm{\tilde{x}}_{l}$；
2. $l$ 节点的估计值在不同时刻的关系为：$\mathbf{\hat{x}}^{(k+n)}_{l}={}^{(k+n)}\mathbf{\tilde{x}}^{(k)}_{l}+\mathbf{\hat{x}}^{(k)}_{l}$，特别的，对于旋转有：${}^{l}_{G}\mathbf{\hat{q}}^{(k+n)}\approx (\mathbf{I}+\lfloor {}^{(k+n)}\theta^{(k)}_{l}\rfloor_{\times}){}^{l}_{G}\mathbf{\hat{q}}^{(k)}$；

由于整片文章公式太多了，其中Notation有些地方可能不是很对笔者也没有检查出来，同时后面为了简化还有些Notation的复用，但是笔者都进行了说明，有问题希望及时提出，笔者及时改正；

&nbsp;

----

## IMU误差状态传播公式的回顾

简单的回顾一下MSCKF对于IMU误差状态（error-state）的传播过程：

1. 通过IMU运动方程得到误差状态的微分方程：
   $$
   \dot{\tilde{{X}}}=\mathbf{F}(\hat{{X}})\tilde{{X}}+\mathbf{G}n \tag{1}
   $$

2. 通过线性系统的离散化得到IMU误差状态递方程的闭式解：
   $$
   \boldsymbol{\tilde{X}}\left(t_{k+1}\right)=\boldsymbol{\Phi}\left(t_{k+1}, t_{k}\right) \boldsymbol{\tilde{X}}\left(t_{k}\right)+\int_{t_{k}}^{t_{k+1}} \boldsymbol{\Phi}\left(t_{k+1}, \tau\right) \boldsymbol{G}(\tau) \boldsymbol{n}(\tau) \mathrm{d} \tau \tag{2}
   $$
   其中$\boldsymbol{\Phi}\left(t_{k+1}, t_{k}\right)=\exp \int_{t_{k}}^{t_{k+1}} \boldsymbol{F}(t) \mathrm{d} t$
   

对于上述的推导过程，我们容易看到，整个推导过程建立在数值分析的层次，也就是闭式解本身是一个数值解，当然有的小伙伴会认为如果假设一段时间内微分量是不变的，直接乘以时间是不是就有理论意义？诚然，笔者认为这样的方法可以，但是不精确，不精确的解去分析整个系统的特性带来的结果必然也是不精确的；所以在最开始，我们需要从理论的角度来重新推导整个IMU误差状态的传导过程。

&nbsp;
---

---

## IMU误差状态传播公式的再推导

如上所述，本节主要是对误差状态传播公式的理论推导，这里主要分析三个参数的传播公式：
1. 旋转部分；
2. 速度部分；
3. 位移部分；

bias部分因为不涉及到能观性的问题，所以这里暂不引入，实际上，引入之后得到的结论也是一样的，后续会稍微提到一下。

>注：以下公式均在时间为${l}$的时刻。

&nbsp;

### 旋转部分的推导

主要依赖两个基本的公式：
$$
\begin{cases}
\begin{aligned}
{}^{I_{l+1}}_{G}R^{(l)}&={}^{I_{l+1}}_{I_{l}}R^{(l)} \quad{}^{I_{l}}_{G}R^{(l)} \\
{}^{I_{l}}_{G}R^{(l)}&=(\mathbf{I}-\left[ {}^{I_{l}}\tilde{\theta} \right]_{\times}){}^{I_{l}}_{G}\hat{R}^{(l)}
\end{aligned}
\end{cases} \tag{3}
$$
将第二行的公式带入到第一行中可以得到：
$$
\begin{aligned}
{}^{I_{l+1}}_{G}R^{(l)}&={}^{I_{l+1}}_{I_{l}}R^{(l)} \quad{}^{I_{l}}_{G}R^{(l)} \\
(\mathbf{I}-\left[ {}^{I_{l+1}}\tilde{\theta} \right]_{\times}){}^{I_{l+1}}_{G}\hat{R}^{(l)}&=(\mathbf{I}-\left[ {}^{I_{l+1}}_{I_{l}}\tilde{\theta} \right]_{\times}){}^{I_{l+1}}_{I_{l}}\hat{R}^{(l)}\quad (\mathbf{I}-\left[ {}^{I_{l}}\tilde{\theta} \right]_{\times}){}^{I_{l}}_{G}\hat{R}^{(l)} \\
{}^{I_{l+1}}_{G}\hat{R}^{(l)}-(\left[ {}^{I_{l+1}}\tilde{\theta} \right]_{\times}){}^{I_{l+1}}_{G}\hat{R}^{(l)}&={}^{I_{l+1}}_{I_{l}}\hat{R}^{(l)}{}^{I_{l}}_{G}\hat{R}^{(l)} -(\left[ {}^{I_{l+1}}_{I_{l}}\tilde{\theta} \right]_{\times}){}^{I_{l+1}}_{I_{l}}\hat{R}^{(l)}{}^{I_{l}}_{G}\hat{R}^{(l)} - {}^{I_{l+1}}_{I_{l}}\hat{R}^{(l)}(\left[ {}^{I_{l}}\tilde{\theta} \right]_{\times}){}^{I_{l}}_{G}\hat{R}^{(l)}+\mathbf{O(2)} \\
\left[ {}^{I_{l+1}}\tilde{\theta} \right]_{\times} &\approx \left[ {}^{I_{l+1}}_{I_{l}}\tilde{\theta} \right]_{\times}+\underbrace{{}^{I_{l+1}}_{I_{l}}\hat{R}^{(l)}(\left[ {}^{I_{l}}\tilde{\theta} \right]_{\times}){}^{I_{l}}_{I_{l+1}}\hat{R}^{(l)}}_{\left[ {}^{I_{l+1}}_{I_{l}}\hat{R}^{(l)}{}^{I_{l}}\tilde{\theta} \right]_{\times}} \\
\end{aligned} \tag{4}
$$
对公式（4）的最后一行使用vee操作可得：
$$
{}^{I_{l+1}}\tilde{\theta}={}^{I_{l+1}}_{I_{l}}\tilde{\theta} + {}^{I_{l+1}}_{I_{l}}\hat{R}^{(l)}{}^{I_{l}}\tilde{\theta} \tag{5}
$$
&nbsp;

### 速度部分的推导

依赖三个基本公式：
$$
\begin{cases}
\begin{aligned}
{}^{G}v_{I_{l+1}}^{(l)} &= {}^{G}v^{(l)} + ({}^{I_{l}}_{G}R^{(l)})^{T} \int_{t_{l}}^{t_{l+1}} ({}^{I_{\tau}}_{I_{l}}R)^{T} \mathbf{a_m} d \tau + g \Delta{t} \\
{}^{G}v_{I_{l}}^{(l)}&={}^{G}\hat{v}_{I_{l}}^{(l)}+{}^{G}\tilde{v}_{I_{l}}^{(l)} \\
{}^{I_{l}}_{G}R^{(l)}&=(\mathbf{I}-\left[ {}^{I_{l}}\tilde{\theta} \right]_{\times}){}^{I_{l}}_{G}\hat{R}^{(l)}
\end{aligned}
\end{cases} \tag{6}
$$
把公式（6）中的第二行和第三行带入第一行之后得到：
$$
\begin{aligned}
{}^{G}v_{I_{l+1}}^{(l)} &= {}^{G}v^{(l)} + ({}^{I_{l}}_{G}R^{(l)})^{T} \int_{t_{l}}^{t_{l+1}} ({}^{I_{\tau}}_{I_{l}}R)^{T} \mathbf{a_m} d \tau + g \Delta{t} \\
{}^{G}\hat{v}_{I_{l+1}}^{(l)}+{}^{G}\tilde{v}_{I_{l+1}}^{(l)} &= {}^{G}\hat{v}_{I_{l}}^{(l)}+{}^{G}\tilde{v}_{I_{l}}^{(l)} + \{(\mathbf{I}-\left[ {}^{I_{l}}\tilde{\theta} \right]_{\times}){}^{I_{l}}_{G}\hat{R}^{(l)}\}^{T} \int_{t_{l}}^{t_{l+1}} \{(\mathbf{I}-\left[{}^{I_{\tau}}\tilde{\theta}\right]_{\times}){}^{I_{\tau}}_{I_{l}}\hat{R}\}^{T} \mathbf{a_m} d \tau + g \Delta{t} \\
&={}^{G}\hat{v}_{I_{l}}^{(l)} + ({}^{I_{l}}_{G}\hat{R}^{(l)})^{T}\int_{t_{l}}^{t_{l+1}} ({}^{I_{\tau}}_{I_{l}}\hat{R})^{T} \mathbf{a_m} d \tau + g \Delta{t}\\ 
&+{}^{G}\tilde{v}_{I_{l}}^{(l)} +\{(-\left[{}^{I_{\tau}}\tilde{\theta}\right]_{\times}){}^{I_{\tau}}_{I_{l}}\hat{R}\}^{T}\int_{t_{l}}^{t_{l+1}} ({}^{I_{\tau}}_{I_{l}}\hat{R})^{T} \mathbf{a_m} d \tau+({}^{I_{l}}_{G}\hat{R}^{(l)})^{T}\int_{t_{l}}^{t_{l+1}} \{(-\left[{}^{I_{\tau}}\tilde{\theta}\right]_{\times}){}^{I_{\tau}}_{I_{l}}\hat{R}\}^{T} \mathbf{a_m} d \tau + \mathbf{O(2)} 
\end{aligned} \tag{7}
$$
此时我们将积分值进行替换，规则如下：
$$
\begin{aligned}
\mathrm{\hat{s}}^{(l)}&=\int_{t_{l}}^{t_{l+1}} ({}^{I_{\tau}}_{I_{l}}\hat{R})^{T} \mathbf{a_m} d \tau = {}^{I_{l}}_{G}R^{(l)}({}^{G}\hat{v}_{l+1}^{(l)}-{}^{G}\hat{v}_{l}^{(l)}-g \Delta{t}) \\
\mathrm{\tilde{s}}^{(l)} &= \int_{t_{l}}^{t_{l+1}} \{(-\left[{}^{I_{\tau}}\tilde{\theta}\right]_{\times}){}^{I_{\tau}}_{I_{l}}\hat{R}\}^{T} \mathbf{a_m} d \tau 
\end{aligned}  \tag{8}
$$
公式（8）中的等价关系由公式（6）的第一行推导出，将公式（8）带入到公式（7）中可以得到：
$$
{}^{G}\tilde{v}_{I_{l+1}}^{(l)}={}^{G}\tilde{v}_{I_{l}}^{(l)}-({}^{I_{l}}_{G}\hat{R}^{(l)})^{T}\left[\hat{\mathrm{s}}^{(l)}\right]_{\times} {}^{I_{l}}\tilde{\theta}+({}^{I_{l}}_{G}\hat{R}^{(l)})^{T} \tilde{\mathrm{s}}^{(l)} \tag{9}
$$
&nbsp;

### 位移部分的推导

与速度相同，这部分推导依赖四个基础公式：
$$
\begin{cases}
\begin{aligned}
{}^{G}p_{I_{l+1}}^{(l)} &= {}^{G}p_{I_l}^{(l)} + {}^{G}v_{I_l}^{(l)}\Delta{t}+ ({}^{I_{l}}_{G}R^{(l)})^{T} \int_{t_{l}}^{t_{l+1}} \int_{t_{l}}^{s} ({}^{I_{\tau}}_{I_{l}}R)^{T} \mathbf{a_m} d \tau d s + \frac{1}{2} g \Delta{t}^2 \\
{}^{G}p_{I_{l}}^{(l)}&={}^{G}\hat{p}_{I_{l}}^{(l)}+{}^{G}\tilde{p}_{I_{l}}^{(l)} \\
{}^{G}v_{I_{l}}^{(l)}&={}^{G}\hat{v}_{I_{l}}^{(l)}+{}^{G}\tilde{v}_{I_{l}}^{(l)} \\
{}^{I_{l}}_{G}R^{(l)}&=(\mathbf{I}-\left[ {}^{I_{l}}\tilde{\theta} \right]_{\times}){}^{I_{l}}_{G}\hat{R}^{(l)}
\end{aligned}
\end{cases} \tag{10}
$$
同样按照速度的推理，可以得到如下结论：
$$
{}^{G}\tilde{p}_{I_{l+1}}^{(l)}={}^{G}\tilde{p}_{I_{l}}^{(l)}+{}^{G}\tilde{v}_{I_l}^{(l)}\Delta{t}-({}^{I_{l}}_{G}\hat{R}^{(l)})^{T}\left[\hat{\mathrm{y}}^{(l)}\right]_{\times} {}^{I_{l}}\tilde{\theta}+({}^{I_{l}}_{G}\hat{R}^{(l)})^{T} \tilde{\mathrm{y}}^{(l)} \tag{11}
$$
其中的变量$\mathrm{y}$和速度中的一样，也是一个替换变量：
$$
\begin{aligned}
\mathrm{\hat{y}}^{(l)}&=\int_{t_{l}}^{t_{l+1}} \int_{t_l}^{s} ({}^{I_{\tau}}_{I_{l}}\hat{R})^{T} \mathbf{a_m} d \tau d s = {}^{I_{l}}_{G}R^{(l)}({}^{G}\hat{p}_{l+1}^{(l)}-{}^{G}\hat{p}_{l}^{(l)}-{}^{G}v_{I_l}^{(l)}\Delta{t}-\frac{1}{2}g \Delta{t}^2) \\
\mathrm{\tilde{y}}^{(l)} &= \int_{t_{l}}^{t_{l+1}} \int_{t_l}^{s} \{(-\left[{}^{I_{\tau}}\tilde{\theta}\right]_{\times}){}^{I_{\tau}}_{I_{l}}\hat{R}\}^{T} \mathbf{a_m} d \tau d s 
\end{aligned}  \tag{12}
$$
&nbsp;

### 状态传递的传递方程

结合公式（5）（9）（12）可以写出在 $l$ 时刻的误差状态的传递方程，对应于KF的预测部分；
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
\end{bmatrix} \tag{13}
$$
上式可以写作状态转移方程为：
$$
{}^{I_{l+1}}_{G}\tilde{\mathrm{x}}^{(l)}= \Phi(\mathrm{x}_{I_{l+1}}^{(l)}, \mathrm{x}_{I_l}^{(l)}) {}^{I_{l}}_{G}\tilde{\mathrm{x}}^{(l)}+\mathrm{w}^{(l)} \tag{14}
$$

这里有必要引用原文中的一些介绍：以位移的误差状态量的递推公式为例，相当于在原先误差状态变量的基础上加上速度的变化，之后角速度的变化与一个杆臂$-({}^{I_l}_{G}R)^{T}\left[\hat{\mathrm{y}}^{(l)}\right]$相乘来影响位移的误差状态。所以对于公式（14）表示的状态转移过程而言，整个过程也具有一定的物理意义。

&nbsp;

---

## MSCKF的观测模型

下面的所有的下角标 $l$ 表示id为 $l$ 的相机，不表示时间，这里暂时不涉及时间，可以认为是理想的观测模型。

为了分析能观性，这里还需要一个步骤就是观测模型，以单个观测点$P_{f_j}$为例，其观测模型为：
$$
\begin{aligned}
z_l&=\pi({}^{C_l}\mathrm{p}_{f_j})+n_{l} \\
{}^{C_l}\mathrm{p}_{f_j}&={}^{C}_{I}R {}^{G}_{l}R({}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_l})+{}^{C}\mathrm{p}_I 
\end{aligned} \tag{15}
$$

所以观测模型为：
$$
\begin{aligned}
H_{(I_l|l)}&=J_{(f_j|l)} \quad {}^{C}_{I}R \quad {}_{G}^{I_l}R\begin{bmatrix} \underbrace{\left[{}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_i}\right]_{\times}({}_{G}^{I_l}R)^{T}}_{{\partial e}/{\partial \theta}} & \underbrace{ -\mathbf{I}_{3\times3}}_{{\partial e}/{\partial \mathrm{p}}} & \underbrace{ \mathbf{0}_{3\times3}}_{{\partial e}/{\partial \mathrm{v}}}\end{bmatrix} \\ 
H_{(f_j|l)}&=J_{(f_j|l)} \quad {}^{C}_{I}R \quad {}_{G}^{I_l}R
\end{aligned} \tag{16}
$$
其中：
$$
J_{(f_j|l)}=\frac{1}{Z}\begin{bmatrix}1 & 0 & -\frac{X}{Z} \\ 0 & 1 & -\frac{Y}{Z} \end{bmatrix}
$$
&nbsp;

----

## 能观性分析

能观性的分析主要依赖于能观性矩阵：
$$
\mathcal{O}=\begin{bmatrix} \mathrm{H}_k \\ \mathrm{H}_{k+1}\Phi_{k} \\ \vdots \\ \mathrm{H}_{k+m}\Phi_{k+m-1} \dots\Phi_{k} \end{bmatrix} \tag{17}
$$
&nbsp;

------

## 理想情况下的能观性矩阵

这里先进行理想情况下的能观性矩阵的推导，这个部分的所有变量均使用理想情况下的状态值（公式上来讲就是$\mathbf{X}_{I_l}^{(l-1)}=\mathbf{X}_{I_l}^{(l)}=\dots=\mathbf{X}_{I_l}^{l+m}$），也就是状态变量从预测出来的时候，就是真值了，后面一直都不变了（提前剧透了FEJ  -.-!!!，不过还是稍有不同，后面会解释）。

于是将公式（13）表示的状态转移矩阵与公式（16）表示的观测矩阵带入公式（17）就可以得到能观性矩阵，这里以 $l$ 时刻为例，有：
$$
\mathcal{O}_{l}=\mathrm{H}_{l}\Phi_{l-1}\Phi_{l-2}\dots\Phi_{k} \tag{18}
$$
这里整个推导十分麻烦，但是好在只要计算前两次乘法我们就能找到规律，下面分别对两次乘法进行展开：
$$
\begin{aligned}
\mathrm{H}_{l}\Phi_{l-1}&=J_{(f_j|l)} \quad {}^{C}_{I}R \quad {}_{G}^{I_l}R
\left\{\underbrace{\begin{bmatrix} \left[{}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_l}\right]_{\times}({}_{G}^{I_l}R)^{T} & -\mathbf{I}_{3\times3} & \mathbf{0}_{3\times3}\end{bmatrix}\Phi_{l-1}}_{S0} \quad \dots \quad \mathbf{I} \quad \dots \quad \mathbf{0} \right\}
\end{aligned} \tag{19}
$$
可以看到，整个乘积的结果其实更加依赖的是IMU的观测部分与状态转移矩阵的乘积部分，也就是上式中的S0部分，下面单独展开：
$$
\begin{aligned}
S0&=
\begin{bmatrix} 
\left[{}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_l}\right]_{\times}({}_{G}^{I_l}R)^{T} & -\mathbf{I}_{3\times3} & \mathbf{0}_{3\times3}
\end{bmatrix}
\begin{bmatrix} 
{}^{I_{l}}_{I_{l-1}}R & \mathbf{0} & \mathbf{0} \\
-({}^{I_{l-1}}_{G}R)^{T}\left[{\mathrm{y}}_{l-1}\right]_{\times} & \mathbf{I} & \mathbf{I}\Delta{t}_{l-1} \\
-({}^{I_{l-1}}_{G}R)^{T}\left[{\mathrm{s}}_{l-1}\right]_{\times} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \\
&=\begin{bmatrix}
\left[{}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_l}\right]_{\times}({}_{G}^{I_{l-1}}R)^{T}+({}^{I_{l-1}}_{G}R)^{T}\left[{\mathrm{y}}_{l-1}\right]_{\times} &
-\mathbf{I} & -\mathbf{I}\Delta{t}_{l-1}
\end{bmatrix} \\ 
&=
\begin{bmatrix}
\left[{}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_l}\right]_{\times}({}_{G}^{I_{l-1}}R)^{T}+\left[ {}^{G}{p}_{l}-{}^{G}{p}_{l-1}-{}^{G}v_{I_{l-1}}\Delta{t}_{l-1}-\frac{1}{2}g \Delta{t}_{l-1}^2 \right]_{\times}({}_{G}^{I_{l-1}}R)^{T} & -\mathbf{I} & -\mathbf{I}\Delta{t}_{l-1} \\
\end{bmatrix} \\
&=
\begin{bmatrix}
\left[{}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_{l-1}}-{}^{G}v_{I_{l-1}}\Delta{t}_{l-1}-\frac{1}{2}g \Delta{t}_{l-1}^2\right]_{\times}({}_{G}^{I_{l-1}}R)^{T} & -\mathbf{I} & -\mathbf{I}\Delta{t}_{l-1}
\end{bmatrix}
\end{aligned} \tag{20}
$$
其中第三行的展开中用到了反对称矩阵的性质：若矩阵R是特殊正交群SO(3)，则有$R[A]_{\times}R^T=[RA]_{\times}$的变换。

接着继续看公式（18）中与$\Phi_{l-2}$的乘积：
$$
\begin{aligned}
S1&=S0\Phi_{l-2}  \\
&=\begin{bmatrix}
\underbrace{\left[{}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_{l-1}}-{}^{G}v_{I_{l-1}}\Delta{t}_{l-1}-\frac{1}{2}g \Delta{t}_{l-1}^2\right]_{\times}}_{m_{l-1}}({}_{G}^{I_{l-1}}R)^{T} & -\mathbf{I} & -\mathbf{I}\Delta{t}_{l-1}
\end{bmatrix}
\begin{bmatrix} 
{}^{I_{l-1}}_{I_{l-2}}R & \mathbf{0} & \mathbf{0} \\
-({}^{I_{l-2}}_{G}R)^{T}\left[{\mathrm{y}}_{l-2}\right]_{\times} & \mathbf{I} & \mathbf{I}\Delta{t}_{l-2} \\
-({}^{I_{l-2}}_{G}R)^{T}\left[{\mathrm{s}}_{l-2}\right]_{\times} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \\
&=
\begin{bmatrix}
(m_{l-1}+\left[{}^{G}{p}_{I_{l-1}}-{}^{G}{p}_{I_{{l-2}}}-{}^{G}v_{I_{l-2}}\Delta{t}_{l-2}-\frac{1}{2}g \Delta{t}_{l-2}^2\right]_{\times}+
\left[{}^{G}{v}_{l-1}-{}^{G}{v}_{l-2}-g \Delta{t}_{l-2}\right]_{\times}\Delta{t}_{l-1})({}_{G}^{I_{l-2}}R)^{T} \\ -\mathbf{I} \\ -\mathbf{I}(\Delta{t}_{l-2}+\Delta{t}_{l-1})
\end{bmatrix}^{T} \\ 
\end{aligned} \tag{21}
$$
把上式中的第一行拿出来分析：
$$
\begin{aligned}
S1_1
&={}^{G}\mathrm{p}_{f_j}-{}^{G}\mathrm{p}_{I_{l-1}}-{}^{G}v_{I_{l-1}}\Delta{t}_{l-1}-\frac{1}{2}g \Delta{t}_{l-1}^2 \\
&+{}^{G}{p}_{I_{l-1}}-{}^{G}{p}_{I_{l-2}}-{}^{G}v_{I_{l-2}}\Delta{t}_{l-2}-\frac{1}{2}g \Delta{t}_{l-2}^2 \\
&+({}^{G}{v}_{l-1}-{}^{G}{v}_{l-2}-g \Delta{t}_{l-2})\Delta{t}_{l-1} \\
&={}^{G}\mathrm{p}_{f_j}-{}^{G}{p}_{I_{l-2}}-{}^{G}v_{l-2}(\Delta{t}_{l-1}+\Delta{t}_{t-2})-\frac{1}{2}g(\Delta{t}_{l-1}^2+2\Delta{t}_{t-1}\Delta{t}_{t-2}+\Delta{t}_{t-2}^2) \\
&={}^{G}\mathrm{p}_{f_j}-{}^{G}{p}_{I_{l-2}}-{}^{G}v_{l-2}(\Delta{t}_{l-1}+\Delta{t}_{t-2})-\frac{1}{2}g(\Delta{t}_{l-1}+\Delta{t}_{t-2})^2 
\end{aligned} \tag{21A}
$$
上式带入公式（21）可以得到：
$$
S1=
\begin{bmatrix}
\left[{}^{G}\mathrm{p}_{f_j}-{}^{G}{p}_{I_{l-2}}-{}^{G}v_{l-2}(\Delta{t}_{l-1}+\Delta{t}_{t-2})-\frac{1}{2}g(\Delta{t}_{l-1}+\Delta{t}_{t-2})^2 \right]_{\times}({}_{G}^{I_{l-2}}R)^{T} \\
-\mathbf{I} \\ -\mathbf{I}(\Delta{t}_{l-2}+\Delta{t}_{l-1})
\end{bmatrix}^{T} \tag{21B}
$$
后面的情况均类似，所以由归纳法可以得到：
$$
\mathcal{O}_{l}=J_{(f_j|l)}{}^{C}_{I}R{}_{G}^{I_l}R
\begin{bmatrix} \underbrace{\left[{}^{G}\mathrm{p}_{f_j}-{}^{G}{p}_{I_{k}}-{}^{G}v_{k}\Delta{t}-\frac{1}{2}g\Delta{t}^2 \right]_{\times}({}_{G}^{I_{k}}R)^{T}, -\mathbf{I},  -\mathbf{I}\Delta{t}}_{IMU} & \underbrace{\dots , \mathbf{I} , \dots , \mathbf{0}}_{feature} \end{bmatrix} \tag{22}
$$
其中$\Delta{t}=\Delta{t}_{l-1}+\dots+\Delta{t}_{k}$。

 容易看出（不是论文这么说真心凑不出啊），在理想情况下，能观性矩阵的零空间为：
$$
\mathbf{N}=\left[\begin{array}{cc}
\mathbf{0}_{3} & {}^{I_k}_{G}\mathbf{R} \mathbf{g} \\
\mathbf{I}_{3} & -\left[^{G} \mathbf{p}_{k}\right]_\times{\mathbf{g}} \\
\mathbf{0}_{3} & -\left[^{G} \mathbf{v}_{k}\right]_\times{\mathbf{g}} \\
\mathbf{I}_{3} & -\left[^{G} \mathbf{p}_{f_1}\right]_\times{\mathbf{g}} \\
\mathbf{I}_{3} & -\left[^{G} \mathbf{p}_{f_2}\right]_\times{\mathbf{g}} \\
\vdots & \vdots \\
\mathbf{I}_{3} & -\left[^{G} \mathbf{p}_{f_N}\right]_\times{\mathbf{g}}
\end{array}\right] \tag{23}
$$
读者可以将公式（22）与公式（23）相乘，发现均为0，其中最后的g其实是为了凑两个g的叉乘而存在的。

&nbsp;

### 小结

在求解理想情况下的能观性矩阵时，主要利用了如下几点，这几点也是对于启发FEJ非常重要的几点：

1. 从公式（20）和公式（21）的旋转矩阵相乘可以看到，由于是理想情况，因此旋转矩阵可以直接消去 $l-1$ 时刻的影响：
   $$
   ({}^{I_{l-2}}_{G}R)^{T}=({}^{I_{l-1}}_{G}R)^{T}{}^{I_{l-1}}_{I_{l-2}}R \tag{24}
   $$

2. 由公式（21）及其附属推导可以看到，整个化简的关键在于位移和速度的量不变化；

&nbsp;

----

## 实际情况下的能观性矩阵

下面终于要分析一下实际的能观性矩阵的东西了。

假设程序运行到了$\alpha_{i+1}$时刻，那么对于第 $l$ 个位姿节点（因为MSCKF在维护一个窗口，窗口内的参数都是会更新的）能观性矩阵的组成部分来说：

1. 状态转移矩阵为$\Phi_{l-1}(\mathrm{\hat{x}}_{I_l}^{(l-1)}, \mathrm{\hat{x}}_{I_{l-1}}^{(l-1)})$，$\Phi_{l-2}(\mathrm{\hat{x}}_{I_{l-1}}^{(l-2)}, \mathrm{\hat{x}}_{I_{l-2}}^{(l-2)}$，$\dots$，$\Phi_{k}(\mathrm{\hat{x}}_{k+1}^{(k)}, \mathrm{\hat{x}}_{I_{k}}^{(k)})$，这部分因为是历史递推过来的，所以**不能**受到后来更新的影响；
2. 依旧对于特征点$P_{f_j} $ ，观测矩阵$\mathbf{H}_{l|\alpha_i}=\partial{\mathbf{e}^{(\alpha_i+1)}_{l}}/\partial{\mathrm{\hat{x}}_{l}^{(\alpha_i)}}$，此时公式中使用了 $l$ 节点最新的估计值，这里有读者可能比较疑惑为什么变量使用的是$\alpha_i$时刻的值，**因为在$\alpha_i+1$时刻，在窗口中的变量还没有被最新的观测更新**，所以在线性化的时候，值依旧是上一次被更新的值，也就是$\mathrm{\hat{x}}_{l}^{\alpha_i}$；

针对上面的两个点，这里写出他们的具体形式：

#### 状态转移矩阵

类比公式（13）可得：
$$
\Phi_{l-1}(\mathrm{\hat{x}}_{I_l}^{(l-1)}, \mathrm{\hat{x}}_{I_{l-1}}^{(l-1)})=
\begin{bmatrix} 
{}^{I_{l}}_{I_{l-1}}R^{(l-1)} & \mathbf{0} & \mathbf{0} \\
-({}^{I_{l-1}}_{G}R^{(l-1)})^{T}\left[\hat{\mathrm{y}}^{(l-1)}_{l-1}\right]_{\times} & \mathbf{I} & \mathbf{I}\Delta{t}_{l-1} \\
-({}^{I_{l-1}}_{G}R^{(l-1)})^{T}\left[\hat{\mathrm{s}}^{(l-1)}_{l-1}\right]_{\times} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \tag{25}
$$

#### 观测矩阵

类比公式（16），这里直接写出综合形式：
$$
\mathrm{H}_{l}^{(\alpha_i)}=J_{(f_j|l)} {}^{C}_{I}R {}_{G}^{I_l}R^{(\alpha_{i})}
\left\{\begin{bmatrix} \left[{}^{G}\mathrm{\hat{p}}_{f_j}-{}^{G}\mathrm{\hat{p}}_{I_l}^{(\alpha_i)}\right]_{\times}({}_{G}^{I_l}R^{(\alpha_i)})^{T} & -\mathbf{I}_{3\times3} & \mathbf{0}_{3\times3}\end{bmatrix} \quad \dots \quad \mathbf{I} \quad \dots \quad \mathbf{0} \right\} \tag{26}
$$
&nbsp;

#### 能观矩阵的构建

这里的推导就超级麻烦了，好在我们依旧可以从前两次乘法运算之后看到问题所在，因此类比于公式（19），先展开第一个乘积：
$$
\begin{aligned}
M0^{(\alpha_i)}&=\begin{bmatrix} \left[{}^{G}\mathrm{\hat{p}}_{f_j}-{}^{G}\mathrm{\hat{p}}_{I_l}^{(\alpha_i)}\right]_{\times}({}_{G}^{I_l}R^{(\alpha_i)})^{T} & -\mathbf{I}_{3\times3} & \mathbf{0}_{3\times3}\end{bmatrix}
\begin{bmatrix} 
{}^{I_{l}}_{I_{l-1}}R^{(l-1)} & \mathbf{0} & \mathbf{0} \\
-({}^{I_{l-1}}_{G}R^{(l-1)})^{T}\left[\hat{\mathrm{y}}^{(l-1)}_{l-1}\right]_{\times} & \mathbf{I} & \mathbf{I}\Delta{t}_{l-1} \\
-({}^{I_{l-1}}_{G}R^{(l-1)})^{T}\left[\hat{\mathrm{s}}^{(l-1)}_{l-1}\right]_{\times} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \\
&=
\begin{bmatrix}
\left[{}^{G}\mathrm{\hat{p}}_{f_j}-{}^{G}\mathrm{\hat{p}}_{I_l}^{(\alpha_i)}\right]_{\times}({}_{G}^{I_l}R^{(\alpha_i)})^{T}({}^{I_{l}}_{I_{l-1}}R^{(l-1)})+({}^{I_{l-1}}_{G}R^{(l-1)})^{T}\left[\hat{\mathrm{y}}^{(l-1)}_{l-1}\right]_{\times} & -\mathbf{I} & -\mathbf{I}\Delta{t}_{l-1} 
\end{bmatrix}
\end{aligned} \tag{27}
$$
可以看到，重点是在上式中的第一个元素，将该部分单独展开：
$$
\begin{aligned}
M0^{(\alpha_i)}_{1}&=\left[\underbrace{\left[{}^{G}\mathrm{\hat{p}}_{f_j}-{}^{G}\mathrm{\hat{p}}_{I_l}^{(\alpha_i)}\right]_{\times}}_{part1}\underbrace{({}_{G}^{I_l}R^{(\alpha_i)})^{T}({}^{I_{l}}_{G}R^{(l-1)})}_{D1}+\underbrace{\left[ {}^{G}\hat{p}_{l}^{(l-1)}-{}^{G}\hat{p}_{l-1}^{(l-1)}-{}^{G}\hat{v}_{I_{l-1}}^{(l-1)}\Delta{t}_{l-1}-\frac{1}{2}g \Delta{t}_{l-1}^2 \right]_{\times}}_{part2}\right]({}^{I_{l-1}}_{G}R^{(l-1)})^{T} \\
&=\left[
part1 \underbrace{\left((\mathbf{I}-\lfloor{}^{(\alpha_i)}\theta_{(l-1)}\rfloor_{\times})({}^{I_{l}}_{G}R^{(l-1)})\right)^{T}({}^{I_{l}}_{G}R^{(l-1)})}_{D1}+part2
\right]
({}^{I_{l-1}}_{G}R^{(l-1)})^{T}  \\
&=\left[
\underbrace{part1+part2}_{D2}+part1\underbrace{\left(({}^{I_{l}}_{G}R^{(l-1)})^{T}\lfloor{}^{(\alpha_i)}\theta^{(l-1)}_{l}\rfloor_{\times}({}^{I_{l}}_{G}R^{(l-1)})\right)}_{D3}
\right]({}^{I_{l-1}}_{G}R^{(l-1)})^{T}  \\
&=\left[
\lfloor
{}^{G}\mathrm{\hat{p}}_{f_j}\underbrace{-{}^{G}\mathrm{\hat{p}}_{I_l}^{(\alpha_i)}+{}^{G}\hat{p}_{l}^{(l-1)}}_{D4}-{}^{G}\hat{p}_{l-1}^{(l-1)}-{}^{G}\hat{v}_{I_{l-1}}^{(l-1)}\Delta{t}_{l-1}-\frac{1}{2}g \Delta{t}_{l-1}^2
\rfloor_{\times} 
+ part1\times D3
\right]({}^{I_{l-1}}_{G}R^{(l-1)})^{T} \\
&=\left[
\underbrace{\lfloor
{}^{G}\mathrm{\hat{p}}_{f_j}-{}^{G}\hat{p}_{l-1}^{(l-1)}-{}^{G}\hat{v}_{I_{l-1}}^{(l-1)}\Delta{t}_{l-1}-\frac{1}{2}g \Delta{t}_{l-1}^2
\rfloor_{\times}}_{part3}+\underbrace{\lfloor-{}^{G}\mathrm{\hat{p}}_{I_l}^{(\alpha_i)}+{}^{G}\hat{p}_{I_l}^{(l-1)}\rfloor_{\times}}_{D5}+\underbrace{part1\times D3}_{D6}
\right]({}^{I_{l-1}}_{G}R^{(l-1)})^{T}
\end{aligned} \tag{28}
$$

其中：

1. 第一行的化简将$\lfloor \hat{\mathrm{y}}^{(l-1)}_{l}\rfloor_{\times}$直接展开，且使用$\lfloor Ra \rfloor_{\times}=R\lfloor a \rfloor_{\times}R^{T}$的性质形成了part2部分；
2. 第三行的化简使用$\lfloor a \rfloor_{\times}^{T}=-\lfloor a \rfloor_{\times}$的性质，把负号消除形成D3部分；
3. 最后一行的化简，因为希望凑出一个类似于公式（20）最后一行的形式，所以这里直接将D4部分移了出来；

至此第一次乘积部分我们就完成了；下面来看第二次乘积：
$$
\begin{aligned}
M1^{(\alpha_i)}&=\begin{bmatrix} M0_{1}^{\alpha_i}({}^{I_{l-1}}_{G}R^{(l-1)})^{T} & -\mathbf{I} & -\mathbf{I}\Delta{t}_{l-1}\end{bmatrix}
\begin{bmatrix} 
{}^{I_{l-1}}_{G}R^{(l-2)}({}^{I_{l-2}}_{G}R^{(l-2)})^{T} & \mathbf{0} & \mathbf{0} \\
-({}^{I_{l-2}}_{G}R^{(l-2)})^{T}\left[\hat{\mathrm{y}}^{(l-2)}_{l-2}\right]_{\times} & \mathbf{I} & \mathbf{I}\Delta{t}_{l-2} \\
-({}^{I_{l-2}}_{G}R^{(l-2)})^{T}\left[\hat{\mathrm{s}}^{(l-2)}_{l-2}\right]_{\times} & \mathbf{0} & \mathbf{I}
\end{bmatrix} \\
&=
\begin{bmatrix}
\left[M0_{1}^{\alpha_i}\underbrace{({}^{I_{l-1}}_{G}R^{(l-1)})^{T}({}^{I_{l-1}}_{G}R^{(l-2)})}_{D7}+\underbrace{ \lfloor \hat{\mathrm{y}}^{(l-2)}_{l-2} \rfloor_{\times}+\lfloor \hat{\mathrm{s}}^{(l-2)}_{l-2}\Delta{t}_{l-1} \rfloor_{\times}}_{part4}\right]({}^{I_{l-2}}_{G}R^{(l-2)})^{T} \\
-\mathbf{I} \\ 
-\mathbf{I}(\Delta{t}_{l-1}+\Delta{t}_{l-2})
\end{bmatrix}^{T}
\end{aligned} \tag{29}
$$

这里休息一下，因为其中做了一些变换和notation上的偷懒：

1. 把公式（28）中方括号中的部分复用了标示$M0_{1}^{\alpha_i}$；

2. 复用了标示$\hat{\mathrm{y}}^{(l-2)}_{l-2}$，如下：
   $$
   -({}^{I_{l-2}}_{G}R^{(l-2)})^{T}\left[\hat{\mathrm{y}}^{(l-2)}_{l-2}\right]_{\times}=-\lfloor \underbrace{{}^{G}\hat{p}^{(l-2)}_{I_{l-1}}-{}^{G}\hat{p}^{(l-2)}_{I_{l-2}}-{}^{G}\hat{v}_{I_{l-2}}^{(l-2)}\Delta{t}_{l-2}-\frac{1}{2}g\Delta{t}_{l-2}^{2}}_{\hat{\mathrm{y}}^{(l-2)}_{l-2}}\rfloor_{\times}({}^{I_{l-2}}_{G}R^{(l-2)})^{T}
   $$

3. 复用了标示$\hat{\mathrm{s}}^{(l-2)}_{l-2}$，如下：
	$$
	-({}^{I_{l-2}}_{G}R^{(l-2)})^{T}\left[\hat{\mathrm{s}}^{(l-2)}_{l-2}\right]_{\times}=-\lfloor \underbrace{{}^{G}\hat{v}^{(l-2)}_{I_{l-1}}-{}^{G}\hat{v}^{(l-2)}_{I_{l-2}}-g\Delta{t}_{l-2}}_{\hat{\mathrm{s}}^{(l-2)}_{l-2}}\rfloor_{\times}({}^{I_{l-2}}_{G}R^{(l-2)})^{T}
	$$

回到公式（29）的化简上来，可以看到重点其实在首个元素的最开始的乘法中，这里单独展开有：
$$
\begin{aligned}
M0_{1}^{\alpha_i}\times D7 &=M0_{1}^{\alpha_i}\left((\mathbf{I}-\lfloor{}^{(l-1)}\theta^{(l-2)}_{l-1}\rfloor_{\times}){}^{I_{l-1}}_{G}R^{(l-2)}\right)^{T}({}^{I_{l-1}}_{G}R^{(l-2)}) \\
&=M0_{1}^{\alpha_i}+\underbrace{M0_{1}^{\alpha_i}\lfloor ({}^{I_{l-1}}_{G}R^{(l-2)})^{T} ({}^{(l-1)}\theta^{(l-2)}_{l-1})\rfloor_{\times}}_{D8}
\end{aligned} \tag{30}
$$
推导到这个地方的时候，结合公式（28）和（30），相信读者可以明显的看到以下几个事实：

1. 公式（28）中的part3和公式（29）中的part4结合就可以得到类似于理想值下的结果，也就是公式（21）的首个元素：
   $$
   \begin{aligned}
   part3+part4&=
   {}^{G}\mathrm{\hat{p}}_{f_j}-{}^{G}\hat{p}_{l-1}^{(l-1)}-{}^{G}\hat{v}_{I_{l-1}}^{(l-1)}\Delta{t}_{l-1}-\frac{1}{2}g \Delta{t}_{l-1}^2 \\
   &+ {}^{G}\hat{p}^{(l-2)}_{I_{l-1}}-{}^{G}\hat{p}^{(l-2)}_{I_{l-2}}-{}^{G}\hat{v}_{I_{l-2}}^{(l-2)}\Delta{t}_{l-2}-\frac{1}{2}g\Delta{t}_{l-2}^{2} \\
   &+ {}^{G}\hat{v}^{(l-2)}_{I_{l-1}}\Delta{t}_{l-1}-{}^{G}\hat{v}^{(l-2)}_{I_{l-2}}\Delta{t}_{l-1}-g\Delta{t}_{l-2}\Delta{t}_{l-1} \\
   &=\underbrace{{}^{G}\mathrm{\hat{p}}_{f_j}-{}^{G}\hat{p}^{(l-2)}_{I_{l-2}}-{}^{G}\hat{v}^{(l-2)}_{I_{l-2}}(\Delta{t}_{l-1}+\Delta{t}_{l-2})-\frac{1}{2}g(\Delta{t}_{l-1}+\Delta{t}_{l-2})^2}_{part5} \\
   &+ \underbrace{({}^{G}\hat{v}_{I_{l-1}}^{(l-2)}-{}^{G}\hat{v}_{I_{l-1}}^{(l-1)})\Delta{t}_{l-1}+({}^{G}\hat{p}^{(l-2)}_{I_{l-1}}-{}^{G}\hat{p}_{I_{l-1}}^{(l-1)})}_{D9}
   \end{aligned} \tag{31}
   $$
   可以看到，part5已经和公式（21）形式上一抹一样了，只不过除了这部分之外，**多了D9的部分**；

2. 在对公式（28）和公式（29）进行化简的时候，发现D1和D7部分真的是惊人的相似，**都是说对于同一个变量，使用了两个时刻的估计值，从而造成在化简的时候多出了角度的扰动量**，形成了D3和D8部分；

3. 同理，也能发现D5和D9部分也是一样的问题，同样的变量因为使用了不同时刻的估计值，产生了扰动项；

后面笔者就不进行展开了，十分麻烦，详细可以看参考1中的公式（42）（43）。

&nbsp;

-----

## First-Estimate-Jacobian

通过上面三点的分析，不难发现，**既然同一个变量使用了不同时刻的估计会产生扰动项，那么都使用一个时刻的估计值不就好了么**，其实这就是FEJ做的事情！

如果我们按照这个思路，那么将公式（25）到公式（31）中的所有的$\mathrm{x}^{(k+)}_{k}$都变为$\mathrm{x}_{k}^{k-1}$，之后回代到公式（28）（30）（31）中：

1. $D3=\left(({}^{I_{l}}_{G}R^{(l-1)})^{T}\lfloor{}^{(\alpha_i)}\theta^{(l-1)}_{l}\rfloor_{\times}({}^{I_{l}}_{G}R^{(l-1)})\right)$，对于 $l$ 节点，在时刻 $\alpha_i$ 与时刻 $l-1$ 使用的都是相同的估计值，所以扰动项 ${}^{(\alpha_i)}\theta^{(l-1)}_{l}={}^{(l-1)}\theta^{(l-1)}_{l}=\mathbf{0}$，所以整个D3值为0，导致后面的D6=part1 x D3也为0；
2. $D5=-{}^{G}\mathrm{\hat{p}}_{I_l}^{(\alpha_i)}+{}^{G}\hat{p}_{I_l}^{(l-1)}=-{}^{G}\mathrm{\hat{p}}_{I_l}^{(l-1)}+{}^{G}\hat{p}_{I_l}^{(l-1)}=\mathbf{0}$；
3. D8与D3同理，由${}^{(l-1)}\theta^{(l-2)}_{l-1}=\mathbf{0}$导致D8为0；
4. D9与D5同理，也全为0；

这么一代入，公式（29）得到了一个非常nice的形式：
$$
\begin{aligned}
M1^{(\alpha_i)}&=
\begin{bmatrix}
\lfloor {}^{G}\mathrm{\hat{p}}_{f_j}-{}^{G}\hat{p}^{(l-3)}_{I_{l-2}}-{}^{G}\hat{v}^{(l-3)}_{I_{l-2}}(\Delta{t}_{l-1}+\Delta{t}_{l-2})-\frac{1}{2}g(\Delta{t}_{l-1}+\Delta{t}_{l-2})^2 \rfloor_{\times}({}^{I_{l-2}}_{G}R^{(l-3)})^{T} \\
-\mathbf{I} \\ 
-\mathbf{I}(\Delta{t}_{l-1}+\Delta{t}_{l-2})
\end{bmatrix}^{T}
\end{aligned} \tag{32}
$$


可以看到，这个形式已经十分贴合公式（21）所表示的理想情况下的能观矩阵了；

这里再往下再算一次矩阵乘法验证，也就是$M2^{(\alpha_i)}\Phi_{l-3}(\hat{\mathrm{x}}^{(l-3)}_{l-2}, \hat{\mathrm{x}}^{(l-4)}_{l-3})$，单独展开得到：
$$
\begin{aligned}
M2^{(\alpha_i)}=
\begin{bmatrix}
M1^{\alpha_i}_1({}^{I_{l-2}}_{G}R^{(l-3)})^{T} & -\mathbf{I} & -\mathbf{I}(\Delta{t}_{l-1}+\Delta{t}_{l-2})
\end{bmatrix}
\begin{bmatrix} 
{}^{I_{l-2}}_{G}R^{(l-3)}({}^{I_{l-3}}_{G}R^{(l-4)})^{T} & \mathbf{0} & \mathbf{0} \\
-({}^{I_{l-3}}_{G}R^{(l-4)})^{T}\left[\hat{\mathrm{y}}^{(l-4)}_{l-3}\right]_{\times} & \mathbf{I} & \mathbf{I}\Delta{t}_{l-3} \\
-({}^{I_{l-3}}_{G}R^{(l-4)})^{T}\left[\hat{\mathrm{s}}^{(l-4)}_{l-3}\right]_{\times} & \mathbf{0} & \mathbf{I}
\end{bmatrix}
\end{aligned} \tag{33}
$$
重点展开首个元素有：
$$
\begin{aligned}
M2^{(\alpha_i)}_{1}&=
{}^{G}\mathrm{\hat{p}}_{f_j}-{}^{G}\hat{p}^{(l-3)}_{I_{l-2}}-{}^{G}\hat{v}^{(l-3)}_{I_{l-2}}(\Delta{t}_{l-1}+\Delta{t}_{l-2})-\frac{1}{2}g(\Delta{t}_{l-1}+\Delta{t}_{l-2})^2 \\
&+ {}^{G}\hat{p}^{(l-3)}_{I_{l-2}}-{}^{G}\hat{p}^{(l-4)}_{I_{l-3}}-{}^{G}\hat{v}_{I_{l-3}}^{(l-4)}\Delta{t}_{l-3}-\frac{1}{2}g\Delta{t}_{l-3}^{2} \\
&+ {}^{G}\hat{v}^{(l-3)}_{I_{l-2}}(\Delta{t}_{l-1}+\Delta{t}_{l-2})-{}^{G}\hat{v}^{(l-4)}_{I_{l-3}}(\Delta{t}_{l-1}+\Delta{t}_{l-2})-g\Delta{t}_{l-3}(\Delta{t}_{l-1}+\Delta{t}_{l-2}) \\
&={}^{G}\mathrm{\hat{p}}_{f_j}-{}^{G}\hat{p}^{(l-4)}_{I_{l-3}}-{}^{G}\hat{v}_{I_{l-3}}^{(l-4)}(\Delta{t}_{l-1}+\Delta{t}_{l-2}+\Delta{t}_{l-3})-\frac{1}{2}g(\Delta{t}_{l-1}+\Delta{t}_{l-2}+\Delta{t}_{l-3})^2
\end{aligned}
$$
将以上结果回代入公式（33）可得：
$$
M2^{(\alpha_i)}=
\begin{bmatrix}
\lfloor {}^{G}\mathrm{\hat{p}}_{f_j}-{}^{G}\hat{p}^{(l-4)}_{I_{l-3}}-{}^{G}\hat{v}_{I_{l-3}}^{(l-4)}(\Delta{t}_{1-3})-\frac{1}{2}g(\Delta{t}_{1-3})^2 \rfloor_{\times}({}^{I_{l-3}}_{G}R^{(l-4)})^{T} \\ 
-\mathbf{I} \\
-\mathbf{I}\Delta{t}_{1-3}
\end{bmatrix}^{T} \tag{34}
$$
可以预见到，一直乘积下去的结果最终的形式和公式（22）一样，如下：
$$
\hat{\mathcal{O}}_{l}=J_{(f_j|l)}{}^{C}_{I}R{}_{G}^{I_l}\hat{R}^{(l-1)}
\begin{bmatrix} \underbrace{\left[{}^{G}\mathrm{\hat{p}}_{f_j}-{}^{G}\hat{p}_{I_{k}}^{(k-1)}-{}^{G}\hat{v}_{k}^{(k-1)}\Delta{t}-\frac{1}{2}g\Delta{t}_{k-\alpha_i}^2 \right]_{\times}({}_{G}^{I_{k}}\hat{R}^{(k-1)})^{T}, -\mathbf{I},  -\mathbf{I}\Delta{t}_{k-\alpha_i}}_{IMU} \\ \underbrace{\dots , \mathbf{I} , \dots , \mathbf{0}}_{feature} \end{bmatrix} \tag{35}
$$
公式中最后递推到了第 k 个节点，使用的是 $k-1$ 时刻的估计值。

所以整个系统的零空间为：
$$
\mathbf{\hat{N}}=\left[\begin{array}{cc}
\mathbf{0}_{3} & {}^{I_k}_{G}\mathbf{\hat{R}}^{(k-1)} \mathbf{g} \\
\mathbf{I}_{3} & -\left[^{G} \mathbf{\hat{p}}_{k}^{(k-1)}\right]_\times{\mathbf{g}} \\
\mathbf{0}_{3} & -\left[^{G} \mathbf{\hat{v}}_{k}^{(k-1)}\right]_\times{\mathbf{g}} \\
\mathbf{I}_{3} & -\left[^{G} \mathbf{\hat{p}}_{f_1}\right]_\times{\mathbf{g}} \\
\mathbf{I}_{3} & -\left[^{G} \mathbf{\hat{p}}_{f_2}\right]_\times{\mathbf{g}} \\
\vdots & \vdots \\
\mathbf{I}_{3} & -\left[^{G} \mathbf{\hat{p}}_{f_N}\right]_\times{\mathbf{g}}
\end{array}\right]  \tag{36}
$$


&nbsp;

-----

## 关于零空间的一些理解

对比公式（36）和公式（23），不难发现：

1. 公式（23）的零空间的基底使用真值表示，亦即这些值不再随时间变化了；
2. 公式（36）的零空间的基底使用的是 k-1​ 时刻估计出的 k 节点的值表示，k 节点此时的值并不一定是最优的！

所以两个零空间一样么？显然是不一样的，但是两个零空间的物理意义却是相同的！

具体而言：

- 前三维影响IMU系的位置和特征点的位置，相当于把整个系统shift起来了；
- 最后一个维度影响**以重力轴为旋转轴的旋转**，也就是世界系的yaw方向，也相当于在yaw方向上shift了整个系统；

这部分读者感兴趣可以看参考1中的附录部分。

其实整个推导下来之后，笔者认为FEJ更多的其实是通过固定节点的优化方向，进而保证整个线性系统能观矩阵的零空间一直保持一致，**但是需要明确的是，该优化方向与零空间并不是正交的**。

进一步可以看到，其实该零空间是第一个观测方程的零空间，即在 k 时刻，整个能观性矩阵为：
$$
\hat{\mathcal{O}}^{(k)}=\left[ \mathrm{H}_{k}^{(k-1)} \right] \tag{37}
$$
而该能观性矩阵的零空间就是如公式（36）所示的零空间；

所以FEJ维护的零空间在笔者看来其实就是以 k 时刻为起点的系统，在 k 时刻的零空间，从实际的角度来说，当系统在 k 时刻的状态确定了，那么整个系统的位置和yaw轴不可观其实都是跟起点紧密相关的（相当于把起点shift起来了），而后面使用FEJ都在维护这样的状态；

&nbsp;

---

## 总结

本文较为详细的推导了：

1. 不同线性化点是如何影响整个能观性的；
2. FEJ为什么可以保持整个系统的能观性不受影响；
3. FEJ不仅保持零空间的维度，同时保持了零空间的物理意义不变了；

还有需要说明的是：本文在解决能观性不同的问题上没有使用参考1中的方法，不过参考1中的方法将整个旋转部分去除的方法确实很强，篇幅所限这里就不展开了，感兴趣的可以自行阅读一下～

下一篇笔者打算写一下强制让优化方向与零空间正交的OC-KF的方法，该方法也是开源工程S-MSCKF中使用的方法。