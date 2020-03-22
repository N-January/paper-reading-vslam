# DSO-Marginalization



## 写在前面

emmm，这部分的代码量瞬间就上来了，各种函数和方法也相对很多，边总结边梳理吧。



---

## 变量和结构

### EnergyFunction

- frames拿着前端的keyframes
- HM，bM是边缘化之后的先验信息
- adHost，adTarget，向量空间的矩阵伴随，$T_w^h$->$T_h^t$，$T_w^t$->$T_h^t$；
- adHTdeltaF，将host和target的delta量变化到$T_t^h$上，使用伴随

### FrameHessian

Notation：

$\zeta_0$为线性化点，$x_0$为margin时刻的增量，$x$为k时刻的增量

- worldToCam_evalPT，帧跟踪之后的位姿，也是FEJ的线性化点$\zeta_0$；
- state，state_scale，scale是state添加了一个scale之后的值；
- PRE_worldToCam，LM迭代的时候使用的变量，线性化点+delta量；
- state_zero是$x_0$；
- get_state_minus_stateZero()表示在线性化方向上的增量delta量；
- dIp是拿着每一层的灰度值和xy的梯度值的量；
- dI是拿着第0层的量；
- step，step_backup是优化的量；
- delta是$x-x_0$；

### FrameFramePrecalc

- PRE_RTll， PRE_tTll是优化中使用的相对位姿$T_h^t $；
- PRE_RTll_0，PRE_RTll_0是线性化点$\zeta_ 0$的相对位姿；
- PRE_KRKiTll， PRE_RKiTll是与K乘积之后的；

### Residual

- state_state是表示residual节点的状态；
- state_NewState表示优化的时候的新状态；
- isActiveAndIsGoodNEW表示residual误差小于阈值，可以被认为是激活状态；
- isLinearized表示有没有经过变形化；
- JpJdF表示Hw部分，因为Jd就一列，因此这个是一维向量；
- centerProjectedTo表示host的uv到target的uv；
- projectedTo是pattern到target的uv；

### RawResidualJacobian

该类主要保存每一个观测的信息，包含该观测的误差，误差对状态变量的导数（因为FEJ的关系，Jacobian固定的很早），该观测的误差能量（误差的平方）。

Notation：

p表示投影点的像素点$p_j=[u, v]$；

- Jpdxi 是2×6，$J_{pdxi}=\partial{p_j}/\partial{T_i^j}$，投影点对位姿的导数；
- Jpdc 是2×4，$J_{pdc}=\partial{p_j}/\partial{C}$，投影点对内参的导数；
- Jpdd 是2×1，$J_{pdd}=\partial{p_j}/\partial{\rho}$，投影点对逆深度的导数；
- JIdx[2]是8×2，$J_{Idx}=\partial{e}/\partial{p_j}$，误差对于投影点的导数，等于图像的梯度；
- JabF[2]是8×2，$J_{ab}=\partial{e}/\partial{ab}$，误差对于相对光度参数的导数；
- JIdx2是2×2，$J_{Idx}^TJ_{Idx}$，构建H矩阵需要；
- JabJIdx是2×2，$J_{ab}^TJ_{Idx}$，构建H矩阵需要；
- Jab2是2×2，$J_{ab}^TJ_{ab}$，构建H矩阵需要；
- resF是8×1，记录每一个pattern所产生的光度误差；



---

## 整体流程

下面是整体流程：

Notation：

state：表示增量方程中的增量；

1. 当有新的关键帧产生的时候，首先设定线性化点$\zeta_0$（setEvalPT_scaled）；

2. 更新未成熟点的逆深度值（traceNewCoarse）；

3. 标记需要被margin的帧；

4. 把当前关键帧添加到关键帧中frames；

5. 将当前关键帧添加到能量函数中EnergyFunction中（ef->insertFrame）;

   - 获取state的delta（fh->takeData）；

   - setAdjointsF函数，设置帧间的伴随矩阵，因为固定住线性化点了，因此要把增量从线性化点映射到帧间位姿空间上（这个后面详细推导）；

   - 更新共视图（connectivityMap：Map类型）；
   
6. 更新帧间位姿$T_h^t $，更新state的delta，同5.1（setPrecalcValues函数）；

   - FrameFramePrecalc::Set函数更新线性化点之间的相对位姿，为后面的H矩阵做准备，由于FEJ的关系，在计算优化方向的时候，要用线性化点时刻的Jacobian，位姿点使用worldToCam_evalPT的值，相对光度使用固定点时刻的相对光度值（后面详细推导）；误差值$e=f(x)$要用优化之后的位姿和相对光度计算，位姿使用PRE_worldToCam，相对光度使用更新之后的值（后面详细推导）；
   - EnergyFunctional::setDeltaF函数是将每个节点的$x-x_0$映射到相对位姿空间上，所以线性化点其实是$\zeta_0+x_0$点
   
7. 遍历keyframe的成熟的feature，之后无脑建立一个residual挂载在feature中，等着后期被删掉；

8. activatePointsMT函数，遍历keyframe的所有未成熟的feature，如果feature可以投影到最新关键帧，则将该点进行优化：

   - optimizeImmaturePoint函数对点进行优化，依旧是光度优化，一旦光度误差值在范围内，则认为可以激活；
   - 把优化过关的residual挂载在feature中；
   
9. 优化函数optimize：

   - 将**所有激活（r->isActive）**的**没有固定线性化点的residual（r->isLinearization=false）**放入activeResidual中，可以认为activeResidual是新的residual边。在没有边缘化之前，所有**激活点**的residual都满足条件，但是由于FEJ的影响，它们的线性化点确定了，但是计算误差时的参数（位姿和光度参数）没有固定；在**关键点**因为一些原因**被固定住但没有被边缘化和drop掉**时，此时线性化点依旧固定，且其误差也被要从固定点开始计算，后期还要对这个关键点进行优化，直到被margin或者drop掉；
   - linearizeAll计算所有的activeResidual的误差以及Jacobian，当前新关键帧的线性化点认为是track时候的位姿，就是第1步的位姿，其他关键帧的位姿就是FEJ的线性化点的位姿，都为worldToCam_evalPT；
   - calcLEnergy函数构建所有被固定线性化点的EnergyFunction中的E，公式为$E = 2b\delta{x}+\delta{x}H\delta{x} = 2J^Te\delta{x}+\delta{x}J^T J\delta{x}$，其中$e=e_0+J_{x_0}(x-x_0)$为被固定时刻的误差；
   - calcMEnergy函数是将边缘化的先验值，公式为$E^{\prime}\left(\boldsymbol{x}_{\alpha} \times\left(\boldsymbol{\zeta}_{0}\right)_{\alpha}\right)=2 \boldsymbol{x}_{\alpha}^{T} \widehat{\mathbf{b}}_{\alpha}^{\prime}+\boldsymbol{x}_{\alpha}^{T} \widehat{\mathbf{H}}_{\alpha \alpha} \boldsymbol{x}_{\alpha}$，其中$b_{\alpha}$为margin时线性化点的b；
   - applyRes_Reductor构建所有H矩阵需要的变量；
   - solveSystem求解整个增量方程；
     - orthogonalize，保证零空间相关；
     - resubstituteF_MT，把每个节点的$\delta{x}$映射到$T_h^t$空间上，之后求解逆深度的更新值；
   - doStepFromBackup，更新增量$x$，更新当前帧位姿PRE_worldToCam和当前landmark的逆深度，这里更新逆深度的时候也把zero点设置为最新的优化值了，所以说对于逆深度来说，线性化点没有固定；重新算一遍相对位姿setPrecalcValues（这里仅仅是更新了相对位姿，因为线性化点是没有变的）；
   - 整个优化过程完毕之后，设置当前最新关键帧的线性化点为当前位姿setEvalPT函数（worldToCam_evalPT = PRE_worldToCam），记录线性化点时的增量为$x_0$（这个地方位姿的增量记录为$x_0$），这个函数中又对最新关键帧PRE_worldToCam进行了计算，其实还是线性化点处的值；
   - ef->setAdjointsF重新计算伴随；
   - setPrecalcValues重新计算各个delta量，逆深度的delta；
   - linearizeAll重新把新的residual（activeResidual）固定住，同时干掉一些误差较大的residual（把residual从point的residual列表中删除）；
   
10. removeOutliers删除外点，这里的判据是point的residual列表为空；

11. 为下一次跟踪准备值CoarseTracker::setCoarseTrackingRef函数，准备跟踪需要的深度，host的uv等等；

12. flagPointsForRemoval，这个函数比较重要，因为要margin掉一些老的关键帧，那么以该关键帧为host帧的Landmark有三个出路：

    - 深度值很差的点要被drop掉；

    - 深度值很好的点要被margin掉，和被margin的帧一起为后面的帧提供先验；

    - 深度值还没有收敛的点，要继续进行深度的优化，同时要为关联帧的参数优化提供一些必要的约束；我个人的理解是：因为host帧没有了，但是自己还不够资格作为先验给margin掉，因此只能建立约束。假设被margin的帧或者landmark是将军的话，那么这部分点就相当于是队伍中的经验丰富老兵，之后自己成长（逆深度被继续优化）的同时，保证队伍的方向不太过偏离原来的方向（参数的增量不能导致误差增长太大）；

      上述的后面两种情况下，当前点的所有关联residual都要被固定在FEJ的线性化点处，因为这个点的一些信息被边缘化掉了，所以为了使零空间完备，使用公式$e_{x_0}=e_{x}-J_{x_0}(x-x_0)$把误差退回到FEJ处，这里特别注意的是因为landmark是没有固定线性化点的，**因此这里公式表达的意思实际上是把位姿、光度参数退回到FEJ的时刻，保证先验或者后面约束的零空间的完备性**；

13. ef->marginalizePointsF函数，主要是边缘化掉point，这里的边缘化是为了加速求解增量方程，这里把point边缘化掉之后加在HM，bM中；

14. 对HM和bM进行schur补操作，得到新的先验信息；



----

## 公式推导

### Jacobian公式

首先光度误差为：
$$
e(T_{ij}, \rho_p, a_i, b_i, a_j, b_j)=\left(I_{j}\left[\mathbf{p}^{\prime}\right]-\frac{t_{j} e^{a_{j}}}{t_{i} e^{a_{i}}}I_{i}[\mathbf{p}]\right)-\left(b_{j}-\frac{t_{j} e^{a_{j}}}{t_{i} e^{a_{i}}}b_{i}\right)
$$
其中：
$$
\begin{aligned}
P_j &= T_{i}^{j}\begin{bmatrix}Pi \\ \rho\end{bmatrix} = R_{i}^{j}\pi_c^{-1}(p)+\rho t_{i}^{j}  \\
p' &= \pi_{c}(P_j)
\end{aligned}
$$
然后定义相对光度参数$\delta{a}， \delta{b}$为：
$$
\begin{aligned}
\delta{a}_{i}^{j} &= -\frac{t_{j} e^{a_{j}}}{t_{i} e^{a_{i}}}  \\
\delta{b}_{i}^{j} &= -(b_j-\frac{t_{j} e^{a_{j}}}{t_{i} e^{a_{i}}}b_i)
\end{aligned}
$$
对每一个节点的状态变量进行求导得：
$$
J_{x}=\frac{\partial{e}}{\partial{x}}=\frac{\partial{e}}{\partial{I_j}}\frac{\partial{I_j}}{\partial{p_j}}
\begin{cases}
\frac{\partial{p_j}}{\partial{T_{i}^{j}}} \text{ where x=Jeo} \\
\frac{\partial{p_j}}{\partial{\rho}} \text{ where x=} \rho \\
\frac{\partial{p_j}}{\partial{C}} \text{ where x=C} \\
\end{cases}
$$
逐步对每一部分进行求导：

#### x=Jeo

​	先对相对位姿进行求导，有：
$$
\begin{aligned}
\frac{\partial{p_{j}}}{\partial T_i^j} &=\frac{\partial{p_{j}}}{\partial P_j}\frac{\partial{P_{j}}}{\partial T_{ji}}  \\
&=\begin{bmatrix} \frac{fx}{Z} & 0 & -\frac{fxX}{Z^2} \\ 0 & \frac{fy}{Z} & -\frac{fyY}{Z^2}\end{bmatrix}\frac{\partial{P_{j}}}{\partial T_{ji}} \\
&=\begin{bmatrix} \frac{fx}{Z} & 0 & -\frac{fxX}{Z^2} \\ 0 & \frac{fy}{Z} & -\frac{fyY}{Z^2}\end{bmatrix}\begin{bmatrix} \rho \mathbf{I}_{3×3} & -[RP_{i}+\rho t]_{\times} \end{bmatrix}
\end{aligned}
$$

#### x=$\rho$

对逆深度进行求导，有：
$$
\begin{aligned}
\frac{\partial{p_{j}}}{\partial{\rho}} &=\frac{\partial{p_{j}}}{\partial P_j}\frac{\partial{P_{j}}}{\partial \rho}  \\
&=\begin{bmatrix} \frac{fx}{Z} & 0 & -\frac{fxX}{Z^2} \\ 0 & \frac{fy}{Z} & -\frac{fyY}{Z^2}\end{bmatrix}\frac{\partial{P_{j}}}{\partial \rho} \\
&=\begin{bmatrix} \frac{fx}{Z} & 0 & -\frac{fxX}{Z^2} \\ 0 & \frac{fy}{Z} & -\frac{fyY}{Z^2}\end{bmatrix}\mathbf{t}_{i}^{j}
\end{aligned}
$$

#### x=C

设$u=\frac{X}{Z}$，$v=\frac{Y}{Z}$，所以得到$p_{jx}=f_{x}u+c_x$，$p_{jy}=f_{y}v+c_y$，所以对于相机内参C(fx,fy,cx,cy)而言，其导数有(这里仅仅将$p_{jx}$对C进行求导，$p_{jy}$是一样的)：
$$
\begin{aligned}
\frac{\partial{p_{jx}}}{\partial{f_x}} &=u+f_x\frac{\partial{u}}{\partial{f_x}} =u+f_x\frac{\partial{u}}{\partial{P_j}}\frac{\partial{P_j}}{\partial{P_i}}\frac{\partial{P_i}}{\partial{f_x}} \\
&= u+f_x[\frac{1}{Z}, 0, -\frac{X}{Z^2}]R_{i}^{j}\begin{bmatrix}-\frac{u_i-c_x}{f_x^2} \\ 0 \\ 0  \end{bmatrix} \\
&= u+\frac{1}{Z}[1, 0, -u]R_i^j \begin{bmatrix}-P_{ix} \\ 0 \\ 0  \end{bmatrix}
\end{aligned}
$$
对于$f_y$而言，导数为：
$$
\begin{aligned}
\frac{\partial{p_{jx}}}{\partial{f_y}} &=0+f_x\frac{\partial{u}}{\partial{f_x}} =0+f_x\frac{\partial{u}}{\partial{P_j}}\frac{\partial{P_j}}{\partial{P_i}}\frac{\partial{P_i}}{\partial{f_y}} \\
&= 0+f_x[\frac{1}{Z}, 0, -\frac{X}{Z^2}]R_{i}^{j}\begin{bmatrix}0 \\ -\frac{v_i-c_y}{f_y^2} \\ 0  \end{bmatrix} \\
&= 0+\frac{1}{Z}[1, 0, -u]R_i^j \begin{bmatrix}0 \\ -\frac{f_X}{f_y}P_{iy} \\ 0 \end{bmatrix}
\end{aligned}
$$


对于$c_x$而言，导数为：
$$
\begin{aligned}
\frac{\partial{p_{jx}}}{\partial{c_x}} &=1+f_x\frac{\partial{u}}{\partial{c_x}} =1+f_x\frac{\partial{u}}{\partial{P_j}}\frac{\partial{P_j}}{\partial{P_i}}\frac{\partial{P_i}}{\partial{c_x}} \\
&= 1+f_x[\frac{1}{Z}, 0, -\frac{X}{Z^2}]R_{i}^{j}\begin{bmatrix}-\frac{1}{f_x} \\ 0 \\ 0  \end{bmatrix} \\
&= 1+\frac{1}{Z}[1, 0, -u]R_i^j \begin{bmatrix}-1 \\ 0 \\ 0  \end{bmatrix}
\end{aligned}
$$
对于$c_y$而言，导数为：
$$
\begin{aligned}
\frac{\partial{p_{jx}}}{\partial{c_y}} &=0+f_x\frac{\partial{u}}{\partial{c_x}} =0+f_x\frac{\partial{u}}{\partial{P_j}}\frac{\partial{P_j}}{\partial{P_i}}\frac{\partial{P_i}}{\partial{c_x}} \\
&= 0+f_x[\frac{1}{Z}, 0, -\frac{X}{Z^2}]R_{i}^{j}\begin{bmatrix}0 \\ -\frac{1}{f_y} \\ 0  \end{bmatrix} \\
&= 1+\frac{1}{Z}[1, 0, -u]R_i^j \begin{bmatrix}0 \\ -\frac{f_x}{f_y} \\ 0  \end{bmatrix}
\end{aligned}
$$


所以把同样的方法应用到对$f_y，c_y$的求导中，最终整体的求导公式为：
$$
\begin{aligned}
\frac{\partial{p_j}}{\partial{C}} &= \begin{bmatrix}u, 0, 1, 0 \\ 0, v, 0, 1\end{bmatrix}+\frac{1}{Z}\begin{bmatrix}
{P_{ix}\left(r_{20} u-r_{00}\right)} & {P_{iy} \frac{f_{s}}{f_{y}}\left(r_{21} u-r_{01}\right)} & {\left(r_{20} u-r_{00}\right)} & {\frac{f_{x}}{f_{y}}\left(r_{21} u-r_{01}\right)} \\
{P_{ix} \frac{f_{y}}{f_{x}}\left(r_{20} v-r_{10}\right)} & {P_{iy}\left(r_{21} v-r_{11}\right)} & {\frac{f_{y}}{f_{x}}\left(r_{20} v-r_{10}\right)} & {\left(r_{21} v-r_{11}\right)}
\end{bmatrix}
\end{aligned}
$$

#### 对于相对光度参数

因为相对光度参数$\delta{a}，\delta{b}$与投影点没有关系，只与灰度有关，因此相对光度的参数要与pattern结合在一起，每一个像素点都要求解一次相对光度参数的导数，与这里另外拎出来求导：
$$
\begin{aligned}
J_{\delta{a}} &= \frac{\partial{e}}{\partial{\delta{a}}}=(I_i(p)-b_i) \\
J_{\delta{b}} &= \frac{\partial{e}}{\partial{\delta{b}}}=I
\end{aligned}
$$





---

### First Estimate Jacobian

因为DSO使用了FEJ方法，所以线性化点（也可以认为是优化方向）就要被固定住，主要是为了系统的能观性不被改变。对于这些线性化点被固定住的点，其优化的Jacobian不会变化（实际上是相对位姿和相对光度不变，而相机内参和逆深度不固定），但是其误差（光度误差）会变化。

在上面的流程中其实可以看到，这个FEJ会影响两个方面的增量方程：

1. 新残差构建的增量方程，这里主要是固定了Jacobian，误差的计算点不固定；
2. 被固定点的增量方程，这部分残差最重要的作用是为了约束关联帧的参数，为了保证零空间的完备性，这个约束要建立在FEJ的线性点处，误差也要退回到FEJ线性化点处，即$e_{x_0}=e_x-J_{x_0}(x-x_0)$，这里强调的一点是这里是把帧参数会退到FEJ处了，逆深度由于没有线性化点，因此不对这一步产生任何的影响；

这里总结一下两种情况的增量方程：

1. 第一种的误差计算方式为 $e_x=f(x+\delta{x})$，所以$J_{x_0}^TJ_{x_0}x=J_{x_0}^Te_x$，所以$E=||f(x)+J_{x_0}\delta{x}||_{\gamma}^2$；
2. 第二种是误差计算点固定住了，即$e_{x}=f(e_{0}+\delta{x})$，所以$J_{x_0}^TJ_{x_0}x=J_{x_0}^Te_{x_0}$，所以$E=||f(x_0)+J_{x_0}\delta{x}||_{\gamma}^2$；

这里引出一个问题：上述公式中的$\delta{x}$如何获得？当然你可以利用新旧相对位姿的差来表示，但是这里还有一个更好的办法——伴随。

---

### 向量空间的矩阵伴随

简单说一下我对伴随的理解，伴随最原始的形式是针对同一个流形上，将**局部坐标系的增量**映射到**全局坐标系的增量**（左乘和右乘的区别），这个映射可以是线性的，也可以是非线性的。具体可以参考[博客](<https://zhuanlan.zhihu.com/p/87438999?utm_source=wechat_session&utm_medium=social&utm_oi=918942813108973568>)。因为伴随的存在，使得很多映射关系都可以用伴随来推导，这个章节主要也在做这个事情。

DSO中主要有两个地方使用到了伴随：

1. 将全局参数的变化量映射为相对参数的变化量，即上面的部分；
2. 将相对参数的Jacobian映射为全局参数的Jacobian，所以在DSO中，一直都在求解相对参数的Jacobian，这样其实更加方便的计算能量函数，同时因为有伴随的存在，也不会在构建全局位姿的Jacobian上花时间；

下面主要对上述的两个方面进行公式推导，看一下伴随是怎么连接全局参数和相对参数的，以下所有推导都在全局坐标系，即李代数上进行。



#### 1a. 相对位姿与host帧位姿的伴随

$$
\begin{aligned}
\mathbf{Exp}(^{\epsilon}x_h^t)T_h^t &= T_w^t(\mathbf{Exp}(^{\epsilon}x_h)T_w^h)^{-1}  \\
&= T_w^tT_h^w\mathbf{Exp}(^{\epsilon}x_h)^{-1} \\
&= T_h^t\mathbf{Exp}(-^{\epsilon}x_h) \\
\\
\mathbf{Exp}(^{\epsilon}x_h^t) &= T_h^t\mathbf{Exp}(-^{\epsilon}x_h)T_t^h \\
^{\epsilon}x_h^t &= -Ad_{T_h^t}(^{\epsilon}x_h)
\end{aligned}
$$

上述公式表示在host帧上的位姿增量可以通过伴随$Ad_{T_h^t}$映射到相对位姿$T_h^t$上。



#### 1b. 相对位姿与target帧位姿的伴随

$$
\begin{aligned}
\mathbf{Exp}(^{\epsilon}x_h^t)T_h^t &= \mathbf{Exp}(^{\epsilon}x_t)T_w^t(T_w^h)^{-1}  \\
&= \mathbf{Exp}(^{\epsilon}x_h)T_h^t \\
\\
\mathbf{Exp}(^{\epsilon}x_h^t) &= \mathbf{Exp}(^{\epsilon}x_t) \\
^{\epsilon}x_h^t &= ^{\epsilon}x_t
\end{aligned}
$$



因此，当我们得到host帧和target帧上的位姿增量之后，可以通过上面的公式转换到相对位姿的向量空间上。



#### 1c. 相对光度与host帧光度参数的伴随

这里把相对光度参数的定义拷贝过来：
$$
\begin{aligned}
\delta{a}_{i}^{j} &= -\frac{t_{j} e^{a_{j}}}{t_{i} e^{a_{i}}}  \\
\delta{b}_{i}^{j} &= -(b_j-\frac{t_{j} e^{a_{j}}}{t_{i} e^{a_{i}}}b_i)
\end{aligned}
$$
然后可以清楚地看到，相对光度参数与host帧光度参数的关系为：
$$
\begin{aligned}
\frac{\partial{\delta{a_i^j}}}{\partial{a_i}} &= \frac{t_{j} e^{a_{j}}}{t_{i} e^{a_{i}}} \\
\frac{\partial{\delta{b_i^j}}}{\partial{b_i}} &= \frac{t_{j} e^{a_{j}}}{t_{i} e^{a_{i}}} 
\end{aligned}
$$


#### 1d. 相对光度与target帧光度参数的伴随

$$
\begin{aligned}
\frac{\partial{\delta{a_i^j}}}{\partial{a_j}} &= -\frac{t_{j} e^{a_{j}}}{t_{i} e^{a_{i}}} \\
\frac{\partial{\delta{b_i^j}}}{\partial{b_j}} &= -I 
\end{aligned}
$$



#### 2a～2d. 全局参数的Jacobian与相对参数的Jacobian的关系

有两种方法可以得到这个关系，下面以host帧位姿为例，另外的参数是一样的方法：

1. 第一种，从求导的原始的定义出发，有：
   $$
   \begin{aligned}
   \frac{\partial{e}}{\partial{\xi_h^t}}
   &=\lim_{\delta{\xi}_h^t->0} \frac{(Exp(\delta{\xi_h^t})T_h^t)P_h-T_h^tP_h}{\delta{\xi_h^t}} \\
   &=\lim_{\delta{\xi}_h->0} \frac{T_w^t(Exp(\delta{\xi_h})T_w^h)^{-1}P_h-T_w^t(T_w^h)^{-1}P_h}{-Ad_{T_h^t}\delta{\xi_h}} \\
   &=\lim_{\delta{\xi}_h->0} \frac{T_w^t(Exp(\delta{\xi_h})T_w^h)^{-1}P_h-T_w^t(T_w^h)^{-1}P_h}{\delta{\xi_h}}(-Ad_{T_h^t})^{-1} \\
   &=\frac{\partial{e}}{\partial{\xi_h}}(-Ad_{T_h^t})^{-1} \\
   
   \frac{\partial{e}}{\partial{\xi_h}} &= \frac{\partial{e}}{\partial{\xi_h^t}}(-Ad_{T_h^t})
   \end{aligned}
   $$
   ​	上面的推导中：
   
- 第二步使用了1a的方法；
  
- 第三步把除法变为乘法之后就能知道$-Ad_{T_h^t}$要乘在整个公式的后面；
   - 第四部可以看到前一部分就是求导的原始定义，所以直接替换就可以了；
   
2. 第二种，使用求导的链式法则，有：
   $$
   \frac{\partial{e}}{\partial{\xi_h}}=\frac{\partial{e}}{\partial{\xi_h^t}}\frac{\partial{\xi_h^t}}{\partial{\xi_h}}
   $$
   可以看到最后后面一项刚好和1a~1d部分对应；
   
   

----

## 整体流程图

这是自己总结的一份流程图，个人看的代码是按照这样的流程来的：

<img src="pictures/workflow.bmp"/>