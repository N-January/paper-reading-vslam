# DSO——初始化



## 写在前面

DSO的初始化着实是十分的难看懂，个人总结为以下三个原因：

- 没有采用常规的（很大程度上是因为笔者目光比较短浅）本质矩阵或者单应性矩阵求解位姿、三角化初始的landmark确定尺度，而是直接作为一个优化问题进行优化（恩，又知道了一种初始化方式）；
- 本身的光度模型就比较麻烦，而且有些公式也没有采用常规的方法（又是目光短浅），所以初看起来会比较“反常识”；
- 作者写代码确实很厉害，基本上优化过程都是自己写的，而且schur补的变量都是边求解Jacobian边构建的，最后直接一个简单的运算；

这篇文章主要是总结一下自己在看初始化代码的过程（主要是CoarseInitializer::trackFrame代码部分），希望能帮助更多的小伙伴。由于作者是在构建Jacobian的时候就在构建边缘化的东西了，且正则项的影响是在构建Schur补用到的矩阵之后加入的，所以推荐的阅读顺序为：

1. 第一部分——光度误差
2. 边缘化
3. 第二部分——正则项



---

## 优化的模型

整个优化的能量函数（也就是我们常说的误差函数）分为两个部分：一部分是光度误差；另一部分是为了帮助收敛而添加的正则项（虽然我不明白为啥添加一个能帮助收敛）；下面分两部分来说这两个部分：



### 第一部分——光度误差

光度误差模型如下：
$$
E_{\mathbf{p} j}:=\sum_{\mathbf{p} \in \mathcal{N}_{\mathbf{p}}} w_{\mathbf{p}}\left\|\left(I_{j}\left[\mathbf{p}^{\prime}\right]-b_{j}\right)-\frac{t_{j} e^{a_{j}}}{t_{i} e^{a_{i}}}\left(I_{i}[\mathbf{p}]-b_{i}\right)\right\|_{\mathcal{Y}}
$$
再次说明一下Notation：

- $i, j$为参考帧和当前帧；
- $t_i, t_j$为参考帧和当前帧的曝光时间，不知道写作1；$a_i, b_i$为待求解的光度响应参数；
- $p\prime$为$p$点在$j$帧的投影点；

下面结合公式来求解Jacobian，首先把整个误差分为两个部分：几何部分和光度参数部分：
$$
\begin{aligned}
E_{\mathbf{p} j}:&=\sum_{\mathbf{p} \in \mathcal{N}_{\mathbf{p}}} w_{\mathbf{p}}\left\|\left(I_{j}\left[\mathbf{p}^{\prime}\right]-b_{j}\right)-\frac{t_{j} e^{a_{j}}}{t_{i} e^{a_{i}}}\left(I_{i}[\mathbf{p}]-b_{i}\right)\right\|_{\mathcal{Y}} \\ 
&=\sum_{\mathbf{p} \in \mathcal{N}_{\mathbf{p}}} w_{\mathbf{p}}\left\|\left(I_{j}\left[\mathbf{p}^{\prime}\right]-\frac{t_{j} e^{a_{j}}}{t_{i} e^{a_{i}}}I_{i}[\mathbf{p}]\right)-\left(b_{j}-\frac{t_{j} e^{a_{j}}}{t_{i} e^{a_{i}}}b_{i}\right)\right\|_{\mathcal{Y}}
\end{aligned}
$$
然后将光度误差$e$拿出来进行分析：
$$
e(T_{ij}, \rho_p, a_i, b_i, a_j, b_j)=\left(I_{j}\left[\mathbf{p}^{\prime}\right]-\frac{t_{j} e^{a_{j}}}{t_{i} e^{a_{i}}}I_{i}[\mathbf{p}]\right)-\left(b_{j}-\frac{t_{j} e^{a_{j}}}{t_{i} e^{a_{i}}}b_{i}\right)
$$
对于初始化过程来说，$a_i, b_i$都是0，因此可以只计算$j$帧的参数就可以了，在求解Jacobian之前，一个需要注意的细节就是在DSO中，作者使用的transform公式与常规的不太一样，对于一个3D点$dP_c$（其中$d$为该点的实际深度，$Pc$为该点在主导帧归一化平面上的坐标），通常使用如下的公式进行坐标系之间的变换，如果使用的是逆深度，通常也是把逆深度作为分母：
$$
P_j = T_{i}^{j}\begin{bmatrix}dPi \\ 1\end{bmatrix} = dR_{j}^{i}P_i+t_{i}^{j}
$$
但是DSO中对于这块的处理个人感觉还是比较好的，作者把3D点的齐次坐标改写为$[P_i, \rho]^T$的形式，即添加的齐次项并不为1，但是本身齐次坐标就没有尺度的概念，因此是完全正确的，所以使用这种形式的话，整个transform过程变作：
$$
\begin{aligned}
P_j = T_{i}^{j}\begin{bmatrix}Pi \\ \rho\end{bmatrix} = R_{j}^{i}P_i+\rho t_{i}^{j}
\end{aligned}
$$
恩，这时候对逆深度求导就舒服多了。



#### 几何误差部分求导

下面分两个小部分对**几何部分**求导：

1. 对相机位姿求导：

$$
\begin{aligned}
J_{geo}&=\frac{\partial{e}}{\partial T_{ji}}=\frac{\partial{e}}{\partial I_j}\frac{\partial{p_{j}}}{\partial P_j}\frac{\partial{P_{j}}}{\partial T_{ji}}  \\
&=\nabla{I_j(p_{j})}\begin{bmatrix} \frac{fx}{Z} & 0 & -\frac{fxX}{Z^2} \\ 0 & \frac{fy}{Z} & -\frac{fyY}{Z^2}\end{bmatrix}\frac{\partial{P_{j}}}{\partial T_{ji}} \\
&=\nabla{I_j(p_{j})}\begin{bmatrix} \frac{fx}{Z} & 0 & -\frac{fxX}{Z^2} \\ 0 & \frac{fy}{Z} & -\frac{fyY}{Z^2}\end{bmatrix}\begin{bmatrix} \rho \mathbf{I}_{3×3} & -[RP_{i}+\rho t]_{\times} \end{bmatrix}
\end{aligned}
$$

> 这里对$P_j$对位姿的偏导进行额外说明：
> $$
> \begin{aligned}
> \frac{\partial{P_{j}}}{\partial T_{ji}} &= \frac{Exp(\delta{\zeta})\begin{bmatrix}R & t\\ 0 & 1\end{bmatrix}\begin{bmatrix}P_{i}\\ \rho\end{bmatrix}-\begin{bmatrix}R & t\\ 0 & 1\end{bmatrix}\begin{bmatrix}P_{i}\\ \rho\end{bmatrix}}{\delta{\zeta}} \\
> &=\frac{[\delta{\zeta}]_{\times} \begin{bmatrix}R & t\\ 0 & 1\end{bmatrix}\begin{bmatrix}P_{i}\\ \rho\end{bmatrix}-\begin{bmatrix}R & t\\ 0 & 1\end{bmatrix}\begin{bmatrix}P_{i}\\ \rho\end{bmatrix}}{\delta{\zeta}} \\
> &=\frac{\begin{bmatrix}[\delta{\theta}]_{\times} & \delta{\epsilon} \\ \mathbf{0} & 0\end{bmatrix}\begin{bmatrix}R & t\\ 0 & 1\end{bmatrix}\begin{bmatrix}P_{i}\\ \rho\end{bmatrix}}{\begin{bmatrix}\delta\epsilon \\ \delta\theta\end{bmatrix}} \\
> &=\frac{\begin{bmatrix}[\delta{\theta}]_{\times} & \delta{\epsilon} \\ \mathbf{0} & 0\end{bmatrix}
> \begin{bmatrix}RP_{i}+\rho t\\ \rho \end{bmatrix}}{\begin{bmatrix}\delta\epsilon \\ \delta\theta\end{bmatrix}} = \frac{\begin{bmatrix}[\delta{\theta}]_{\times}(RP_{i}+\rho t)+\rho \delta{\epsilon} \\ 0\end{bmatrix}}{\begin{bmatrix}\delta\epsilon \\ \delta\theta\end{bmatrix}} \\
> &=\begin{bmatrix} \rho \mathbf{I}_{3×3} & -[RP_{i}+\rho t]_{\times} \end{bmatrix}
> \end{aligned}
> $$

2. 对逆深度进行求导：

$$
\begin{aligned}
J_{geo}&=\frac{\partial{e}}{\partial \rho}=\frac{\partial{e}}{\partial I_j}\frac{\partial{p_{j}}}{\partial P_j}\frac{\partial{P_{j}}}{\partial \rho}  \\
&=\nabla{I_j(p_{j})}\begin{bmatrix} \frac{fx}{Z} & 0 & -\frac{fxX}{Z^2} \\ 0 & \frac{fy}{Z} & -\frac{fyY}{Z^2}\end{bmatrix}\frac{\partial{P_{j}}}{\partial \rho} \\
&=\nabla{I_j(p_{j})}\begin{bmatrix} \frac{fx}{Z} & 0 & -\frac{fxX}{Z^2} \\ 0 & \frac{fy}{Z} & -\frac{fyY}{Z^2}\end{bmatrix}\mathbf{t_i^j}
\end{aligned}
$$



#### 光度校正参数求导

由于是初始化阶段，$a_i，b_i$都为0，所以仅仅对当前帧$j$的参数进行丢到就行了，过程比较简单，如下：
$$
\begin{aligned}
J_{photo}&=\frac{\partial{e}}{\partial\begin{bmatrix}a_j \\ b_j\end{bmatrix}} \\
&=\begin{bmatrix} -\frac{t_{j} e^{a_{j}}}{t_{i} e^{a_{i}}}\left(I_{i}[\mathbf{p}]-b_{i}\right) & -1\end{bmatrix}
\end{aligned}
$$


上面的部分对应DSO代码的中如下：

```c++
int dx = patternP[idx][0];
int dy = patternP[idx][1];

// 这边公式用的是齐次的公式，也就是Pc=K*T*Pw, Pw=[x y 1 \rho]^T
Vec3f pt = RKi * Vec3f(point->u + dx, point->v + dy, 1) + t * point->idepth_new;

// 投影到当前帧
float u = pt[0] / pt[2];
float v = pt[1] / pt[2];
float Ku = fxl * u + cxl;
float Kv = fyl * v + cyl;

// 中间变量
float new_idepth = point->idepth_new / pt[2];

if (!(Ku > 1 && Kv > 1 && Ku < wl - 2 && Kv < hl - 2 && new_idepth > 0)) {
    isGood = false;
    break;
}

// 得到插值后的值
// hitColor是一个vector3f，[0]：gray value，[1]: gradientX，[2]: gradientY
Vec3f hitColor = getInterpolatedElement33(colorNew, Ku, Kv, wl);
//Vec3f hitColor = getInterpolatedElement33BiCub(colorNew, Ku, Kv, wl);

//float rlR = colorRef[point->u+dx + (point->v+dy) * wl][0];
// 得到插值之后的灰度值
float rlR = getInterpolatedElement31(colorRef, point->u + dx, point->v + dy, wl);

if (!std::isfinite(rlR) || !std::isfinite((float) hitColor[0])) {
    isGood = false;
    break;
}

// 得到光度误差，有huber函数
float residual = hitColor[0] - r2new_aff[0] * rlR - r2new_aff[1];
float hw = fabs(residual) < setting_huberTH ? 1 : setting_huberTH / fabs(residual);
energy += hw * residual * residual * (2 - hw);

// 中间变量，为雅可比做准备，主要针对深度的jacobian
float dxdd = (t[0] - t[2] * u) / pt[2];
float dydd = (t[1] - t[2] * v) / pt[2];

if (hw < 1) hw = sqrtf(hw);
float dxInterp = hw * hitColor[1] * fxl;
float dyInterp = hw * hitColor[2] * fyl;

// 0-5是6FOD(位姿) 
dp0[idx] = new_idepth * dxInterp;
dp1[idx] = new_idepth * dyInterp;
dp2[idx] = -new_idepth * (u * dxInterp + v * dyInterp);
dp3[idx] = -u * v * dxInterp - (1 + v * v) * dyInterp;
dp4[idx] = (1 + u * u) * dxInterp + u * v * dyInterp;
dp5[idx] = -v * dxInterp + u * dyInterp;

// 对光度系数a b进行求导
dp6[idx] = -hw * r2new_aff[0] * rlR;
dp7[idx] = -hw * 1;

// 逆深度的求导
dd[idx] = dxInterp * dxdd + dyInterp * dydd;

// insentity error
r[idx] = hw * residual;
```



### 第二部分——正则项

为了使得整个优化过程能够快速收敛，作者在整个误差方程中添加了L2正则项，根据位移的不同，添加的正则项也是不同的，整体的能量函数变为：

1. 当位移比较小的时候：

$$
E = E_{proj}+ \alpha_w(\underbrace{\left( d_{p_i}-1 \right)^2}_{H_{vv} part} + \underbrace{ \left\| t_i^j \right\|^2*N}_{H_{uu} part})
$$

2. 当位移较大的时候：
   $$
   E = E_{proj}+ 1*(\underbrace{\left\| d_{p_i}-d_{IR} \right\|^2}_{H_{vv} part})
   $$

注意，这里能量函数是误差$e(x)$的平方，所以在求导Jacobian的时候，要用$e=d_{p_i}-1$，$e=t_i^j$ 或者$e=d_{d_i}-d_{IR}$，所以：

1. 当位移比较小的时候，1）正则项对于逆深度（$H_\beta$部分）的Jacobian为单位向量$I$，对于位移（$t_i^j$，$H_\alpha$部分）的Jacobian依旧为单位向量$I$。2）正则项对于$J^Tb$的影响为$I^T\times e$；
2. 当位移很大的时候，按上面的同理可得对于$H$矩阵和$b$向量的影响，这里不再赘述；

这部分对应的代码为，由于这部分还涉及到Schur补的构建，这个在后面部分会介绍：

```c++
Accumulator11 EAlpha;
EAlpha.initialize();
for (int i = 0; i < npts; i++) {
    Pnt *point = ptsl + i;
    if (!point->isGood_new) {
        E.updateSingle((float) (point->energy[1]));
    } else {
        point->energy_new[1] = (point->idepth_new - 1) * (point->idepth_new - 1);
        E.updateSingle((float) (point->energy_new[1]));
    }
}
EAlpha.finish();
float alphaEnergy = alphaW * (EAlpha.A + refToNew.translation().squaredNorm() * npts);

// compute alpha opt.
float alphaOpt;
if (alphaEnergy > alphaK * npts) {
    alphaOpt = 0;
    alphaEnergy = alphaK * npts;
} else {
    alphaOpt = alphaW;
}

acc9SC.initialize();
for (int i = 0; i < npts; i++) {
    Pnt *point = ptsl + i;
    if (!point->isGood_new)
        continue;

    point->lastHessian_new = JbBuffer_new[i][9];
	
    // 当位移较小的时候，添加||dp-1||_2
    // 注意alphaOpt是权重，相当于要加权两个H矩阵
    JbBuffer_new[i][8] += alphaOpt * (point->idepth_new - 1); 
    JbBuffer_new[i][9] += alphaOpt;

    if (alphaOpt == 0) {
        // 当位移较大的时候，添加||dp-diR||_2
        JbBuffer_new[i][8] += couplingWeight * (point->idepth_new - point->iR);
        JbBuffer_new[i][9] += couplingWeight;
    }
}

// 将位移较小情况的正则项加入
H_out(0, 0) += alphaOpt * npts;
H_out(1, 1) += alphaOpt * npts;
H_out(2, 2) += alphaOpt * npts;

Vec3f tlog = refToNew.log().head<3>().cast<float>();
b_out[0] += tlog[0] * alphaOpt * npts;
b_out[1] += tlog[1] * alphaOpt * npts;
b_out[2] += tlog[2] * alphaOpt * npts；
```



---

## 边缘化（这里的边缘化是为了加速求解增量方程）

笔者在开始的时候就比较吐槽这个部分，因为作者在求解Jacobian的时候就在构建Schur补要用的各项矩阵用于边缘化，笔者在看代码的时候感到着实不好理解，所以这部分也重点记录一下：

### 理论部分

对于一个点$P_i$，其投影误差的Jacobian可以分为$\alpha$和$\beta$两个部分，其中$\alpha$部分是要保留的部分，这里是位姿和光度校正参数，$\beta$为边缘化掉的部分，这里为逆深度参数，公式如下：
$$
J_{n\times1} = [J_{\alpha}, J_{\beta}]
$$
所以对于增量方程有：
$$
\begin{bmatrix}H_{\alpha \alpha} & H_{\alpha \beta} \\ H_{\alpha \beta}^T & H_{\beta \beta}\end{bmatrix}\begin{bmatrix}\delta{x_{\alpha}} \\ \delta{x_{\beta}} \end{bmatrix} = \begin{bmatrix}b_{\alpha} \\ b_{\beta} \end{bmatrix}
$$
其中：
$$
\begin{aligned}
H_{\alpha \alpha}&=J_{\alpha}^T J_{\alpha} \\
H_{\alpha \beta} &=J_{\alpha}^T J_{\beta} \\
H_{\beta \beta}  &=J_{\beta}^T J_{\beta} \\
\end{aligned}
$$
而Schur补的公式如下，过程其实就跟解二元一次方程一样，这里不再赘述：
$$
\begin{aligned}
(H_{\alpha \alpha}-H_{\alpha \beta}H_{\beta \beta}^{-1}H_{\beta \alpha})\delta{x_{\alpha}}&=(b_{\alpha}-H_{\alpha \beta}H_{\beta \beta}^{-1}b_{\beta}) \\
\overline{H_{\alpha \alpha}} \delta{x_{\alpha}} &= \overline{b_{\alpha}}
\end{aligned}
$$
求解完$\delta{x_{\alpha}}$之后，带入原方程之中求解$\delta{x_{\beta}}$



### 代码部分

这部分在代码中有两个地方，第一个地方如下，求解的是要保留的$\alpha$部分的矩阵$H_{\alpha \alpha}$，其中dp0-7是$J_{\alpha}$，$r$是误差$e$，acc9最终矩阵的大小为$9\times9$，前$8\times8$为$H_{\alpha \alpha}=J_{alpha}^{T} J_{alpha}$，最后一列为$b_{\alpha}=J_{\alpha}^T e$：

```c++
// SSE每次处理4个数据，内部在做n×1×1×n的事情
for (int i = 0; i + 3 < patternNum; i += 4)
    acc9.updateSSE(
    _mm_load_ps(((float *) (&dp0)) + i),
    _mm_load_ps(((float *) (&dp1)) + i),
    _mm_load_ps(((float *) (&dp2)) + i),
    _mm_load_ps(((float *) (&dp3)) + i),
    _mm_load_ps(((float *) (&dp4)) + i),
    _mm_load_ps(((float *) (&dp5)) + i),
    _mm_load_ps(((float *) (&dp6)) + i),
    _mm_load_ps(((float *) (&dp7)) + i),
    _mm_load_ps(((float *) (&r)) + i));

// 处理剩下的数据，因为pattern有时候是奇数
for (int i = ((patternNum >> 2) << 2); i < patternNum; i++)
    acc9.updateSingle(
    (float) dp0[i], (float) dp1[i], (float) dp2[i], (float) dp3[i],
    (float) dp4[i], (float) dp5[i], (float) dp6[i], (float) dp7[i],
    (float) r[i]);
```

另一部分如下，其中JbBuffer0~7是$J^T_{\alpha}J_{\beta}$部分，而JbBuffer[8]是$J^T_{\beta}e$部分，JbBuffer[9]是$J^T_{\beta}J_{\beta}$部分：

```c++
// immediately compute dp*dd' and dd*dd' in JbBuffer1.
// 因为要把逆深度merge掉用于schur补计算，因此这里算的其实是W阵，merge掉深度值
JbBuffer_new[i][0] += dp0[idx] * dd[idx];
JbBuffer_new[i][1] += dp1[idx] * dd[idx];
JbBuffer_new[i][2] += dp2[idx] * dd[idx];
JbBuffer_new[i][3] += dp3[idx] * dd[idx];
JbBuffer_new[i][4] += dp4[idx] * dd[idx];
JbBuffer_new[i][5] += dp5[idx] * dd[idx];
JbBuffer_new[i][6] += dp6[idx] * dd[idx];
JbBuffer_new[i][7] += dp7[idx] * dd[idx];

// Jb part
JbBuffer_new[i][8] += r[idx] * dd[idx];

// Hv part
JbBuffer_new[i][9] += dd[idx] * dd[idx];
```

最后一部分是构建$H_{\alpha \beta}H_{\beta \beta}^{-1}H_{\beta \alpha}$和$H_{\alpha \beta}H_{\beta \beta}^{-1}b_{\beta}$，该部分均放在acc9SC中，其中前$8\times8$是$H_{\alpha \beta}H_{\beta \beta}^{-1}H_{\beta \alpha}$，最后一列$8\times1$是$H_{\alpha \beta}H_{\beta \beta}^{-1}b_{\beta}$，如下：

```c++
// 因为H_beta是对角矩阵，因此逆为个元素分之1，但是没有搞懂为啥分母多加了1
JbBuffer_new[i][9] = 1 / (1 + JbBuffer_new[i][9]);
// 内部在做n×1×1×1×1×n的事情，其中n=9（8+1）,中间的1是JbBuffer[9]，H_beta_beta
acc9SC.updateSingleWeighted(
    (float) JbBuffer_new[i][0], (float) JbBuffer_new[i][1], (float) JbBuffer_new[i][2],
    (float) JbBuffer_new[i][3],
    (float) JbBuffer_new[i][4], (float) JbBuffer_new[i][5], (float) JbBuffer_new[i][6],
    (float) JbBuffer_new[i][7],
    (float) JbBuffer_new[i][8], (float) JbBuffer_new[i][9]);
```



---

## 之后的部分

上述过程基本上都是函数calcResAndGS内部做的事情，其实整个trackFrame函数在做的就是一个L-M过程，这个过程可以在网上找到很多讲解，这里不赘述。

这里再记录一些变量：

- point->ir就是逆深度（至少笔者目前是这么觉得的）；
- point->iR是在neighbor窗口中的平滑值，且这个值会最后在金字塔的相邻层级下进行再一次的平滑（见propagateDown和propagateUp函数）；
- point->depth和point->newdepth都是在优化过程中迭代的量；

