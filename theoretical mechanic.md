主要是理论力学中的拉格朗日力学以及虚功原理部分，对于后续状态空间方程的推导以及VMC解算的推导比较重要。

# 理论力学基础

这部分内容是将偏置并联构型机器人抽象为轮式倒立摆机构、对整体轮式倒立摆进行分析的理论基础，对后续VMC相关内容的理解有很大帮助，但是概念较为抽象，可以先看后面的实际推导，遇到相应概念再往前查看。

## 约束

### 机械运动

机械运动的分类为

1. 自由运动。坐标、速度完全取决于有明确形式的力和初始条件。
2. 有约束运动（非自由运动）。坐标和速度存在一些形式上不涉及任何力的限制关系—约束物体的坐标和速度（初始条件）之间存在的一些限制关系。

### 约束

约束可以有以下分类方法

1. 完整约束（几何约束）；非完整约束（微分约束）
2. 稳定约束；非稳定约束
3. 可解约束；不可解约束

>[!note] **完整约束**（几何约束）
>$$
>f(\vec{r}_{1},\vec{r}_{2},\dots \vec{r}_{N};t)=0
>$$
>即仅与各质点坐标有关， 而与各质点的速度无关。例如，质点被约束在约束在曲线或者曲面上运动， 约束方程就是该曲线或者曲面。
>1. 稳定的几何约束 $f(\vec{r}_{1},\vec{r}_{2},\dots \vec{r}_{N})=0$
>2. 不稳定的几何约束 $f(\vec{r}_{1},\vec{r}_{2},\dots \vec{r}_{N};t)=0$

>[!note] **非完整约束**
>$$
>f(\vec{r},\dot{\vec{r}},t)=0
>$$
>或微分形式
>$$
>\sum c_{i}d\vec{r}_{i}+dt=0
>$$
>如果可以积分，则可积分为完整约束；不可积分，才是非完整约束

## 广义坐标

### 自由度

>[!note] **自由度：系统独立变量数**
>1. 自由运动。$N$ 个质点共 $3N$ 个独立参量描述，自由度为：$3N$ 
>2. 约束运动。$N$ 个质点间存在 $k$ 个完整约束。自由度为：$s = 3N - k$

### 广义坐标

>[!note] 广义坐标
>如果系统可以用 $s$ （系统自由度）个独立坐标来描述，这些独立变量称为广义坐标，如
>$$
>\vec{r}_{i}=\vec{r}_{i}(q_{1},q_{2},\dots,q_{s};t),i=1,2,\dots,N
>$$
>>[!tip] 
>>对于广义坐标的选用，可以选取长度、角度、坐标等等，有无数多选取方法，遵循**简洁**且最适合实际物理模型的原则。

>[!note] 位形空间
>广义坐标张成的空间 $\mathbb{R}^{s}$

>[!note] 广义速度
>广义坐标的微分 $\dot{q}_{\alpha}(\alpha=1,2,\dots,s)$

## 拉格朗日量

>[!note] 拉格朗日量
>定义为广义坐标下，动能与势能之差
>$$
>L(q,\dot{q};t)=T(\dot{q};t)-V(q;t)
>$$
## 虚功原理

### 虚位移

>[!note] 虚位移
>假想系统的各质点瞬时发生了微小的符合约束条件的位移，称为虚位移
>1. 不需要时间
>2. 不必实际发生，无限多种
>3. 满足约束条件

>[!abstract] 虚位移（变分）与实位移（微分）
>符号约定
>4. 实位移：$d\vec{r}$，全微分
>5. 虚位移：$\delta \vec{r}$，等时变分 $\delta t=0$
>
>区别
>6. 虚位移是瞬时完成的（$dt = 0$）或者说是假想的偏离真实物理位置的移动， 不需要时间；而实位移需要一小段时间（$dt \neq 0$）
>7. 虚位移在满足约束的条件下可以任意选取，并未真实发生，而实位移一般与质点的真实运动相关
>8. 虚位移的方向无论是稳定约束还是非稳定约束，都是沿着约束的切线方向， 而实位移在非稳定约束时，不一定沿着约束的切线方向

### 理想约束

>[!note] 理想约束
>$$
>\sum \vec{R}_{i}\cdot \delta \vec{r}_{i}=0
>$$

例如

1. 物体在光滑曲面运动 $\vec{R}\perp \delta \vec{r}$
2. 刚性约束：约束力成对出现 $\vec{R}_{1}+\vec{R}_{2}=0$
3. 接触约束：将摩擦力当成主动力

### 虚功原理

>[!note] 虚功原理
>在理想约束条件下，主动力所做的总虚功为零
>$$
>\delta W=\sum F_{i}\cdot \delta r_{i}=0
>$$

>[!note] 虚功
>力在虚位移下所做的假想的功
>$$
>\delta W=\sum(F_{i}+R_{i})\cdot r_{i}
>$$

虚功原理主要用于静力学分析，所有质点处于平衡态时，主动力与约束力平衡
$$
F_{i}+R_{i}=0
$$
$$
\delta W=\sum(F_{i}+R_{i})\cdot r_{i}=0
$$
对于理想约束
$$
\delta W=\sum F_{i}\cdot \delta r_{i}=0
$$
使用广义坐标，方程可以化为
$$
\sum_{\alpha=1}^{s}\sum_{i=1}^{N}F_{i}\cdot \frac{ \partial r_{i} }{ \partial q_{\alpha} } \delta q_{\alpha}=0
$$
由于广义坐标是独立变量，引入广义力定义

>[!note] 广义力
>$$
>Q_{\alpha}=\sum_{i=1}^{N}F_{i}\cdot \frac{ \partial r_{i} }{ \partial q_{\alpha} } 
>$$

那么
$$
\sum_{\alpha=1}^{s}Q_{\alpha}\delta q_{\alpha}=0
$$
由广义坐标的独立性
$$
Q_{\alpha}=0
$$
对于保守力体系
$$
F_{i}=-\nabla_{i}V
$$
$$
Q_{\alpha}=-\sum_{i=1}^{N}\nabla _{i}V\cdot \frac{ \partial r_{i} }{ \partial q_{\alpha} } =-\frac{ \partial V }{ \partial q_{\alpha} } =0
$$
$$
\delta W=\sum_{i=1}^{N}F_{i}\cdot \delta r_{i}=\sum_{\alpha=1}^{s}Q_{\alpha}\delta q_{\alpha}=-\sum_{\alpha=1}^{s}\frac{ \partial V }{ \partial q_{\alpha} } \delta q_{\alpha}=-\delta V=0
$$
## 达朗贝尔原理

>[!note] 达朗贝尔原理
>主动力和惯性力所做的总虚功为0
>$$
>\sum(F_{i}-m_{i}\ddot{r_{i}})\cdot \delta r_{i}=0
>$$

从每个质点的运动方程开始
$$
m_{i}\ddot{r_{i}}=F_{i}+R_{i},i=1,2,\dots,N
$$
$$
F_{i}+R_{i}-m_{i}\ddot{r_{i}}=0,i=1,2,\dots,N
$$
即
$$
\sum(F_{i}+R_{i}-m_{i}\ddot{r_{i}})\delta r_{i}=0
$$
代入理想约束，即为
$$
\sum(F_{i}-m_{i}\ddot{r_{i}})\cdot \delta r_{i}=0
$$
## 拉格朗日方程

这部分是将前面“达朗贝尔原理”与“广义坐标”结合的核心结论，它避免了直接对系统内部约束力进行复杂的矢量分析，而是纯粹从系统的能量标量出发推导系统的动力学方程，是多刚体系统（如机器人）建模的最主要工具。

### 动力学普遍方程

将达朗贝尔原理推导出的方程通过广义坐标展开。
已知达朗贝尔原理（在理想约束下）：
$$
\sum(F_{i}-m_{i}\ddot{\vec{r}_{i}})\cdot \delta \vec{r}_{i}=0
$$
利用实位移与广义坐标的关系 $\vec{r}_{i}=\vec{r}_{i}(q_{1},q_{2},\dots,q_{s};t)$，其虚位移可以表示为广义坐标变分的线性组合：
$$
\delta \vec{r}_{i}=\sum_{\alpha=1}^{s}\frac{\partial \vec{r}_{i}}{\partial q_{\alpha}}\delta q_{\alpha}
$$
代入达朗贝尔原理并交换求和顺序，可得到由广义坐标表示的**动力学普遍方程**：
$$
\sum_{\alpha=1}^{s}\left[ \sum_{i=1}^{N}F_{i}\cdot \frac{\partial \vec{r}_{i}}{\partial q_{\alpha}} - \sum_{i=1}^{N}m_{i}\ddot{\vec{r}_{i}}\cdot \frac{\partial \vec{r}_{i}}{\partial q_{\alpha}} \right]\delta q_{\alpha}=0
$$

### 第二类拉格朗日方程

由前文可知，主动力所做的虚功系数就是**广义力** $Q_{\alpha}$：
$$
Q_{\alpha}=\sum_{i=1}^{N}F_{i}\cdot \frac{ \partial \vec{r}_{i} }{ \partial q_{\alpha} }
$$
对于惯性力做功部分，引入系统的总**动能** $T$：
$$
T = \sum_{i=1}^{N}\frac{1}{2}m_{i}\dot{\vec{r}}_{i}\cdot \dot{\vec{r}}_{i}
$$
经过运动学求导的代数变换（利用偏导数恒等式 $\frac{\partial \dot{\vec{r}}_{i}}{\partial \dot{q}_{\alpha}} = \frac{\partial \vec{r}_{i}}{\partial q_{\alpha}}$ 以及对时间的全导数法则），惯性力项可以奇迹般地化为纯动能的偏导数形式：
$$
\sum_{i=1}^{N}m_{i}\ddot{\vec{r}_{i}}\cdot \frac{\partial \vec{r}_{i}}{\partial q_{\alpha}} = \frac{d}{dt}\left( \frac{\partial T}{\partial \dot{q}_{\alpha}} \right) - \frac{\partial T}{\partial q_{\alpha}}
$$

>[!note] 拉格朗日方程（一般形式）
>由于系统的广义坐标 $q_{\alpha}$ 是相互独立的（其变分 $\delta q_{\alpha}$ 任意且不为0），动力学普遍方程中各项的系数必然等于零。由此得到**第二类拉格朗日方程**：
>$$
>\frac{d}{dt}\left( \frac{\partial T}{\partial \dot{q}_{\alpha}} \right) - \frac{\partial T}{\partial q_{\alpha}} = Q_{\alpha} \quad (\alpha=1, 2, \dots, s)
>$$

### 保守系统与拉格朗日方程

当系统受到的主动力均为**保守力**（如重力、理想弹簧力等）时，广义力可以完全由系统的势能 $V$ 导出：
$$
Q_{\alpha} = -\frac{\partial V}{\partial q_{\alpha}}
$$
由于宏观力学中势能 $V$ 通常只是位置坐标 $q$ 的函数，与广义速度 $\dot{q}$ 无关（即 $\frac{\partial V}{\partial \dot{q}_{\alpha}} = 0$）。代入前面定义的**拉格朗日量** $L = T - V$：
$$
\frac{\partial L}{\partial \dot{q}_{\alpha}} = \frac{\partial T}{\partial \dot{q}_{\alpha}} \quad , \quad \frac{\partial L}{\partial q_{\alpha}} = \frac{\partial T}{\partial q_{\alpha}} - \frac{\partial V}{\partial q_{\alpha}}
$$

>[!note] 拉格朗日方程（保守系统）
>对于保守系统，动力学方程可以表示为极其优美对称的形式，完全由拉格朗日量 $L$ 决定：
>$$
>\frac{d}{dt}\left( \frac{\partial L}{\partial \dot{q}_{\alpha}} \right) - \frac{\partial L}{\partial q_{\alpha}} = 0
>$$

### 非保守力与受控系统（机器人建模基础）

在实际的机械与控制系统中（如轮式倒立摆、偏置并联构型机器人），除了重力等保守力外，不可避免地存在电机的驱动力矩、地面的摩擦力等**非保守力**。

>[!note] 包含非保守广义力的拉格朗日方程
>将广义力 $Q_{\alpha}$ 分解为保守力部分（归入势能 $V$ 形成拉格朗日量 $L$）和非保守力部分 $Q_{\alpha}^{nc}$（包含控制输入 $\tau$ 和阻尼/摩擦等）：
>$$
>\frac{d}{dt}\left( \frac{\partial L}{\partial \dot{q}_{\alpha}} \right) - \frac{\partial L}{\partial q_{\alpha}} = Q_{\alpha}^{nc}
>$$

### 机器人标准动力学方程的推导

在多刚体系统（如轮式倒立摆、机械臂）的建模与控制中，通常需要将拉格朗日方程化为极其规律的矩阵形式。以下是从拉格朗日方程到标准动力学方程 $M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = B(q)\tau$ 的严密推导。

#### 1. 系统的能量表达
假设系统有 $n$ 个自由度，广义坐标为向量 $q \in \mathbb{R}^n$。
- **动能 $T$** 通常可以表示为广义速度的二次型：
  $$T = \frac{1}{2}\dot{q}^T M(q)\dot{q} = \frac{1}{2}\sum_{i=1}^{n}\sum_{j=1}^{n} M_{ij}(q)\dot{q}_i\dot{q}_j$$
  其中，$M(q)$ 是 $n \times n$ 的**惯性矩阵**（对称且正定），它包含了系统质量分布随位形 $q$ 变化的信息。
- **势能 $V$** 在一般机械系统中通常仅取决于系统的位置（如重力势能）：
  $$V = V(q)$$
- **拉格朗日量 $L$** 定义为：
  $$L = T - V = \frac{1}{2}\sum_{i=1}^{n}\sum_{j=1}^{n} M_{ij}(q)\dot{q}_i\dot{q}_j - V(q)$$

#### 2. 代入拉格朗日方程
对第 $k$ 个广义坐标，应用包含非保守广义力 $\tau_k$ 的拉格朗日方程：
$$\frac{d}{dt}\left( \frac{\partial L}{\partial \dot{q}_k} \right) - \frac{\partial L}{\partial q_k} = \tau_k$$

**第一步：计算对广义速度的偏导与时间全导数**
先求对速度的偏导（即广义动量）：
$$\frac{\partial L}{\partial \dot{q}_k} = \sum_{j=1}^{n} M_{kj}(q)\dot{q}_j$$
再对其求时间 $t$ 的全导数。由于惯性矩阵元素 $M_{kj}(q)$ 随位形 $q$ 变化，而 $q$ 随时间变化，这里必须应用链式法则：
$$\frac{d}{dt}\left( \frac{\partial L}{\partial \dot{q}_k} \right) = \sum_{j=1}^{n} M_{kj}(q)\ddot{q}_j + \sum_{j=1}^{n} \dot{M}_{kj}(q)\dot{q}_j$$
将 $\dot{M}_{kj}(q)$ 展开为 $\sum_{i=1}^{n} \frac{\partial M_{kj}}{\partial q_i}\dot{q}_i$，得到：
$$\frac{d}{dt}\left( \frac{\partial L}{\partial \dot{q}_k} \right) = \sum_{j=1}^{n} M_{kj}(q)\ddot{q}_j + \sum_{i=1}^{n}\sum_{j=1}^{n} \frac{\partial M_{kj}}{\partial q_i}\dot{q}_i\dot{q}_j$$

**第二步：计算对广义坐标的偏导**
$$\frac{\partial L}{\partial q_k} = \frac{1}{2}\sum_{i=1}^{n}\sum_{j=1}^{n} \frac{\partial M_{ij}}{\partial q_k}\dot{q}_i\dot{q}_j - \frac{\partial V}{\partial q_k}$$

**第三步：合并方程**
将上述两部分的推导代回拉格朗日方程中，得到第 $k$ 个坐标的动力学展开式：
$$\sum_{j=1}^{n} M_{kj}(q)\ddot{q}_j + \sum_{i=1}^{n}\sum_{j=1}^{n} \left( \frac{\partial M_{kj}}{\partial q_i} - \frac{1}{2}\frac{\partial M_{ij}}{\partial q_k} \right)\dot{q}_i\dot{q}_j + \frac{\partial V}{\partial q_k} = \tau_k$$

#### 3. 整理出标准矩阵项（Christoffel符号引入）
方程中间那一团复杂的双重求和 $\sum\sum(\dots)\dot{q}_i\dot{q}_j$ 代表了科氏力与离心力。为了写成优美的矩阵相乘形式，我们利用 $i$ 和 $j$ 的对称性，将括号内的系数重写为**第一类克里斯托费尔符号（Christoffel symbols of the first kind）** $c_{ijk}$：
$$c_{ijk} = \frac{1}{2}\left( \frac{\partial M_{kj}}{\partial q_i} + \frac{\partial M_{ki}}{\partial q_j} - \frac{\partial M_{ij}}{\partial q_k} \right)$$
由此，我们可以定义**科氏力与离心力矩阵** $C(q, \dot{q})$，其第 $k$ 行第 $j$ 列的元素定义为：
$$C_{kj} = \sum_{i=1}^{n} c_{ijk}\dot{q}_i$$
这样一来，中间复杂的非线性项就被完美打包成了向量 $C(q,\dot{q})\dot{q}$ 的第 $k$ 个元素。

同时，将势能对坐标的偏导定义为**重力项向量** $G(q)$ 的第 $k$ 个元素：
$$G_k = \frac{\partial V}{\partial q_k}$$

>[!note] 机器人标准动力学方程
>将所有 $k = 1, 2, \dots, n$ 个标量方程按照列向量的形式堆叠起来，并引入执行器映射矩阵 $B(q)$（将实际驱动器的力矩向量 $\tau$ 映射到广义力上），最终推导出了控制理论中标准的矩阵方程：
>$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = B(q)\tau$$
>
>这个推导过程完美展示了如何从系统**能量标量**出发，通过变分与求导计算，自然而然地剥离并归纳出**惯性项 $M$**、**非线性交叉耦合项 $C$** 和**保守力项 $G$**。这是连接理论力学与现代控制理论（尤其是状态空间与反馈线性化控制）的核心桥梁。

