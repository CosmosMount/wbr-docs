# 物理建模与控制器

  

物理建模的方式直接决定了控制器的设计和控制上限，对于建模而言，并不是一定是越精确越细致就越好，模型精确依赖数据精确，模型粗糙依赖更多处理，下面的物理建模按照从易到难，从简单到复杂，展现了三个发展阶段。

  

在三种建模的发展历程中，受力分析完全可复用，转动分析和加速度分析部分可复用，那么这就很好地简化了我们在切换模型时的分析成本，按照顺序阅读三种建模，会对整个系统有更好地认识。下面的建模均是从原来学校的开源发展修改而来，统一并优化了符号定义和坐标系约定，使各种建模具有更好的一致性和连贯性，每一种都会具备完整的推导，但可以注意可复用部分的内容。

  

## 约定

  

一切力以竖直向上、水平向左为正方向

  

| 物理意义      | 数学符号                                            |

| --------- | ----------------------------------------------- |

| 轮主动力矩     | $\tau_{w,l},\tau_{w,r}$                         |

| 髋主动力矩     | $\tau_{l,l},\tau_{l,r}$                         |

| 腿对机身水平作用力 | $F_{l,l}^{h},F_{l,r}^h$                         |

| 腿对机身垂直作用力 | $F_{l,l}^{v},F_{l,r}^v$                         |

| 轮对腿水平作用力  | $F_{w,l}^{h},F_{w,r}^h$                         |

| 轮对腿垂直作用力  | $F_{w,l}^{v},F_{w,r}^v$                         |

| 轮质量       | $m_{w}$                                         |

| 腿质量       | $m_{l}$                                         |

| 机体质量      | $m_{b}$                                         |

| 位移相关      | $x,\dot{x},\ddot{x}$                            |

| yaw       | $\phi,\dot{\phi}, \ddot{\phi}$                  |

| pitch     | $\theta_{b},\dot{\theta}_{b},\ddot{\theta}_{b}$ |

| 摆角        | $\theta_{l},\dot{\theta}_{l},\ddot{\theta}_{l}$ |

| 轮角        | $\theta_{w},\dot{\theta}_{w},\ddot{\theta}_{w}$ |

| 轮转轴到腿转轴距离 | $l,l_{l},l_{r}$ |

| 轮转轴到腿质心距离 | $l_{w},l_{w,l},l_{w,r}$ |

| 腿转轴到腿质心距离 | $l_{b},l_{b,l},l_{b,r}$ |

| 轮加速度      | $a_{w}^h,a_{w}^v$                               |

| 腿加速度      | $a_{l}^h,a_{l}^v$                               |

| 机体加速度     | $a_{b}^h,a_{b}^v$                               |

| 机体质心偏移    | $d_{b},\theta_{b}^0$                            |

| 腿质心偏移     | $d_{l},\theta_{l}^0$                            |

  

我们的分析过程遵循自下而上的顺序，保证连贯性和前后的依赖。

  

## 单腿建模

  

将双腿等价为一条腿进行统一建模，极大程度地降低了建模复杂度，但同时会带来较多层级的控制器设计，由哈工程王洪玺首次提出。

  

### 轮质心

受力分析

$$

-F_{w}^{h}+f=m_{w}a_{w}^{h},\quad -F_{w}^{v}+F_{N}-m_{w}g=m_{w}a_{w}^{v}

$$

（单腿建模下竖直式一般不用）

加速度：$a_{w}^h=\ddot{x},\,a_{w}^{v}=0$

转动：$I \ddot{\theta}_{w}=\tau_{w}-fR_{w}$

### 腿质心

受力分析

$$

-F_{l}^{h}+F_{w}^{h}=m_{l}a_{l}^{h},\quad -F_{l}^{v}+F_{w}^{v}-m_{l}g=m_{l}a_{l}^{v}

$$

加速度

$$

a_{l}^{h}=a_{w}^{h}+\frac{ \partial ^2 }{ \partial t } (l_{w}\sin \theta_{l})=\ddot{x}+l_{w}\cos \theta_{l}\ddot{\theta_{l}}-l_{w}\sin \theta_{l}\dot{\theta}_{l}^{2}

$$

$$

a_{l}^{v}=a_{w}^{v}+\frac{ \partial ^{2} }{ \partial t } (l_{w}\cos \theta_{l})=-l_{w}\sin \theta_{l}\ddot{\theta_{l}}-l_{w}\cos \theta_{l}\dot{\theta}_{l}^{2}

$$

转动

$$

I \ddot{\theta}_{l}=\tau_{l}-\tau_{w}+(F_{w}^{v}l_{w}+F_{l}^{v}l_{b})\sin \theta_{l}-(F_{w}^{h}l_{w}+F_{l}^{h}l_{b})\cos \theta_{l}

$$

### 机体质心

受力分析

$$

F_{l}^{h}=m_{b}a_{b}^{h},\quad F_{l}^{v}-m_{b}g=m_{b}a_{b}^v

$$

加速度

$$

a_{b}^{h}=a_{w}^{h}+\frac{ \partial ^2 }{ \partial t } (l\sin \theta_{l})=\ddot{x}+l\cos \theta_{l}\ddot{\theta_{l}}-l\sin \theta_{l}\dot{\theta}_{l}^{2}

$$

$$

a_{b}^{v}=a_{w}^{v}+\frac{ \partial ^{2} }{ \partial t } (l\cos \theta_{l})=-l\sin \theta_{l}\ddot{\theta_{l}}-l\cos \theta_{l}\dot{\theta}_{l}^{2}

$$

转动：$I \ddot{\theta}_{b}=-\tau_{l}-m_{b}gd_{b}\cos(\theta_{b}+\theta_{b}^{0})+F_{l}^{v}\sin \theta_{b}-F_{l}^{h}d_{b}\cos \theta_{b}$

### 控制律

  

我们需要

$$

x=\left[ \begin{matrix}

x \\

\dot{x} \\

\theta_{l} \\

\dot{\theta}_{l} \\

\theta_{b} \\

\dot{\theta}_{b}

\end{matrix}

 \right]

 ,

 \dot{x}=\left[ \begin{matrix}

\dot{x} \\

\ddot{x} \\

\dot{\theta}_{l} \\

\ddot{\theta}_{l} \\

\dot{\theta}_{b} \\

\ddot{\theta}_{b}

\end{matrix}

 \right]

 ,

  u=\left[ \begin{matrix}

\tau_{w} \\

\tau_{l}

\end{matrix}

 \right]

$$

最终期望整理出

$$

\dot{x}=Ax+Bu

$$

### 消去力

  

由 $F_{w,l}^{v}=F_{w,r}^{v}$、水平/竖直受力平衡及 $f_i=(\tau_{w,i}-I\ddot{\theta}_{w,i})/R_w$（单腿 $\ddot{\theta}_w=\ddot{x}/R_w$）联立，消去全部 $F,f,a$。下列仅含 $x,\theta_l,\theta_b$ 及其导数与 $\tau$。

  

水平（消 $f$）：

$$

\big(m_{w}+m_{l}+m_{b}+\tfrac{I}{R_{w}^{2}}\big)\ddot{x}+(m_{l}l_{w}+m_{b}l)\cos \theta_{l}\ddot{\theta}_{l}=\frac{\tau_{w}}{R_{w}}+(m_{l}l_{w}+m_{b}l)\sin \theta_{l}\dot{\theta}_{l}^{2}

$$

单腿无 yaw；左右对称时 $\dot{\phi}=0$。

腿部转动：

$$

\begin{aligned}

I\ddot{\theta}_{l}=&\,\tau_{l}-\tau_{w} \\

&+\Big[(m_{l}+m_{b})g+m_{l}\big(-l_{w}\sin \theta_{l}\ddot{\theta}_{l}-l_{w}\cos \theta_{l}\dot{\theta}_{l}^{2}\big)+m_{b}\big(-l\sin \theta_{l}\ddot{\theta}_{l}-l\cos \theta_{l}\dot{\theta}_{l}^{2}\big)\Big]l_{w}\sin \theta_{l} \\

&+m_{b}\Big(g-l\sin \theta_{l}\ddot{\theta}_{l}-l\cos \theta_{l}\dot{\theta}_{l}^{2}\Big)l_{b}\sin \theta_{l} \\

&-\Big[m_{l}l_{w}\big(\ddot{x}+l_{w}\cos \theta_{l}\ddot{\theta}_{l}-l_{w}\sin \theta_{l}\dot{\theta}_{l}^{2}\big)+m_{b}(l_{w}+l_{b})\big(\ddot{x}+l\cos \theta_{l}\ddot{\theta}_{l}-l\sin \theta_{l}\dot{\theta}_{l}^{2}\big)\Big]\cos \theta_{l}

\end{aligned}

$$

机体转动：

$$

\begin{aligned}

I\ddot{\theta}_{b}=&\,-\tau_{l}-m_{b}gd_{b}\cos(\theta_{b}+\theta_{b}^{0})+m_{b}g\sin\theta_{b} \\

&+m_{b}\big(l\sin\theta_{l}\ddot{\theta}_{l}+l\cos\theta_{l}\dot{\theta}_{l}^{2}\big)\sin\theta_{b} \\

&-m_{b}\big(\ddot{x}+l\cos\theta_{l}\ddot{\theta}_{l}-l\sin\theta_{l}\dot{\theta}_{l}^{2}\big)d_{b}\cos\theta_{b}

\end{aligned}

$$

  

## 双腿建模

  

将双腿分开建模，并将yaw实际作为状态变量进行计算，能够对双腿实现更精细的控制，并消除了调节劈叉环的过程，由上交首次提出，和单腿建模均为当前最主流建模。

  

### 轮质心

#### 左轮

$$

-F_{w,l}^{h}+f_{l}=m_{w}a_{w,l}^{h},\quad -F_{w,l}^{v}+F_{N,l}-m_{w}g=m_{w}a_{w,l}^{v}

$$

$a_{w,l}^h=\ddot{x},\,a_{w,l}^{v}=0$，$I \ddot{\theta}_{w,l}=\tau_{w,l}-f_{l}R_{w}$

#### 右轮

$$

-F_{w,r}^{h}+f_{r}=m_{w}a_{w,r}^{h},\quad -F_{w,r}^{v}+F_{N,r}-m_{w}g=m_{w}a_{w,r}^{v}

$$

$a_{w,r}^h=\ddot{x},\,a_{w,r}^{v}=0$，$I \ddot{\theta}_{w,r}=\tau_{w,r}-f_{r}R_{w}$

### 腿质心

#### 左腿

$$

-F_{l,l}^{h}+F_{w,l}^{h}=m_{l}a_{l,l}^{h},\quad -F_{l,l}^{v}+F_{w,l}^{v}-m_{l}g=m_{l}a_{l,l}^{v}

$$

$$

a_{l,l}^{h}=a_{w,l}^{h}+\frac{ \partial ^{2} }{ \partial t } (l_{w,l}\sin \theta_{l,l})=\ddot{x}+l_{w,l}\cos \theta_{l,l}\ddot{\theta}_{l,l}-l_{w,l}\sin \theta_{l,l}\dot{\theta}_{l,l}^{2}

$$

$$

a_{l,l}^{v}=a_{w,l}^{v}+\frac{ \partial ^{2} }{ \partial t } (l_{w,l}\cos \theta_{l,l})=-l_{w,l}\sin \theta_{l,l}\ddot{\theta_{l,l}}-l_{w,l}\cos \theta_{l,l}\dot{\theta}_{l,l}^{2}

$$

$$

I_{l,l}\ddot{\theta}_{l,l}=\tau_{l,l}-\tau_{w,l}+(F_{w,l}^{v}l_{w,l}+F_{l,l}^{v}l_{b,l})\sin \theta_{l,l}-(F_{w,l}^{h}l_{w,l}+F_{l,l}^{h}l_{b,l})\cos \theta_{l,l}

$$

#### 右腿

$$

-F_{l,r}^{h}+F_{w,r}^{h}=m_{l}a_{l,r}^{h},\quad -F_{l,r}^{v}+F_{w,r}^{v}-m_{l}g=m_{l}a_{l,r}^{v}

$$

$$

a_{l,r}^{h}=a_{w,r}^{h}+\frac{ \partial ^{2} }{ \partial t } (l_{w,r}\sin \theta_{l,r})=\ddot{x}+l_{w,r}\cos \theta_{l,r}\ddot{\theta}_{l,r}-l_{w,r}\sin \theta_{l,r}\dot{\theta}_{l,r}^{2}

$$

$$

a_{l,r}^{v}=a_{w,r}^{v}+\frac{ \partial ^{2} }{ \partial t } (l_{w,r}\cos \theta_{l,r})=-l_{w,r}\sin \theta_{l,r}\ddot{\theta_{l,r}}-l_{w,r}\cos \theta_{l,r}\dot{\theta}_{l,r}^{2}

$$

$$

I_{l,r}\ddot{\theta}_{l,r}=\tau_{l,r}-\tau_{w,r}+(F_{w,r}^{v}l_{w,r}+F_{l,r}^{v}l_{b,r})\sin \theta_{l,r}-(F_{w,r}^{h}l_{w,r}+F_{l,r}^{h}l_{b,r})\cos \theta_{l,r}

$$

### 机体质心

$$

F_{l,l}^{h}+F_{l,r}^{h}=m_{b}a_{b}^{h},\quad F_{l,l}^{v}+F_{l,r}^{v}-m_{b}g=m_{b}a_{b}^{v}

$$

$$

\begin{align}

a_{b}^{h} & =\frac{ \partial ^2 }{ \partial t } \frac{1}{2}(l_{l}\sin \theta_{l,l}+l_{r}\sin \theta_{l,r}) \\

 & =\ddot{x}+\frac{1}{2}l_{l}\cos \theta_{l,l}\ddot{\theta}_{l,l}-\frac{1}{2}l_{l}\sin \theta_{l,l}\dot{\theta}_{l,l}^{2}+\frac{1}{2}l_{r}\cos \theta_{l,r}\ddot{\theta}_{l,r}-\frac{1}{2}l_{r}\sin \theta_{l,r}\dot{\theta}_{l,r}^{2}

\end{align}

$$

$$

\begin{align}

a_{b}^{v} & =\frac{ \partial ^{2} }{ \partial t } \frac{1}{2}(l_{l} \cos \theta_{l,l}+l_{r}\cos \theta_{l,r}) \\

 & =-\frac{1}{2}l_{l}\sin \theta_{l,l}\ddot{\theta}_{l,l}-\frac{1}{2}l_{l}\cos \theta_{l,l}\dot{\theta}_{l,l}^{2}-\frac{1}{2}l_{r}\sin \theta_{l,r}\ddot{\theta}_{l,r}-\frac{1}{2}l_{r}\cos \theta_{l,r}\dot{\theta}_{l,r}^{2}

\end{align}

$$

$$

I_{b}\ddot{\theta}_{b}=-\tau_{l,l}-\tau_{l,r}-m_{b}gd_{b}\cos(\theta_{b}+\theta_{b}^{0})+(F_{l,l}^{v}+F_{l,r}^{v})\sin \theta_{b}-(F_{l,l}^{h}+F_{l,r}^{h})d_{b}\cos \theta_{b}

$$

$$

I_{\phi}\ddot{\phi}=(f_{r}-f_{l})R_{b}

$$

  

### 消去力

  

由 $F_{w,l}^{v}=F_{w,r}^{v}$、水平/竖直受力平衡及 $f_i=(\tau_{w,i}-I\ddot{\theta}_{w,i})/R_w$ 联立，消去全部 $F,f,a$。下列仅含 $x,\phi,\theta_{l,i},\theta_b,\theta_{w,i}$ 及其导数与 $\tau$。

  

水平（消 $f$）：

$$

\big(2m_{w}+2m_{l}+m_{b}+\tfrac{2I}{R_{w}^{2}}\big)\ddot{x}+\frac{m_{b}}{2}\big(l_{l}\cos\theta_{l,l}\ddot{\theta}_{l,l}+l_{r}\cos\theta_{l,r}\ddot{\theta}_{l,r}\big)+m_{l}\big(l_{w,l}\cos\theta_{l,l}\ddot{\theta}_{l,l}+l_{w,r}\cos\theta_{l,r}\ddot{\theta}_{l,r}\big)=\frac{\tau_{w,l}+\tau_{w,r}}{R_{w}}+m_{b}\Big[-\tfrac{l_{l}}{2}\sin\theta_{l,l}\dot{\theta}_{l,l}^{2}-\tfrac{l_{r}}{2}\sin\theta_{l,r}\dot{\theta}_{l,r}^{2}\Big]+m_{l}\Big[-l_{w,l}\sin\theta_{l,l}\dot{\theta}_{l,l}^{2}-l_{w,r}\sin\theta_{l,r}\dot{\theta}_{l,r}^{2}\Big]

$$

yaw（消 $f$）：

$$

I_{\phi}\ddot{\phi}=\frac{R_{b}}{R_{w}}\Big[(\tau_{w,r}-I\ddot{\theta}_{w,r})-(\tau_{w,l}-I\ddot{\theta}_{w,l})\Big]

$$

左腿转动（$l_l=l_{w,l}+l_{b,l}$）：

$$

\begin{aligned}

I_{l,l}\ddot{\theta}_{l,l}=&\,\tau_{l,l}-\tau_{w,l}\big(1+\tfrac{l_l}{R_w}\big)+\tfrac{I}{R_w}l_l\ddot{\theta}_{w,l}+\tfrac{m_b+2m_l}{2}gl_l\sin\theta_{l,l}-m_l l_{b,l}g\sin\theta_{l,l} \\

&+m_l l_{b,l}l_{w,l}\sin^2\theta_{l,l}\,\ddot{\theta}_{l,l}+m_l l_{b,l}l_{w,l}\sin\theta_{l,l}\cos\theta_{l,l}\,\dot{\theta}_{l,l}^{2} \\

&-\tfrac{m_b}{4}l_l\sin\theta_{l,l}\big[l_l\sin\theta_{l,l}\ddot{\theta}_{l,l}+l_l\cos\theta_{l,l}\dot{\theta}_{l,l}^{2}+l_r\sin\theta_{l,r}\ddot{\theta}_{l,r}+l_r\cos\theta_{l,r}\dot{\theta}_{l,r}^{2}\big] \\

&-\tfrac{m_l}{2}l_l\sin\theta_{l,l}\big[l_{w,l}\sin\theta_{l,l}\ddot{\theta}_{l,l}+l_{w,l}\cos\theta_{l,l}\dot{\theta}_{l,l}^{2}+l_{w,r}\sin\theta_{l,r}\ddot{\theta}_{l,r}+l_{w,r}\cos\theta_{l,r}\dot{\theta}_{l,r}^{2}\big] \\

&-m_w\ddot{x}\,l_{w,l}\cos\theta_{l,l}-(m_w-m_l)\ddot{x}\,l_{b,l}\cos\theta_{l,l}+m_l l_{w,l}l_{b,l}\cos^2\theta_{l,l}\,\ddot{\theta}_{l,l}-m_l l_{w,l}l_{b,l}\sin\theta_{l,l}\cos\theta_{l,l}\,\dot{\theta}_{l,l}^{2}

\end{aligned}

$$

右腿转动（下标 $l\to r$，$l_r=l_{w,r}+l_{b,r}$）：

$$

\begin{aligned}

I_{l,r}\ddot{\theta}_{l,r}=&\,\tau_{l,r}-\tau_{w,r}\big(1+\tfrac{l_r}{R_w}\big)+\tfrac{I}{R_w}l_r\ddot{\theta}_{w,r}+\tfrac{m_b+2m_l}{2}gl_r\sin\theta_{l,r}-m_l l_{b,r}g\sin\theta_{l,r} \\

&+m_l l_{b,r}l_{w,r}\sin^2\theta_{l,r}\,\ddot{\theta}_{l,r}+m_l l_{b,r}l_{w,r}\sin\theta_{l,r}\cos\theta_{l,r}\,\dot{\theta}_{l,r}^{2} \\

&-\tfrac{m_b}{4}l_r\sin\theta_{l,r}\big[l_l\sin\theta_{l,l}\ddot{\theta}_{l,l}+l_l\cos\theta_{l,l}\dot{\theta}_{l,l}^{2}+l_r\sin\theta_{l,r}\ddot{\theta}_{l,r}+l_r\cos\theta_{l,r}\dot{\theta}_{l,r}^{2}\big] \\

&-\tfrac{m_l}{2}l_r\sin\theta_{l,r}\big[l_{w,l}\sin\theta_{l,l}\ddot{\theta}_{l,l}+l_{w,l}\cos\theta_{l,l}\dot{\theta}_{l,l}^{2}+l_{w,r}\sin\theta_{l,r}\ddot{\theta}_{l,r}+l_{w,r}\cos\theta_{l,r}\dot{\theta}_{l,r}^{2}\big] \\

&-m_w\ddot{x}\,l_{w,r}\cos\theta_{l,r}-(m_w-m_l)\ddot{x}\,l_{b,r}\cos\theta_{l,r}+m_l l_{w,r}l_{b,r}\cos^2\theta_{l,r}\,\ddot{\theta}_{l,r}-m_l l_{w,r}l_{b,r}\sin\theta_{l,r}\cos\theta_{l,r}\,\dot{\theta}_{l,r}^{2}

\end{aligned}

$$

机体转动：

$$

\begin{aligned}

I_{b}\ddot{\theta}_{b}=&\,-\tau_{l,l}-\tau_{l,r}-m_{b}gd_{b}\cos(\theta_{b}+\theta_{b}^{0})+m_{b}g\sin\theta_{b} \\

&+m_{b}\Big[\tfrac{l_{l}}{2}\sin\theta_{l,l}\ddot{\theta}_{l,l}+\tfrac{l_{l}}{2}\cos\theta_{l,l}\dot{\theta}_{l,l}^{2}+\tfrac{l_{r}}{2}\sin\theta_{l,r}\ddot{\theta}_{l,r}+\tfrac{l_{r}}{2}\cos\theta_{l,r}\dot{\theta}_{l,r}^{2}\Big]\sin\theta_{b} \\

&-m_{b}\Big[\ddot{x}+\tfrac{l_{l}}{2}\cos\theta_{l,l}\ddot{\theta}_{l,l}-\tfrac{l_{l}}{2}\sin\theta_{l,l}\dot{\theta}_{l,l}^{2}+\tfrac{l_{r}}{2}\cos\theta_{l,r}\ddot{\theta}_{l,r}-\tfrac{l_{r}}{2}\sin\theta_{l,r}\dot{\theta}_{l,r}^{2}\Big]d_{b}\cos\theta_{b}

\end{aligned}

$$

  

## 双腿质心偏移建模

  

### 轮转轴

#### 左轮

$$

-F_{w,l}^{h}+f_{l}=m_{w}a_{w,l}^{h},\quad -F_{w,l}^{v}+F_{N,l}-m_{w}g=m_{w}a_{w,l}^{v}

$$

$a_{w,l}^h=\ddot{x},\,a_{w,l}^{v}=0$，$I \ddot{\theta}_{w,l}=\tau_{w,l}-f_{l}R_{w}$

#### 右轮

$$

-F_{w,r}^{h}+f_{r}=m_{w}a_{w,r}^{h},\quad -F_{w,r}^{v}+F_{N,r}-m_{w}g=m_{w}a_{w,r}^{v}

$$

$a_{w,r}^h=\ddot{x},\,a_{w,r}^{v}=0$，$I \ddot{\theta}_{w,r}=\tau_{w,r}-f_{r}R_{w}$

### 腿转轴

#### 左腿

$$

-F_{l,l}^{h}+F_{w,l}^{h}=m_{l}a_{l,l}^{h},\quad -F_{l,l}^{v}+F_{w,l}^{v}-m_{l}g=m_{l}a_{l,l}^{v}

$$

$$

a_{l,l}^{h}=a_{w,l}^{h}+\frac{ \partial ^{2} }{ \partial t } (l_{l}\sin \theta_{l,l})=\ddot{x}+l_{l}\cos \theta_{l,l}\ddot{\theta}_{l,l}-l_{l}\sin \theta_{l,l}\dot{\theta}_{l,l}^{2}

$$

$$

a_{l,l}^{v}=a_{w,l}^{v}+\frac{ \partial ^{2} }{ \partial t } (l_{l}\cos \theta_{l,l})=-l_{l}\sin \theta_{l,l}\ddot{\theta_{l,l}}-l_{l}\cos \theta_{l,l}\dot{\theta}_{l,l}^{2}

$$

$$

I_{l,l}\ddot{\theta}_{l,l}=\tau_{l,l}-\tau_{w,l}-m_{l}gd_{l}\sin(\theta_{l,l}+\theta_{l,l}^{0})-F_{w,l}^{h}l_{l}\cos \theta_{l,l}+F_{w,l}^{v}l_{l}\sin \theta_{l,l}

$$

#### 右腿

$$

-F_{l,r}^{h}+F_{w,r}^{h}=m_{l}a_{l,r}^{h},\quad -F_{l,r}^{v}+F_{w,r}^{v}-m_{l}g=m_{l}a_{l,r}^{v}

$$

$$

a_{l,r}^{h}=a_{w,r}^{h}+\frac{ \partial ^{2} }{ \partial t } (l_{r}\sin \theta_{l,r})=\ddot{x}+l_{r}\cos \theta_{l,r}\ddot{\theta}_{l,r}-l_{r}\sin \theta_{l,r}\dot{\theta}_{l,r}^{2}

$$

$$

a_{l,r}^{v}=a_{w,r}^{v}+\frac{ \partial ^{2} }{ \partial t } (l_{r}\cos \theta_{l,r})=-l_{r}\sin \theta_{l,r}\ddot{\theta_{l,r}}-l_{r}\cos \theta_{l,r}\dot{\theta}_{l,r}^{2}

$$

$$

I_{l,r}\ddot{\theta}_{l,r}=\tau_{l,r}-\tau_{w,r}-m_{l}gd_{l}\sin(\theta_{l,r}+\theta_{l,r}^{0})-F_{w,r}^{h}l_{r}\cos \theta_{l,r}+F_{w,r}^{v}l_{r}\sin \theta_{l,r}

$$

### 机体转轴

$$

F_{l,l}^{h}+F_{l,r}^{h}=m_{b}a_{b}^h,\quad F_{l,l}^{v}+F_{l,r}^{v}-m_{b}g=m_{b}a_{b}^{v}

$$

$$

a_{b}^{h}=\frac{1}{2}(a_{l,l}^{h}+a_{l,r}^{h})=\ddot{x}+\frac{1}{2}l_{l}\cos \theta_{l,l}\ddot{\theta}_{l,l}-\frac{1}{2}l_{l}\sin \theta_{l,l}\dot{\theta}_{l,l}^{2}+\frac{1}{2}l_{r}\cos \theta_{l,r}\ddot{\theta}_{l,r}-\frac{1}{2}l_{r}\sin \theta_{l,r}\dot{\theta}_{l,r}^{2}

$$

$$

a_{b}^{v}=\frac{1}{2}(a_{l,l}^{v}+a_{l,r}^{v})=-\frac{1}{2}l_{l}\sin \theta_{l,l}\ddot{\theta}_{l,l}-\frac{1}{2}l_{l}\cos \theta_{l,l}\dot{\theta}_{l,l}^{2}-\frac{1}{2}l_{r}\sin \theta_{l,r}\ddot{\theta}_{l,r}-\frac{1}{2}l_{r}\cos \theta_{l,r}\dot{\theta}_{l,r}^{2}

$$

$$

I_{b}\ddot{\theta}_{b}=-\tau_{l,l}-\tau_{l,r}-m_{b}gd_{b}\cos(\theta_{b}+\theta_{b}^{0})+(F_{l,l}^{v}+F_{l,r}^{v})\sin \theta_{b}-(F_{l,l}^{h}+F_{l,r}^{h})d_{b}\cos \theta_{b}

$$

$$

I_{\phi}\ddot{\phi}=(f_{r}-f_{l})R_{b}

$$

  

### 消去力

  

由 $F_{w,l}^{v}=F_{w,r}^{v}$、水平/竖直受力平衡及 $f_i=(\tau_{w,i}-I\ddot{\theta}_{w,i})/R_w$ 联立，消去全部 $F,f,a$。下列仅含状态变量及其导数与 $\tau$（腿方程以髋轴 $l_i$ 计）。

  

水平（消 $f$，$l_{w,i}\to l_i$ 于 $m_l$ 水平耦合）：

$$

\big(2m_{w}+2m_{l}+m_{b}+\tfrac{2I}{R_{w}^{2}}\big)\ddot{x}+\frac{m_{b}}{2}\big(l_{l}\cos\theta_{l,l}\ddot{\theta}_{l,l}+l_{r}\cos\theta_{l,r}\ddot{\theta}_{l,r}\big)+m_{l}\big(l_{l}\cos\theta_{l,l}\ddot{\theta}_{l,l}+l_{r}\cos\theta_{l,r}\ddot{\theta}_{l,r}\big)=\frac{\tau_{w,l}+\tau_{w,r}}{R_{w}}+m_{b}\Big[-\tfrac{l_{l}}{2}\sin\theta_{l,l}\dot{\theta}_{l,l}^{2}-\tfrac{l_{r}}{2}\sin\theta_{l,r}\dot{\theta}_{l,r}^{2}\Big]+m_{l}\Big[-l_{l}\sin\theta_{l,l}\dot{\theta}_{l,l}^{2}-l_{r}\sin\theta_{l,r}\dot{\theta}_{l,r}^{2}\Big]

$$

yaw（消 $f$）：

$$

I_{\phi}\ddot{\phi}=\frac{R_{b}}{R_{w}}\Big[(\tau_{w,r}-I\ddot{\theta}_{w,r})-(\tau_{w,l}-I\ddot{\theta}_{w,l})\Big]

$$

左腿转动：

$$

\begin{aligned}

I_{l,l}\ddot{\theta}_{l,l}=&\,\tau_{l,l}-\tau_{w,l}\big(1+\tfrac{l_l}{R_w}\big)+\tfrac{I}{R_w}l_l\ddot{\theta}_{w,l}-m_{l}gd_{l}\sin(\theta_{l,l}+\theta_{l,l}^{0})-m_{w}\ddot{x}\,l_{l}\cos\theta_{l,l} \\

&+\tfrac{m_b+2m_l}{2}gl_{l}\sin\theta_{l,l}-\tfrac{m_b}{4}l_{l}\sin\theta_{l,l}\big[l_l\sin\theta_{l,l}\ddot{\theta}_{l,l}+l_l\cos\theta_{l,l}\dot{\theta}_{l,l}^{2}+l_r\sin\theta_{l,r}\ddot{\theta}_{l,r}+l_r\cos\theta_{l,r}\dot{\theta}_{l,r}^{2}\big] \\

&-\tfrac{m_l}{2}l_{l}\sin\theta_{l,l}\big[l_l\sin\theta_{l,l}\ddot{\theta}_{l,l}+l_l\cos\theta_{l,l}\dot{\theta}_{l,l}^{2}+l_r\sin\theta_{l,r}\ddot{\theta}_{l,r}+l_r\cos\theta_{l,r}\dot{\theta}_{l,r}^{2}\big]

\end{aligned}

$$

右腿转动（下标 $l\to r$）：

$$

\begin{aligned}

I_{l,r}\ddot{\theta}_{l,r}=&\,\tau_{l,r}-\tau_{w,r}\big(1+\tfrac{l_r}{R_w}\big)+\tfrac{I}{R_w}l_r\ddot{\theta}_{w,r}-m_{l}gd_{l}\sin(\theta_{l,r}+\theta_{l,r}^{0})-m_{w}\ddot{x}\,l_{r}\cos\theta_{l,r} \\

&+\tfrac{m_b+2m_l}{2}gl_{r}\sin\theta_{l,r}-\tfrac{m_b}{4}l_{r}\sin\theta_{l,r}\big[l_l\sin\theta_{l,l}\ddot{\theta}_{l,l}+l_l\cos\theta_{l,l}\dot{\theta}_{l,l}^{2}+l_r\sin\theta_{l,r}\ddot{\theta}_{l,r}+l_r\cos\theta_{l,r}\dot{\theta}_{l,r}^{2}\big] \\

&-\tfrac{m_l}{2}l_{r}\sin\theta_{l,r}\big[l_l\sin\theta_{l,l}\ddot{\theta}_{l,l}+l_l\cos\theta_{l,l}\dot{\theta}_{l,l}^{2}+l_r\sin\theta_{l,r}\ddot{\theta}_{l,r}+l_r\cos\theta_{l,r}\dot{\theta}_{l,r}^{2}\big]

\end{aligned}

$$

机体转动（同双腿）：

$$

\begin{aligned}

I_{b}\ddot{\theta}_{b}=&\,-\tau_{l,l}-\tau_{l,r}-m_{b}gd_{b}\cos(\theta_{b}+\theta_{b}^{0})+m_{b}g\sin\theta_{b} \\

&+m_{b}\Big[\tfrac{l_{l}}{2}\sin\theta_{l,l}\ddot{\theta}_{l,l}+\tfrac{l_{l}}{2}\cos\theta_{l,l}\dot{\theta}_{l,l}^{2}+\tfrac{l_{r}}{2}\sin\theta_{l,r}\ddot{\theta}_{l,r}+\tfrac{l_{r}}{2}\cos\theta_{l,r}\dot{\theta}_{l,r}^{2}\Big]\sin\theta_{b} \\

&-m_{b}\Big[\ddot{x}+\tfrac{l_{l}}{2}\cos\theta_{l,l}\ddot{\theta}_{l,l}-\tfrac{l_{l}}{2}\sin\theta_{l,l}\dot{\theta}_{l,l}^{2}+\tfrac{l_{r}}{2}\cos\theta_{l,r}\ddot{\theta}_{l,r}-\tfrac{l_{r}}{2}\sin\theta_{l,r}\dot{\theta}_{l,r}^{2}\Big]d_{b}\cos\theta_{b}

\end{aligned}

$$

  

## 动力学矩阵概要

  

消 $f$ 与 $F_{w,l}^{v}=F_{w,r}^{v}$ 后标准形 $M(q)\ddot{q}+C(q,\dot{q})\dot{q}+G(q)=B(q)\,\tau$；$C$ 汇集 $\dot{\theta}^{2}$ 离心项，此处不展开 $M,B$ 元素，只给维数与 $G$。

  

### 单腿建模

  

$q=[x,\theta_{l},\theta_{b}]^{T}$，$\tau=[\tau_{w},\tau_{l}]^{T}$，$\ddot{\theta}_{w}=\ddot{x}/R_w$。**$M$**：$3\times 3$；**$B$**：$3\times 2$（$\tau_w$ 经 $1/R_w$ 进水平行，$\tau_l$ 进腿/机体行）。**$G(q)$**：

$$

G=\begin{bmatrix}

0 \\

\big[(m_{l}+m_{b})gl_{w}+m_{b}gl_{b}\big]\sin\theta_{l} \\

m_{b}gd_{b}\cos(\theta_{b}+\theta_{b}^{0})-m_{b}g\sin\theta_{b}

\end{bmatrix}

$$

  

### 双腿建模

  

$q=[\theta_{w,l},\theta_{w,r},\theta_{l,l},\theta_{l,r},\theta_{b}]^{T}$，$\tau=[\tau_{w,l},\tau_{w,r},\tau_{l,l},\tau_{l,r}]^{T}$。**$M$**：$5\times 5$；**$B$**：$5\times 4$（五行：水平、yaw、左/右腿、机体；$\tau_{w,i}$ 系数含 $1/R_w$、$R_b/R_w$、$-(1+l_i/R_w)$、$-d_b/R_w$）。**$G(q)$**：

$$

G=\begin{bmatrix}

0 \\

0 \\

\Big[\big(m_{l}+\tfrac{m_{b}}{2}\big)gl_{w,l}+\tfrac{m_{b}}{2}gl_{b,l}\Big]\sin\theta_{l,l} \\

\Big[\big(m_{l}+\tfrac{m_{b}}{2}\big)gl_{w,r}+\tfrac{m_{b}}{2}gl_{b,r}\Big]\sin\theta_{l,r} \\

m_{b}gd_{b}\cos(\theta_{b}+\theta_{b}^{0})-m_{b}g\sin\theta_{b}

\end{bmatrix}

$$

小角度：$\sin\theta\approx\theta$，$G$ 第三、四、五行分别退化为 $K_{l,l}\theta_{l,l}$、$K_{l,r}\theta_{l,r}$、$m_b g d_b\,\theta_b$。

  

### 双腿质心偏移建模

  

$q,\tau$ 维数同双腿（$M$ 为 $5\times 5$，$B$ 为 $5\times 4$）。**$G(q)$** 在双腿基础上，腿行用 $l_i$ 代 $l_{w,i},l_{b,i}$，并各多 $m_l g d_l\sin(\theta_{l,i}+\theta_{l,i}^{0})$：

$$

G_{3}=\big(m_{l}+\tfrac{m_{b}}{2}\big)gl_{l}\sin\theta_{l,l}+m_{l}gd_{l}\sin(\theta_{l,l}+\theta_{l,l}^{0}),\quad

G_{4}=\big(m_{l}+\tfrac{m_{b}}{2}\big)gl_{r}\sin\theta_{l,r}+m_{l}gd_{l}\sin(\theta_{l,r}+\theta_{l,r}^{0})

$$

  

### 与一阶状态的关系

  

在平衡点线性化并令 $\dot{q}\approx 0$ 得 $\dot{x}=Ax+Bu$；$G(q_0)$ 由平衡腿力矩抵消，LQR 作偏置处理。

  

## 小角度线性化

  

平衡点 $\theta_{l},\theta_{l,l},\theta_{l,r},\theta_{b},\phi\approx 0$，$\theta_{l}^{0},\theta_{b}^{0},\theta_{l,l}^{0},\theta_{l,r}^{0}\approx 0$；$\sin\theta\approx\theta$，$\cos\theta\approx 1$，$\sin(\theta+\theta^{0})\approx\theta+\theta^{0}$，$\cos(\theta+\theta^{0})\approx 1$；忽略 $\dot{\theta}^{2}$ 及 $\theta^{2},\theta\dot{\theta}$ 等二阶小量。对「消去力」各式逐项线性化，顺序与结构相同。

  

### 单腿建模

  

水平：

$$

\big(m_{w}+m_{l}+m_{b}+\tfrac{I}{R_{w}^{2}}\big)\ddot{x}+(m_{l}l_{w}+m_{b}l)\ddot{\theta}_{l}=\frac{\tau_{w}}{R_{w}}

$$

腿部转动：

$$

\big[I+m_{l}l_{w}^{2}+m_{b}l(l_{w}+l_{b})\big]\ddot{\theta}_{l}+(m_{l}l_{w}+m_{b}l)\ddot{x}=\tau_{l}-\tau_{w}+\big[(m_{l}+m_{b})gl_{w}+m_{b}gl_{b}\big]\theta_{l}

$$

机体转动：

$$

I\ddot{\theta}_{b}+m_{b}ld_{b}\ddot{\theta}_{l}=-\tau_{l}-m_{b}gd_{b}+m_{b}g\theta_{b}-m_{b}d_{b}\ddot{x}

$$

  

### 双腿建模

  

水平：

$$

\big(2m_{w}+2m_{l}+m_{b}+\tfrac{2I}{R_{w}^{2}}\big)\ddot{x}+\frac{m_{b}}{2}\big(l_{l}\ddot{\theta}_{l,l}+l_{r}\ddot{\theta}_{l,r}\big)+m_{l}\big(l_{w,l}\ddot{\theta}_{l,l}+l_{w,r}\ddot{\theta}_{l,r}\big)=\frac{\tau_{w,l}+\tau_{w,r}}{R_{w}}

$$

yaw：

$$

I_{\phi}\ddot{\phi}=\frac{R_{b}}{R_{w}}\Big[(\tau_{w,r}-I\ddot{\theta}_{w,r})-(\tau_{w,l}-I\ddot{\theta}_{w,l})\Big]

$$

左腿转动（$l_l=l_{w,l}+l_{b,l}$）：

$$

\big[I_{l,l}+m_{l}l_{w,l}l_{b,l}\big]\ddot{\theta}_{l,l}=\tau_{l,l}-\tau_{w,l}\big(1+\tfrac{l_l}{R_w}\big)+\tfrac{I}{R_w}l_l\ddot{\theta}_{w,l}-(m_w l_l-m_l l_{b,l})\ddot{x}+\Big[\big(m_{l}+\tfrac{m_{b}}{2}\big)gl_{w,l}+\tfrac{m_{b}}{2}gl_{b,l}\Big]\theta_{l,l}

$$

右腿转动（$l_r=l_{w,r}+l_{b,r}$，下标 $l\to r$）：

$$

\big[I_{l,r}+m_{l}l_{w,r}l_{b,r}\big]\ddot{\theta}_{l,r}=\tau_{l,r}-\tau_{w,r}\big(1+\tfrac{l_r}{R_w}\big)+\tfrac{I}{R_w}l_r\ddot{\theta}_{w,r}-(m_w l_r-m_l l_{b,r})\ddot{x}+\Big[\big(m_{l}+\tfrac{m_{b}}{2}\big)gl_{w,r}+\tfrac{m_{b}}{2}gl_{b,r}\Big]\theta_{l,r}

$$

机体转动：

$$

I_{b}\ddot{\theta}_{b}=-\tau_{l,l}-\tau_{l,r}-m_{b}gd_{b}+m_{b}g\theta_{b}-m_{b}d_{b}\ddot{x}-\frac{m_{b}d_{b}}{2}\big(l_{l}\ddot{\theta}_{l,l}+l_{r}\ddot{\theta}_{l,r}\big)

$$

  

### 双腿质心偏移建模

  

水平（$m_l$ 水平耦合中 $l_{w,i}\to l_i$）：

$$

\big(2m_{w}+2m_{l}+m_{b}+\tfrac{2I}{R_{w}^{2}}\big)\ddot{x}+\frac{m_{b}}{2}\big(l_{l}\ddot{\theta}_{l,l}+l_{r}\ddot{\theta}_{l,r}\big)+m_{l}\big(l_{l}\ddot{\theta}_{l,l}+l_{r}\ddot{\theta}_{l,r}\big)=\frac{\tau_{w,l}+\tau_{w,r}}{R_{w}}

$$

yaw：

$$

I_{\phi}\ddot{\phi}=\frac{R_{b}}{R_{w}}\Big[(\tau_{w,r}-I\ddot{\theta}_{w,r})-(\tau_{w,l}-I\ddot{\theta}_{w,l})\Big]

$$

左腿转动（髋轴）：

$$

I_{l,l}\ddot{\theta}_{l,l}=\tau_{l,l}-\tau_{w,l}\big(1+\tfrac{l_l}{R_w}\big)+\tfrac{I}{R_w}l_l\ddot{\theta}_{w,l}-m_{w}l_{l}\ddot{x}-m_{l}gd_{l}\,(\theta_{l,l}+\theta_{l,l}^{0})+\big(m_{l}+\tfrac{m_{b}}{2}\big)gl_{l}\,\theta_{l,l}

$$

右腿转动（下标 $l\to r$）：

$$

I_{l,r}\ddot{\theta}_{l,r}=\tau_{l,r}-\tau_{w,r}\big(1+\tfrac{l_r}{R_w}\big)+\tfrac{I}{R_w}l_r\ddot{\theta}_{w,r}-m_{w}l_{r}\ddot{x}-m_{l}gd_{l}\,(\theta_{l,r}+\theta_{l,r}^{0})+\big(m_{l}+\tfrac{m_{b}}{2}\big)gl_{r}\,\theta_{l,r}

$$

机体转动：

$$

I_{b}\ddot{\theta}_{b}=-\tau_{l,l}-\tau_{l,r}-m_{b}gd_{b}+m_{b}g\theta_{b}-m_{b}d_{b}\ddot{x}-\frac{m_{b}d_{b}}{2}\big(l_{l}\ddot{\theta}_{l,l}+l_{r}\ddot{\theta}_{l,r}\big)

$$

  

### 对称退化

$l_{w,l}=l_{w,r}=l_{w}$，$l_{b,l}=l_{b,r}=l_{b}$，$l_{l}=l_{r}=l$，$\theta_{l,l}=\theta_{l,r}=\theta_{l}$ 时，双腿水平方程退化为

$$

\big(2m_{w}+2m_{l}+m_{b}+\tfrac{2I}{R_{w}^{2}}\big)\ddot{x}+2(m_{l}l_{w}+m_{b}l)\ddot{\theta}_{l}=\frac{\tau_{w,l}+\tau_{w,r}}{R_{w}}

$$

与单腿水平式在 $\tau_{w,l}+\tau_{w,r}\to\tau_w$、$2m_w\to m_w$ 时一致；偏移模型 $d_{l}=0$、$l_{w}=l-l_{b}$ 时腿方程与双腿质心模型一阶对齐。

  

## 三种建模一致性审查

  

| 项目 | 单腿建模 | 双腿建模 | 双腿质心偏移建模 |

| --- | --- | --- | --- |

| 自由度 | $x,\theta_l,\theta_b$（无 yaw） | 左右 $\theta_{l,i}$ + $\phi,\theta_b$ | 同双腿 |

| 腿转动参考点 | 腿质心（力臂 $l_w,l_b$） | 腿质心（$l_{w,i},l_{b,i}$） | **髋转轴**（$l_{l},l_{r}$，含 $d_l,\theta_l^0$） |

| 机体 pitch | 推导用 $F_{l,i}^{h,v}$；消力后仅含状态变量 | 同单腿 | 同双腿 |

| 标准形 | $M$ $3\times 3$，$B$ $3\times 2$（已消 $F,f,a$） | $M$ $5\times 5$，$B$ $5\times 4$ | 同双腿 |

| 对称极限 | — | $\theta_{l,l}=\theta_{l,r}$ 时退化为 2×单腿 | $l_{w,i}=l_i-l_{b,i},\,d_l=0$ 时趋近质心模型 |

  

**几何关系**：一般 $l=l_{w}+l_{b}$（质心在轮轴与髋轴连线上）；双腿分别记为 $l_{l}=l_{w,l}+l_{b,l}$，$l_{r}=l_{w,r}+l_{b,r}$。

  

**仍须注意**：

1. 竖直消力设 $F_{w,l}^{v}=F_{w,r}^{v}$，在「消去力」中已完全代入，不保留 $F_{w}^{v},F_{l,i}^{v}$。

2. 推导段仍用 $F,f,a$ 作中间量；最终动力学方程仅含 $x,\phi,\theta,\tau$ 及导数。

3. $B$ 只映射 $\tau_{w,i},\tau_{l,i}$。

**层次关系**：受力分析与轮转动方程三者可复用；加速度分析在模型 2/3 差 $l_{w,i}$ vs $l_{l},l_{r}$；模型 3 腿转动绕髋轴并列重力偏移项。