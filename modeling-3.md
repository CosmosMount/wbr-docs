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
消去力

#### 水平运动方程

$$

f=(m_{w}+m_{l}+m_{b})\ddot{x}+(m_{l}l_{w}+m_{b}l)\cos \theta_{l}\ddot{\theta}_{l}-(m_{l}l_{w}+m_{b}l)\sin \theta_{l}\dot{\theta}_{l}^{2}

$$
代入 $f$ 
$$

\big(m_{w}+m_{l}+m_{b}+\tfrac{I}{R_{w}^{2}}\big)\ddot{x}+(m_{l}l_{w}+m_{b}l)\cos \theta_{l}\ddot{\theta}_{l}=\frac{\tau_{w}}{R_{w}}+(m_{l}l_{w}+m_{b}l)\sin \theta_{l}\dot{\theta}_{l}^{2}

$$
#### 腿部转动方程

$$

\begin{aligned}

I_{l}\ddot{\theta}_{l}=&\,\tau_{l}-\tau_{w} \\

&+\Big[(m_{l}+m_{b})g+m_{l}\big(-l_{w}\sin \theta_{l}\ddot{\theta}_{l}-l_{w}\cos \theta_{l}\dot{\theta}_{l}^{2}\big)+m_{b}\big(-l\sin \theta_{l}\ddot{\theta}_{l}-l\cos \theta_{l}\dot{\theta}_{l}^{2}\big)\Big]l_{w}\sin \theta_{l} \\

&+m_{b}\Big(g-l\sin \theta_{l}\ddot{\theta}_{l}-l\cos \theta_{l}\dot{\theta}_{l}^{2}\Big)l_{b}\sin \theta_{l} \\

&-\Big[m_{l}l_{w}\big(\ddot{x}+l_{w}\cos \theta_{l}\ddot{\theta}_{l}-l_{w}\sin \theta_{l}\dot{\theta}_{l}^{2}\big)+m_{b}(l_{w}+l_{b})\big(\ddot{x}+l\cos \theta_{l}\ddot{\theta}_{l}-l\sin \theta_{l}\dot{\theta}_{l}^{2}\big)\Big]\cos \theta_{l}

\end{aligned}

$$
#### 机体转动方程

$$

\begin{aligned}

I_{b}\ddot{\theta}_{b}=&\,-\tau_{l}+m_{b}gd_{b}\sin\theta_{b} \\

&+m_{b}\big(l\sin\theta_{l}\ddot{\theta}_{l}+l\cos\theta_{l}\dot{\theta}_{l}^{2}\big)d_{b}\sin\theta_{b} \\

&-m_{b}\big(\ddot{x}+l\cos\theta_{l}\ddot{\theta}_{l}-l\sin\theta_{l}\dot{\theta}_{l}^{2}\big)d_{b}\cos\theta_{b}

\end{aligned}

$$







### 控制律

我们需要
$$
x=\left[ \begin{matrix}
x \\
\dot{x} \\
\theta_{l,l} \\
\dot{\theta}_{l,l} \\
\theta_{l,r} \\
\dot{\theta}_{l,r} \\
\theta_{b} \\
\dot{\theta}_{b}
\end{matrix}
 \right]
 ,
 \dot{x}=\left[ \begin{matrix}
\dot{x} \\
\ddot{x} \\
\dot{\theta}_{l,l} \\
\ddot{\theta}_{l,l} \\
\dot{\theta}_{l,r} \\
\ddot{\theta}_{l,r} \\
\dot{\theta}_{b} \\
\ddot{\theta}_{b}
\end{matrix}
 \right]
 ,
  u=\left[ \begin{matrix}
\tau_{w,l}  \\
\tau_{w,r} \\
\tau_{l,l} \\
\tau_{l,r}
\end{matrix}
 \right]
$$
最终期望整理出
$$
\dot{x}=Ax+Bu
$$
消去力

#### 水平运动方程

$$
\begin{align}

\big(2m_{w}+2m_{l}+m_{b}+\tfrac{2I}{R_{w}^{2}}\big)\ddot{x}+\frac{m_{b}}{2}\big(l_{l}\cos\theta_{l,l}\ddot{\theta}_{l,l}+l_{r}\cos\theta_{l,r}\ddot{\theta}_{l,r}\big)+m_{l}\big(l_{w,l}\cos\theta_{l,l}\ddot{\theta}_{l,l}+l_{w,r}\cos\theta_{l,r}\ddot{\theta}_{l,r}\big) \\
=\frac{\tau_{w,l}+\tau_{w,r}}{R_{w}}+m_{b}\Big[-\tfrac{l_{l}}{2}\sin\theta_{l,l}\dot{\theta}_{l,l}^{2}-\tfrac{l_{r}}{2}\sin\theta_{l,r}\dot{\theta}_{l,r}^{2}\Big]+m_{l}\Big[-l_{w,l}\sin\theta_{l,l}\dot{\theta}_{l,l}^{2}-l_{w,r}\sin\theta_{l,r}\dot{\theta}_{l,r}^{2}\Big]

\end{align}
$$
#### yaw转动方程

$$

I_{\phi}\ddot{\phi}=\frac{R_{b}}{R_{w}}\Big[(\tau_{w,r}-I\ddot{\theta}_{w,r})-(\tau_{w,l}-I\ddot{\theta}_{w,l})\Big]

$$
无需进一步线性化

#### 腿部转动方程

左腿

$$

\begin{aligned}

I_{l,l}\ddot{\theta}_{l,l}=&\,\tau_{l,l}-\tau_{w,l}\big(1+\tfrac{l_l}{R_w}\big)+\tfrac{I}{R_w}l_l\ddot{\theta}_{w,l}+\tfrac{m_b+2m_l}{2}gl_l\sin\theta_{l,l}-m_l l_{b,l}g\sin\theta_{l,l} \\

&+m_l l_{b,l}l_{w,l}\sin^2\theta_{l,l}\,\ddot{\theta}_{l,l}+m_l l_{b,l}l_{w,l}\sin\theta_{l,l}\cos\theta_{l,l}\,\dot{\theta}_{l,l}^{2} \\

&-\tfrac{m_b}{4}l_l\sin\theta_{l,l}\big[l_l\sin\theta_{l,l}\ddot{\theta}_{l,l}+l_l\cos\theta_{l,l}\dot{\theta}_{l,l}^{2}+l_r\sin\theta_{l,r}\ddot{\theta}_{l,r}+l_r\cos\theta_{l,r}\dot{\theta}_{l,r}^{2}\big] \\

&-\tfrac{m_l}{2}l_l\sin\theta_{l,l}\big[l_{w,l}\sin\theta_{l,l}\ddot{\theta}_{l,l}+l_{w,l}\cos\theta_{l,l}\dot{\theta}_{l,l}^{2}+l_{w,r}\sin\theta_{l,r}\ddot{\theta}_{l,r}+l_{w,r}\cos\theta_{l,r}\dot{\theta}_{l,r}^{2}\big] \\

&-m_w\ddot{x}\,l_{w,l}\cos\theta_{l,l}-(m_w-m_l)\ddot{x}\,l_{b,l}\cos\theta_{l,l}+m_l l_{w,l}l_{b,l}\cos^2\theta_{l,l}\,\ddot{\theta}_{l,l}-m_l l_{w,l}l_{b,l}\sin\theta_{l,l}\cos\theta_{l,l}\,\dot{\theta}_{l,l}^{2}

\end{aligned}

$$

右腿

$$

\begin{aligned}

I_{l,r}\ddot{\theta}_{l,r}=&\,\tau_{l,r}-\tau_{w,r}\big(1+\tfrac{l_r}{R_w}\big)+\tfrac{I}{R_w}l_r\ddot{\theta}_{w,r}+\tfrac{m_b+2m_l}{2}gl_r\sin\theta_{l,r}-m_l l_{b,r}g\sin\theta_{l,r} \\

&+m_l l_{b,r}l_{w,r}\sin^2\theta_{l,r}\,\ddot{\theta}_{l,r}+m_l l_{b,r}l_{w,r}\sin\theta_{l,r}\cos\theta_{l,r}\,\dot{\theta}_{l,r}^{2} \\

&-\tfrac{m_b}{4}l_r\sin\theta_{l,r}\big[l_l\sin\theta_{l,l}\ddot{\theta}_{l,l}+l_l\cos\theta_{l,l}\dot{\theta}_{l,l}^{2}+l_r\sin\theta_{l,r}\ddot{\theta}_{l,r}+l_r\cos\theta_{l,r}\dot{\theta}_{l,r}^{2}\big] \\

&-\tfrac{m_l}{2}l_r\sin\theta_{l,r}\big[l_{w,l}\sin\theta_{l,l}\ddot{\theta}_{l,l}+l_{w,l}\cos\theta_{l,l}\dot{\theta}_{l,l}^{2}+l_{w,r}\sin\theta_{l,r}\ddot{\theta}_{l,r}+l_{w,r}\cos\theta_{l,r}\dot{\theta}_{l,r}^{2}\big] \\

&-m_w\ddot{x}\,l_{w,r}\cos\theta_{l,r}-(m_w-m_l)\ddot{x}\,l_{b,r}\cos\theta_{l,r}+m_l l_{w,r}l_{b,r}\cos^2\theta_{l,r}\,\ddot{\theta}_{l,r}-m_l l_{w,r}l_{b,r}\sin\theta_{l,r}\cos\theta_{l,r}\,\dot{\theta}_{l,r}^{2}

\end{aligned}

$$

#### 机体转动方程

$$

\begin{aligned}

I_{b}\ddot{\theta}_{b}=&\,-\tau_{l,l}-\tau_{l,r}-m_{b}gd_{b}\cos(\theta_{b}+\theta_{b}^{0})+m_{b}gd_{b}\sin\theta_{b} \\

&+m_{b}\Big[\tfrac{l_{l}}{2}\sin\theta_{l,l}\ddot{\theta}_{l,l}+\tfrac{l_{l}}{2}\cos\theta_{l,l}\dot{\theta}_{l,l}^{2}+\tfrac{l_{r}}{2}\sin\theta_{l,r}\ddot{\theta}_{l,r}+\tfrac{l_{r}}{2}\cos\theta_{l,r}\dot{\theta}_{l,r}^{2}\Big]\sin\theta_{b} \\

&-m_{b}\Big[\ddot{x}+\tfrac{l_{l}}{2}\cos\theta_{l,l}\ddot{\theta}_{l,l}-\tfrac{l_{l}}{2}\sin\theta_{l,l}\dot{\theta}_{l,l}^{2}+\tfrac{l_{r}}{2}\cos\theta_{l,r}\ddot{\theta}_{l,r}-\tfrac{l_{r}}{2}\sin\theta_{l,r}\dot{\theta}_{l,r}^{2}\Big]d_{b}\cos\theta_{b}

\end{aligned}

$$





### 控制律

我们需要
$$
x=\left[ \begin{matrix}
x \\
\dot{x} \\
\theta_{l,l} \\
\dot{\theta}_{l,l} \\
\theta_{l,r} \\
\dot{\theta}_{l,r} \\
\theta_{b} \\
\dot{\theta}_{b}
\end{matrix}
 \right]
 ,
 \dot{x}=\left[ \begin{matrix}
\dot{x} \\
\ddot{x} \\
\dot{\theta}_{l,l} \\
\ddot{\theta}_{l,l} \\
\dot{\theta}_{l,r} \\
\ddot{\theta}_{l,r} \\
\dot{\theta}_{b} \\
\ddot{\theta}_{b}
\end{matrix}
 \right]
 ,
  u=\left[ \begin{matrix}
\tau_{w,l}  \\
\tau_{w,r} \\
\tau_{l,l} \\
\tau_{l,r}
\end{matrix}
 \right]
$$
最终期望整理出
$$
\dot{x}=Ax+Bu
$$
消去力

#### 水平运动方程

$$
\begin{align}

\big(2m_{w}+2m_{l}+m_{b}+\tfrac{2I}{R_{w}^{2}}\big)\ddot{x}+\frac{m_{b}}{2}\big(l_{l}\cos\theta_{l,l}\ddot{\theta}_{l,l}+l_{r}\cos\theta_{l,r}\ddot{\theta}_{l,r}\big)+m_{l}\big(l_{l}\cos\theta_{l,l}\ddot{\theta}_{l,l}+l_{r}\cos\theta_{l,r}\ddot{\theta}_{l,r}\big) \\
=\frac{\tau_{w,l}+\tau_{w,r}}{R_{w}}+m_{b}\Big[-\tfrac{l_{l}}{2}\sin\theta_{l,l}\dot{\theta}_{l,l}^{2}-\tfrac{l_{r}}{2}\sin\theta_{l,r}\dot{\theta}_{l,r}^{2}\Big]+m_{l}\Big[-l_{l}\sin\theta_{l,l}\dot{\theta}_{l,l}^{2}-l_{r}\sin\theta_{l,r}\dot{\theta}_{l,r}^{2}\Big]

\end{align}
$$
小角度线性化
$$

\big(2m_{w}+2m_{l}+m_{b}+\tfrac{2I}{R_{w}^{2}}\big)\ddot{x}+\frac{m_{b}}{2}\big(l_{l}\ddot{\theta}_{l,l}+l_{r}\ddot{\theta}_{l,r}\big)+m_{l}\big(l_{l}\ddot{\theta}_{l,l}+l_{r}\ddot{\theta}_{l,r}\big)=\frac{\tau_{w,l}+\tau_{w,r}}{R_{w}}

$$

#### yaw转动方程

$$

I_{\phi}\ddot{\phi}=\frac{R_{b}}{R_{w}}\Big[(\tau_{w,r}-I\ddot{\theta}_{w,r})-(\tau_{w,l}-I\ddot{\theta}_{w,l})\Big]

$$

#### 腿部转动方程

左腿

$$

\begin{aligned}

I_{l,l}\ddot{\theta}_{l,l}=&\,\tau_{l,l}-\tau_{w,l}\big(1+\tfrac{l_l}{R_w}\big)+\tfrac{I}{R_w}l_l\ddot{\theta}_{w,l}-m_{l}gd_{l}\sin(\theta_{l,l}+\theta_{l,l}^{0})-m_{w}\ddot{x}\,l_{l}\cos\theta_{l,l} \\

&+\tfrac{m_b+2m_l}{2}gl_{l}\sin\theta_{l,l}-\tfrac{m_b}{4}l_{l}\sin\theta_{l,l}\big[l_l\sin\theta_{l,l}\ddot{\theta}_{l,l}+l_l\cos\theta_{l,l}\dot{\theta}_{l,l}^{2}+l_r\sin\theta_{l,r}\ddot{\theta}_{l,r}+l_r\cos\theta_{l,r}\dot{\theta}_{l,r}^{2}\big] \\

&-\tfrac{m_l}{2}l_{l}\sin\theta_{l,l}\big[l_l\sin\theta_{l,l}\ddot{\theta}_{l,l}+l_l\cos\theta_{l,l}\dot{\theta}_{l,l}^{2}+l_r\sin\theta_{l,r}\ddot{\theta}_{l,r}+l_r\cos\theta_{l,r}\dot{\theta}_{l,r}^{2}\big]

\end{aligned}

$$
小角度线性化
$$

I_{l,l}\ddot{\theta}_{l,l}=\tau_{l,l}-\tau_{w,l}\big(1+\tfrac{l_l}{R_w}\big)+\tfrac{I}{R_w}l_l\ddot{\theta}_{w,l}-m_{w}l_{l}\ddot{x}-m_{l}gd_{l}\,(\theta_{l,l}+\theta_{l,l}^{0})+\big(m_{l}+\tfrac{m_{b}}{2}\big)gl_{l}\,\theta_{l,l}

$$

右腿

$$

\begin{aligned}

I_{l,r}\ddot{\theta}_{l,r}=&\,\tau_{l,r}-\tau_{w,r}\big(1+\tfrac{l_r}{R_w}\big)+\tfrac{I}{R_w}l_r\ddot{\theta}_{w,r}-m_{l}gd_{l}\sin(\theta_{l,r}+\theta_{l,r}^{0})-m_{w}\ddot{x}\,l_{r}\cos\theta_{l,r} \\

&+\tfrac{m_b+2m_l}{2}gl_{r}\sin\theta_{l,r}-\tfrac{m_b}{4}l_{r}\sin\theta_{l,r}\big[l_l\sin\theta_{l,l}\ddot{\theta}_{l,l}+l_l\cos\theta_{l,l}\dot{\theta}_{l,l}^{2}+l_r\sin\theta_{l,r}\ddot{\theta}_{l,r}+l_r\cos\theta_{l,r}\dot{\theta}_{l,r}^{2}\big] \\

&-\tfrac{m_l}{2}l_{r}\sin\theta_{l,r}\big[l_l\sin\theta_{l,l}\ddot{\theta}_{l,l}+l_l\cos\theta_{l,l}\dot{\theta}_{l,l}^{2}+l_r\sin\theta_{l,r}\ddot{\theta}_{l,r}+l_r\cos\theta_{l,r}\dot{\theta}_{l,r}^{2}\big]

\end{aligned}

$$
小角度线性化
$$

I_{l,r}\ddot{\theta}_{l,r}=\tau_{l,r}-\tau_{w,r}\big(1+\tfrac{l_r}{R_w}\big)+\tfrac{I}{R_w}l_r\ddot{\theta}_{w,r}-m_{w}l_{r}\ddot{x}-m_{l}gd_{l}\,(\theta_{l,r}+\theta_{l,r}^{0})+\big(m_{l}+\tfrac{m_{b}}{2}\big)gl_{r}\,\theta_{l,r}

$$
#### 机体转动方程

$$

\begin{aligned}

I_{b}\ddot{\theta}_{b}=&\,-\tau_{l,l}-\tau_{l,r}-m_{b}gd_{b}\cos(\theta_{b}+\theta_{b}^{0})+m_{b}gd_{b}\sin\theta_{b} \\

&+m_{b}\Big[\tfrac{l_{l}}{2}\sin\theta_{l,l}\ddot{\theta}_{l,l}+\tfrac{l_{l}}{2}\cos\theta_{l,l}\dot{\theta}_{l,l}^{2}+\tfrac{l_{r}}{2}\sin\theta_{l,r}\ddot{\theta}_{l,r}+\tfrac{l_{r}}{2}\cos\theta_{l,r}\dot{\theta}_{l,r}^{2}\Big]\sin\theta_{b} \\

&-m_{b}\Big[\ddot{x}+\tfrac{l_{l}}{2}\cos\theta_{l,l}\ddot{\theta}_{l,l}-\tfrac{l_{l}}{2}\sin\theta_{l,l}\dot{\theta}_{l,l}^{2}+\tfrac{l_{r}}{2}\cos\theta_{l,r}\ddot{\theta}_{l,r}-\tfrac{l_{r}}{2}\sin\theta_{l,r}\dot{\theta}_{l,r}^{2}\Big]d_{b}\cos\theta_{b}

\end{aligned}

$$
小角度线性化
$$

I_{b}\ddot{\theta}_{b}=-\tau_{l,l}-\tau_{l,r}-m_{b}gd_{b}+m_{b}gd_{b}\theta_{b}-m_{b}d_{b}\ddot{x}-\frac{m_{b}d_{b}}{2}\big(l_{l}\ddot{\theta}_{l,l}+l_{r}\ddot{\theta}_{l,r}\big)

$$
### 标准动力学

$$
M(q)\ddot{q}+C(q,\dot{q})\dot{q}+G(q)=B(q)\tau
$$
$C$ 汇集 $\dot{\theta}^{2}$ 离心项，此处不展开 $M,B$ 元素，只给维数与 $G$。
 
$q=[\theta_{w,l},\theta_{w,r},\theta_{l,l},\theta_{l,r},\theta_{b}]^{T}$，$\tau=[\tau_{w,l},\tau_{w,r},\tau_{l,l},\tau_{l,r}]^{T}$。**$M$**：$5\times 5$；**$B$**：$5\times 4$

$$

G=\begin{bmatrix}

0 \\

0 \\

\big(m_{l}+\tfrac{m_{b}}{2}\big)gl_{l}\sin\theta_{l,l}+m_{l}gd_{l}\sin(\theta_{l,l}+\theta_{l,l}^{0}) \\

\big(m_{l}+\tfrac{m_{b}}{2}\big)gl_{r}\sin\theta_{l,r}+m_{l}gd_{l}\sin(\theta_{l,r}+\theta_{l,r}^{0}) \\

m_{b}gd_{b}\cos(\theta_{b}+\theta_{b}^{0})-m_{b}gd_{b}\sin\theta_{b}

\end{bmatrix}

$$
#### 平衡点计算

对于 $q$ 在平衡点，满足 $G(q)=0$，解得
$$
\theta_{l,l}^{\text{eq}} = \arctan\left( \dfrac{-m_l d_l \sin\theta_{l,l}^0}{\left(m_l + \frac{m_b}{2}\right) l_l + m_l d_l \cos\theta_{l,l}^0} \right)
$$
$$
\theta_{l,r}^{\text{eq}} = \arctan\left( \dfrac{-m_l d_l \sin\theta_{l,r}^0}{\left(m_l + \frac{m_b}{2}\right) l_r + m_l d_l \cos\theta_{l,r}^0} \right) \\[15pt]
$$
$$
\theta_{b} = \dfrac{\pi}{4} - \dfrac{\theta_b^0}{2}
$$
测算之后将平衡点代入式子，计算得到 $A,B$ 矩阵