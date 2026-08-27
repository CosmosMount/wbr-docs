---
share_link: https://share.note.sx/kiano7tx
share_updated: 2026-07-28T14:33:26+08:00
---
## 离地检测

离地检测主要包含支持力解算和离地控制逻辑处理，两方面对下台阶、飞坡的稳定和优雅都很重要。

```C++
this->P  = (Trev[0]+this->Fs) * cos_alpha + Trev[1] / this->len * sin_alpha;
this->ddlen   = (this->dlen - this->prev_dlen) * 1000.0f;
```


![[experiments-modules-1.png]]

```C++
this->Zw = (_az - Numeric::Gravity)
			- this->ddlen * cos_alpha
			+ 2.0f * this->dlen * this->dalpha * sin_alpha
			+ this->len * this->ddalpha * sin_alpha
			+ this->len * this->dalpha * this->dalpha * cos_alpha;
```

![[experiments-modules-2.png]]

![[experiments-modules-3.png]]

![[experiments-modules-4.png]]

```C++
this->Zw = (_az - Numeric::Gravity);
```

```C++
this->Zw = (_az - Numeric::Gravity) + this->len * this->dalpha * this->dalpha * cos_alpha;
```

![[experiments-modules-5.png]]


![[experiments-modules-7.png]]

![[experiments-modules-8.png]]

在受到撞击的时候会出现离地

![[experiments-modules-9.png]]

只开 $a_{z}-g$ 也会出现离地的情况

其实主导是P，把摆动量关闭之后会好

![[experiments-modules-10.png]]

![[experiments-modules-11.png]]

![[experiments-modules-12.png]]

![[experiments-modules-13.png]]

这里的观测量换成了len_pd 的结果

![[experiments-modules-14.png]]

![[experiments-modules-15.png]]

![[experiments-modules-16.png]]

![[experiments-modules-17.png]]

![[experiments-modules-18.png]]

![[experiments-modules-19.png]]

![[experiments-modules-20.png]]

![[experiments-modules-21.png]]

