# L-System (Lindenmayer System)

## 原理
基于字符串重写规则的递归系统，用于模拟植物生长。

## 语法
- **F**: 向前画线
- **+**: 左转
- **-**: 右转
- **[**: 保存状态（入栈）
- **]**: 恢复状态（出栈）

## 经典案例

### 1. 科赫曲线 (Koch Curve)
```
Axiom: F
Rule: F → F+F--F+F
Angle: 60°
```

迭代3次：
- n=0: F
- n=1: F+F--F+F
- n=2: F+F--F+F+F+F--F+F--F+F--F+F+F+F--F+F
- 形成雪花形状

### 2. 龙形曲线 (Dragon Curve)
```
Axiom: FX
Rules:
  X → X+YF+
  Y → -FX-Y
Angle: 90°
```

### 3. 植物 A (Fractal Plant)
```
Axiom: X
Rules:
  X → F+[[X]-X]-F[-FX]+X
  F → FF
Angle: 25°
```
形成逼真的分枝植物！

### 4. 灌木 (Bush)
```
Axiom: F
Rule: F → FF+[+F-F-F]-[-F+F+F]
Angle: 22.5°
```

## 实现策略
1. 字符串重写 n 次
2. 用栈保存/恢复 (x, y, angle)
3. 根据规则绘制

## 参数化 L-System
```
Axiom: A(1)
Rules:
  A(t) → F(t)[+A(t*r)][-A(t*r)]
  F(t) → F(t*growth)
```
参数控制分支长度和粗细！
