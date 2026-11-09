# 仿真记录

## 平衡控制+腿部控制仿真

### 平衡控制LQR参数
``` matlab
balance_controller_t.Q =

    10           0           0           0           0           0
    0         500           0           0           0           0
    0           0        2000           0           0           0
    0           0           0          50           0           0
    0           0           0           0       10000           0
    0           0           0           0           0         100

balance_controller_t.R =
    10     0
     0     1
```

### 状态反馈矩阵K(l)插值拟合参数

$$
K(l) = k(:,:,1)+k(:,:,2)l+k(:,:,3)l^2
$$

```matlab
balance_controller_t.k(:,:,1) =
   -0.6578   -5.3292  -19.4966   -6.1604  -30.2483   -8.7311
    2.8963   22.0211   60.0414   13.3426  -63.8742   -1.5015
balance_controller_t.k(:,:,2) =
   -2.1589  -13.7424  -83.1162    8.2557  138.9527   44.8030
  -13.1366 -101.2443 -248.5760  -65.3543 -230.0723  -71.4180
balance_controller_t.k(:,:,3) =
    3.5490   21.8760   91.2113  -35.3716 -194.0204  -69.0502
   18.1246  141.3095  326.9953  100.3417  379.8627  121.0402
```

### 腿部控制

滚转角roll控制（PID，暂未加入）

```matlab
kp: 0
ki: 0
kd: 0
```

腿长（PID+前馈，模拟弹簧阻尼系统）

```matlab
kp: 300
ki: 10
kd: 150
```

### 仿真结果

<center>
<img src="assets\sim_balance_and_leg_control\state.jpg", height="400"/>
</center>

<center>
<img src="assets\sim_balance_and_leg_control\controller_output.jpg", height="200"/>
</center>

<center>
<img src="assets\sim_balance_and_leg_control\torque.jpg", height="200"/>
</center>

## 简化模型仿真

### 状态空间模型

```matlab
ss_WLR =
 
  A = 
              x1         x2         x3         x4         x5         x6
   x1          0          1          0          0          0          0
   x2          0          0     -6.716          0    0.00029          0
   x3          0          0          0          1          0          0
   x4          0          0      74.95          0  -0.003236          0
   x5          0          0          0          0          0          1
   x6          0          0    -0.6186          0     0.5384          0
 
  B = 
            u1       u2
   x1        0        0
   x2    5.546  -0.3061
   x3        0        0
   x4   -27.17    3.416
   x5        0        0
   x6  0.05345   -3.746
 
  C = 
       x1  x2  x3  x4  x5  x6
   y1   1   0   0   0   0   0
   y2   0   1   0   0   0   0
   y3   0   0   1   0   0   0
   y4   0   0   0   1   0   0
   y5   0   0   0   0   1   0
   y6   0   0   0   0   0   1
 
  D = 
       u1  u2
   y1   0   0
   y2   0   0
   y3   0   0
   y4   0   0
   y5   0   0
   y6   0   0
```

### LQR参数

```matlab
Q =
    5           0           0           0           0           0
    0         500           0           0           0           0
    0           0        5000           0           0           0
    0           0           0          50           0           0
    0           0           0           0       10000           0
    0           0           0           0           0         100
R =
    10     0
     0     1
K =
   -0.6738   -7.1017  -34.5667   -4.4942   -9.8158   -2.1156
    0.6780    7.0792   26.6372    3.3570  -95.2629  -11.2667
```

### 仿真结果

<center>
<img src="assets\sim_balance_control_simple\simscape.jpg", height="400"/>
</center>
