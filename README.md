# 简易平衡车实现

## 简介

本平衡车采用了 STM32F103C6T6 作为主控芯片，结合 MPU6050 传感器实现自平衡功能。通过 PID 控制算法，平衡车能够在不同地形下保持稳定，并响应用户的控制指令。

## 引脚分配

![引脚分配图](https://cdn.jsdelivr.net/gh/zhangnjsd/img-jsdelivr-go/f103junc/p1r.png)

- **USART1**: 用于调试，波特率 115200。
- **I2C1**: 连接 MPU6050 传感器。
- **TIM1**: 用于电机 PWM 控制。
  - **CH1 (PA8)**: 左电机 PWM 输出。
  - **CH2 (PA9)**: 右电机 PWM 输出。
- **TIM2**: 用于左轮速度测量。
  - **CH1 (PA0)**: 左轮速度输入。
  - **CH2 (PA1)**: 右轮速度输入。
- **TIM3**: 用于右轮速度测量。
  - **CH1 (PA6)**: 左轮速度输入。
  - **CH2 (PA7)**: 右轮速度输入。
- **GPIO (PA4)**: STDBY 引脚，用于控制电机的启停。
- **GPIO (PB12)**: AIN 1（左）。
- **GPIO (PB13)**: AIN 2（左）。
- **GPIO (PB14)**: BIN 1（右）。
- **GPIO (PB15)**: BIN 2（右）。

## 实现理论

- 初始化 PID 数据(a, v, w)：

  ```C
  PIDInit(&aPID, APID_KP, APID_KI, APID_KD, MECHANICAL_MEDIAN, APID_I_LIMIT, APID_OUT_LIMIT);
  PIDInit(&wPID, WPID_KP, WPID_KI, WPID_KD, 0.0f, WPID_I_LIMIT, WPID_OUT_LIMIT);
  PIDInit(&vPID, VPID_KP, VPID_KI, VPID_KD, VELO_TARGET, VPID_I_LIMIT, VPID_OUT_LIMIT);
  ```

- 对获得的数据进行处理。`mpu6050_quat_update(const RawAccGyro *raw_sample, float dt)` 函数将原始加速度和陀螺仪数据转换为四元数表示的姿态信息（姿态解算算法）。
  - 加速度归一化：

    $$
    \vec{a} = \frac{1}{\|\vec{a}\|}(a_x, a_y, a_z)
    $$

    对应了：

    ```C
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm > 1e-6f) {
      ax /= norm;
      ay /= norm;
      az /= norm;
    }
    ```

  - 陀螺仪角速度转换为弧度：

    $$
    \vec{\omega} = \left(\frac{g_x}{k} \cdot \text{rad}, \frac{g_y}{k} \cdot \text{rad}, \frac{g_z}{k} \cdot \text{rad}\right)
    $$

    对应了：

    ```C
    float gx = ((float)raw_sample->gx / MPU6050_GYRO_LSB_PER_DPS) * DEG2RAD;
    float gy = ((float)raw_sample->gy / MPU6050_GYRO_LSB_PER_DPS) * DEG2RAD;
    float gz = ((float)raw_sample->gz / MPU6050_GYRO_LSB_PER_DPS) * DEG2RAD;
    ```

  - 四元数预测重力方向：

    $$
    v_x = 2(q_1 q_3 - q_0 q_2), \quad
    v_y = 2(q_0 q_1 + q_2 q_3), \quad
    v_z = q_0^2 - q_1^2 - q_2^2 + q_3^2
    $$

    对应了：

    ```C
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    ```

  - Error:

    $$
    e_x = a_y v_z - a_z v_y, \quad
    e_y = a_z v_x - a_x v_z, \quad
    e_z = a_x v_y - a_y v_x
    $$

    对应了：

    ```C
    float ex = ay * vz - az * vy;
    float ey = az * vx - ax * vz;
    float ez = ax * vy - ay * vx;
    ```

  - 陀螺仪修正：

    $$
    \omega_x' = \omega_x + K_p e_x, \quad
    \omega_y' = \omega_y + K_p e_y, \quad
    \omega_z' = \omega_z + K_p e_z
    $$

    对应了：

    ```C
    gx += QUAT_CORR_KP * ex;
    gy += QUAT_CORR_KP * ey;
    gz += QUAT_CORR_KP * ez;
    ```

  - 四元数积分：

    $$
    \begin{aligned}
    q_0' &= q_0 + \tfrac{1}{2}(-q_1 \omega_x' - q_2 \omega_y' - q_3 \omega_z') \Delta t \\
    q_1' &= q_1 + \tfrac{1}{2}(q_0 \omega_x' + q_2 \omega_z' - q_3 \omega_y') \Delta t \\
    q_2' &= q_2 + \tfrac{1}{2}(q_0 \omega_y' - q_1 \omega_z' + q_3 \omega_x') \Delta t \\
    q_3' &= q_3 + \tfrac{1}{2}(q_0 \omega_z' + q_1 \omega_y' - q_2 \omega_x') \Delta t
    \end{aligned}
    $$

    对应了：

    ```C
    float half_dt = 0.5f * dt;
    float nq0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * half_dt;
    float nq1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * half_dt;
    float nq2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * half_dt;
    float nq3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * half_dt;
    ```

  - 四元数归一化：

    $$
    q = \frac{q'}{\|q'\|}
    $$

    对应了：

    ```C
    float qnorm = sqrtf(nq0 * nq0 + nq1 * nq1 + nq2 * nq2 + nq3 * nq3);
    if (qnorm > 1e-6f) {
      q0 = nq0 / qnorm;
      q1 = nq1 / qnorm;
      q2 = nq2 / qnorm;
      q3 = nq3 / qnorm;
    }
    ```

- 四元数转欧拉角：

  $$
  \begin{aligned}
  \theta _{\text{pitch}} &= \arcsin(2(q_0 q_2 - q_1 q_3)) \\
  \phi _{\text{roll}} &= \arctan2(2(q_0 q_1 + q_2 q_3), 1 - 2(q_1^2 + q_2^2)) \\
  \psi _{\text{yaw}} &= \arctan2(2(q_0 q_3 + q_1 q_2), 1 - 2(q_2^2 + q_3^2))
  \end{aligned}
  $$

- 三环 PID 串级控制系统：
  
  本系统采用了位置（角度）、速度、角速度三环串级的 PID 控制。利用 MPU6050 读取的姿态信息及双侧电机的编码器速度反馈，共同维持平衡车的直立以及抵御位移干扰。控制频率分配如下：角度环周期 `5ms`，角速度环周期 `5ms`，速度环周期 `10ms`。

  PID 的计算公式：

  $$
  u(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de(t)}{dt}
  $$

  对应代码实现中的离散化 PID 步进函数：

  ```C
  static float pid_step(PID *pp, float measured, float dt) {
    float err = pp->SetPoint - measured;
    pp->SumError += err * dt;
    pp->SumError = clampf(pp->SumError, -pp->ILimit, pp->ILimit);

    float d = (err - pp->LastError) / dt;
    pp->LastError = err;

    float out = pp->Kp * err + pp->Ki * pp->SumError + pp->Kd * d;
    return clampf(out, -pp->OutLimit, pp->OutLimit);
  }
  ```

  - **速度环（最外环）**：通过控制车身产生极小的倾角进而控制车速或保持抵御位移干扰。当受到外力推动或偏离位置时，速度环会修正平衡的目标倾角。
    - **输入**：左右电机编码器读数之和，并经过低通滤波平滑处理的值 `encoder_lpf`。
  
      ```C
      float speed_sum = (float)(speed.l + speed.r);
      encoder_lpf = encoder_lpf * 0.8f + speed_sum * 0.2f;
      ```

    - **设定值**：目标速度 `VELO_TARGET`（当前为 0，即保持原地平衡）。
    - **输出**：角度环的输入偏置 `vPIDOutput`（加到 `MECHANICAL_MEDIAN` 机械中值上）。由于只有 P 与 I 参数介入，其起到了提供长效纠偏角度趋势的作用。
  
  - **角度环（中环）**：负责控制车身的宏观直立平衡，计算出需要向哪个方向倒伏多少角度，并输出期望的角速度给内环。
    - **输入**：经过姿态解算得到的俯仰角 `eula_incl.pitch`。
    - **设定值**：机械物理中值（`MECHANICAL_MEDIAN`）+ 速度外环提供的修正偏角。
    - **输出**：角速度环的期望给定量 `aPIDOutput`。因为该环主要是 PD 控制，利用 PD 参数的快速响应性把倾角快速转化为旋转角速度期望。

  - **角速度环（最内环）**：响应最快，直接控制电机的转速变化执行器，以使车身角速度跟踪中环下达的期望值。
    - **输入**：从 MPU6050 直接获取并转换为 DPS 的俯仰角速度 `gyro_pitch_dps`，比经过解算的角度响应更加即时。
    - **设定值**：角度环 PID 输出的期望角速度 `wPID.SetPoint = aPIDOutput;`。
    - **输出**：将 `wPIDOutput` 缩放后即为最终电机的 PWM 占空比。极高的 P 参数让底层电机迅速转动跟上姿态变化。

## 实现过程反馈

为了保证控制的实时性和稳定性，整个系统基于 FreeRTOS 划分为三个主要任务（Task）进行调度：

- **姿态更新任务（`updateInclTask`）**：
  - 运行周期为 `ANGLE_PERIOD_MS` (5ms)。
  - 负责定期读取 MPU6050 传感器的加速度、角速度和温度原始数据。
  - 执行传感器数据融合与四元数更新算法，最终将其转换为直观反映小车姿态的欧拉角（如俯仰角），并提取实际的俯仰角速度供下级 PID 直接使用。

- **电机控制与计算任务（`motorControlTask`）**：
  - 基础运行周期为 `CTRL_PERIOD_MS` (5ms)。串级 PID 逻辑在此任务内计算和调度：
    - 每逢 5ms 计算一次角度环和角速度内环：`PID_A(&aPID, ...)` / `PID_W(&wPID, ...)`。
    - 每逢 10ms (`SPEED_PERIOD_MS`) 计算一次速度外环：读取 TIM2 与 TIM3 编码器反馈获得双轮速度和，经滤波处理后带入 `PID_V(&vPID, ...)`计算。计算结果附加为角度环的新目标点。
  - 最后根据计算所得的最终输出 `wPIDOutput`、以及缩放比例限制出安全的 PWM 给定值，通过 `__HAL_TIM_SET_COMPARE` 应用于 TIM1；并判断占空比符号，用来控制左右两路电机的 `IN1` 与 `IN2` 引脚翻转方向。

- **主控与初始化任务（`mainTask`）**：
  - 系统加电和 FreeRTOS 启动时首先开始执行的模块。
  - 负责对编码器（TIM2/TIM3）、电机输出 PWM 定时器（TIM1）进行启动运算，初始化 MPU6050 数据。
  - 实例化外环与内环的所有 PID 控制器参数。
  - 拉高电机驱动芯片的待机引脚（`TB6612_STDBY_PIN`）。
  - 执行完毕后退化为延时挂起状态（每秒唤醒一次），作为基本的心跳功能。

## 未完成的一些构思

- :x: **行进：** 即使现在平衡车可以平衡，但没有实现前进和后退的功能。不过预留了速度环来实现这个功能。
- :x: **蓝牙控制：** 可将 ESP32C6 广播蓝牙与 STM32F103C6T6 通过 CAN(TWAI) 通信实现远程控制平衡车的功能。
- :x: **PID调优：** 即使可以稳定平衡车，但 PID 参数还没有经过调优，所以会发生抖动。
- :x: **电池管理：** 目前没有实现电池电量监测和管理功能，未来可以添加相关模块来提高系统的可靠性和安全性。
