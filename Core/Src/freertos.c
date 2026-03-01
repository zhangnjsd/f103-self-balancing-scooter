/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/**
 *
 * ฅ^._.^ฅ
 *
 * ᓚ₍⑅^..^₎♡₍^. .^₎⟆
 *
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "main.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_tim.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/_intsup.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;
} RawAccGyro;

typedef struct {
  float roll;
  float pitch;
  float yaw;
} Eula;

typedef struct {
  short l;
  short r;
} Speed;

typedef struct {
  float Kp;
  float Ki;
  float Kd;
  float SetPoint;
  float LastError;
  float SumError;
  float ILimit;
  float OutLimit;
} PID;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ? MPU6050 I2C address and registers START
#define MPU6050_PHY_ADDR 0x68 << 1
#define MPU6050_REG_ADDR 0x6B
#define MPU6050_DATA_ADDR 0x3B
// ? MPU6050 I2C address and registers END

// ? Control loop timing
#define CTRL_PERIOD_MS 5U
#define ANGLE_PERIOD_MS 5U
#define SPEED_PERIOD_MS 10U

#define CTRL_DT_S ((float)CTRL_PERIOD_MS / 1000.0f)
#define ANGLE_DT_S ((float)ANGLE_PERIOD_MS / 1000.0f)
#define SPEED_DT_S ((float)SPEED_PERIOD_MS / 1000.0f)
#define SAMPLE_ALPHA 0.98
#define MPU6050_GYRO_LSB_PER_DPS                                               \
  131.0f // MPU6050 sensitivity for gyroscope in LSB per degree per second
#define DEG2RAD 0.01745329251f // Conversion factor from degrees to radians
#define RAD2DEG 57.2957795f    // Conversion factor from radians to degrees
#define QUAT_CORR_KP                                                           \
  0.8f // Proportional gain for quaternion correction in sensor fusion

// ? PID constants START
// * ANGALE PID
#define APID_KP 285.0f
#define APID_KI 0.0f
#define APID_KD 1.74f
// * ANGULAR VELOCITY PID
#define WPID_KP 1.4f
#define WPID_KI (WPID_KP / 200.0f)
#define WPID_KD 0.0f
// * VELOCITY PID
#define VPID_KP 0.15f
#define VPID_KI (VPID_KP / 200.0f)
#define VPID_KD 0.0f

// * Limitations and targets, refers to GitHub proj..
#define MECHANICAL_MEDIAN 0.0f // Mechanical median point for angle PID setpoint
#define VELO_TARGET 0.0f       // Target velocity for velocity PID
#define APID_I_LIMIT 3000.0f   // Integral limit for angle PID
#define WPID_I_LIMIT 3000.0f   // Integral limit for angular velocity PID
#define VPID_I_LIMIT 3000.0f   // Integral limit for velocity PID
#define APID_OUT_LIMIT 2000.0f // Output limit for angle PID
#define WPID_OUT_LIMIT 2000.0f // Output limit for angular velocity PID
#define VPID_OUT_LIMIT 100.0f  // Output limit for velocity PID
#define MOTOR_MIN_EFFECTIVE_PWM                                                \
  20U // Minimum effective PWM value for the motors
#define PWM_OUTPUT_SCALE                                                       \
  1.0f // Scale factor for converting PID output to PWM value
// ? PID constants END
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
RawAccGyro raw;
volatile Eula eula_incl;
Speed speed;
int16_t temp;
volatile float gyro_pitch_dps = 0.0f;
float encoder_lpf = 0.0f;
float aPIDOutput = 0.0f;
float wPIDOutput = 0.0f;
float vPIDOutput = 0.0f;
PID aPID;
PID wPID;
PID vPID;
float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
// ! Redirect printf to UART
int fputc(int ch, FILE *f) {
  uint8_t temp[1] = {(uint8_t)ch};
  HAL_UART_Transmit(&huart1, temp, 1, 2);
  return ch;
}

// ! Check and clamp a float value between a lower and upper bound (U)
static float clampf(float v, float lo, float hi) {
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

// ! Initialize PID
static void PIDInit(PID *pp, float kp, float ki, float kd, float setpoint,
                    float ilimit, float outlimit) {
  pp->Kp = kp;
  pp->Ki = ki;
  pp->Kd = kd;
  pp->SetPoint = setpoint;
  pp->LastError = 0.0f;
  pp->SumError = 0.0f;
  pp->ILimit = ilimit;
  pp->OutLimit = outlimit;
}

// ! PID step calculation
static float pid_step(PID *pp, float measured, float dt) {
  float err = pp->SetPoint - measured;
  pp->SumError += err * dt;
  pp->SumError = clampf(pp->SumError, -pp->ILimit, pp->ILimit);

  float d = (err - pp->LastError) / dt;
  pp->LastError = err;

  float out = pp->Kp * err + pp->Ki * pp->SumError + pp->Kd * d;
  return clampf(out, -pp->OutLimit, pp->OutLimit);
}

// ! PID calculation for angle control
static float PID_A(PID *pp, float pitch_deg) {
  return pid_step(pp, pitch_deg, ANGLE_DT_S);
}

// ! PID calculation for angular velocity control
static float PID_W(PID *pp, float gyro_dps) {
  return pid_step(pp, gyro_dps, CTRL_DT_S);
}

// ! PID calculation for velocity control
static float PID_V(PID *pp, float filtered_speed_sum) {
  return pid_step(pp, filtered_speed_sum, SPEED_DT_S);
}

// ! MPU6050 Init.
static void mpu6050_init(void) {
  uint8_t data = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_PHY_ADDR, MPU6050_REG_ADDR, 1, &data,
                    sizeof(data), HAL_MAX_DELAY);
}

/**
 * ! MPU6050 Read All Data
 *
 * @param accel_x Pointer to store accelerometer X-axis data
 * @param accel_y Pointer to store accelerometer Y-axis data
 * @param accel_z Pointer to store accelerometer Z-axis data
 * @param temp Pointer to store temperature data
 * @param gyro_x Pointer to store gyroscope X-axis data
 * @param gyro_y Pointer to store gyroscope Y-axis data
 * @param gyro_z Pointer to store gyroscope Z-axis data
 *
 */
static void mpu6050_read_all(RawAccGyro *raw, int16_t *temp) {
  uint8_t buf[14];
  HAL_I2C_Mem_Read(&hi2c1, MPU6050_PHY_ADDR, MPU6050_DATA_ADDR, 1, buf,
                   sizeof(buf), HAL_MAX_DELAY);

  raw->ax = (buf[0] << 8) | buf[1];
  raw->ay = (buf[2] << 8) | buf[3];
  raw->az = (buf[4] << 8) | buf[5];
  *temp = (buf[6] << 8) | buf[7];
  raw->gx = (buf[8] << 8) | buf[9];
  raw->gy = (buf[10] << 8) | buf[11];
  raw->gz = (buf[12] << 8) | buf[13];
}

// ! MPU6050 Quaternion Update using a simple sensor fusion algorithm
static void mpu6050_quat_update(const RawAccGyro *raw_sample, float dt) {
  float ax = (float)raw_sample->ax;
  float ay = (float)raw_sample->ay;
  float az = (float)raw_sample->az;

  float norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm > 1e-6f) {
    ax /= norm;
    ay /= norm;
    az /= norm;
  }

  float gx = ((float)raw_sample->gx / MPU6050_GYRO_LSB_PER_DPS) * DEG2RAD;
  float gy = ((float)raw_sample->gy / MPU6050_GYRO_LSB_PER_DPS) * DEG2RAD;
  float gz = ((float)raw_sample->gz / MPU6050_GYRO_LSB_PER_DPS) * DEG2RAD;

  float vx = 2.0f * (q1 * q3 - q0 * q2);
  float vy = 2.0f * (q0 * q1 + q2 * q3);
  float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  float ex = ay * vz - az * vy;
  float ey = az * vx - ax * vz;
  float ez = ax * vy - ay * vx;

  gx += QUAT_CORR_KP * ex;
  gy += QUAT_CORR_KP * ey;
  gz += QUAT_CORR_KP * ez;

  float half_dt = 0.5f * dt;
  float nq0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * half_dt;
  float nq1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * half_dt;
  float nq2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * half_dt;
  float nq3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * half_dt;

  float qnorm = sqrtf(nq0 * nq0 + nq1 * nq1 + nq2 * nq2 + nq3 * nq3);
  if (qnorm > 1e-6f) {
    q0 = nq0 / qnorm;
    q1 = nq1 / qnorm;
    q2 = nq2 / qnorm;
    q3 = nq3 / qnorm;
  }
}

// ! Convert quaternion to Euler angles (roll, pitch, yaw)
static void mpu6050_quat_to_euler(volatile Eula *eula) {
  float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
  float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
  eula->roll = atan2f(sinr_cosp, cosr_cosp) * RAD2DEG;

  float sinp = 2.0f * (q0 * q2 - q3 * q1);
  sinp = clampf(sinp, -1.0f, 1.0f);
  eula->pitch = asinf(sinp) * RAD2DEG;

  float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
  float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
  eula->yaw = atan2f(siny_cosp, cosy_cosp) * RAD2DEG;
}

// ! Clamp PWM magnitude to be within effective range and hardware limits
static uint16_t clamp_pwm_mag(int pwm) {
  uint16_t max_pwm = (uint16_t)__HAL_TIM_GET_AUTORELOAD(&htim1);
  int mag = (pwm >= 0) ? pwm : -pwm;
  if (mag < 0) {
    mag = 0;
  } else if (mag > 0 && mag < (int)MOTOR_MIN_EFFECTIVE_PWM) {
    mag = (int)MOTOR_MIN_EFFECTIVE_PWM;
  } else if (mag > (int)max_pwm) {
    mag = (int)max_pwm;
  }
  return (uint16_t)mag;
}

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void updateInclTask() {
  // ! Once...
  uint32_t next_tick = osKernelGetTickCount();
  for (;;) {
    // ! Loop...
    mpu6050_read_all(&raw, &temp);
    /* printf("RAW: AX: %d, AY: %d, AZ: %d, GX: %d, GY: %d, GZ: %d, TEMP:
       %d\r\n", raw.ax, raw.ay, raw.az, raw.gx, raw.gy, raw.gz, temp); */
    mpu6050_quat_update(&raw, ANGLE_DT_S);
    mpu6050_quat_to_euler(&eula_incl);
    gyro_pitch_dps = (float)raw.gx / MPU6050_GYRO_LSB_PER_DPS;
    next_tick += ANGLE_PERIOD_MS;
    osDelayUntil(next_tick);
  }
}

void motorControlTask() {
  // ! Once...
  uint32_t next_tick = osKernelGetTickCount();
  uint32_t angle_tick_acc = 0U;
  uint32_t speed_tick_acc = 0U;
  for (;;) {
    // ! Loop...
    angle_tick_acc += CTRL_PERIOD_MS;
    speed_tick_acc += CTRL_PERIOD_MS;

    if (angle_tick_acc >= ANGLE_PERIOD_MS) {
      angle_tick_acc = 0U;
      aPIDOutput = PID_A(&aPID, eula_incl.pitch);
      wPID.SetPoint = aPIDOutput;
    }

    if (speed_tick_acc >= SPEED_PERIOD_MS) {
      speed_tick_acc = 0U;
      speed.l = (short)__HAL_TIM_GET_COUNTER(&htim2);
      __HAL_TIM_SET_COUNTER(&htim2, 0);
      speed.r = (short)__HAL_TIM_GET_COUNTER(&htim3);
      __HAL_TIM_SET_COUNTER(&htim3, 0);

      float speed_sum = (float)(speed.l + speed.r);
      encoder_lpf = encoder_lpf * 0.8f + speed_sum * 0.2f;
      vPIDOutput = PID_V(&vPID, encoder_lpf);
      aPID.SetPoint = MECHANICAL_MEDIAN + vPIDOutput;
    }

    wPIDOutput = PID_W(&wPID, gyro_pitch_dps);
    int pwm_val = (int)(wPIDOutput * PWM_OUTPUT_SCALE);

    uint16_t pwm_mag = clamp_pwm_mag(pwm_val);

    // Set same spd for 2 channels
    if (pwm_val >= 0) {
      // ! Forward
      HAL_GPIO_WritePin(MOTOR_L_INPUT_1_GPIO_Port, MOTOR_L_INPUT_1_PIN,
                        GPIO_PIN_SET);
      HAL_GPIO_WritePin(MOTOR_L_INPUT_2_GPIO_Port, MOTOR_L_INPUT_2_PIN,
                        GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MOTOR_R_INPUT_1_GPIO_Port, MOTOR_R_INPUT_1_PIN,
                        GPIO_PIN_SET);
      HAL_GPIO_WritePin(MOTOR_R_INPUT_2_GPIO_Port, MOTOR_R_INPUT_2_PIN,
                        GPIO_PIN_RESET);
    } else {
      // ! Backward
      HAL_GPIO_WritePin(MOTOR_L_INPUT_1_GPIO_Port, MOTOR_L_INPUT_1_PIN,
                        GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MOTOR_L_INPUT_2_GPIO_Port, MOTOR_L_INPUT_2_PIN,
                        GPIO_PIN_SET);
      HAL_GPIO_WritePin(MOTOR_R_INPUT_1_GPIO_Port, MOTOR_R_INPUT_1_PIN,
                        GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MOTOR_R_INPUT_2_GPIO_Port, MOTOR_R_INPUT_2_PIN,
                        GPIO_PIN_SET);
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_mag);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_mag);

    next_tick += CTRL_PERIOD_MS;
    osDelayUntil(next_tick);
  }
}

void mainTask(void) {
  // ! Once...
  // ? Start Encoders...
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  // ? Start PWM...
  // * L
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  // * R
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  // ? Init MPU6050...
  mpu6050_init();
  PIDInit(&aPID, APID_KP, APID_KI, APID_KD, MECHANICAL_MEDIAN, APID_I_LIMIT,
          APID_OUT_LIMIT);
  PIDInit(&wPID, WPID_KP, WPID_KI, WPID_KD, 0.0f, WPID_I_LIMIT, WPID_OUT_LIMIT);
  PIDInit(&vPID, VPID_KP, VPID_KI, VPID_KD, VELO_TARGET, VPID_I_LIMIT,
          VPID_OUT_LIMIT);
  // ? PWRUP STDBY...
  HAL_GPIO_WritePin(TB6612_STDBY_GPIO_Port, TB6612_STDBY_PIN, GPIO_PIN_SET);
  for (;;) {
    // ! Loop...
    // * Heartbeat...
    osDelay(1000);
  }
}
/* USER CODE END Application */
