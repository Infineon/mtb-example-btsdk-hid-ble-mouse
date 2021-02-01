/* Inactivity threshold register */
/*
 * Copyright 2016-2021, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
 *
 * lsm9ds1.c
 *
 */
#ifdef USE_LSM9DS1
#include "wiced.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_i2c.h"

/* LSM9DS1 Register defines *******************************************************/
/* Accelerometer and gyroscope registers */

#define ACCEL_ADDRESS     (0x6A) /* This is 0xD4 shifted right by 1 */

#define LSM9DS1_ACT_THS          0x04

 #define LSM9DS1_ACT_THS_SLEEP_ON_INACT_EN      0x80  /* [7]   SLEEP_ON_INACT_EN Gyroscope operating mode during inactivity */
 #define LSM9DS1_ACT_THS_ACT_THS_MASK           0x7f  /* [0:6] Inactivity threshold */

/* Inactivity duration register */
#define LSM9DS1_ACT_DUR          0x05 /* [0:7] Inactivity duration register */

/* Accelerometer interrupt configuration register */
#define LSM9DS1_INT_GEN_CFG_XL   0x06

 #define LSM9DS1_INT_GEN_CFG_XL_AOI_XL          0x80 /* [7] AND/OR combination of accelerometer’s interrupt events.
                                                        Default value: 0
                                                        0: OR combination
                                                        1: AND combination) */
 #define LSM9DS1_INT_GEN_CFG_XL_6D              0x40 /* [6] 6-direction detection function for interrupt.
                                                        Default value: 0
                                                        0: disabled
                                                        1: enabled) */
 #define LSM9DS1_INT_GEN_CFG_XL_ZHIE_XL         0x20 /* [5] Z-axis high byte interrupt enable
                                                            Enable interrupt generation on accelerometer’s Z-axis high event.
                                                            Default value: 0
                                                            0: disable interrupt request
                                                            1: interrupt request on measured acceleration value higher than preset threshold) */
 #define LSM9DS1_INT_GEN_CFG_XL_ZLIE_XL         0x10 /* [4] Z-axis low byte interrupt enable
                                                            Enable interrupt generation on accelerometer’s Z-axis low event.
                                                            Default value: 0
                                                            0: disable interrupt request
                                                            1: interrupt request on measured acceleration value lower than preset threshold)*/
 #define LSM9DS1_INT_GEN_CFG_XL_YHIE_XL         0x08 /* [3] Y-axis high byte interrupt enable
                                                            Enable interrupt generation on accelerometer’s Y-axis high event.
                                                            Default value: 0
                                                            0: disable interrupt request
                                                            1: interrupt request on measured acceleration value higher than preset threshold)*/
 #define LSM9DS1_INT_GEN_CFG_XL_YLIE_XL         0x04 /* [2] Y-axis low byte interrupt enable
                                                            Enable interrupt generation on accelerometer’s Y-axis low event.
                                                            Default value: 0
                                                            0: disable interrupt request
                                                            1: interrupt request on measured acceleration value lower than preset threshold)*/
 #define LSM9DS1_INT_GEN_CFG_XL_XHIE_XL         0x02 /* [1] X-axis high byte interrupt enable
                                                            Enable interrupt generation on accelerometer’s X-axis high event.
                                                            Default value: 0
                                                            0: disable interrupt request
                                                            1: interrupt request on measured acceleration value lower than preset threshold)*/
 #define LSM9DS1_INT_GEN_CFG_XL_XLIE_XL         0x01 /* [0] X-axis low byte interrupt enable
                                                            Enable interrupt generation on accelerometer’s X-axis low event.
                                                            Default value: 0
                                                            0: disable interrupt request
                                                            1: interrupt request on measured acceleration value lower than preset threshold)*/

/* Linear acceleration sensor interrupt threshold registers */
#define LSM9DS1_INT_GEN_THS_X_XL 0x07 /* Accelerometer X interrupt threshold */
                                      /* THS_XL_X [7:0] X-axis interrupt threshold. Default value: 0000 0000 */
#define LSM9DS1_INT_GEN_THS_Y_XL 0x08 /* Accelerometer Y interrupt threshold */
                                      /* THS_XL_Y [7:0] Y-axis interrupt threshold. Default value: 0000 0000 */
#define LSM9DS1_INT_GEN_THS_Z_XL 0x09 /* Accelerometer Z interrupt threshold */
                                                     /* THS_XL_Z [7:0] Z-axis interrupt threshold. Default value: 0000 0000 */

/* Linear acceleration sensor interrupt duration register. */
#define LSM9DS1_INT_GEN_DUR_XL   0x0a /* Accelerometer interrupt duration */

 #define LSM9DS1_INT_GEN_DUR_XL_WAIT_XL         0x80  /* [7] Wait function enabled on duration counter.
                                                             Default value: 0
                                                             0: wait function off
                                                             1: wait for DUR_XL [6:0] samples before exiting interrupt) */
 #define LSM9DS1_INT_GEN_DUR_XL_DUR_XL_MASK     0x7f  /* [0:6] Enter/exit interrupt duration value. Default value: 000 0000 */

/* Angular rate sensor reference value register for digital high-pass filter (r/w). */
#define LSM9DS1_REFERENCE_G                     0x0b /* Gyroscope reference value for high-pass filter */
                                                     /* REF_G [7:0] Reference value for gyroscope’s digital high-pass filter (r/w).
                                                        Default value: 0000 0000 */

#define LSM9DS1_INT1_CTRL        0x0c /* INT1_A/G pin control */

 #define LSM9DS1_INT1_CTRL_INT1_IG_G            0x80 /* [7] Gyroscope interrupt enable on INT 1_A/G pin. Default value: 0 (0: disabled; 1: enabled) */
 #define LSM9DS1_INT1_CTRL_INT_IG_XL            0x40 /* [6] Accelerometer interrupt generator on INT 1_A/G pin. Default value: 0 (0: disabled; 1: enabled) */
 #define LSM9DS1_INT1_CTRL_INT_FSS5             0x20 /* [5] FSS5 interrupt enable on INT 1_A/G pin. Default value: 0 (0: disabled; 1: enabled) */
 #define LSM9DS1_INT1_CTRL_INT_OVR              0x10 /* [4] Overrun interrupt on INT 1_A/G pin. Default value: 0 (0: disabled; 1: enabled) */
 #define LSM9DS1_INT1_CTRL_INT_FTH              0x08 /* [3] FIFO threshold interrupt on INT 1_A/G pin. Default value: 0 (0: disabled; 1: enabled) */
 #define LSM9DS1_INT1_CTRL_INT_                 0x04 /* [2] Boot Boot status available on INT 1_A/G pin. Default value: 0 (0: disabled; 1: enabled) */
 #define LSM9DS1_INT1_CTRL_INT_DRDY_G           0x02 /* [1] Gyroscope data ready on INT 1_A/G pin. Default value: 0 (0: disabled; 1: enabled) */
 #define LSM9DS1_INT1_CTRL_INT_DRDY_XL          0x01 /* [0] Accelerometer data ready on INT 1_A/G pin. Default value: 0 (0: disabled; 1: enabled) */

/* INT2_A/G pin control register.*/
#define LSM9DS1_INT2_CTRL        0x0d /* INT2_A/G pin control */

 #define LSM9DS1_INT2_CTRL_INT2_INACT           0x80 /* [7] Inactivity interrupt output signal. Default value: 0
                                                           (0: no interrupt has been generated; 1: one or more interrupt events have been generated) */
                                                     /* [6] Always write 0 */
 #define LSM9DS1_INT2_CTRL_INT2_FSS5            0x20 /* [5] FSS5 interrupt enable on INT2_A/G pin. Default value: 0 (0: disabled; 1: enabled) */
 #define LSM9DS1_INT2_CTRL_INT2_OVR             0x10 /* [4] Overrun interrupt on INT2_A/G pin. Default value: 0 (0: disabled; 1: enabled) */
 #define LSM9DS1_INT2_CTRL_INT2_FTH             0x08 /* [3] FIFO threshold interrupt on INT2_A/G pin. Default value: 0 (0: disabled; 1: enabled) */
 #define LSM9DS1_INT2_CTRL_INT2_DRDY_TEMP       0x04 /* [2] Temperature data ready on INT2_A/G pin. Default value: 0 (0: disabled; 1: enabled) */
 #define LSM9DS1_INT2_CTRL_INT2_DRDY_G          0x02 /* [1] Gyroscope data ready on INT2_A/G pin. Default value: 0 (0: disabled; 1: enabled) */
 #define LSM9DS1_INT2_CTRL_INT2_DRDY_XL         0x01 /* [0] Accelerometer data ready on INT2_A/G pin. Default value: 0 (0: disabled; 1: enabled) */

/* Who_AM_I register */
#define LSM9DS1_WHO_AM_I         0x0f /* Accelerometer and gyroscope device identification */

 #define LSM9DS1_WHO_AM_I_VALUE                 0x68 /* always return 0x68 */

/* Gyroscope control register 1 */
#define LSM9DS1_CTRL_REG1_G      0x10

 #define LSM9DS1_CTRL_REG1_G_ODR_G_MASK         0xe0 /* [7:5] Gyroscope output data rate selection. Default value: 000 */
 #define LSM9DS1_CTRL_REG1_G_ODR_G_POWER_DOWN   0x00 /* [7:5] Value: 000 Power down */
 #define LSM9DS1_CTRL_REG1_G_ODR_G_1            0x20 /* [7:5] value: 001 Ord: 14.9 Hz   Cutoff: 5 Hz */
 #define LSM9DS1_CTRL_REG1_G_ODR_G_2            0x40 /* [7:5] value: 010 Ord: 59.5 Hz   Cutoff: 19 Hz */
 #define LSM9DS1_CTRL_REG1_G_ODR_G_3            0x60 /* [7:5] value: 011 Ord: 119 Hz    Cutoff: 38 Hz */
 #define LSM9DS1_CTRL_REG1_G_ODR_G_4            0x80 /* [7:5] value: 100 Ord: 238 Hz    Cutoff: 76 Hz */
 #define LSM9DS1_CTRL_REG1_G_ODR_G_5            0xa0 /* [7:5] value: 101 Ord: 476 Hz    Cutoff: 100 Hz */
 #define LSM9DS1_CTRL_REG1_G_ODR_G_6            0xc0 /* [7:5] value: 110 Ord: 952 Hz    Cutoff: 100 Hz */
                                                     /* [7:5] value: 111 Not valid */
 #define LSM9DS1_CTRL_REG1_G_FS_G_MASK          0x18 /* [4:3] Gyroscope full-scale selection. Default value: 00 */
 #define LSM9DS1_CTRL_REG1_G_FS_G_0             0x00 /* [4:3] value: 00 245 dps */
 #define LSM9DS1_CTRL_REG1_G_FS_G_1             0x08 /* [4:3] value: 01 500 dps */
                                                     /* [4:3] value: 10 Not available */
 #define LSM9DS1_CTRL_REG1_G_FS_G_2             0x18 /* [4:3] value: 11 2000 dps */
 #define LSM9DS1_CTRL_REG1_G_BW_G_MASK          0x03 /* [1:0] Gyroscope bandwidth selection. Default value: 00 */
 #define LSM9DS1_CTRL_REG1_G_BW_G_0             0x00 /* [1:0] Value 00 */
 #define LSM9DS1_CTRL_REG1_G_BW_G_1             0x01 /* [1:0] Value 01 */
 #define LSM9DS1_CTRL_REG1_G_BW_G_2             0x02 /* [1:0] Value 10 */
 #define LSM9DS1_CTRL_REG1_G_BW_G_3             0x03 /* [1:0] Value 11 */

/* Gyroscope control register 2 */
#define LSM9DS1_CTRL_REG2_G      0x11
                                                     /* [7:4] Must be 0 */
 #define LSM9DS1_CTRL_REG2_G_INT_SEL_MASK       0x0c /* [3:2] INT selection configuration. Default value: 00 */
 #define LSM9DS1_CTRL_REG2_G_INT_SEL_0          0x00 /* [3:2] 00 */
 #define LSM9DS1_CTRL_REG2_G_INT_SEL_1          0x04 /* [3:2] 01 */
 #define LSM9DS1_CTRL_REG2_G_INT_SEL_2          0x08 /* [3:2] 10 */
 #define LSM9DS1_CTRL_REG2_G_INT_SEL_3          0x0c /* [3:2] 11 */
 #define LSM9DS1_CTRL_REG2_G_OUT_SEL_MASK       0x03 /* [1:0] Out selection configuration. Default value: 00 */
 #define LSM9DS1_CTRL_REG2_G_OUT_SEL_0          0x00 /* [1:0] 00 */
 #define LSM9DS1_CTRL_REG2_G_OUT_SEL_1          0x01 /* [1:0] 10 */
 #define LSM9DS1_CTRL_REG2_G_OUT_SEL_2          0x02 /* [1:0] 01 */
 #define LSM9DS1_CTRL_REG2_G_OUT_SEL_3          0x03 /* [1:0] 11 */

/* Gyroscope control register 3 */
#define LSM9DS1_CTRL_REG3_G      0x12

 #define LSM9DS1_CTRL_REG3_G_LP_mode            0x80 /* [7] Low-power mode enable. Default value: 0 (0: Low-power disabled; 1: Low-power enabled) */
 #define LSM9DS1_CTRL_REG3_G_HP_EN              0x40 /* [6] High-pass filter enable. Default value: 0 (0: HPF disabled; 1: HPF enabled) */
                                                     /* [5:4] Must be 0 */
 #define LSM9DS1_CTRL_REG3_G_HPCF_G_MASK        0x0f /* [3:0] Gyroscope high-pass filter cutoff frequency selection. Default value: 0000 */
 #define LSM9DS1_CTRL_REG3_G_HPCF_G(n)             n

/* Gyroscope sign and orientation */
#define LSM9DS1_ORIENT_CFG_G     0x13

/* Gyroscope interrupt source */
#define LSM9DS1_INT_GEN_SRC_G    0x14

/* Temperature low byte */
#define LSM9DS1_OUT_TEMP_L       0x15

/* Temperature high byte */
#define LSM9DS1_OUT_TEMP_H       0x16

/* Status register */
#define LSM9DS1_STATUS_REG       0x17
                                                     /* [7] Must be 0 */
 #define LSM9DS1_STATUS_REG_IG_XL               0x40 /* [6] Accelerometer interrupt output signal. Default value: 0
                                                            (0: no interrupt has been generated; 1: one or more interrupt events have been generated) */
 #define LSM9DS1_STATUS_REG_IG_G                0x20 /* [5] Gyroscope interrupt output signal. Default value: 0
                                                            (0: no interrupt has been generated; 1: one or more interrupt events have been generated) */
 #define LSM9DS1_STATUS_REG_INACT               0x10 /* [4] Inactivity interrupt output signal. Default value: 0
                                                            (0: no interrupt has been generated; 1: one or more interrupt events have been generated) */
 #define LSM9DS1_STATUS_REG__BOOT_STATUS        0x08 /* [3] Boot running flag signal. Default value: 0
                                                            (0: no boot running; 1: boot running) */
 #define LSM9DS1_STATUS_REG_TDA                 0x04 /* [2] Temperature sensor new data available. Default value: 0
                                                            (0: new data is not yet available; 1: new data is available) */
 #define LSM9DS1_STATUS_REG_GDA                 0x02 /* [1] Gyroscope new data available. Default value: 0
                                                            (0: a new set of data is not yet available; 1: a new set of data is available) */
 #define LSM9DS1_STATUS_REG_XLDA                0x01 /* [0] Accelerometer new data available. Default value: 0
                                                            (0: a new set of data is not yet available; 1: a new set of data is available) */
#define LSM9DS1_OUT_X_L_G        0x18 /* Gyroscope pitch (X) low byte */
#define LSM9DS1_OUT_X_H_G        0x19 /* Gyroscope pitch (X) high byte */
#define LSM9DS1_OUT_Y_L_G        0x1a /* Gyroscope roll (Y) low byte */
#define LSM9DS1_OUT_Y_H_G        0x1b /* Gyroscope roll (Y) high byte */
#define LSM9DS1_OUT_Z_L_G        0x1c /* Gyroscope yaw (Z) low byte */
#define LSM9DS1_OUT_Z_H_G        0x1d /* Gyroscope yaw (Z) high byte */
#define LSM9DS1_CTRL_REG4        0x1e /* Control register 4 */

/* Accelerometer control register 5 */
#define LSM9DS1_CTRL_REG5_XL     0x1f

 #define LSM9DS1_CTRL_REG5_XL_DEC_MASK          0xc0 /* [7:6] Decimation of acceleration data on OUT REG and FIFO. Default value: 00 */
 #define LSM9DS1_CTRL_REG5_XL_DEC_0             0x00 /* [7:6] 00: no decimation */
 #define LSM9DS1_CTRL_REG5_XL_DEC_1             0x40 /* [7:6] 01: update every 2 samples */
 #define LSM9DS1_CTRL_REG5_XL_DEC_2             0x80 /* [7:6] 10: update every 4 samples */
 #define LSM9DS1_CTRL_REG5_XL_DEC_3             0xc0 /* [7:6] 11: update every 8 samples */
 #define LSM9DS1_CTRL_REG5_XL_DEC_zEN_XL        0x20 /* [5] Accelerometer’s Z-axis output enable. Default value: 1 */
 #define LSM9DS1_CTRL_REG5_XL_DEC_yEN_XL        0x10 /* [4] Accelerometer’s y-axis output enable. Default value: 1 */
 #define LSM9DS1_CTRL_REG5_XL_DEC_xEN_XL        0x08 /* [3] Accelerometer’s x-axis output enable. Default value: 1 */
                                                     /* [2:0] must be zero */
/* Accelerometer control register 6 */
#define LSM9DS1_CTRL_REG6_XL     0x20

 #define LSM9DS1_CTRL_REG6_XL_ODR_XL_MASK       0xe0 /* [7:5] Output data rate and power mode selection. default value: 000 (*/
 #define LSM9DS1_CTRL_REG6_XL_ODR_XL_POWER_DOWN 0x00 /* [7:5] 000 Power down */
 #define LSM9DS1_CTRL_REG6_XL_ODR_XL_1          0x20 /* [7:5] 001 10 Hz */
 #define LSM9DS1_CTRL_REG6_XL_ODR_XL_2          0x40 /* [7:5] 010 50 Hz */
 #define LSM9DS1_CTRL_REG6_XL_ODR_XL_3          0x60 /* [7:5] 011 119 Hz */
 #define LSM9DS1_CTRL_REG6_XL_ODR_XL_4          0x80 /* [7:5] 100 238 Hz */
 #define LSM9DS1_CTRL_REG6_XL_ODR_XL_5          0xa0 /* [7:5] 101 476 Hz */
 #define LSM9DS1_CTRL_REG6_XL_ODR_XL_6          0xc0 /* [7:5] 110 952 Hz */
                                                     /* [7:5] 111 Not Valid */
 #define LSM9DS1_CTRL_REG6_XL_FS_XL_MASK        0x18 /* [4:3] Accelerometer full-scale selection. Default value: 00 */
 #define LSM9DS1_CTRL_REG6_XL_FS_XL_2G          0x00 /* [4:3] 00 ±2g */
 #define LSM9DS1_CTRL_REG6_XL_FS_XL_16G         0x08 /* [4:3] 01 ±16g */
 #define LSM9DS1_CTRL_REG6_XL_FS_XL_4G          0x10 /* [4:3] 10 ±4g */
 #define LSM9DS1_CTRL_REG6_XL_FS_XL_8G          0x18 /* [4:3] 11 ±8g */

 #define LSM9DS1_CTRL_REG6_XL_BW_SCAL_ODR       0x04 /* [2] Bandwidth selection. Default value: 0
                                                        0: bandwidth determined by ODR selection:
                                                            - BW = 408 Hz when ODR = 952 Hz, 50 Hz, 10 Hz;
                                                            - BW = 211 Hz when ODR = 476 Hz;
                                                            - BW = 105 Hz when ODR = 238 Hz;
                                                            - BW = 50 Hz when ODR = 119 Hz;
                                                        1: bandwidth selected according to BW_XL [2:1] selection */
 #define LSM9DS1_CTRL_REG6_XL_BW_XL_MASK        0x03 /* [1:0] When LSM9DS1_CTRL_REG6_XL_BW_SCAL_ODR=1, Anti-aliasing filter bandwidth selection. Default value: 00 */
 #define LSM9DS1_CTRL_REG6_XL_BW_XL_408_HZ      0x00 /* [1:0] 00 408 Hz */
 #define LSM9DS1_CTRL_REG6_XL_BW_XL_211_HZ      0x01 /* [1:0] 01 211 Hz */
 #define LSM9DS1_CTRL_REG6_XL_BW_XL_105_HZ      0x02 /* [1:0] 10 105 Hz */
 #define LSM9DS1_CTRL_REG6_XL_BW_XL_50_HZ       0x03 /* [1:0] 11 50 Hz */

/* Accelerometer control register 7 */
#define LSM9DS1_CTRL_REG7_XL     0x21

 #define LSM9DS1_CTRL_REG7_XL_HR                0x80 /* [7] High resolution mode for accelerometer enable. Default value: 0 */
 #define LSM9DS1_CTRL_REG7_XL_HR_DISABLE        0x00 /* [7]=0 High resolution mode for accelerometer disabled */
 #define LSM9DS1_CTRL_REG7_XL_HR_0              0x80 /* [7]=1,[6:5]=00 Accelerometer digital filter (high pass and low pass) cutoff frequency selection: ODR/50 */
 #define LSM9DS1_CTRL_REG7_XL_HR_1              0xa0 /* [7]=1,[6:5]=01 Accelerometer digital filter (high pass and low pass) cutoff frequency selection: ODR/100 */
 #define LSM9DS1_CTRL_REG7_XL_HR_2              0xc0 /* [7]=1,[6:5]=10 Accelerometer digital filter (high pass and low pass) cutoff frequency selection: ODR/9 */
 #define LSM9DS1_CTRL_REG7_XL_HR_3              0xe0 /* [7]=1,[6:5]=11 Accelerometer digital filter (high pass and low pass) cutoff frequency selection: ODR/400 */
                                                     /* [4:3] Must be zero */
 #define LSM9DS1_CTRL_REG7_XL_FDS               0x04 /* [2] FDS Filtered data selection. Default value: 0
                                                            (0: internal filter bypassed; 1: data from internal filter sent to output register and FIFO) */
                                                     /* [1] Must be zero */
 #define LSM9DS1_CTRL_REG7_XL_HPIS1             0x01 /* [0] High-pass filter enabled for acceleration sensor interrupt function on Interrupt. Default value: 0
                                                            (0: filter bypassed; 1: filter enabled) */

/* Control register 8 */
#define LSM9DS1_CTRL_REG8        0x22

 #define LSM9DS1_CTRL_REG8_BOOT                 0x80 /* [7] Reboot memory content. Default value: 0 (0: normal mode; 1: reboot memory content.
                                                            Boot request is executed as soon as internal oscillator is turned-on. It is possible
                                                            to set bit while in powerdown mode, in this case it will be served at the next normal
                                                            mode or sleep mode.  */
 #define LSM9DS1_CTRL_REG8_BDU                  0x40 /* [6] Block data update. Default value: 0
                                                            (0: continuous update; 1: output registers not updated until MSB and LSB read) */
 #define LSM9DS1_CTRL_REG8_H_LACTIVE            0x20 /* [5] Interrupt activation level. Default value: 0
                                                            (0: interrupt output pins active high; 1: interrupt output pins active low) */
 #define LSM9DS1_CTRL_REG8_PP_OD                0x10 /* [4] Push-pull/open-drain selection on the INT1_A/G pin and INT2_A/G pin. Default value: 0
                                                            (0: push-pull mode; 1: open-drain mode) */
 #define LSM9DS1_CTRL_REG8_SIM                  0x08 /* [3] SPI serial interface mode selection. Default value: 0
                                                            (0: 4-wire interface; 1: 3-wire interface). */
 #define LSM9DS1_CTRL_REG8_IF_ADD_INC           0x04 /* [2] Register address automatically incremented during a multiple byte access with a serial interface (I2C or SPI). Default value: 1
                                                            (0: disabled; 1: enabled) */
 #define LSM9DS1_CTRL_REG8_BLE                  0x02 /* [1] Big/Little Endian data selection. Default value 0
                                                            (0: data LSB @ lower address; 1: data MSB @ lower address) */
 #define LSM9DS1_CTRL_REG8_SW_RESET             0x01 /* [0] Software reset. Default value: 0
                                                            (0: normal mode; 1: reset device) This bit is cleared by hardware after next flash boot. */

/* Control register 9 */
#define LSM9DS1_CTRL_REG9        0x23
                                                     /* [7] Must be 0 */
 #define LSM9DS1_CTRL_REG9_SLEEP_G              0x40 /* [6] Gyroscope sleep mode enable. Default value: 0 (0: disabled; 1: enabled) */
                                                     /* [5] Must be 0 */
 #define LSM9DS1_CTRL_REG9_FIFO_TEMP_EN         0x10 /* [4] Temperature data storage in FIFO enable. Default value: 0 (0: temperature data not stored in FIFO; 1: temperature data stored in FIFO) */
 #define LSM9DS1_CTRL_REG9_DRDY_mask_bit        0x08 /* [3] Data available enable bit. Default value: 0 (0: DA timer disabled; 1: DA timer enabled) */
 #define LSM9DS1_CTRL_REG9_I2C_DISABLE          0x04 /* [2] Disable I2C interface. Default value: 0 (0: both I2C and SPI enabled; 1: I2C disabled, SPI only) */
 #define LSM9DS1_CTRL_REG9_FIFO_EN              0x02 /* [1] FIFO memory enable. Default value: 0 (0: disabled; 1: enabled) */
 #define LSM9DS1_CTRL_REG9_STOP_ON_FTH          0x01 /* [0] Enable FIFO threshold level use. Default value: 0 (0: FIFO depth is not limited; 1: FIFO depth is limited to threshold level) */

/* Control register 10 */
#define LSM9DS1_CTRL_REG10       0x24
                                                     /* [7:3] Must be 0 */
 #define LSM9DS1_CTRL_REG10_ST_G                0x04 /* [2] Angular rate sensor self-test enable. Default value: 0 (0: Self-test disabled; 1: Self-test enabled) */
                                                     /* [1] Must be 0 */
 #define LSM9DS1_CTRL_REG10_ST_XL               0x01 /* [0] Linear acceleration sensor self-test enable. Default value: 0 (0: Self-test disabled; 1: Self-test enabled) */

/* Accelerometer interrupt source */
#define LSM9DS1_INT_GEN_SRC_XL   0x26
                                                     /* [7] Must be 0 */
 #define LSM9DS1_INT_GEN_SRC_XL_IA_XL           0x40 /* [6] Interrupt active. Default value: 0. (0: no interrupt has been generated; 1: one or more interrupts have been generated) */
 #define LSM9DS1_INT_GEN_SRC_XL_ZH_XL           0x20 /* [5] Accelerometer’s Z high event. Default value: 0 (0: no interrupt, 1: Z high event has occurred) */
 #define LSM9DS1_INT_GEN_SRC_XL_ZL_XL           0x10 /* [4] Accelerometer’s Z low event. Default value: 0 (0: no interrupt; 1: Z low event has occurred) */
 #define LSM9DS1_INT_GEN_SRC_XL_YH_XL           0x08 /* [3] Accelerometer’s Y high event. Default value: 0 (0: no interrupt, 1: Y high event has occurred) */
 #define LSM9DS1_INT_GEN_SRC_XL_YL_XL           0x04 /* [2] Accelerometer’s Y low event. Default value: 0 (0: no interrupt, 1: Y low event has occurred) */
 #define LSM9DS1_INT_GEN_SRC_XL_XH_XL           0x02 /* [1] Accelerometer’s X high event. Default value: 0 (0: no interrupt, 1: X high event has occurred) */
 #define LSM9DS1_INT_GEN_SRC_XL_XL_XL           0x01 /* [0] Accelerometer’s X low. event. Default value: 0 (0: no interrupt, 1: X low event has occurred) */

/* Status register 2 */
#define LSM9DS1_STATUS_REG2      0x27
                                                     /* [7] Must be 0 */
 #define LSM9DS1_STATUS_REG2_IG_XL              0x40 /* [6] Accelerometer interrupt output signal. Default value: 0
                                                            (0: no interrupt has been generated; 1: one or more interrupt events have been generated) */
 #define LSM9DS1_STATUS_REG2_IG_G               0x20 /* [5] Gyroscope interrupt output signal. Default value: 0
                                                            (0: no interrupt has been generated; 1: one or more interrupt events have been generated) */
 #define LSM9DS1_STATUS_REG2_INACT              0x10 /* [4] Inactivity interrupt output signal. Default value: 0
                                                            (0: no interrupt has been generated; 1: one or more interrupt events have been generated) */
 #define LSM9DS1_STATUS_REG2_BOOT_STATUS        0x08 /* [3] Boot running flag signal. Default value: 0 (0: no boot running; 1: boot running) */
 #define LSM9DS1_STATUS_REG2_TDA                0x04 /* [2] Temperature sensor new data available. Default value: 0 (0: a new data is not yet available; 1: a new data is available) */
 #define LSM9DS1_STATUS_REG2_GDA                0x02 /* [1] Gyroscope new data available. Default value: 0 (0: a new set of data is not yet available; 1: a new set of data is available) */
 #define LSM9DS1_STATUS_REG2_XLDA               0x01 /* [0] Accelerometer new data available. Default value: 0
                                                            (0: a new set of data is not yet available; 1: a new set of data is available) */

#define LSM9DS1_OUT_X_L_XL       0x28 /* Accelerometer X low byte */
#define LSM9DS1_OUT_X_H_XL       0x29 /* Accelerometer X high byte */
#define LSM9DS1_OUT_Y_L_XL       0x2a /* Accelerometer Y low byte */
#define LSM9DS1_OUT_Y_H_XL       0x2b /* Accelerometer Y high byte */
#define LSM9DS1_OUT_Z_L_XL       0x2c /* Accelerometer Z low byte */
#define LSM9DS1_OUT_Z_H_XL       0x2d /* Accelerometer Z high byte */
#define LSM9DS1_FIFO_CTRL        0x2e /* FIFO control register */
#define LSM9DS1_FIFO_SRC         0x2f /* FIFO status control register */
#define LSM9DS1_INT_GEN_CFG_G    0x30 /* Gyroscope interrupt configuration */
#define LSM9DS1_INT_GEN_THS_XH_G 0x31 /* Gyroscope pitch (X) interrupt threshold high byte */
#define LSM9DS1_INT_GEN_THS_XL_G 0x32 /* Gyroscope pitch (X) interrupt threshold low byte */
#define LSM9DS1_INT_GEN_THS_YH_G 0x33 /* Gyroscope roll (Y) interrupt threshold high byte */
#define LSM9DS1_INT_GEN_THS_YL_G 0x34 /* Gyroscope roll (Y) interrupt threshold low byte */
#define LSM9DS1_INT_GEN_THS_ZH_G 0x35 /* Gyroscope yaw (Z) interrupt threshold high byte */
#define LSM9DS1_INT_GEN_THS_ZL_G 0x36 /* Gyroscope yaw (Z) interrupt threshold low byte */
#define LSM9DS1_INT_GEN_DUR_G    0x37 /* Gyroscope interrupt duration */

typedef struct {
  uint8_t disabled:1;
} lsm9ds1_t;

static lsm9ds1_t lsm9ds1={};

/*******************************************************************************
 * Function Name: LSMDS1_read()
 ********************************************************************************
 * Summary: read register value
 *
 * Parameters:
 *  uint8_t reg -- register to read
 *
 * Return:
 *  read value
 *
 *******************************************************************************/
static uint8_t LSM9DS1_read(uint8_t reg)
{
    uint8_t data=0;
    if (!lsm9ds1.disabled)
    {
        wiced_hal_i2c_combined_read(&data,1,&reg,1,ACCEL_ADDRESS);
    }
    return data;
}

/*******************************************************************************
 * Function Name: LSMDS1_write()
 ********************************************************************************
 * Summary: write register with value
 *
 * Parameters:
 *  uint8_t reg -- register to write
 *  uint8_t data -- data to write
 *
 * Return:
 *  none
 *
 *******************************************************************************/
static void LSM9DS1_write(uint8_t reg, uint8_t data)
{
    if (!lsm9ds1.disabled)
    {
        uint8_t buff[2];

        buff[0] = reg;
        buff[1] = data;

        wiced_hal_i2c_write(buff,2,ACCEL_ADDRESS);
    }
}

/*******************************************************************************
 * Function Name: LSMDS1_read_xy(int * x, int *y)
 ********************************************************************************
 * Summary: write register with value
 *
 * Parameters:
 *  uint16_t *x  -- pointer ot x data
 *  uint16_t *y  -- pointer ot y data
 *
 * Return:
 *  none, *y, *x read data
 *
 *******************************************************************************/
static int LSM9DS1_read_xy(int16_t * x, int16_t *y)
{
    if (!lsm9ds1.disabled)
    {
        struct
        {
           int16_t ax;
           int16_t ay;
        } __attribute__((packed)) accelData;

        uint8_t  reg = LSM9DS1_OUT_X_L_XL;

        wiced_hal_i2c_combined_read((uint8_t *)&accelData,sizeof(accelData),&reg,1,ACCEL_ADDRESS);
        *x = accelData.ax;
        *y = accelData.ay;
        return TRUE;
    }
    return FALSE;
}

/*******************************************************************************
 * Function Name: lsm9ds1_init()
 ********************************************************************************
 * Summary: Initialize lsm9ds1
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void lsm9ds1_poll()
{
    int16_t x,y;

    if (LSM9DS1_read_xy(&x, &y))
    {
        WICED_BT_TRACE("\nAcel x,y = %d, %d",x,y);
    }
}

/*******************************************************************************
 * Function Name: lsm9ds1_init()
 ********************************************************************************
 * Summary: Initialize lsm9ds1
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void lsm9ds1_init()
{
    /*Initialize I2C and set speed to 400kHz */
    wiced_hal_i2c_init();
    wiced_hal_i2c_set_speed(I2CM_SPEED_400KHZ);

    if (LSM9DS1_read(LSM9DS1_WHO_AM_I)!= LSM9DS1_WHO_AM_I_VALUE)
    {
        WICED_BT_TRACE("\n*** Failed to read LSM9DDS1 motion sensor device");
        lsm9ds1.disabled = 1;
    }
    else
    {
        LSM9DS1_write(LSM9DS1_CTRL_REG6_XL, LSM9DS1_CTRL_REG6_XL_ODR_XL_3);
        WICED_BT_TRACE("\nMotion sensor initialized");
    }
}

/*******************************************************************************
 * Function Name: lsm9ds1_shudown()
 ********************************************************************************
 * Summary: shut down lsm9ds1
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void lsm9ds1_shutdown()
{
    LSM9DS1_write(LSM9DS1_CTRL_REG1_G, LSM9DS1_CTRL_REG1_G_ODR_G_POWER_DOWN);
    LSM9DS1_write(LSM9DS1_CTRL_REG6_XL, LSM9DS1_CTRL_REG6_XL_ODR_XL_POWER_DOWN);
}

#endif // USE_LSM9DS1
