#ifndef __BNO085_H__
#define __BNO085_H__

#include "main.h"
#include <stdbool.h>

/* BNO085 Euler angle output structure */
typedef struct {
  float yaw;           /* degrees, 0~360 (heading from magnetic north) */
  float pitch;         /* degrees, -90~90 */
  float roll;          /* degrees, -180~180 */
  float accuracy_deg;  /* heading accuracy estimate in degrees */
  bool valid;          /* true if data has been received at least once */
} bno085_euler_t;

/* Calibration accuracy levels (from sensor status bits) */
typedef enum {
  BNO085_CAL_UNRELIABLE = 0, /* 보정 안 됨 */
  BNO085_CAL_LOW        = 1, /* 낮은 정확도 */
  BNO085_CAL_MEDIUM     = 2, /* 중간 정확도 */
  BNO085_CAL_HIGH       = 3  /* 높은 정확도 - 보정 완료! */
} bno085_cal_accuracy_t;

/* Initialization: reset, boot, configure Rotation Vector report */
bool bno085_init(void);

/* Legacy compatibility entry point used by main bring-up path */
void bno085_communication_test(void);

/* Non-blocking process: call periodically. Returns true if new data parsed. */
bool bno085_process(void);

/* Get latest Euler angles */
bno085_euler_t bno085_get_euler(void);

/* ---- Calibration API ---- */

/* Begin magnetometer calibration (figure-8 motion required).
 * Enables dynamic calibration for accel + gyro + mag.
 * Call bno085_process() and monitor bno085_get_cal_accuracy()
 * until it reaches BNO085_CAL_HIGH (3). */
void bno085_calibrate_begin(void);

/* Stop dynamic calibration and lock current calibration values */
void bno085_calibrate_end(void);

/* Save current calibration data to sensor flash (persists across power cycles) */
bool bno085_calibrate_save(void);

/* Get current magnetometer calibration accuracy level (0~3) */
bno085_cal_accuracy_t bno085_get_cal_accuracy(void);

#endif /* __BNO085_H__ */
