#include <stdint.h>
#include "stm32f7xx_hal.h"

typedef enum
{
    OpModeIdle = 0x00u,
    OpModeSingleShoot = 0x01u,
    OpModeContinuous = 0x02u,
    OpModeNumOfModes,
    OpModeInvalid = 0xFFu
} OpMode_t;

typedef void (*MeasurementFinishedCallback_t)(uint16_t);

/**
 * @brief Get production if of a C02Click
 * 
 * @return uint8_t production id
 */
extern uint8_t C02Click_GetProdId(void);

/**
 * @brief Get REV of a C02Click
 * 
 * @return uint8_t REV of board.
 */
extern uint8_t C02Click_GetRev(void);

/**
 * @brief Set measurement rate of C02Click sensor
 *  min 5s max 4095s
 * @param mesRate measurement rate to be set.
 */
extern void C02Click_SetMeasurementRate(uint16_t mesRate);

/**
 * @brief REad C02 PPM of C02Click
 * 
 * @return uint16_t C02 ppm
 */
extern uint16_t C02Click_GetC02Ppm(void);

/**
 * @brief Set ref pressure of C02 sensor.
 * min 750, max 1150
 * @param refPressure ref pressure to set.
 */
extern void C02Click_SetRefPressure(uint16_t refPressure);

/**
 * @brief Set op mode of C02 click
 * 
 * @param OpMode op mode to set.
 */
extern void C02Click_SetOpMode(OpMode_t OpMode);

/**
 * @brief Initialize i2c handle of C02 click
 * 
 * @param handle handle to set.
 */
extern void C02Click_Init(I2C_HandleTypeDef handle, MeasurementFinishedCallback_t cbk);

/**
 * @brief C02Click cyclic job for continuous measurements
 * 
 */
extern void C02Click_CyclicJob1ms(void);

/**
 * @brief Read ref pressure
 * 
 * @return uint16_t 
 */
extern uint16_t C02Click_GetRefPressure(void);

/**
 * @brief Trigger continuous measurement sequence.
 * Function will set measurement rate, set mode to continuous and start measurement timer
 * Measurement finished callback will be called every \p MesRate seconds
 * 
 * @param MesRate measurement rate in seconds
 */
extern void C02Click_TriggerContinuousMeasurement(uint16_t MesRate);

/**
 * @brief Function stops measurement.
 * Function will set measurement op mode to idle, set mes rate to invalid and stop measurement timer.
 */
extern void C02Click_StopMeasurement(void);


