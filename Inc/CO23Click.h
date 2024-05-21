#include <stdint.h>
#include "stm32f7xx_hal.h"

#define CO23_I2C_ADDR (0x28u << 1u)

/* Register map */
#define PROD_ID         (0x00u)
#define SENS_STS        (0x01u)
#define MEAS_RATE_H     (0x02u)
#define MEAS_RATE_L     (0x03u)   
#define MEAS_CFG        (0x04u)
#define CO2PPM_H        (0x05u)
#define CO2PPM_L        (0x06u)
#define MEAS_STS        (0x07u)
#define INT_CFG         (0x08u)
#define ALARM_TH_H      (0x09u)
#define ALARM_TH_L      (0x0Au)
#define PRESS_REF_H     (0x0Bu)
#define PRESS_REF_L     (0x0Cu)
#define CALIB_REF_H     (0x0Du)
#define CALIB_REF_L     (0x0Eu)
#define SCRATCH_PAD     (0x0Fu)
#define SENS_RST        (0x10u)

/*PROD_ID*/
#define PROD_POS        (3u)
#define PROD_ID_MASK    (0xE0u)
#define REV_POS         (3u)
#define REV_LEN         (5u)
/*SENS_STS*/
#define SEN_RDY_POS     (0u)
#define PWM_DIS_ST_POS  (1u)
#define ORTPM_POS       (2u)
#define ORVS_POS        (3u)
#define ICCER_POS       (4u)
#define ORTPM_CLR_POS   (5u)
#define ORVS_CLR_POS    (6u)
#define ICCER_CLR_POS   (7u)
/*MEAS_CFG*/
#define PWM_OUTEN_POS   (2u)
#define PWM_MODE_POS    (3u)
#define BOC_CFG_POS     (4u)
#define BOC_CFG_LEN     (2u)
#define OP_MODE_POS     (6u)
#define OP_MODE_LEN     (2u)
/*MEAS_STS*/
#define RES_POS         (2u)
#define DRDY_pos        (3u)
#define INT_STS         (4u)
#define ALARM_POS       (5u)
#define INT_STS_CLR_POS (6u)
#define ALARM_CLR_POS   (7u)
/*INT_CFG*/
#define INT_TYP_POS     (3u)
#define INT_FUNC_POS    (4u)
#define INT_FUNC_LEN    (3u)
#define ALARM_TYP_pos   (7u)

// typedef struct {
//     uint8_t PROD : 3;
//     uint8_t REV : 5;
// }ProdId_t;

typedef struct  {
    // uint8_t SEN_RDY : 1;
    // uint8_t PWM_DIS_ST : 1;
    // uint8_t ORTPM : 1;
    // uint8_t ORVS : 1;
    // uint8_t ICCER : 1;
    // uint8_t ORTPM_CLR :1;
    // uint8_t ORVS_CLR : 1;
    // uint8_t ICCER_CLR : 1;
    uint8_t SensStatus;
}SentSts_t;

// typedef struct {
//     uint8_t PWM_OUTEN : 1;
//     uint8_t PWM_MODE : 1;
//     uint8_t BOC_CFG : 2;
//     uint8_t OP_MODE : 2;
// }MeasCfg_t;

// typedef struct {
//     uint8_t RES : 1;
//     uint8_t DRDY : 1;
//     uint8_t INT_STS : 1;
//     uint8_t ALARM : 1;
//     uint8_t INT_STS_CLR : 1;
//     uint8_t ALARM_CLR : 1;
// }MeasSts_t;

// typedef struct {
//     uint8_t INT_TYP : 1;
//     uint8_t INT_FUNC : 3;
//     uint8_t ALARM_TYP : 1;
// }IntCfg_t;

typedef enum
{
    OpModeIdle = 0x00u,
    OpModeSingleShoot = 0x01u,
    OpModeContinuous = 0x02u,
    OpModeNumOfModes,
    OpModeInvalid = 0xFFu
} OpMode_t;

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
 * @brief Get status of a sensor
 * 
 * @return SentSts_t sensor status structure.
 */
extern SentSts_t C02Click_GetSensorStatus(void);

/**
 * @brief Set measurement rate of C02Click sensor
 * 
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
 * @brief Perform soft reset of C02 sensor.
 * 
 */
extern void C02Click_SoftReset(void);

/**
 * @brief Set ref pressure of C02 sensor.
 * 
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
extern void C02Click_Init(I2C_HandleTypeDef handle);

