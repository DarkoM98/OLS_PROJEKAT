#include "CO23Click.h"

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
#define REV_MASK        (0x1Fu)
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
/* MES RATE */
#define MES_RATE_LEN        (2u)
#define MES_RATE_LSB        (0u)
#define MES_RATE_MSB        (1u)
/* CO2PPM */
#define CO2PPM_LEN      (2u)
#define CO2PPM_LSB      (0u)
#define CO2PPM_MSB      (1u)
/* REF_PRESSURE */
#define REF_PRES_LEN    (2u)
#define REF_PRES_LSB    (0u)
#define REF_PRES_MSB    (1u)

// Maximum timeout
#define CO2_TIMEOUT			0x3000
#define BITS_IN_BYTE        8u
#define MEASUREMENT_RATE_INVALID    (uint64_t)(~(0u))
#define SECONDS_TO_MS_RATE          (1000u)
#define UINT16_LSB_MASK  (0xFFu)
#define UINT16_MSB_MASK   (0xFF00u)

static I2C_HandleTypeDef i2cHandle;
static MeasurementFinishedCallback_t MesFinishedCbk = NULL;
static uint64_t SystemTime = 0u;
static uint64_t MeasurementRateMs = MEASUREMENT_RATE_INVALID;
static uint64_t MeasurementTimer = MEASUREMENT_RATE_INVALID;
/**
 * @brief Read I2C register
 * 
 * @param Reg register to read
 * @return uint8_t output of the register
 */
static uint8_t I2CReadReg(uint8_t Reg);

/**
 * @brief Write i2C register
 * 
 * @param Reg register to write to
 * @param Data Data to write to register
 */
static void I2CWriteReg(uint8_t Reg, uint8_t Data);

static uint8_t I2CReadReg(uint8_t Reg)
{
    uint8_t Output = 0u;
    while(HAL_I2C_Master_Transmit(&i2cHandle, CO23_I2C_ADDR, &Reg, 1, CO2_TIMEOUT)!= HAL_OK);
    while(HAL_I2C_Master_Receive(&i2cHandle, CO23_I2C_ADDR, &Output, 1, CO2_TIMEOUT) != HAL_OK);

    return Output;
}

static void I2CWriteReg(uint8_t Reg, uint8_t Data)
{
    uint8_t WriteMsg[2u] = { Reg, Data};
    while(HAL_I2C_Master_Transmit(&i2cHandle, CO23_I2C_ADDR, WriteMsg, 2, CO2_TIMEOUT)!= HAL_OK);
}

void C02Click_Init(I2C_HandleTypeDef handle, MeasurementFinishedCallback_t cbk)
{
    i2cHandle = handle;
    MesFinishedCbk = cbk;
}

uint8_t C02Click_GetProdId(void)
{
    uint8_t ProdId = I2CReadReg(PROD_ID);
    ProdId &= PROD_ID_MASK;
    ProdId >>= (BITS_IN_BYTE - PROD_POS);

    return ProdId;
}

uint8_t C02Click_GetRev(void)
{
    uint8_t Rev = I2CReadReg(PROD_ID);
    Rev &= REV_MASK;

    return Rev;
}

void C02Click_SetMeasurementRate(uint16_t mesRate)
{
    uint8_t MesRateBuffer[MES_RATE_LEN] = { 0u };

    MesRateBuffer[MES_RATE_LSB] = (UINT16_LSB_MASK)&mesRate;
    MesRateBuffer[MES_RATE_MSB] = ((UINT16_MSB_MASK)&mesRate) >> BITS_IN_BYTE;

    I2CWriteReg(MEAS_RATE_H, MesRateBuffer[MES_RATE_MSB]);
    I2CWriteReg(MEAS_RATE_L, MesRateBuffer[MES_RATE_LSB]);

    MeasurementRateMs = mesRate*SECONDS_TO_MS_RATE;
}

uint16_t C02Click_GetC02Ppm(void)
{
    uint8_t Co2PpmBuffer[CO2PPM_LEN] = {0xFFu, 0xFFu};
    Co2PpmBuffer[CO2PPM_LSB] = I2CReadReg(CO2PPM_L);
    Co2PpmBuffer[CO2PPM_MSB] = I2CReadReg(CO2PPM_H);

    return Co2PpmBuffer[CO2PPM_LSB] | (Co2PpmBuffer[CO2PPM_MSB] << BITS_IN_BYTE);
}

void C02Click_SetRefPressure(uint16_t refPressure)
{
    uint8_t RefPressureBuffer[REF_PRES_LEN] = { 0u };

    RefPressureBuffer[REF_PRES_LSB] = refPressure & UINT16_LSB_MASK;
    RefPressureBuffer[REF_PRES_MSB] = (refPressure & UINT16_MSB_MASK) >> BITS_IN_BYTE;

    I2CWriteReg(PRESS_REF_L, RefPressureBuffer[REF_PRES_LSB]);
    I2CWriteReg(PRESS_REF_H, RefPressureBuffer[REF_PRES_MSB]);
}

uint16_t C02Click_GetRefPressure(void)
{
    uint8_t RefPressureBuffer[REF_PRES_LEN] = {0xFFu, 0xFFu};

    RefPressureBuffer[REF_PRES_LSB] = I2CReadReg(PRESS_REF_L);
    RefPressureBuffer[REF_PRES_MSB] = I2CReadReg(PRESS_REF_H);

    return RefPressureBuffer[REF_PRES_LSB] | (RefPressureBuffer[REF_PRES_MSB] << BITS_IN_BYTE);
}

void C02Click_CyclicJob1ms(void)
{
    SystemTime++;
    if(MEASUREMENT_RATE_INVALID != MeasurementRateMs && MEASUREMENT_RATE_INVALID != MeasurementTimer)
    {
        if(NULL != MesFinishedCbk)
        {
            MeasurementTimer--;
            if(0u == MeasurementTimer)
            {
                uint16_t Co2Ppm = C02Click_GetC02Ppm();
                MesFinishedCbk(Co2Ppm);
                MeasurementTimer = MeasurementRateMs;
            }
        }
    }
}

void C02Click_SetOpMode(OpMode_t OpMode)
{
    if(OpMode < OpModeInvalid)
    {
        uint8_t CfgReg = 0u;
        CfgReg |= (uint8_t)OpMode;
        I2CWriteReg(MEAS_CFG, CfgReg);
    }
}

void C02Click_TriggerContinuousMeasurement(uint16_t MesRate)
{
    C02Click_SetMeasurementRate(MesRate);
    C02Click_SetOpMode(OpModeContinuous);

    MeasurementRateMs = MesRate * SECONDS_TO_MS_RATE;
    MeasurementTimer = MeasurementRateMs; //start measurement rate timer
}

void C02Click_StopMeasurement(void)
{
    C02Click_SetOpMode(OpModeIdle);

    MeasurementRateMs = MEASUREMENT_RATE_INVALID;
    MeasurementTimer = MEASUREMENT_RATE_INVALID;
}
