#include "CO23Click.h"

// Maximum timeout
#define CO2_TIMEOUT			0x3000
#define BITS_IN_BYTE        8u
static I2C_HandleTypeDef i2cHandle;

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

void C02Click_Init(I2C_HandleTypeDef handle)
{
    i2cHandle = handle;
}

SentSts_t C02Click_GetSensorStatus(void)
{
    SentSts_t StatusMsg = { 0u };
    StatusMsg.SensStatus = I2CReadReg(SENS_STS);

    return StatusMsg;
}

uint8_t C02Click_GetProdId(void)
{
    uint8_t ProdId = I2CReadReg(PROD_ID);
//    ProdId &= PROD_ID_MASK;
//    ProdId >>= (BITS_IN_BYTE - PROD_POS);

    return ProdId;
}
