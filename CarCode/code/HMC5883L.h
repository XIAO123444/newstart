#ifndef __HMC5883L_H
#define __HMC5883L_H

#include "zf_common_headfile.h"
#include <stdint.h>

#define HMC5883L_USE_SOFT_IIC        (1)

#if HMC5883L_USE_SOFT_IIC
#include "zf_driver_soft_iic.h"
#else
#include "zf_driver_iic.h"
#endif

//==================================================== 软件 IIC 驱动 ====================================================
#define HMC5883L_SOFT_IIC_DELAY      (10U)
#define HMC5883L_SCL_PIN             (B2)
#define HMC5883L_SDA_PIN             (B0)
//==================================================== 软件 IIC 驱动 ====================================================

#define HMC5883L_I2C_ADDR            (0x1Eu)                  // 7bit 地址
#define HMC5883L_I2C_ADDR_8BIT       (HMC5883L_I2C_ADDR << 1) // 读写字节地址

#define HMC5883L_REG_CRA             (0x00u)
#define HMC5883L_REG_CRB             (0x01u)
#define HMC5883L_REG_MODE            (0x02u)
#define HMC5883L_REG_DATA_X_MSB      (0x03u)
#define HMC5883L_REG_DATA_X_LSB      (0x04u)
#define HMC5883L_REG_DATA_Z_MSB      (0x05u)
#define HMC5883L_REG_DATA_Z_LSB      (0x06u)
#define HMC5883L_REG_DATA_Y_MSB      (0x07u)
#define HMC5883L_REG_DATA_Y_LSB      (0x08u)
#define HMC5883L_REG_ID_A            (0x0Au)
#define HMC5883L_REG_ID_B            (0x0Bu)
#define HMC5883L_REG_ID_C            (0x0Cu)

uint8_t HMC5883L_Init(void);
void    HMC5883L_ReadRawData(void);

#endif