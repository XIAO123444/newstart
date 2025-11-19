#include "HMC5883L.h"
#include "zf_driver_soft_iic.h"

int16_t Compass_x;         // X轴原始数据
int16_t Compass_y;         // Y轴原始数据
int16_t Compass_z;         // Z轴原始数据

// 用库的枚举保存 IIC 通道

// 先按 7bit 地址使用；若读不到 ID，再把 HMC_ADDR 改为 (HMC5883L_I2C_ADDR << 1)
#define HMC_ADDR   (HMC5883L_I2C_ADDR_8BIT)

#if HMC5883L_USE_SOFT_IIC
static soft_iic_info_struct hmc_soft_iic;
#else
static const iic_index_enum hmc_iic_index = HMC5883L_IIC;
#endif

static inline void hmc_iic_write_reg(uint8_t reg, uint8_t val)
{
#if HMC5883L_USE_SOFT_IIC
    soft_iic_write_8bit_register(&hmc_soft_iic, reg, val);
#else
    iic_write_8bit_register(hmc_iic_index, HMC_ADDR, reg, val);
#endif
}

static inline void hmc_iic_read_regs(uint8_t reg, uint8_t* buf, uint16_t len)
{
#if HMC5883L_USE_SOFT_IIC
    soft_iic_read_8bit_registers(&hmc_soft_iic, reg, buf, len);
#else
    iic_read_8bit_registers(hmc_iic_index, HMC_ADDR, reg, buf, len);
#endif
}

uint8_t HMC5883L_Init(void)
{
#if HMC5883L_USE_SOFT_IIC
    soft_iic_init(&hmc_soft_iic,
                  HMC5883L_I2C_ADDR,      // ← 只传 0x1E
                  HMC5883L_SOFT_IIC_DELAY,
                  HMC5883L_SCL_PIN,
                  HMC5883L_SDA_PIN);
#else
    iic_init(hmc_iic_index,
             HMC_ADDR,
             HMC5883L_IIC_SPEED,
             HMC5883L_SCL_PIN,
             HMC5883L_SDA_PIN);
#endif

    uint8_t id[3] = {0xFF, 0xFF, 0xFF};  // 初始化为明显错误值
    hmc_iic_read_regs(HMC5883L_REG_ID_A, id, 3);
    printf("HMC5883L ID: 0x%02X 0x%02X 0x%02X\n", id[0], id[1], id[2]);

    if (!(id[0] == 'H' && id[1] == '4' && id[2] == '3'))
    {
        return 0;
    }

    hmc_iic_write_reg(HMC5883L_REG_CRA, 0x70u);
    hmc_iic_write_reg(HMC5883L_REG_CRB, 0x20u);
    hmc_iic_write_reg(HMC5883L_REG_MODE, 0x00u);

    system_delay_ms(67);
    return 1;
}

void HMC5883L_ReadRawData(void)
{
    uint8_t buf[6] = {0};
    hmc_iic_read_regs(HMC5883L_REG_DATA_X_MSB, buf, 6);

    Compass_x = (int16_t)((buf[0] << 8) | buf[1]);
    Compass_z = (int16_t)((buf[2] << 8) | buf[3]);
    Compass_y = (int16_t)((buf[4] << 8) | buf[5]);
}