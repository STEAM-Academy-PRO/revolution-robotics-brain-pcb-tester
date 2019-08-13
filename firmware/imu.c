/*
 * imu.c
 *
 * Created: 2019. 08. 07. 17:17:14
 *  Author: Dániel Buga
 */ 

#include "test.h"
#include <hal_spi_m_sync.h>
#include <hal_delay.h>

#define IMU_REGISTER_WHOAMI ((uint8_t) 0x0Fu)

static struct spi_m_sync_descriptor spi;

static inline void _imu_send_write_address(uint8_t addr)
{
    uint8_t address = addr & 0x7Fu;

    struct spi_xfer xfer;

    xfer.txbuf = &address;
    xfer.rxbuf = NULL;
    xfer.size = 1u;
    
    spi_m_sync_transfer(&spi, &xfer);
}

static inline void _imu_send_read_address(uint8_t addr)
{
    uint8_t address = 0x80u | (addr & 0x7Fu);

    struct spi_xfer xfer;

    xfer.txbuf = &address;
    xfer.rxbuf = NULL;
    xfer.size = 1u;
    
    spi_m_sync_transfer(&spi, &xfer);
}

static void _imu_write_registers(uint8_t reg, uint8_t* pData, size_t data_count)
{
    struct spi_xfer xfer;

    xfer.txbuf = pData;
    xfer.rxbuf = NULL;
    xfer.size = data_count;
    
    gpio_set_pin_level(IMU_CS_pin, false);
    _imu_send_write_address(reg);
    spi_m_sync_transfer(&spi, &xfer);
    gpio_set_pin_level(IMU_CS_pin, true);
}

static void _imu_read_registers(uint8_t reg, uint8_t* pData, size_t data_count)
{
    struct spi_xfer xfer;

    xfer.txbuf = NULL;
    xfer.rxbuf = pData;
    xfer.size = data_count;
    
    gpio_set_pin_level(IMU_CS_pin, false);
    _imu_send_read_address(reg);
    spi_m_sync_transfer(&spi, &xfer);
    gpio_set_pin_level(IMU_CS_pin, true);
}

static void _imu_write_register(uint8_t reg, uint8_t data)
{
    struct spi_xfer xfer;
    
    uint8_t address = reg & 0x7Fu;
    uint8_t buffer[] = {address, data};

    xfer.txbuf = buffer;
    xfer.rxbuf = NULL;
    xfer.size = 2u;
    
    gpio_set_pin_level(IMU_CS_pin, false);
    spi_m_sync_transfer(&spi, &xfer);
    gpio_set_pin_level(IMU_CS_pin, true);
}

static uint8_t _imu_read_register(uint8_t reg)
{
    struct spi_xfer xfer;
    
    uint8_t address = 0x80u | (reg & 0x7Fu);
    uint8_t txBuffer[] = {address, 0xFFu};
    uint8_t rxBuffer[2];

    xfer.txbuf = txBuffer;
    xfer.rxbuf = rxBuffer;
    xfer.size = 2u;
    
    gpio_set_pin_level(IMU_CS_pin, false);
    spi_m_sync_transfer(&spi, &xfer);
    gpio_set_pin_level(IMU_CS_pin, true);

    return rxBuffer[1];
}

static void _imu_modify_register(uint8_t addr, uint8_t mask, uint8_t value)
{
    uint8_t reg = _imu_read_register(addr);
    reg &= ~mask;
    reg |= value;
    _imu_write_register(addr, reg);
}

void imu_init(void)
{
    hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM5_GCLK_ID_CORE, CONF_GCLK_SERCOM5_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
    hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM5_GCLK_ID_SLOW, CONF_GCLK_SERCOM5_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
    hri_mclk_set_APBDMASK_SERCOM5_bit(MCLK);

    gpio_set_pin_level(IMU_MOSI_pin, false);
    gpio_set_pin_direction(IMU_MOSI_pin, GPIO_DIRECTION_OUT);
    gpio_set_pin_function(IMU_MOSI_pin, IMU_MOSI_pin_function);

    gpio_set_pin_level(IMU_SCLK_pin, false);
    gpio_set_pin_direction(IMU_SCLK_pin, GPIO_DIRECTION_OUT);
    gpio_set_pin_function(IMU_SCLK_pin, IMU_SCLK_pin_function);

    gpio_set_pin_direction(IMU_MISO_pin, GPIO_DIRECTION_IN);
    gpio_set_pin_pull_mode(IMU_MISO_pin, GPIO_PULL_OFF);
    gpio_set_pin_function(IMU_MISO_pin, IMU_MISO_pin_function);

    gpio_set_pin_direction(IMU_INT1_pin, GPIO_DIRECTION_IN);
    gpio_set_pin_pull_mode(IMU_INT1_pin, GPIO_PULL_OFF);
    gpio_set_pin_function(IMU_INT1_pin, GPIO_PIN_FUNCTION_OFF);

    gpio_set_pin_direction(IMU_INT2_pin, GPIO_DIRECTION_IN);
    gpio_set_pin_pull_mode(IMU_INT2_pin, GPIO_PULL_OFF);
    gpio_set_pin_function(IMU_INT2_pin, GPIO_PIN_FUNCTION_OFF);

    /* CS permanently tied to ground */
    gpio_set_pin_level(IMU_CS_pin, true);
    gpio_set_pin_direction(IMU_CS_pin, GPIO_DIRECTION_OUT);
    gpio_set_pin_function(IMU_CS_pin, IMU_CS_pin_function);

    spi_m_sync_init(&spi, IMU_SERCOM);
    spi_m_sync_enable(&spi);
    
    /* disable i2c */
    delay_ms(20u);
    _imu_write_register(0x13u, 0x04u);
}

uint8_t imu_read_whoami(void)
{
    return _imu_read_register(IMU_REGISTER_WHOAMI);
}

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} imu_axl_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} imu_rot_t;

static bool is_axl_interrupt_set(void)
{
    return gpio_get_pin_level(IMU_INT1_pin);
}

static bool is_rot_interrupt_set(void)
{
    return gpio_get_pin_level(IMU_INT2_pin);
}

static void axl_set_positive_test_signal(void)
{
    _imu_modify_register(0x14u, 0x03u, 0x01u);
    delay_ms(20u);
}

static void axl_set_negative_test_signal(void)
{
    _imu_modify_register(0x14u, 0x03u, 0x02u);
    delay_ms(20u);
}

static void axl_clear_test_signal(void)
{
    _imu_modify_register(0x14u, 0x03u, 0x00u);
}

static void rot_set_positive_test_signal(void)
{
    _imu_modify_register(0x14u, 0x0Cu, 0x04u);
    delay_ms(20u);
}

static void rot_set_negative_test_signal(void)
{
    _imu_modify_register(0x14u, 0x0Cu, 0x0Cu);
    delay_ms(20u);
}

static void rot_clear_test_signal(void)
{
    _imu_modify_register(0x14u, 0x0Cu, 0x00u);
}

void read_axl_sample(imu_axl_t* data)
{
    while (!is_axl_interrupt_set())
    {
        /* wait */
    }
    /* 
    registers: (L, H)
    accel X: 28, 29
    accel Y: 2A, 2B
    accel Z: 2C, 2D
    */
    uint8_t regs[6];
    _imu_read_registers(0x28u, regs, sizeof(regs));
    
    data->x = regs[0] | (regs[1] << 8u);
    data->y = regs[2] | (regs[3] << 8u);
    data->z = regs[4] | (regs[5] << 8u);
}

void read_averaged_axl_sample(imu_axl_t* data)
{
    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;

    /* discard first */
    imu_axl_t axl;
    read_axl_sample(&axl);

    for (uint8_t i = 0u; i < 8u; i++)
    {
        read_axl_sample(&axl);
        
        x += axl.x;
        y += axl.y;
        z += axl.z;
    }
    
    data->x = x >> 3u;
    data->y = y >> 3u;
    data->z = z >> 3u;
}

void read_rot_sample(imu_rot_t* data)
{
    while (!is_rot_interrupt_set())
    {
        /* wait */
    }
    /* 
    registers: (L, H)
    rotat X: 22, 23
    rotat Y: 24, 25
    rotat Z: 26, 27
    */
    uint8_t regs[6];
    _imu_read_registers(0x22u, regs, sizeof(regs));
    
    data->x = regs[0] | (regs[1] << 8u);
    data->y = regs[2] | (regs[3] << 8u);
    data->z = regs[4] | (regs[5] << 8u);
}

void read_averaged_rot_sample(imu_rot_t* data)
{
    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;

    /* discard first */
    imu_rot_t rot;
    read_rot_sample(&rot);

    for (uint8_t i = 0u; i < 8u; i++)
    {
        read_rot_sample(&rot);
        
        x += rot.x;
        y += rot.y;
        z += rot.z;
    }
    
    data->x = x >> 3u;
    data->y = y >> 3u;
    data->z = z >> 3u;
}

    imu_axl_t idle_acceleration;
    imu_axl_t active_acceleration;
    imu_rot_t idle_rotation;
    imu_rot_t active_rotation;

bool imu_run_selftest(void)
{
    /* common configuration */

    /* reset device */
    _imu_write_register(0x12u, 0x01u);
    delay_ms(20u);

    /* disable i2c */
    _imu_write_register(0x13u, 0x04u);

    /* by default, INTx are open drain, active high */

    /* configure INT1 as XL, INT2 as gyro data ready */
    _imu_write_register(0x0Du, 0x01u);
    _imu_write_register(0x0Eu, 0x02u);
    
    /* by default, low address = low byte */
    /* default: FIFO bypass */
    
    _imu_write_register(0x10u, 0x60u); /**< axl 416Hz, +/-2g, LPF1=0 */
    _imu_write_register(0x11u, 0x60u); /**< gyro 416Hz, +/-245dps */
    
    _imu_write_register(0x16u, 0x00u); /**< gyro HPF off */
    _imu_write_register(0x17u, 0x00u); /**< axl LPF off */

    /* read idle acceleration & rotation */
    read_averaged_axl_sample(&idle_acceleration);

    bool success = true;

    /* test signal is somewhere between 90..1700mg of acceleration */
    
    /* set positive test signal on acceleration */
    axl_set_positive_test_signal();
    read_averaged_axl_sample(&active_acceleration);
    success &= (active_acceleration.x - idle_acceleration.x) > 1500; /* 1LSb = 0.061 mg at ±2 g full scale */
    success &= (active_acceleration.y - idle_acceleration.y) > 1500;
    success &= (active_acceleration.z - idle_acceleration.z) > 1500;

    /* set negative test signal on acceleration */
    axl_set_negative_test_signal();
    read_averaged_axl_sample(&active_acceleration);
    success &= (active_acceleration.x - idle_acceleration.x) < -1500; /* 1LSb = 0.061 mg at ±2 g full scale */
    success &= (active_acceleration.y - idle_acceleration.y) < -1500;
    success &= (active_acceleration.z - idle_acceleration.z) < -1500;

    axl_clear_test_signal();

    /* An angular rate gyroscope is a device that produces a positive-going digital output for counterclockwise rotation around the axis considered [datasheet 4.6.1] */
    delay_ms(20u);

    read_averaged_rot_sample(&idle_rotation);

    /* test signal is around 20..80dps */
    /* set positive test signal on rotation */
    rot_set_positive_test_signal();
    read_averaged_rot_sample(&active_rotation);
    success &= (active_rotation.x - idle_rotation.x) > 200; /* 1LSb = 70 mdps at ±2000 dps full scale */
    success &= (active_rotation.y - idle_rotation.y) > 200;
    success &= (active_rotation.z - idle_rotation.z) > 200;
    
    /* set negative test signal on rotation */
    rot_set_negative_test_signal();
    read_averaged_rot_sample(&active_rotation);
    success &= (active_rotation.x - idle_rotation.x) < -200; /* 1LSb = 70 mdps at ±2000 dps full scale */
    success &= (active_rotation.y - idle_rotation.y) < -200;
    success &= (active_rotation.z - idle_rotation.z) < -200;

    rot_clear_test_signal();

    return success;
}
