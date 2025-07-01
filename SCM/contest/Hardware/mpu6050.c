#include "mpu6050.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"
#include "i2c.h"

//#define MPU6050_ADDR 0xD0 // 默认地址0x68左移一位

#define MPU6050_ADDR 0x68 // 使用7位设备地址（0x68或0x69）

uint8_t MPU6050_InIt(void)
{
    uint8_t check;
    uint8_t Data;
    
    // 1. 检查设备是否在线
    if (HAL_I2C_IsDeviceReady(&hi2c2, MPU6050_ADDR << 1, 3, 100) != HAL_OK) {
        return 1; // 设备未响应
    }
    
    // 2. 检查设备ID (WHO_AM_I 寄存器)
    if (HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR << 1, 0x75, I2C_MEMADD_SIZE_8BIT, &check, 1, 1000) != HAL_OK) {
        return 1; // 读取失败
    }
    
    if (check != 0x68) { // 设备ID应为0x68
        return 1; // 初始化失败
    }
    
    // 3. 重置设备（可选但推荐）
    Data = 0x80; // 复位位
    if (HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR << 1, 0x6B, I2C_MEMADD_SIZE_8BIT, &Data, 1, 1000) != HAL_OK) {
        return 1; // 复位失败
    }
    
    HAL_Delay(100); // 等待复位完成
    
    // 4. 唤醒MPU6050并选择时钟源
    Data = 0x01; // 选择X轴陀螺仪作为时钟源
    if (HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR << 1, 0x6B, I2C_MEMADD_SIZE_8BIT, &Data, 1, 1000) != HAL_OK) {
        return 1; // 唤醒失败
    }
    
    // 5. 设置加速度计量程 (±8g)
    Data = 0x10; // ±8g (灵敏度 4096 LSB/g)
    if (HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR << 1, 0x1C, I2C_MEMADD_SIZE_8BIT, &Data, 1, 1000) != HAL_OK) {
        return 1; // 加速度计量程设置失败
    }
    
    // 6. 设置陀螺仪量程 (±2000°/s)
    Data = 0x18; // ±2000°/s (灵敏度 16.4 LSB/°/s)
    if (HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR << 1, 0x1B, I2C_MEMADD_SIZE_8BIT, &Data, 1, 1000) != HAL_OK) {
        return 1; // 陀螺仪量程设置失败
    }
    
    // 7. 设置数字低通滤波器 (DLPF)
    Data = 0x06; // 带宽 5Hz
    if (HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR << 1, 0x1A, I2C_MEMADD_SIZE_8BIT, &Data, 1, 1000) != HAL_OK) {
        return 1; // 滤波器设置失败
    }
    
//    // 8. 禁用所有中断（可选）
//    Data = 0x00;
//    if (HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR << 1, 0x38, I2C_MEMADD_SIZE_8BIT, &Data, 1, 1000) != HAL_OK) {
//        return 1; // 中断设置失败
//    }
//    
//    // 9. 禁用FIFO（可选）
//    Data = 0x00;
//    if (HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR << 1, 0x23, I2C_MEMADD_SIZE_8BIT, &Data, 1, 1000) != HAL_OK) {
//        return 1; // FIFO设置失败
//    }
    
    return 0; // 初始化成功
}

void MPU6050_Read_Accel(float *Accel)
{
    uint8_t buf[6];
    int16_t raw[3];
    
    // 读取加速度计原始数据
    HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR<<1, 0x3B, 1, buf, 6, 10000);
    
    // 合并高低字节
    raw[0] = (int16_t)(buf[0] << 8 | buf[1]);
    raw[1] = (int16_t)(buf[2] << 8 | buf[3]);
    raw[2] = (int16_t)(buf[4] << 8 | buf[5]);
    
    // 转换为g值 (±8g对应灵敏度4096 LSB/g)
    Accel[0] = raw[0] / 4096.0;
    Accel[1] = raw[1] / 4096.0;
    Accel[2] = raw[2] / 4096.0;
}

void MPU6050_Read_Gyro(float *Gyro)
{
    uint8_t buf[6];
    int16_t raw[3];
    
    // 读取陀螺仪原始数据
    HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR<<1, 0x43, 1, buf, 6, 10000);
    
    // 合并高低字节
    raw[0] = (int16_t)(buf[0] << 8 | buf[1]);
    raw[1] = (int16_t)(buf[2] << 8 | buf[3]);
    raw[2] = (int16_t)(buf[4] << 8 | buf[5]);
    
    // 转换为°/s (±2000°/s对应灵敏度16.4 LSB/°/s)
    Gyro[0] = raw[0] / 16.4;
    Gyro[1] = raw[1] / 16.4;
    Gyro[2] = raw[2] / 16.4;
}



#define	MPU6050_SMPLRT_DIV		0x19
#define	MPU6050_ACCEL_XOUT_H	0x3B
#define	MPU6050_GYRO_XOUT_H		0x43
#define	MPU6050_PWR_MGMT_1		0x6B
#define	MPU6050_WHO_AM_I		0x75

uint8_t MPU6050_init[6] = {0x01 ,0x00 ,0x09 ,0x06 ,0x18 ,0x18};
void MPU6050_Init(void)
{
	HAL_I2C_Mem_Write(&hi2c2,0xD0,MPU6050_PWR_MGMT_1,  I2C_MEMADD_SIZE_8BIT,&MPU6050_init[0],2,10);
	HAL_I2C_Mem_Write(&hi2c2,0xD0,MPU6050_SMPLRT_DIV,  I2C_MEMADD_SIZE_8BIT,&MPU6050_init[2],4,10);
}
void MPU6050_GetData(uint8_t *Arr)
{
	HAL_I2C_Mem_Read(&hi2c2,0xD0,MPU6050_ACCEL_XOUT_H ,I2C_MEMADD_SIZE_8BIT,Arr,6,10000); 
   	HAL_I2C_Mem_Read(&hi2c2,0xD0,MPU6050_GYRO_XOUT_H  ,I2C_MEMADD_SIZE_8BIT,Arr+6,6,10000); 
}
void convert_uint8_to_uint16(float *Accel,float *Gyro)
{
	static uint8_t arr[12];
	uint16_t output[6];
	MPU6050_GetData(arr);
    for (int i = 0; i < 12; i += 2)
    {
        output[i / 2] = (arr[i] << 8) | arr[i + 1];
    }
		
		Accel[0] = output[0] / 4096.0;
    Accel[1] = output[1] / 4096.0;
    Accel[2] = output[2] / 4096.0;
		Gyro[0] = output[3] / 16.4;
    Gyro[1] = output[4] / 16.4;
    Gyro[2] = output[5] / 16.4;
}
