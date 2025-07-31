//#include "mpu6050.h"
//#include "stm32f1xx_hal.h"
//#include "stm32f1xx_hal_def.h"
//#include "i2c.h"
//#include "math.h"
//#include "global.h"
//#include "filter.h"
//#include "usart.h"

//Quaternion NumQ = {1, 0, 0, 0};

//Angle_t Angle;
//MPU6050_Data_t mpu_data = {0};
//Quaternion q = {1.0f, 0.0f, 0.0f, 0.0f};  // 初始化四元数 表示初始姿态为水平。
//uint8_t G_ACCEL_RANGE = 0x00;  // ±2g
//uint8_t G_GYRO_RANGE = 0x18;   // ±2000°/s
//float mpu_offset[6] = {-1196.00800000000f,275.660000000000f,14781.4640000000f,-130.498000000000f,-2.87800000000000f,-2.60600000000000f};

//Klm_Flt_t EKF[3] = {{0.02,0,0.001,0.543,0,0},{0.02,0,0.001,0.543,0,0},{0.02,0,0.001,0.543,0,0}};	
//INT16_XYZ_t	 GYRO_OFFSET_RAW,ACC_OFFSET_RAW;		 				//零漂数据
//INT16_XYZ_t	 MPU6050_ACC_RAW,MPU6050_GYRO_RAW;	     		//读取值原始数据
//	
//	
//uint8_t MPU6050_OffSet_cail(INT16_XYZ_t value,INT16_XYZ_t *offset,uint16_t sensivity){
//	static int32_t tempgx=0,tempgy=0,tempgz=0;
//	static uint16_t cnt_a=0;//使用static修饰的局部变量，表明次变量具有静态存储周期，也就是说该函数执行完后不释放内存
//	if(cnt_a==0)
//	{
//		tempgx = 0;
//		tempgy = 0;
//		tempgz = 0;
//		cnt_a = 1;
//		offset->X = 0;
//		offset->Y = 0;
//		offset->Z = 0;
//	}
//	while(cnt_a!=201){
//		tempgx += value.X;
//		tempgy += value.Y; 
//		tempgz += value.Z-sensivity;			//加速度计校准 sensivity 等于 MPU6050初始化时设置的灵敏度值（8196LSB/g）;陀螺仪校准 sensivity = 0；

//		if(cnt_a==200)               //200个数值求平均
//		{
//			offset->X=tempgx/cnt_a;
//			offset->Y=tempgy/cnt_a;
//			offset->Z=tempgz/cnt_a;
//			cnt_a = 0;

//			return 1;
//		}
//		cnt_a++;
//	}
//	return 0;
//}

//int cail_flag = 1;
//void MPU6050_Offset(void){
//		uint16_t sensivity = 0;
//		if(cail_flag){
//				MPU6050_OffSet_cail(MPU6050_GYRO_RAW,&GYRO_OFFSET_RAW,0);
//				switch (G_ACCEL_RANGE) {
//						case 0x00: sensivity = 16384; break; // ±2g，转换因子为16384
//						case 0x08: sensivity = 8192; break;  // ±4g，转换因子为8192
//						case 0x10: sensivity = 4096; break;  // ±8g，转换因子为4096
//						case 0x18: sensivity = 2048; break;  // ±16g，转换因子为2048
//						default: sensivity = 16384; break;    // 默认量程范围为±2g
//				}
//				MPU6050_OffSet_cail(MPU6050_ACC_RAW,&ACC_OFFSET_RAW,sensivity);
//				cail_flag = 0;
//		}
//		//加速度去零偏AD值 
//		MPU6050_ACC_RAW.X -= ACC_OFFSET_RAW.X;
//		MPU6050_ACC_RAW.Y -= ACC_OFFSET_RAW.Y;
//		MPU6050_ACC_RAW.Z -= (ACC_OFFSET_RAW.Z-sensivity);
//				
//		//陀螺仪去零偏AD值 
//		MPU6050_GYRO_RAW.X -= GYRO_OFFSET_RAW.X;
//		MPU6050_GYRO_RAW.Y -= GYRO_OFFSET_RAW.Y;
//		MPU6050_GYRO_RAW.Z -= GYRO_OFFSET_RAW.Z;
//}


///******************************************************************************
//* 函  数：void MPU6050_AccRead(FLOAT_XYZ *Acc_filt)
//* 功  能：读取加速度的原始数据，并做一阶卡尔曼滤波
//* 参  数：*accData 原始数据的指针
//* 返回值：无
//* 备  注：无
//*******************************************************************************/
//void MPU6050_AccRead(FLOAT_XYZ_t *Acc_filt)
//{
//	  int16_t accData[3];
//	  uint8_t i;
//    accData[0] = (int16_t)MPU6050_ACC_RAW.X;
//    accData[1] = (int16_t)MPU6050_ACC_RAW.Y;
//    accData[2] = (int16_t)MPU6050_ACC_RAW.Z;
//	
//	  for(i=0;i<3;i++)//对加速度原始数据进行一阶卡尔曼滤波
//		{ 				
//			Kalman_Filter(&EKF[i], (float)accData[i]);  
//			accData[i] = EKF[i].Out;
//		}
//	  Acc_filt->X  = (float)accData[0];
//	  Acc_filt->Y  = (float)accData[1];
//	  Acc_filt->Z  = (float)accData[2];
//}

///******************************************************************************
//* 函  数：void MPU6050_GyroRead(FLOAT_XYZ *Gyr_filt)
//* 功  能：读取陀螺仪的原始数据，并做一阶低通滤波
//* 参  数：*gyroData 原始数据的指针
//* 返回值：无
//* 备  注：无
//*******************************************************************************/
//void MPU6050_GyroRead(FLOAT_XYZ_t *Gyr_filt)
//{
//	  int16_t gyroData[3];
//	  uint8_t i;
//    gyroData[0] = (int16_t)MPU6050_GYRO_RAW.X ;
//    gyroData[1] = (int16_t)MPU6050_GYRO_RAW.Y ;
//    gyroData[2] = (int16_t)MPU6050_GYRO_RAW.Z ;
//	
//	  for(i=0;i<3;i++)//对角速度原始数据进行一阶低通滤波
//		{  
//       const float factor = 0.15f;  //滤波参数			
//			 static float tBuff[3];
//			 gyroData[i] = tBuff[i] = tBuff[i] * (1 - factor) + gyroData[i] * factor;
//		}
//		Gyr_filt->X  = (float)gyroData[0];
//	  Gyr_filt->Y  = (float)gyroData[1];
//	  Gyr_filt->Z  = (float)gyroData[2];
//}

//// 添加原始读取函数
//void MPU6050_Read_Accel(INT16_XYZ_t *value)
//{
//		uint8_t buf[6];
//    HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR<<1, 0x3B, 1, buf, 6, 10000);
//    value->X = (buf[0] << 8) | buf[1];
//		value->Y = (buf[2] << 8) | buf[3];
//		value->Z = (buf[4] << 8) | buf[5];
//}

//void MPU6050_Read_Gyro(INT16_XYZ_t *value)
//{
//		uint8_t buf[6];
//    HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR<<1, 0x43, 1, buf, 6, 10000);
//    value->X = (buf[0] << 8) | buf[1];
//		value->Y = (buf[2] << 8) | buf[3];
//		value->Z = (buf[4] << 8) | buf[5];
//}


//uint8_t MPU6050_init[6] = {0x01 ,0x00 ,0x09 ,0x06 ,0x18 ,0x18};

//void MPU6050_Init(void)
//{
//    uint8_t readValue = 0;
//    
//    // 1. 检查设备ID (WHO_AM_I)
//    HAL_I2C_Mem_Read(&hi2c2, 0xD0, MPU6050_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &readValue, 1, 100);
//    if(readValue != 0x68) {
//        debug_printf("MPU6050 not found! ID: 0x%02X\r\n", readValue);
//				while(1);
//    }
//    
//    // 2. 写入配置
//    HAL_I2C_Mem_Write(&hi2c2, 0xD0, MPU6050_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &MPU6050_init[0], 2, 100);
//    // 5. 设置采样率分频器并验证
//    HAL_I2C_Mem_Write(&hi2c2, 0xD0, MPU6050_SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT, &MPU6050_init[2], 4, 100);
//    HAL_Delay(10); // 等待寄存器写入完成
//    
//		// 3. 设置加速度计量程并验证
//    uint8_t accel_range = G_ACCEL_RANGE;
//    HAL_I2C_Mem_Write(&hi2c2, 0xD0, MPU6050_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &accel_range, 1, 100);
//    HAL_Delay(10); // 等待寄存器写入完成
//    
//    HAL_I2C_Mem_Read(&hi2c2, 0xD0, MPU6050_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &readValue, 1, 100);
//    if((readValue & 0x18) != G_ACCEL_RANGE) {
//        debug_printf("Accel range config failed! Set: 0x%02X, Read: 0x%02X\r\n", 
//               G_ACCEL_RANGE, readValue & 0x18);
//				while(1);
//    }
//    
//    // 4. 设置陀螺仪量程并验证
//    uint8_t gyro_range = G_GYRO_RANGE;
//    HAL_I2C_Mem_Write(&hi2c2, 0xD0, MPU6050_GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &gyro_range, 1, 100);
//    HAL_Delay(10); // 等待寄存器写入完成
//    
//    HAL_I2C_Mem_Read(&hi2c2, 0xD0, MPU6050_GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &readValue, 1, 100);
//    if((readValue & 0x18) != G_GYRO_RANGE) {
//        debug_printf("Gyro range config failed! Set: 0x%02X, Read: 0x%02X\r\n", 
//               G_GYRO_RANGE, readValue & 0x18);
//				while(1);
//    }
//    
//    // 6. 最终状态检查
//    //debug_printf("MPU6050 initialized successfully!\r\n");
//}


//float MPU6050_ConvertAccel(float value, uint8_t range) 
//{
//    float factor; // 用于存储转换因子
//    // 根据加速度计的量程选择合适的转换因子
//    switch (range) {
//        case 0x00: factor = 16384.0; break; // ±2g，转换因子为16384
//        case 0x08: factor = 8192.0; break;  // ±4g，转换因子为8192
//        case 0x10: factor = 4096.0; break;  // ±8g，转换因子为4096
//        case 0x18: factor = 2048.0; break;  // ±16g，转换因子为2048
//        default: factor = 16384.0; break;    // 默认量程范围为±2g
//    }
//    // 将原始加速度值转换为g单位
//    return value / factor;
//}
// 
//float MPU6050_ConvertGyro(float value, uint8_t range) 
//{
//    float factor; // 用于存储转换因子
//    // 根据陀螺仪的量程选择合适的转换因子
//    switch (range) {
//        case 0x00: factor = 131.0; break;  // ±250°/s，转换因子为131
//        case 0x08: factor = 65.5; break;   // ±500°/s，转换因子为65.5
//        case 0x10: factor = 32.8; break;   // ±1000°/s，转换因子为32.8
//        case 0x18: factor = 16.4; break;   // ±2000°/s，转换因子为16.4
//        default: factor = 131.0; break;     // 默认量程范围为±250°/s
//    }
//    // 将原始陀螺仪值转换为°/s单位
//    return value / factor;
//}

//////读取传感器数据
////void MPU6050_ReadSensors(MPU6050_Data_t *Mpu)
////{
////		int16_t raw[6];
////		MPU6050_Read_Accel(raw);
////		MPU6050_Read_Gyro(raw+3);

////		float data[6];
////		for(int i=0; i<6; i++) {
////				if(i < 3) { // 加速度计卡尔曼滤波
////					Kalman_Filter(&EKF[i], (float)raw[i] - mpu_offset[i]);  
////					data[i] = EKF[i].Out;
//////						data[i] = (float)raw[i] - mpu_offset[i];
////				}
////				else { // 陀螺仪互补滤波
////						static float filtered[3];
////						const float factor = 0.15f;
////						int idx = i-3;
////						filtered[idx] = filtered[idx]*(1-factor) + (raw[i]-mpu_offset[i])*factor;
////						data[i] = filtered[idx];
////				}
////		}
//// 
////    Mpu->ax = MPU6050_ConvertAccel(data[0], G_ACCEL_RANGE);
////    Mpu->ay = MPU6050_ConvertAccel(data[1], G_ACCEL_RANGE);
////    Mpu->az = MPU6050_ConvertAccel(data[2], G_ACCEL_RANGE);
////    Mpu->gx = MPU6050_ConvertGyro(data[3], G_GYRO_RANGE) * AtR; // 转换为rad/s
////    Mpu->gy = MPU6050_ConvertGyro(data[4], G_GYRO_RANGE) * AtR;
////    Mpu->gz = MPU6050_ConvertGyro(data[5], G_GYRO_RANGE) * AtR;
////}




/////* 四元数解法初始化 */
////void imu_rest(void)
////{
////	NumQ.q0 =1;
////	NumQ.q1 = 0;
////	NumQ.q2 = 0;
////	NumQ.q3 = 0;	
////	GyroIntegError.x = 0;
////	GyroIntegError.y = 0;
////	GyroIntegError.z = 0;
////	Angle.pitch = 0;
////	Angle.roll = 0;
////	Angle.yaw = 0;
////}

////void GetAngle(const MPU6050_Data_t *pMpu,Angle_t *pAngE, float dt) 
////{		
////	volatile struct V Gravity,Acc,Gyro,AccGravity;
//// 
////	static  float KpDef = 0.5f ;
////	static  float KiDef = 0.0001f;
//////static  float KiDef = 0.00001f;
////	
////	float q0_t,q1_t,q2_t,q3_t;
////  //float NormAcc;	
////	float NormQuat; 
////	float HalfTime = dt * 0.5f;
//// 
////	//提取等效旋转矩阵中的重力分量 
////	Gravity.x = 2*(NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);								
////	Gravity.y = 2*(NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);						  
////	Gravity.z = 1-2*(NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);	
////	// 加速度归一化
////	//printf("accX:%d\r\n",MPU6050.accX);
////	float NormAcc = 1/sqrt(squa(pMpu->ax)+ squa(pMpu->ay) +squa(pMpu->az));
////	//printf("NorAcc%f\r\n",NormAcc);
////	//	NormAcc = Q_rsqrt(squa(MPU6050.accX)+ squa(MPU6050.accY) +squa(MPU6050.accZ));
////	
////  Acc.x = pMpu->ax * NormAcc;
////  Acc.y = pMpu->ay * NormAcc;
////  Acc.z = pMpu->az * NormAcc;
////	
//// 	//向量差乘得出的值
////	AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
////	AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
////	AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);
////	
////	//再做加速度积分补偿角速度的补偿值
////  GyroIntegError.x += AccGravity.x * KiDef;
////  GyroIntegError.y += AccGravity.y * KiDef;
////  GyroIntegError.z += AccGravity.z * KiDef;
////	
////	//角速度融合加速度积分补偿值
////  Gyro.x = pMpu->gx + KpDef * AccGravity.x  +  GyroIntegError.x;//弧度制
////  Gyro.y = pMpu->gy + KpDef * AccGravity.y  +  GyroIntegError.y;
////  Gyro.z = pMpu->gz + KpDef * AccGravity.z  +  GyroIntegError.z;
////	
////	// 一阶龙格库塔法, 更新四元数
////	q0_t = (-NumQ.q1*Gyro.x - NumQ.q2*Gyro.y - NumQ.q3*Gyro.z) * HalfTime;
////	q1_t = ( NumQ.q0*Gyro.x - NumQ.q3*Gyro.y + NumQ.q2*Gyro.z) * HalfTime;
////	q2_t = ( NumQ.q3*Gyro.x + NumQ.q0*Gyro.y - NumQ.q1*Gyro.z) * HalfTime;
////	q3_t = (-NumQ.q2*Gyro.x + NumQ.q1*Gyro.y + NumQ.q0*Gyro.z) * HalfTime;
////	
////	NumQ.q0 += q0_t;
////	NumQ.q1 += q1_t;
////	NumQ.q2 += q2_t;
////	NumQ.q3 += q3_t;
////	// 四元数归一化
////	NormQuat = 1/sqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
////	NumQ.q0 *= NormQuat;
////	NumQ.q1 *= NormQuat;
////	NumQ.q2 *= NormQuat;
////	NumQ.q3 *= NormQuat;	
////	
////	// 四元数转欧拉角
//////			pAngE->yaw = atan2f(2 * NumQ.q1 *NumQ.q2 + 2 * NumQ.q0 * NumQ.q3, 1 - 2 * NumQ.q2 *NumQ.q2 - 2 * NumQ.q3 * NumQ.q3) * RtA;  //yaw
////			
////			float yaw_G = pMpu->gz*RtA;
////			if((yaw_G > 0.03f) || (yaw_G < -0.03f)) //数据太小可以认为是干扰，不是偏航动作
////			{
////				pAngE->yaw  += yaw_G * dt;
////			}
////			pAngE->pitch  =  asin(2 * NumQ.q0 *NumQ.q2 - 2 * NumQ.q1 * NumQ.q3) * RtA;						
////			pAngE->roll	= atan2(2 * NumQ.q2 *NumQ.q3 + 2 * NumQ.q0 * NumQ.q1, 1 - 2 * NumQ.q1 *NumQ.q1 - 2 * NumQ.q2 * NumQ.q2) * RtA;			
////}
////void GetAngles(float *pitch, float *roll, float *yaw, float dt) 
////{
////		
////    MPU6050_ReadSensors(&mpu_data);
////		GetAngle(&mpu_data,&Angle,dt);
////		*pitch = Angle.pitch;
////		*roll = Angle.roll;
////		*yaw = Angle.yaw;
////		
////}







//////加速度计归一化
////void NormalizeAccel(float *ax, float *ay, float *az) 
////{
////    float norm = sqrt(*ax * *ax + *ay * *ay + *az * *az);
////    *ax /= norm;
////    *ay /= norm;
////    *az /= norm;
////}

//////计算姿态误差
////void ComputeError(float ax, float ay, float az, float *error_x, float *error_y, float *error_z) 
////{
////    float gravity_x = 2 * (q.q1 * q.q3 - q.q0 * q.q2);
////    float gravity_y = 2 * (q.q0 * q.q1 + q.q2 * q.q3);
////    float gravity_z = 1 - 2 * (q.q1 * q.q1 + q.q2 * q.q2);
//// 
////    *error_x = (ay * gravity_z - az * gravity_y);
////    *error_y = (az * gravity_x - ax * gravity_z);
////    *error_z = (ax * gravity_y - ay * gravity_x);
////}

//////四元数更新
////void UpdateQuaternion(float gx, float gy, float gz, 
////                      float error_x, float error_y, float error_z, 
////                      float dt) 
////{
////    // 互补滤波系数
////    float Kp = 2.0f * 5.0f;   // 比例增益
////    float Ki = 0.0f;           // 积分增益（可设为0）
////    static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
////    
////    // 应用误差补偿
////    gx += Kp * error_x + Ki * integralFBx;
////    gy += Kp * error_y + Ki * integralFBy;
////    gz += Kp * error_z + Ki * integralFBz;
////    
////    integralFBx += Ki * error_x * dt;
////    integralFBy += Ki * error_y * dt;
////    integralFBz += Ki * error_z * dt;
////    
////    // 正确的四元数导数
////    float q0 = q.q0, q1 = q.q1, q2 = q.q2, q3 = q.q3;
////    
////    // 四元数导数 (正确的旋转运动学方程)
////    float q0_dot = (-q1 * gx - q2 * gy - q3 * gz) * 0.5f;
////    float q1_dot = ( q0 * gx - q3 * gy + q2 * gz) * 0.5f;
////    float q2_dot = ( q3 * gx + q0 * gy - q1 * gz) * 0.5f;
////    float q3_dot = (-q2 * gx + q1 * gy + q0 * gz) * 0.5f;
////    
////    // 积分更新
////    q.q0 += q0_dot * dt;
////    q.q1 += q1_dot * dt;
////    q.q2 += q2_dot * dt;
////    q.q3 += q3_dot * dt;
////    
////    // 归一化四元数
////    float norm = sqrtf(q.q0*q.q0 + q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3);
////    if (norm == 0.0f) return;
////    
////    norm = 1.0f / norm;
////    q.q0 *= norm;
////    q.q1 *= norm;
////    q.q2 *= norm;
////    q.q3 *= norm;
////}
//////void UpdateQuaternion(float gx, float gy, float gz, 
//////                      float error_x, float error_y, float error_z, 
//////                      float dt) 
//////{
//////    float Kp = 0.5f;  // 互补滤波系数
////// 
//////    float q0_dot = -q.q1 * gx - q.q2 * gy - q.q3 * gz;
//////    float q1_dot = q.q0 * gx - q.q3 * gy + q.q2 * gz;
//////    float q2_dot = q.q3 * gx + q.q0 * gy - q.q1 * gz;
//////    float q3_dot = -q.q2 * gx + q.q1 * gy + q.q0 * gz;
////// 
//////    q.q0 += (q0_dot + Kp * error_x) * dt;
//////    q.q1 += (q1_dot + Kp * error_y) * dt;
//////    q.q2 += (q2_dot + Kp * error_z) * dt;
//////    q.q3 += (q3_dot) * dt;
////// 
//////    // 归一化四元数，避免数值误差会越积越大
//////    float norm = sqrt(q.q0 * q.q0 + q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3);
//////    q.q0 /= norm;
//////    q.q1 /= norm;
//////    q.q2 /= norm;
//////    q.q3 /= norm;
//////}
//////计算欧拉角
////void ComputeEulerAngles(float *pitch, float *roll, float *yaw) 
////{
////    float q0 = q.q0, q1 = q.q1, q2 = q.q2, q3 = q.q3;
////    
////    // 正确的欧拉角计算 (航空航天序列: Z-Y-X)
////    *roll = atan2f(2.0f * (q0*q1 + q2*q3), 
////                   q0*q0 - q1*q1 - q2*q2 + q3*q3);
////    
////    *pitch = asinf(2.0f * (q0*q2 - q1*q3));
////    
////    *yaw = atan2f(2.0f * (q0*q3 + q1*q2), 
////                  q0*q0 + q1*q1 - q2*q2 - q3*q3);
////    
////    // 转换为角度
////    *roll *= RtA;
////    *pitch *= RtA;
////    *yaw *= RtA;
////}
////void ComputeEulerAngles(float *pitch, float *roll, float *yaw) 
////{
////    *roll = atan2(2 * (q.q2 * q.q3 + q.q0 * q.q1), q.q0 * q.q0 - q.q1 * q.q1 - q.q2 * q.q2 + q.q3 * q.q3);
////    *pitch = asin(-2 * (q.q1 * q.q3 - q.q0 * q.q2));
////    *yaw = atan2(2 * (q.q1 * q.q2 + q.q0 * q.q3), q.q0 * q.q0 + q.q1 * q.q1 - q.q2 * q.q2 - q.q3 * q.q3);
//// 
////    *roll *= RtA;  // 弧度转角度
////    *pitch *= RtA;
////    *yaw *= RtA;
////}

////void GetAngles(float *pitch, float *roll, float *yaw, float dt) 
////{
//////    float ax, ay, az, gx, gy, gz;
////		
////    MPU6050_ReadSensors(&mpu_data);
////		GetAngle(&mpu_data,&Angle,dt);
////		*pitch = Angle.pitch;
////		*roll = Angle.roll;
////		*yaw = Angle.yaw;
////    
//////    // 坐标系转换：MPU6050 (X-右, Y-前, Z-上) -> 航空航天标准 (X-前, Y-右, Z-下)
//////    float ax_corrected = ay;
//////    float ay_corrected = ax;
//////    float az_corrected = -az;
//////    
//////    float gx_corrected = gy;
//////    float gy_corrected = gx;
//////    float gz_corrected = -gz;

//////    NormalizeAccel(&ax, &ay, &az);
//////    
//////    float error_x, error_y, error_z;
//////    ComputeError(ax, ay, az, &error_x, &error_y, &error_z);
//////    
//////    UpdateQuaternion(gx, gy, gz, error_x, error_y, error_z, dt);
//////    
//////    ComputeEulerAngles(pitch, roll, yaw);
////		
////}








