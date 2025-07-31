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
//Quaternion q = {1.0f, 0.0f, 0.0f, 0.0f};  // ��ʼ����Ԫ�� ��ʾ��ʼ��̬Ϊˮƽ��
//uint8_t G_ACCEL_RANGE = 0x00;  // ��2g
//uint8_t G_GYRO_RANGE = 0x18;   // ��2000��/s
//float mpu_offset[6] = {-1196.00800000000f,275.660000000000f,14781.4640000000f,-130.498000000000f,-2.87800000000000f,-2.60600000000000f};

//Klm_Flt_t EKF[3] = {{0.02,0,0.001,0.543,0,0},{0.02,0,0.001,0.543,0,0},{0.02,0,0.001,0.543,0,0}};	
//INT16_XYZ_t	 GYRO_OFFSET_RAW,ACC_OFFSET_RAW;		 				//��Ư����
//INT16_XYZ_t	 MPU6050_ACC_RAW,MPU6050_GYRO_RAW;	     		//��ȡֵԭʼ����
//	
//	
//uint8_t MPU6050_OffSet_cail(INT16_XYZ_t value,INT16_XYZ_t *offset,uint16_t sensivity){
//	static int32_t tempgx=0,tempgy=0,tempgz=0;
//	static uint16_t cnt_a=0;//ʹ��static���εľֲ������������α������о�̬�洢���ڣ�Ҳ����˵�ú���ִ������ͷ��ڴ�
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
//		tempgz += value.Z-sensivity;			//���ٶȼ�У׼ sensivity ���� MPU6050��ʼ��ʱ���õ�������ֵ��8196LSB/g��;������У׼ sensivity = 0��

//		if(cnt_a==200)               //200����ֵ��ƽ��
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
//						case 0x00: sensivity = 16384; break; // ��2g��ת������Ϊ16384
//						case 0x08: sensivity = 8192; break;  // ��4g��ת������Ϊ8192
//						case 0x10: sensivity = 4096; break;  // ��8g��ת������Ϊ4096
//						case 0x18: sensivity = 2048; break;  // ��16g��ת������Ϊ2048
//						default: sensivity = 16384; break;    // Ĭ�����̷�ΧΪ��2g
//				}
//				MPU6050_OffSet_cail(MPU6050_ACC_RAW,&ACC_OFFSET_RAW,sensivity);
//				cail_flag = 0;
//		}
//		//���ٶ�ȥ��ƫADֵ 
//		MPU6050_ACC_RAW.X -= ACC_OFFSET_RAW.X;
//		MPU6050_ACC_RAW.Y -= ACC_OFFSET_RAW.Y;
//		MPU6050_ACC_RAW.Z -= (ACC_OFFSET_RAW.Z-sensivity);
//				
//		//������ȥ��ƫADֵ 
//		MPU6050_GYRO_RAW.X -= GYRO_OFFSET_RAW.X;
//		MPU6050_GYRO_RAW.Y -= GYRO_OFFSET_RAW.Y;
//		MPU6050_GYRO_RAW.Z -= GYRO_OFFSET_RAW.Z;
//}


///******************************************************************************
//* ��  ����void MPU6050_AccRead(FLOAT_XYZ *Acc_filt)
//* ��  �ܣ���ȡ���ٶȵ�ԭʼ���ݣ�����һ�׿������˲�
//* ��  ����*accData ԭʼ���ݵ�ָ��
//* ����ֵ����
//* ��  ע����
//*******************************************************************************/
//void MPU6050_AccRead(FLOAT_XYZ_t *Acc_filt)
//{
//	  int16_t accData[3];
//	  uint8_t i;
//    accData[0] = (int16_t)MPU6050_ACC_RAW.X;
//    accData[1] = (int16_t)MPU6050_ACC_RAW.Y;
//    accData[2] = (int16_t)MPU6050_ACC_RAW.Z;
//	
//	  for(i=0;i<3;i++)//�Լ��ٶ�ԭʼ���ݽ���һ�׿������˲�
//		{ 				
//			Kalman_Filter(&EKF[i], (float)accData[i]);  
//			accData[i] = EKF[i].Out;
//		}
//	  Acc_filt->X  = (float)accData[0];
//	  Acc_filt->Y  = (float)accData[1];
//	  Acc_filt->Z  = (float)accData[2];
//}

///******************************************************************************
//* ��  ����void MPU6050_GyroRead(FLOAT_XYZ *Gyr_filt)
//* ��  �ܣ���ȡ�����ǵ�ԭʼ���ݣ�����һ�׵�ͨ�˲�
//* ��  ����*gyroData ԭʼ���ݵ�ָ��
//* ����ֵ����
//* ��  ע����
//*******************************************************************************/
//void MPU6050_GyroRead(FLOAT_XYZ_t *Gyr_filt)
//{
//	  int16_t gyroData[3];
//	  uint8_t i;
//    gyroData[0] = (int16_t)MPU6050_GYRO_RAW.X ;
//    gyroData[1] = (int16_t)MPU6050_GYRO_RAW.Y ;
//    gyroData[2] = (int16_t)MPU6050_GYRO_RAW.Z ;
//	
//	  for(i=0;i<3;i++)//�Խ��ٶ�ԭʼ���ݽ���һ�׵�ͨ�˲�
//		{  
//       const float factor = 0.15f;  //�˲�����			
//			 static float tBuff[3];
//			 gyroData[i] = tBuff[i] = tBuff[i] * (1 - factor) + gyroData[i] * factor;
//		}
//		Gyr_filt->X  = (float)gyroData[0];
//	  Gyr_filt->Y  = (float)gyroData[1];
//	  Gyr_filt->Z  = (float)gyroData[2];
//}

//// ���ԭʼ��ȡ����
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
//    // 1. ����豸ID (WHO_AM_I)
//    HAL_I2C_Mem_Read(&hi2c2, 0xD0, MPU6050_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &readValue, 1, 100);
//    if(readValue != 0x68) {
//        debug_printf("MPU6050 not found! ID: 0x%02X\r\n", readValue);
//				while(1);
//    }
//    
//    // 2. д������
//    HAL_I2C_Mem_Write(&hi2c2, 0xD0, MPU6050_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &MPU6050_init[0], 2, 100);
//    // 5. ���ò����ʷ�Ƶ������֤
//    HAL_I2C_Mem_Write(&hi2c2, 0xD0, MPU6050_SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT, &MPU6050_init[2], 4, 100);
//    HAL_Delay(10); // �ȴ��Ĵ���д�����
//    
//		// 3. ���ü��ٶȼ����̲���֤
//    uint8_t accel_range = G_ACCEL_RANGE;
//    HAL_I2C_Mem_Write(&hi2c2, 0xD0, MPU6050_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &accel_range, 1, 100);
//    HAL_Delay(10); // �ȴ��Ĵ���д�����
//    
//    HAL_I2C_Mem_Read(&hi2c2, 0xD0, MPU6050_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &readValue, 1, 100);
//    if((readValue & 0x18) != G_ACCEL_RANGE) {
//        debug_printf("Accel range config failed! Set: 0x%02X, Read: 0x%02X\r\n", 
//               G_ACCEL_RANGE, readValue & 0x18);
//				while(1);
//    }
//    
//    // 4. �������������̲���֤
//    uint8_t gyro_range = G_GYRO_RANGE;
//    HAL_I2C_Mem_Write(&hi2c2, 0xD0, MPU6050_GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &gyro_range, 1, 100);
//    HAL_Delay(10); // �ȴ��Ĵ���д�����
//    
//    HAL_I2C_Mem_Read(&hi2c2, 0xD0, MPU6050_GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &readValue, 1, 100);
//    if((readValue & 0x18) != G_GYRO_RANGE) {
//        debug_printf("Gyro range config failed! Set: 0x%02X, Read: 0x%02X\r\n", 
//               G_GYRO_RANGE, readValue & 0x18);
//				while(1);
//    }
//    
//    // 6. ����״̬���
//    //debug_printf("MPU6050 initialized successfully!\r\n");
//}


//float MPU6050_ConvertAccel(float value, uint8_t range) 
//{
//    float factor; // ���ڴ洢ת������
//    // ���ݼ��ٶȼƵ�����ѡ����ʵ�ת������
//    switch (range) {
//        case 0x00: factor = 16384.0; break; // ��2g��ת������Ϊ16384
//        case 0x08: factor = 8192.0; break;  // ��4g��ת������Ϊ8192
//        case 0x10: factor = 4096.0; break;  // ��8g��ת������Ϊ4096
//        case 0x18: factor = 2048.0; break;  // ��16g��ת������Ϊ2048
//        default: factor = 16384.0; break;    // Ĭ�����̷�ΧΪ��2g
//    }
//    // ��ԭʼ���ٶ�ֵת��Ϊg��λ
//    return value / factor;
//}
// 
//float MPU6050_ConvertGyro(float value, uint8_t range) 
//{
//    float factor; // ���ڴ洢ת������
//    // ���������ǵ�����ѡ����ʵ�ת������
//    switch (range) {
//        case 0x00: factor = 131.0; break;  // ��250��/s��ת������Ϊ131
//        case 0x08: factor = 65.5; break;   // ��500��/s��ת������Ϊ65.5
//        case 0x10: factor = 32.8; break;   // ��1000��/s��ת������Ϊ32.8
//        case 0x18: factor = 16.4; break;   // ��2000��/s��ת������Ϊ16.4
//        default: factor = 131.0; break;     // Ĭ�����̷�ΧΪ��250��/s
//    }
//    // ��ԭʼ������ֵת��Ϊ��/s��λ
//    return value / factor;
//}

//////��ȡ����������
////void MPU6050_ReadSensors(MPU6050_Data_t *Mpu)
////{
////		int16_t raw[6];
////		MPU6050_Read_Accel(raw);
////		MPU6050_Read_Gyro(raw+3);

////		float data[6];
////		for(int i=0; i<6; i++) {
////				if(i < 3) { // ���ٶȼƿ������˲�
////					Kalman_Filter(&EKF[i], (float)raw[i] - mpu_offset[i]);  
////					data[i] = EKF[i].Out;
//////						data[i] = (float)raw[i] - mpu_offset[i];
////				}
////				else { // �����ǻ����˲�
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
////    Mpu->gx = MPU6050_ConvertGyro(data[3], G_GYRO_RANGE) * AtR; // ת��Ϊrad/s
////    Mpu->gy = MPU6050_ConvertGyro(data[4], G_GYRO_RANGE) * AtR;
////    Mpu->gz = MPU6050_ConvertGyro(data[5], G_GYRO_RANGE) * AtR;
////}




/////* ��Ԫ���ⷨ��ʼ�� */
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
////	//��ȡ��Ч��ת�����е��������� 
////	Gravity.x = 2*(NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);								
////	Gravity.y = 2*(NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);						  
////	Gravity.z = 1-2*(NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);	
////	// ���ٶȹ�һ��
////	//printf("accX:%d\r\n",MPU6050.accX);
////	float NormAcc = 1/sqrt(squa(pMpu->ax)+ squa(pMpu->ay) +squa(pMpu->az));
////	//printf("NorAcc%f\r\n",NormAcc);
////	//	NormAcc = Q_rsqrt(squa(MPU6050.accX)+ squa(MPU6050.accY) +squa(MPU6050.accZ));
////	
////  Acc.x = pMpu->ax * NormAcc;
////  Acc.y = pMpu->ay * NormAcc;
////  Acc.z = pMpu->az * NormAcc;
////	
//// 	//������˵ó���ֵ
////	AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
////	AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
////	AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);
////	
////	//�������ٶȻ��ֲ������ٶȵĲ���ֵ
////  GyroIntegError.x += AccGravity.x * KiDef;
////  GyroIntegError.y += AccGravity.y * KiDef;
////  GyroIntegError.z += AccGravity.z * KiDef;
////	
////	//���ٶ��ںϼ��ٶȻ��ֲ���ֵ
////  Gyro.x = pMpu->gx + KpDef * AccGravity.x  +  GyroIntegError.x;//������
////  Gyro.y = pMpu->gy + KpDef * AccGravity.y  +  GyroIntegError.y;
////  Gyro.z = pMpu->gz + KpDef * AccGravity.z  +  GyroIntegError.z;
////	
////	// һ�����������, ������Ԫ��
////	q0_t = (-NumQ.q1*Gyro.x - NumQ.q2*Gyro.y - NumQ.q3*Gyro.z) * HalfTime;
////	q1_t = ( NumQ.q0*Gyro.x - NumQ.q3*Gyro.y + NumQ.q2*Gyro.z) * HalfTime;
////	q2_t = ( NumQ.q3*Gyro.x + NumQ.q0*Gyro.y - NumQ.q1*Gyro.z) * HalfTime;
////	q3_t = (-NumQ.q2*Gyro.x + NumQ.q1*Gyro.y + NumQ.q0*Gyro.z) * HalfTime;
////	
////	NumQ.q0 += q0_t;
////	NumQ.q1 += q1_t;
////	NumQ.q2 += q2_t;
////	NumQ.q3 += q3_t;
////	// ��Ԫ����һ��
////	NormQuat = 1/sqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
////	NumQ.q0 *= NormQuat;
////	NumQ.q1 *= NormQuat;
////	NumQ.q2 *= NormQuat;
////	NumQ.q3 *= NormQuat;	
////	
////	// ��Ԫ��תŷ����
//////			pAngE->yaw = atan2f(2 * NumQ.q1 *NumQ.q2 + 2 * NumQ.q0 * NumQ.q3, 1 - 2 * NumQ.q2 *NumQ.q2 - 2 * NumQ.q3 * NumQ.q3) * RtA;  //yaw
////			
////			float yaw_G = pMpu->gz*RtA;
////			if((yaw_G > 0.03f) || (yaw_G < -0.03f)) //����̫С������Ϊ�Ǹ��ţ�����ƫ������
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







//////���ٶȼƹ�һ��
////void NormalizeAccel(float *ax, float *ay, float *az) 
////{
////    float norm = sqrt(*ax * *ax + *ay * *ay + *az * *az);
////    *ax /= norm;
////    *ay /= norm;
////    *az /= norm;
////}

//////������̬���
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

//////��Ԫ������
////void UpdateQuaternion(float gx, float gy, float gz, 
////                      float error_x, float error_y, float error_z, 
////                      float dt) 
////{
////    // �����˲�ϵ��
////    float Kp = 2.0f * 5.0f;   // ��������
////    float Ki = 0.0f;           // �������棨����Ϊ0��
////    static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
////    
////    // Ӧ������
////    gx += Kp * error_x + Ki * integralFBx;
////    gy += Kp * error_y + Ki * integralFBy;
////    gz += Kp * error_z + Ki * integralFBz;
////    
////    integralFBx += Ki * error_x * dt;
////    integralFBy += Ki * error_y * dt;
////    integralFBz += Ki * error_z * dt;
////    
////    // ��ȷ����Ԫ������
////    float q0 = q.q0, q1 = q.q1, q2 = q.q2, q3 = q.q3;
////    
////    // ��Ԫ������ (��ȷ����ת�˶�ѧ����)
////    float q0_dot = (-q1 * gx - q2 * gy - q3 * gz) * 0.5f;
////    float q1_dot = ( q0 * gx - q3 * gy + q2 * gz) * 0.5f;
////    float q2_dot = ( q3 * gx + q0 * gy - q1 * gz) * 0.5f;
////    float q3_dot = (-q2 * gx + q1 * gy + q0 * gz) * 0.5f;
////    
////    // ���ָ���
////    q.q0 += q0_dot * dt;
////    q.q1 += q1_dot * dt;
////    q.q2 += q2_dot * dt;
////    q.q3 += q3_dot * dt;
////    
////    // ��һ����Ԫ��
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
//////    float Kp = 0.5f;  // �����˲�ϵ��
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
//////    // ��һ����Ԫ����������ֵ����Խ��Խ��
//////    float norm = sqrt(q.q0 * q.q0 + q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3);
//////    q.q0 /= norm;
//////    q.q1 /= norm;
//////    q.q2 /= norm;
//////    q.q3 /= norm;
//////}
//////����ŷ����
////void ComputeEulerAngles(float *pitch, float *roll, float *yaw) 
////{
////    float q0 = q.q0, q1 = q.q1, q2 = q.q2, q3 = q.q3;
////    
////    // ��ȷ��ŷ���Ǽ��� (���պ�������: Z-Y-X)
////    *roll = atan2f(2.0f * (q0*q1 + q2*q3), 
////                   q0*q0 - q1*q1 - q2*q2 + q3*q3);
////    
////    *pitch = asinf(2.0f * (q0*q2 - q1*q3));
////    
////    *yaw = atan2f(2.0f * (q0*q3 + q1*q2), 
////                  q0*q0 + q1*q1 - q2*q2 - q3*q3);
////    
////    // ת��Ϊ�Ƕ�
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
////    *roll *= RtA;  // ����ת�Ƕ�
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
//////    // ����ϵת����MPU6050 (X-��, Y-ǰ, Z-��) -> ���պ����׼ (X-ǰ, Y-��, Z-��)
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








