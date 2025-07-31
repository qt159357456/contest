
//#include "imu.h"

//#define Kp_New      0.9f           				    //�����˲���ǰ���ݵ�Ȩ��
//#define Kp_Old      0.1f           				    //�����˲���ʷ���ݵ�Ȩ��  
//#define Acc_Gain  	0.0001220f		    			  //���ٶȱ��G (��ʼ�����ٶ�������-+4g LSBa = 2*4/65535.0)
//#define Gyro_Gain 	0.0609756f							  //���ٶȱ�ɶ� (��ʼ��������������+-2000 LSBg = 2*2000/65535.0)
//#define Gyro_Gr	    0.0010641f							  //���ٶȱ�ɻ���(3.1415/180 * LSBg)       

//Angle_t Att_Angle;                        //�ɻ���̬����
//FLOAT_XYZ_t 	Acc_filt,Gyr_filt;	  //�˲���ĸ�ͨ������
//float   accb[3];                              //���ٶȼ����ݴ洢
//float   gyrb[3];                               //���ٶȼ����ݴ洢
//float   DCMgb[3][3];                          //���������󣨽� ��������ϵ ת��Ϊ ��������ϵ��
//uint8_t AccbUpdate = 0;                       //���ٶȼ����ݸ��±�־λ
//uint8_t GyrbUpdate = 0;                       //���ٶȼ����ݸ��±�־λ

///*********************************************************************************************************
//* ��  ����void Prepare_Data(void)
//* �����ܣ���������ȥ��ƫ��������˲��������������壬Ϊ��̬������׼��
//* ��  ������
//* ����ֵ����
//**********************************************************************************************************/	
//void Prepare_Data(void)
//{
//	MPU6050_Read_Accel(&MPU6050_ACC_RAW);
//	MPU6050_Read_Gyro(&MPU6050_GYRO_RAW);
//	
//	MPU6050_Offset();  //��MPU6050���д�����ȥ��ƫ�����û�м�����ƫ(���ݱ�־λ�ж�)�ͼ�����ƫ
//	

//	MPU6050_AccRead(&Acc_filt);//��ȡ���ٶȼ�ԭʼ���ݣ������п������˲�
//	MPU6050_GyroRead(&Gyr_filt);//��ȡ���ٶȼ�ԭʼ���ݣ�������һ�׵�ͨ�˲�
//	
////	Aver_FilterXYZ(&Acc_filt,12);//�˴��Լ��ٶȼƽ��л��������˲����� 
////  Aver_FilterXYZ(&Gyr_filt,12);//�˴��Խ��ٶȼƽ��л��������˲�����
//	
//	//���ٶ�ADֵ ת���� ��/ƽ���� 
//	Acc_filt.X = (float)Acc_filt.X * Acc_Gain * G;
//	Acc_filt.Y = (float)Acc_filt.Y * Acc_Gain * G;
//	Acc_filt.Z = (float)Acc_filt.Z * Acc_Gain * G;
////	printf("ax=%0.2f ay=%0.2f az=%0.2f\r\n",Acc_filt.X,Acc_filt.Y,Acc_filt.Z);

//	//������ADֵ ת���� ����/��    
//	Gyr_filt.X = (float) Gyr_filt.X * Gyro_Gr;  
//	Gyr_filt.Y = (float) Gyr_filt.Y * Gyro_Gr;
//	Gyr_filt.Z = (float) Gyr_filt.Z * Gyro_Gr;
////	printf("gx=%0.2f gy=%0.2f gz=%0.2f\r\n",Gyr_filt.X,Gyr_filt.Y,Gyr_filt.Z);
//}
///*********************************************************************************************************
//* ��  ����void IMUupdate(FLOAT_XYZ_t *Gyr_filt,FLOAT_XYZ_t *Acc_filt,Angle_t *Att_Angle)
//* �����ܣ���ȡ��̬��
//* ��  ����Gyr_filt 	ָ����ٶȵ�ָ�루ע�ⵥλ�����ǻ��ȣ�
//*         Acc_filt 	ָ����ٶȵ�ָ��
//*         Att_Angle ָ����̬�ǵ�ָ��
//* ����ֵ����
//* ��  ע�������Ԫ����ŷ���Ƕ��ڴ˺��������
//**********************************************************************************************************/	
////kp=ki=0 ������ȫ����������
//#define Kp 1.5f                          // proportional gain governs rate of convergence to accelerometer/magnetometer
//                                          //����������Ƽ��ٶȼƣ������Ƶ���������
//#define Ki 0.005f                         // integral gain governs rate of convergence of gyroscope biases  
//                                          //���������������ƫ��������ٶ�

//float q0 = 1, q1 = 0, q2 = 0, q3 = 0;     // quaternion elements representing the estimated orientation
//float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error

//void IMUupdate(FLOAT_XYZ_t *Gyr_filt,FLOAT_XYZ_t *Acc_filt,Angle_t *Att_Angle,float dt)
//{
//	float ax = Acc_filt->X,ay = Acc_filt->Y,az = Acc_filt->Z;
//	float gx = Gyr_filt->X,gy = Gyr_filt->Y,gz = Gyr_filt->Z;
//	float vx, vy, vz;
//	float ex, ey, ez;
//	float norm;

//	float q0q0 = q0*q0;
//	float q0q1 = q0*q1;
//	float q0q2 = q0*q2;
//	float q0q3 = q0*q3;
//	float q1q1 = q1*q1;
//	float q1q2 = q1*q2;
//	float q1q3 = q1*q3;
//	float q2q2 = q2*q2;
//	float q2q3 = q2*q3;
//	float q3q3 = q3*q3;
//	float halfT = 0.5*dt;

//	if(ax*ay*az==0)
//	return;

//	//���ٶȼ�<����>���������ٶ�����(��������ϵ) 
//	norm = invSqrt(ax*ax + ay*ay + az*az); 
//	ax = ax * norm;
//	ay = ay * norm;
//	az = az * norm;
////	printf("ax=%0.2f ay=%0.2f az=%0.2f\r\n",ax,ay,az);

//	//�����ǻ���<����>��������(��������ϵ) 
//	vx = 2*(q1q3 - q0q2);	               //����(3,1)��											
//	vy = 2*(q0q1 + q2q3);                //����(3,2)��
//	vz = q0q0 - q1q1 - q2q2 + q3q3 ;     //����(3,3)��
//	
//	// printf("vx=%0.2f vy=%0.2f vz=%0.2f\r\n",vx,vy,vz); 

//	//����������õ�ֵ 
//	ex = (ay*vz - az*vy);                     
//	ey = (az*vx - ax*vz); 
//	ez = (ax*vy - ay*vx);

//	//��������������л���
//	exInt = exInt + ex * Ki;								 
//	eyInt = eyInt + ey * Ki;
//	ezInt = ezInt + ez * Ki;

//	//�����PI�󲹳���������
//	gx = gx + Kp*ex + exInt;					   		  	
//	gy = gy + Kp*ey + eyInt;
//	gz = gz + Kp*ez + ezInt;//�����gz����û�й۲��߽��н��������Ư�ƣ����ֳ����ľ��ǻ����������Լ�

//	//��Ԫ�ص�΢�ַ���
//	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
//	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
//	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
//	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

//	//��λ����Ԫ�� 
//	norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
//	q0 = q0 * norm;
//	q1 = q1 * norm;
//	q2 = q2 * norm;  
//	q3 = q3 * norm;

////  ������ʽ
////	matrix[0] = q0q0 + q1q1 - q2q2 - q3q3;	 // 11
////	matrix[1] = 2.f * (q1q2 + q0q3);	       // 12
////	matrix[2] = 2.f * (q1q3 - q0q2);	       // 13
////	matrix[3] = 2.f * (q1q2 - q0q3);	       // 21
////	matrix[4] = q0q0 - q1q1 + q2q2 - q3q3;	 // 22
////	matrix[5] = 2.f * (q2q3 + q0q1);	       // 23
////	matrix[6] = 2.f * (q1q3 + q0q2);	       // 31
////	matrix[7] = 2.f * (q2q3 - q0q1);	       // 32
////	matrix[8] = q0q0 - q1q1 - q2q2 + q3q3;	 // 33

//	//��Ԫ��ת����ŷ����(Z->Y->X) 
//	
//	//ƫ����YAW
//	if((Gyr_filt->Z *RadtoDeg > 0.08f) || (Gyr_filt->Z *RadtoDeg < -0.08f)) //����̫С������Ϊ�Ǹ��ţ�����ƫ������
//	{
//		Att_Angle->yaw += Gyr_filt->Z *RadtoDeg*0.01f;
//	}   
////	printf("yaw: %f\r\n",Att_Angle->yaw);//YAW�����ݣ��ǳ��ȶ���
//	
//	//�����ROLL
//	Att_Angle->roll = -asin(2.f * (q1q3 - q0q2))* 57.3f;
////  printf("%f\r\n",Att_Angle->rol);//ROLL������

//	//������PITCH
//	Att_Angle->pitch = -atan2(2.f * q2q3 + 2.f * q0q1, q0q0 - q1q1 - q2q2 + q3q3)* 57.3f ;
////	printf("pitch: %f\r\n",Att_Angle->pit);//PITCH������
////    printf("%f,%f,%f\r\n",Att_Angle->pit,Att_Angle->rol,Att_Angle->yaw);//PITCH������
//	
//}

