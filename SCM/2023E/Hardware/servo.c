#include "servo.h" 


// ����Ļ����ת��Ϊ����Ƕ�
void Point_To_Angles(Point p, float* angle_x, float* angle_y) {
  // ����ת����ʽ (��Ļ����Ϊԭ��)
  *angle_x = 90.0f + atan2f(p.x, SCREEN_DISTANCE) * 180.0f / PI;
  *angle_y = 90.0f + atan2f(p.y, SCREEN_DISTANCE) * 180.0f / PI;
  
  // �Ƕ��޷�
  *angle_x = fmaxf(SERVO_MIN_ANGLE, fminf(*angle_x, SERVO_MAX_ANGLE));
  *angle_y = fmaxf(SERVO_MIN_ANGLE, fminf(*angle_y, SERVO_MAX_ANGLE));
}

// ���ö��PWM
void Set_Servo_PWM(TIM_HandleTypeDef* htim, uint32_t channel, float angle) {
  uint32_t pulse = 500 + (angle / 180.0f) * 2000;  // 500-2500��s
  __HAL_TIM_SET_COMPARE(htim, channel, pulse);
}

// �ƶ���Ŀ���
void Move_To_Point(Point target) {
  float angle_x, angle_y;
  Point_To_Angles(target, &angle_x, &angle_y);
  
  Set_Servo_PWM(&htim3, TIM_CHANNEL_1, angle_x);  // ˮƽ���
  Set_Servo_PWM(&htim3, TIM_CHANNEL_2, angle_y);  // ��ֱ���
  
  vTaskDelay(20);  // �ȶ�ʱ��
}

// ��λ�����ĵ�
void Reset_Position(void) {
  Point center = {0, 0};
  Move_To_Point(center);
  vTaskDelay(1000);  // �ȶ�ʱ��
}

// ������·���˶�
void Move_Square_Path(void) {
  Point path[4 * POINTS_PER_SIDE];
  const float half_size = SQUARE_SIZE / 2;
  
  // ����·���� (˳ʱ��)
  for (int i = 0; i < POINTS_PER_SIDE; i++) {
    // �ϱ� (�����)
    path[i].x = -half_size + (SQUARE_SIZE * i) / POINTS_PER_SIDE;
    path[i].y = half_size;
    
    // �ұ� (�ϡ���)
    path[i + POINTS_PER_SIDE].x = half_size;
    path[i + POINTS_PER_SIDE].y = half_size - (SQUARE_SIZE * i) / POINTS_PER_SIDE;
    
    // �±� (�ҡ���)
    path[i + 2*POINTS_PER_SIDE].x = half_size - (SQUARE_SIZE * i) / POINTS_PER_SIDE;
    path[i + 2*POINTS_PER_SIDE].y = -half_size;
    
    // ��� (�¡���)
    path[i + 3*POINTS_PER_SIDE].x = -half_size;
    path[i + 3*POINTS_PER_SIDE].y = -half_size + (SQUARE_SIZE * i) / POINTS_PER_SIDE;
  }
  
  // ִ��·��
  for (int i = 0; i < 4 * POINTS_PER_SIDE; i++) {
    Move_To_Point(path[i]);
    vTaskDelay(MOVE_DELAY_MS);
  }
}

// �Զ���·���˶�
void Move_Custom_Path(const Point* path, uint16_t points) {
  for (int i = 0; i < points; i++) {
    Move_To_Point(path[i]);
    vTaskDelay(MOVE_DELAY_MS);
  }
}


