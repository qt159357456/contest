#include "servo.h" 


// 将屏幕坐标转换为舵机角度
void Point_To_Angles(Point p, float* angle_x, float* angle_y) {
  // 几何转换公式 (屏幕中心为原点)
  *angle_x = 90.0f + atan2f(p.x, SCREEN_DISTANCE) * 180.0f / PI;
  *angle_y = 90.0f + atan2f(p.y, SCREEN_DISTANCE) * 180.0f / PI;
  
  // 角度限幅
  *angle_x = fmaxf(SERVO_MIN_ANGLE, fminf(*angle_x, SERVO_MAX_ANGLE));
  *angle_y = fmaxf(SERVO_MIN_ANGLE, fminf(*angle_y, SERVO_MAX_ANGLE));
}

// 设置舵机PWM
void Set_Servo_PWM(TIM_HandleTypeDef* htim, uint32_t channel, float angle) {
  uint32_t pulse = 500 + (angle / 180.0f) * 2000;  // 500-2500μs
  __HAL_TIM_SET_COMPARE(htim, channel, pulse);
}

// 移动到目标点
void Move_To_Point(Point target) {
  float angle_x, angle_y;
  Point_To_Angles(target, &angle_x, &angle_y);
  
  Set_Servo_PWM(&htim3, TIM_CHANNEL_1, angle_x);  // 水平舵机
  Set_Servo_PWM(&htim3, TIM_CHANNEL_2, angle_y);  // 垂直舵机
  
  vTaskDelay(20);  // 稳定时间
}

// 复位到中心点
void Reset_Position(void) {
  Point center = {0, 0};
  Move_To_Point(center);
  vTaskDelay(1000);  // 稳定时间
}

// 正方形路径运动
void Move_Square_Path(void) {
  Point path[4 * POINTS_PER_SIDE];
  const float half_size = SQUARE_SIZE / 2;
  
  // 生成路径点 (顺时针)
  for (int i = 0; i < POINTS_PER_SIDE; i++) {
    // 上边 (左→右)
    path[i].x = -half_size + (SQUARE_SIZE * i) / POINTS_PER_SIDE;
    path[i].y = half_size;
    
    // 右边 (上→下)
    path[i + POINTS_PER_SIDE].x = half_size;
    path[i + POINTS_PER_SIDE].y = half_size - (SQUARE_SIZE * i) / POINTS_PER_SIDE;
    
    // 下边 (右→左)
    path[i + 2*POINTS_PER_SIDE].x = half_size - (SQUARE_SIZE * i) / POINTS_PER_SIDE;
    path[i + 2*POINTS_PER_SIDE].y = -half_size;
    
    // 左边 (下→上)
    path[i + 3*POINTS_PER_SIDE].x = -half_size;
    path[i + 3*POINTS_PER_SIDE].y = -half_size + (SQUARE_SIZE * i) / POINTS_PER_SIDE;
  }
  
  // 执行路径
  for (int i = 0; i < 4 * POINTS_PER_SIDE; i++) {
    Move_To_Point(path[i]);
    vTaskDelay(MOVE_DELAY_MS);
  }
}

// 自定义路径运动
void Move_Custom_Path(const Point* path, uint16_t points) {
  for (int i = 0; i < points; i++) {
    Move_To_Point(path[i]);
    vTaskDelay(MOVE_DELAY_MS);
  }
}


