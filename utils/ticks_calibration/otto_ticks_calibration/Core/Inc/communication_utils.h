// __attribute__((packed)) -> no padding bytes
typedef struct __attribute__((packed)) {
  float angular_velocity;
  float linear_velocity;
  float delta_time;
} odometry_msg;

typedef struct __attribute__((packed)) {
  float linear_velocity;
  float angular_velocity;
} velocity_msg;

typedef struct __attribute__((packed)) {
  float pid_select;
  float pid_setpoint_fixed;
  float pid_setpoint_lin;
  float pid_setpoint_ang;
  float pid_kp;
  float pid_ki;
  float pid_kd;
} pid_setup_msg;

typedef struct __attribute__((packed)) {
  float velocity;
//  float millis;
} plot_msg;

