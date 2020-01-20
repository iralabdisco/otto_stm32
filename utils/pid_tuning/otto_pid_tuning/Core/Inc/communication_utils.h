// __attribute__((packed)) -> no padding bytes
typedef struct __attribute__((packed)) {
  float angular_velocity;
  float linear_velocity;
  float delta_time;
} odometry_msg;

typedef struct __attribute__((packed)) {
  float angular_velocity;
  float linear_velocity;
} velocity_msg;

typedef struct __attribute__((packed)) {
  float pid_select;
  float pid_setpoint;
  float pid_kp;
  float pid_ki;
  float pid_kd;
} pid_setup_msg;

typedef struct __attribute__((packed)) {
  float velocity;
} plot_msg;

