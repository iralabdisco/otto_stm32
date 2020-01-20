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

