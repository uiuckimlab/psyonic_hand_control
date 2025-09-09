#include <Arduino.h>
#include <Wire.h>

#include "helper_functions.h"
#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

#define NODE_NAME "psyonic_controller_node"
#define RIGHT_TOPIC_NAME "right/psyonic_hand_vals"
#define LEFT_TOPIC_NAME "left/psyonic_hand_vals"
#define RIGHT_COMMAND_TOPIC_SUB "psyonic_controller/right/command"
#define LEFT_COMMAND_TOPIC_SUB "psyonic_controller/left/command"
#define ENABLE_UPSAMPLE_SUB "psyonic_controller/enable_upsample"
#define POSITION_SIZE 6
#define CURRENT_SIZE 6
#define VELOCITY_SIZE 6
#define FINGERTIP_SENSOR_SIZE 36

rcl_publisher_t right_publisher;
rcl_publisher_t left_publisher;
rcl_subscription_t right_command_subscriber;
rcl_subscription_t left_command_subscriber;
rcl_subscription_t enable_thumb_subscriber;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;


// Error Handling
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

// Setup ROS Related Msgs
psyonic_hand_control__msg__HandVal right_hand_msg;
psyonic_hand_control__msg__HandVal left_hand_msg;
std_msgs__msg__Float64MultiArray right_command_msg;
std_msgs__msg__Float64MultiArray left_command_msg;
std_msgs__msg__Bool enable_thumb_up_msg;

// Command msg array buffer
double right_array_buffer[NUM_CHANNELS];
double left_array_buffer[NUM_CHANNELS];

// Setup finger position variable
float right_fpos[NUM_CHANNELS] = {30.f,30.f,30.f,30.f,30.f, -30.f};
float left_fpos[NUM_CHANNELS] = {30.f,30.f,30.f,30.f,30.f, -30.f};
uint8_t rx_buffer1[API_RX_SIZE];
uint8_t rx_buffer2[API_RX_SIZE];

/*Moves Fingers to the position user has set*/
void moveRightHandCallback(const void * msgin){
  const std_msgs__msg__Float64MultiArray * msg = (const std_msgs__msg__Float64MultiArray *)msgin;
  for (uint32_t i = 0; i < msg->data.size  && i < NUM_CHANNELS; i++) {
    right_fpos[i] = msg->data.data[i] * 180.0 / M_PI;
  }
}
void moveLeftHandCallback(const void * msgin){
  const std_msgs__msg__Float64MultiArray * msg = (const std_msgs__msg__Float64MultiArray *)msgin;
  for (uint32_t i = 0; i < msg->data.size  && i < NUM_CHANNELS; i++) {
    left_fpos[i] = msg->data.data[i] * 180.0 / M_PI;
  }
}

void enableThumbCallback(const void * msgin){
  const std_msgs__msg__Bool * msg1 = (const std_msgs__msg__Bool *)msgin;
  int time = 1000;
  delayMicroseconds(time*5);
  uint8_t tx_buf_enable1[API_TX_SIZE] = {0};
  enable_thumb_upsample(tx_buf_enable1, msg1->data);
  Serial1.write(tx_buf_enable1, API_TX_SIZE);
  delayMicroseconds(time);

  delayMicroseconds(time*5);
  uint8_t tx_buf_enable2[API_TX_SIZE] = {0};
  enable_thumb_upsample(tx_buf_enable2, msg1->data);
  Serial2.write(tx_buf_enable2, API_TX_SIZE);
  delayMicroseconds(time);
}

void setup()
{

    // Configure serial transport
  Serial.begin(4000000);
  set_microros_serial_transports(Serial);
  delay(2000);
  
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));
  
  std_msgs__msg__Float64MultiArray__init(&right_command_msg);
  right_command_msg.data.capacity = NUM_CHANNELS;
  right_command_msg.data.size = 0;
  right_command_msg.data.data = right_array_buffer;
  
  std_msgs__msg__Float64MultiArray__init(&left_command_msg);
  left_command_msg.data.capacity = NUM_CHANNELS;
  left_command_msg.data.size = 0;
  left_command_msg.data.data = left_array_buffer;

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &right_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(psyonic_hand_control, msg, HandVal),
    RIGHT_TOPIC_NAME));
  RCCHECK(rclc_publisher_init_default(
    &left_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(psyonic_hand_control, msg, HandVal),
    LEFT_TOPIC_NAME));
  
  // Create Subscribers
  RCCHECK(rclc_subscription_init_default(
    &right_command_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    RIGHT_COMMAND_TOPIC_SUB));
  RCCHECK(rclc_subscription_init_default(
    &left_command_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    LEFT_COMMAND_TOPIC_SUB));

  RCCHECK(rclc_subscription_init_default(
    &enable_thumb_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    ENABLE_UPSAMPLE_SUB));
  

       // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &right_command_subscriber,
    &right_command_msg,
    &moveRightHandCallback,
    ON_NEW_DATA));

  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &left_command_subscriber,
    &left_command_msg,
    &moveLeftHandCallback,
    ON_NEW_DATA));

  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &enable_thumb_subscriber,
    &enable_thumb_up_msg,
    &enableThumbCallback,
    ON_NEW_DATA));

  for(size_t i = 0; i < POSITION_SIZE; i++){
    right_hand_msg.positions[i] = right_fpos[i];
    left_hand_msg.positions[i] = left_fpos[i];
  }
  for(size_t i = 0; i < CURRENT_SIZE; i++){
    right_hand_msg.currents[i] = 0.0;
    left_hand_msg.currents[i] = 0.0;
  }
  for(size_t i = 0; i < VELOCITY_SIZE; i++){
    right_hand_msg.velocities[i] = 0.0;
    left_hand_msg.velocities[i] = 0.0;
  }
  for(size_t i = 0; i < FINGERTIP_SENSOR_SIZE; i++){
    right_hand_msg.fingertips[i] = 0.0;
    left_hand_msg.fingertips[i] = 0.0;
  }

  Serial1.begin(460800);
  Serial1.addMemoryForRead(rx_buffer1,API_RX_SIZE-63); // 63 is default buffer size, so trying to make it 72 bytes
  Serial1.clear();
  Serial2.begin(460800);
  Serial2.addMemoryForRead(rx_buffer2,API_RX_SIZE-63); // 63 is default buffer size, so trying to make it 72 bytes
  Serial2.clear();
  int time = 1000;
  delayMicroseconds(time*5);
  uint8_t tx_buf_enable1[API_TX_SIZE] = {0};
  enable_thumb_upsample(tx_buf_enable1, true);
  Serial1.write(tx_buf_enable1, API_TX_SIZE);
  delayMicroseconds(time);
  uint8_t tx_buf_enable2[API_TX_SIZE] = {0};
  enable_thumb_upsample(tx_buf_enable2, true);
  Serial2.write(tx_buf_enable2, API_TX_SIZE);
  delayMicroseconds(time);
  

}

unsigned long initial_time = millis();

void loop()
{ 
  float start_time = millis();
  uint8_t tx_buf1[API_TX_SIZE] = {0};
  format_packet(right_fpos, tx_buf1);
  Serial1.write(tx_buf1, API_TX_SIZE);

  uint8_t tx_buf2[API_TX_SIZE] = {0};
  format_packet(left_fpos, tx_buf2);
  Serial2.write(tx_buf2, API_TX_SIZE);

  int time = 1000;
  // delayMicroseconds(time); // needed for correct read data
  // Serial1.flush();
  // Serial2.flush();
  // delayMicroseconds(time); // needed for correct read data

  read_values(right_hand_msg,Serial1);
  read_values(left_hand_msg,Serial2);
  
  right_hand_msg.fingertips[35] = millis() - start_time;
  left_hand_msg.fingertips[35] = millis() - start_time;


  RCSOFTCHECK(rcl_publish(&right_publisher, &right_hand_msg, NULL));
  RCSOFTCHECK(rcl_publish(&left_publisher, &left_hand_msg, NULL));
  // Spin executor to receive messages
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}