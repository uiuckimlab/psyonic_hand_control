#include <Arduino.h>
#include <Wire.h>

#include "helper_functions.h"
#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

#define NODE_NAME "psyonic_controller_node"
#define TOPIC_NAME "psyonic_hand_vals"
#define COMMAND_TOPIC_SUB "psyonic_controller/command"
#define ENABLE_UPSAMPLE_SUB "psyonic_controller/enable_upsample"
#define POSITION_SIZE 6
#define CURRENT_SIZE 6
#define VELOCITY_SIZE 6
#define FINGERTIP_SENSOR_SIZE 36

rcl_publisher_t publisher;
rcl_subscription_t command_subscriber;
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
psyonic_hand_control__msg__HandVal hand_msg;
std_msgs__msg__Float64MultiArray command_msg;
std_msgs__msg__Bool enable_thumb_up_msg;

// Command msg array buffer
double array_buffer[NUM_CHANNELS];

// Setup finger position variable
float fpos[NUM_CHANNELS] = {30.f,30.f,30.f,30.f,30.f, -30.f};
uint8_t rx_buffer[API_RX_SIZE];

/*Moves Fingers to the position user has set*/
void moveHandCallback(const void * msgin){
  const std_msgs__msg__Float64MultiArray * msg = (const std_msgs__msg__Float64MultiArray *)msgin;
  for (uint32_t i = 0; i < msg->data.size  && i < NUM_CHANNELS; i++) {
    fpos[i] = msg->data.data[i] * 180.0 / M_PI;
  }
}

void enableThumbCallback(const void * msgin){
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  int time = 1000;
  delayMicroseconds(time*5);
  uint8_t tx_buf_enable[API_TX_SIZE] = {0};
  enable_thumb_upsample(tx_buf_enable, msg->data);
  Serial1.write(tx_buf_enable, API_TX_SIZE);
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
  
  std_msgs__msg__Float64MultiArray__init(&command_msg);
  command_msg.data.capacity = NUM_CHANNELS;
  command_msg.data.size = 0;
  command_msg.data.data = array_buffer;

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(psyonic_hand_control, msg, HandVal),
    TOPIC_NAME));
  
  // Create Subscribers
  RCCHECK(rclc_subscription_init_default(
    &command_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    COMMAND_TOPIC_SUB));

  RCCHECK(rclc_subscription_init_default(
    &enable_thumb_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    ENABLE_UPSAMPLE_SUB));
  

       // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &command_subscriber,
    &command_msg,
    &moveHandCallback,
    ON_NEW_DATA));

  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &enable_thumb_subscriber,
    &enable_thumb_up_msg,
    &enableThumbCallback,
    ON_NEW_DATA));

  for(size_t i = 0; i < POSITION_SIZE; i++){
    hand_msg.positions[i] = fpos[i];
  }
  for(size_t i = 0; i < CURRENT_SIZE; i++){
    hand_msg.currents[i] = 0.0;
  }
  for(size_t i = 0; i < VELOCITY_SIZE; i++){
    hand_msg.velocities[i] = 0.0;
  }
  for(size_t i = 0; i < FINGERTIP_SENSOR_SIZE; i++){
    hand_msg.fingertips[i] = 0.0;
  }

  Serial1.begin(460800);
  Serial1.addMemoryForRead(rx_buffer,API_RX_SIZE-63); // 63 is default buffer size, so trying to make it 72 bytes
  Serial1.clear();
  int time = 1000;
  delayMicroseconds(time*5);
  uint8_t tx_buf_enable[API_TX_SIZE] = {0};
  enable_thumb_upsample(tx_buf_enable, true);
  Serial1.write(tx_buf_enable, API_TX_SIZE);
  delayMicroseconds(time);


}

void loop()
{
  float start_time = millis();
  uint8_t tx_buf[API_TX_SIZE] = {0};
  format_packet(fpos, tx_buf);
  Serial1.write(tx_buf, API_TX_SIZE);
  int time = 1000;
  delayMicroseconds(time); // needed for correct read data
  Serial1.flush();
  delayMicroseconds(time); // needed for correct read data
  read_values(hand_msg,Serial1);
  hand_msg.fingertips[35] = millis() - start_time;
  RCSOFTCHECK(rcl_publish(&publisher, &hand_msg, NULL));
  // Spin executor to receive messages
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}