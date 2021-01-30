#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float64.h> 

#define MPU6050_WHO_AM_I   0x75  // Read Only
#define MPU6050_PWR_MGMT_1 0x6B  // Read and Write
#define MPU_ADDR  0x68
#define RESOL_GYRO 131.0

ros::NodeHandle nh;
std_msgs::Float64 yaw;
ros::Publisher pub("gyro", &yaw);

int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw, tmp;

void setup() {
  Wire.begin();
  //Serial.begin(57600); //115200bps
 
  // 初回の読み出し
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU6050_WHO_AM_I);  //MPU6050_PWR_MGMT_1
  Wire.write(0x00);
  Wire.endTransmission();

  // 動作モードの読み出し
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU6050_PWR_MGMT_1);  //MPU6050_PWR_MGMT_1レジスタの設定
  Wire.write(0x00);
  Wire.endTransmission();

  nh.initNode();
  nh.advertise(pub);
}


void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  while (Wire.available() < 14);

  axRaw = Wire.read() << 8 | Wire.read();
  ayRaw = Wire.read() << 8 | Wire.read();
  azRaw = Wire.read() << 8 | Wire.read();
  tmp = Wire.read() << 8 | Wire.read();
  gxRaw = Wire.read() << 8 | Wire.read();
  gyRaw = Wire.read() << 8 | Wire.read();
  gzRaw = Wire.read() << 8 | Wire.read();

  // 角速度値を分解能で割って角速度(degrees per sec)に変換する
  float gyro_z = gzRaw / RESOL_GYRO;//FS_SEL_0 131 LSB / (°/s)

  yaw.data = gyro_z;
  pub.publish(&yaw);
  /* nh.spinOnce(); */
  /* Serial.println(gyro_z); */
}
