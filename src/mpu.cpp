#include "mpu.h"

void mpuInit()
{
  Serial.print("Inicjalizacja MPU6050 ");
  mpu.initialize();
  Serial.print(" . ");
  if(mpu.testConnection()) Serial.println("OK");
}

int mpuX()
{
  return mpu.getAccelerationX();
}
int mpuY()
{
  return mpu.getAccelerationY();
}
int mpuZ()
{
  return mpu.getAccelerationZ();
}