#include "./mpu6050.h"
#include "./delay.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "i2c.h"
#include <stdio.h>
float pitch, roll, yaw; // ŷ-އ
// ԵʼۯMPU6050
// ׵ܘֵ:0,ԉ٦
//     Ǥ̻,խϳպë
u8 MPU_Init(void)
{
	u8 res;
	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X80); // شλMPU6050
	delay_ms(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X00); // ۽ёMPU6050
	MPU_Set_Gyro_Fsr(3);					 // ΓÝӇԫِǷ,2000dps
	MPU_Set_Accel_Fsr(0);					 // ݓ̙׈ԫِǷ,2g
	MPU_Set_Rate(50);						 // ʨ׃ӉҹÊ50Hz
	MPU_Write_Byte(MPU_INT_EN_REG, 0X00);	 // ژҕ̹Ԑא׏
	MPU_Write_Byte(MPU_USER_CTRL_REG, 0X00); // I2C׷ģʽژҕ
	MPU_Write_Byte(MPU_FIFO_EN_REG, 0X00);	 // ژҕFIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG, 0X80); // INTӽޅ֍֧ƽԐЧ
	res = MPU_Read_Byte(MPU_DEVICE_ID_REG);
	printf("%d",res);
	if (res == MPU_ADDR) // ǷݾIDֽȷ
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X01); // ʨ׃CLKSEL,PLL XסΪӎ߼
		MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0X00); // ݓ̙׈ԫΓÝӇּ٤ط
		MPU_Set_Rate(50);						 // ʨ׃ӉҹÊΪ50Hz
	}
	else
		return 1;
	return 0;
}
// ʨ׃MPU6050ΓÝӇԫِǷúԌ׶Χ
// fsr:0,250dps;1,500dps;2,1000dps;3,2000dps
// ׵ܘֵ:0,ʨ׃ԉ٦
//     Ǥ̻,ʨ׃ʧќ
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3); // ʨ׃ΓÝӇúԌ׶Χ
}
// ʨ׃MPU6050ݓ̙׈ԫِǷúԌ׶Χ
// fsr:0,2g;1,4g;2,8g;3,16g
// ׵ܘֵ:0,ʨ׃ԉ٦
//     Ǥ̻,ʨ׃ʧќ
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3); // ʨ׃ݓ̙׈ԫِǷúԌ׶Χ
}
// ʨ׃MPU6050ք˽ؖ֍ͨËҨǷ
// lpf:˽ؖ֍ͨËҨƵÊ(Hz)
// ׵ܘֵ:0,ʨ׃ԉ٦
//     Ǥ̻,ʨ׃ʧќ
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data = 0;
	if (lpf >= 188)
		data = 1;
	else if (lpf >= 98)
		data = 2;
	else if (lpf >= 42)
		data = 3;
	else if (lpf >= 20)
		data = 4;
	else if (lpf >= 10)
		data = 5;
	else
		data = 6;
	return MPU_Write_Byte(MPU_CFG_REG, data); // ʨ׃˽ؖ֍ͨËҨǷ
}
// ʨ׃MPU6050քӉҹÊ(ݙ֨Fs=1KHz)
// rate:4~1000(Hz)
// ׵ܘֵ:0,ʨ׃ԉ٦
//     Ǥ̻,ʨ׃ʧќ
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if (rate > 1000)
		rate = 1000;
	if (rate < 4)
		rate = 4;
	data = 1000 / rate - 1;
	data = MPU_Write_Byte(MPU_SAMPLE_RATE_REG, data); // ʨ׃˽ؖ֍ͨËҨǷ
	return MPU_Set_LPF(rate / 2);					  // ؔ֯ʨ׃LPFΪӉҹÊքһѫ
}

// փսς׈ֵ
// ׵ܘֵ:ς׈ֵ()ճ100Ѷ)
short MPU_Get_Temperature(void)
{
	u8 buf[2];
	short raw;
	float temp;
	MPU_Read_Len(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf);
	raw = ((u16)buf[0] << 8) | buf[1];
	temp = 36.53 + ((double)raw) / 340;
	return temp * 100;
}
// փսΓÝӇֵ(ԭʼֵ)
// gx,gy,gz:ΓÝӇx,y,zסքԭʼׁ˽(ոػۅ)
// ׵ܘֵ:0,ԉ٦
//     Ǥ̻,խϳպë
u8 MPU_Get_Gyroscope(short *gx, short *gy, short *gz)
{
	u8 buf[6], res;
	res = MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
	if (res == 0)
	{
		*gx = ((u16)buf[0] << 8) | buf[1];
		*gy = ((u16)buf[2] << 8) | buf[3];
		*gz = ((u16)buf[4] << 8) | buf[5];
	}
	return res;
	;
}
// փսݓ̙׈ֵ(ԭʼֵ)
// gx,gy,gz:ΓÝӇx,y,zסքԭʼׁ˽(ոػۅ)
// ׵ܘֵ:0,ԉ٦
//     Ǥ̻,խϳպë
u8 MPU_Get_Accelerometer(short *ax, short *ay, short *az)
{
	u8 buf[6], res;
	res = MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);
	if (res == 0)
	{
		*ax = ((u16)buf[0] << 8) | buf[1];
		*ay = ((u16)buf[2] << 8) | buf[3];
		*az = ((u16)buf[4] << 8) | buf[5];
	}
	return res;
	;
}
// IIClѸд
// addr:Ƿݾַ֘
// reg:݄զǷַ֘
// len:дɫӤ׈
// buf:˽ߝȸ
// ׵ܘֵ:0,ֽӣ
//     Ǥ̻,խϳպë
u8 MPU_Write_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
	u8 sta;
	sta = HAL_I2C_Mem_Write(&hi2c1, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
	if (sta == HAL_OK)
		return 0;
	else
		return 1;
}
// IIClѸׁ
// addr:Ƿݾַ֘
// reg:Ҫׁȡք݄զǷַ֘
// len:ҪׁȡքӤ׈
// buf:ׁȡսք˽ߝզԢȸ
// ׵ܘֵ:0,ֽӣ
//     Ǥ̻,խϳպë
u8 MPU_Read_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
	u8 sta;
	sta = HAL_I2C_Mem_Read(&hi2c1, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
	if (sta == HAL_OK)
		return 0;
	else
		return 1;
}
// IICдһٶؖޚ
// reg:݄զǷַ֘
// data:˽ߝ
// ׵ܘֵ:0,ֽӣ
//     Ǥ̻,խϳպë
u8 MPU_Write_Byte(u8 reg, u8 data)
{
	u8 sta;
	sta = HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
	if (sta != HAL_OK)
		return 1;
	else
		return 0;
}

// IICׁһٶؖޚ
// reg:݄զǷַ֘
// ׵ܘֵ:ׁսք˽ߝ
u8 MPU_Read_Byte(u8 reg)
{
	u8 dat, sta;

	sta = HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, &dat, 1, 100);
	if (sta != HAL_OK)
		return 1;
	else
		return dat;
}

float get_yaw(void)
{

	while (mpu_dmp_get_data(&pitch, &roll, &yaw) != 0)
	{
	};
	// printf("pitch=  %.2f\t", pitch);
	// printf("roll=  %.2f\t", roll);
	// printf("yaw=  %.2f\r\n", yaw);

	return yaw;
}
