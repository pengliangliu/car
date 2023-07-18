#include "oled.h"
#include "font.h"    
#include "i2c.h"


uint8_t CMD_Data[]={
0xAE, 0x00, 0x10, 0x40, 0xB0, 0x81, 0xFF, 0xA1, 0xA6, 0xA8, 0x3F,
					
0xC8, 0xD3, 0x00, 0xD5, 0x80, 0xD8, 0x05, 0xD9, 0xF1, 0xDA, 0x12,
					
0xD8, 0x30, 0x8D, 0x14, 0xAF};      //初始化命令
//显示浮点数
//x,y :起点坐标
//num:数值	 
//a:整数的位数
//b:小数的位数
//size:字体大小
void OLED_SHowfloat(uint8_t x,uint8_t y,float num,uint8_t a,uint8_t b,uint8_t size)
{
    if(num<0)
    {
        OLED_ShowChar(x,y,'-',size);
        x+=size/2;
        num=-num;
        
    OLED_ShowNum(x,y,num,a,16);
    OLED_ShowChar(x+a*(size/2),y,'.',16);
    for(int i=1;i<=b;i++)
        {
            OLED_ShowNum(x+(a+i)*(size/2),y,(uint16_t)(num*10)%10,1,size);
            num*=10;
        }
    }
    
    else
    {
    OLED_ShowNum(x,y,num,a,16);
    OLED_ShowChar(x+a*(size/2),y,'.',16);
    for(int i=1;i<=b;i++)
        {
            OLED_ShowNum(x+(a+i)*(size/2),y,(uint16_t)(num*10)%10,1,size);
            num*=10;
        }
    }
}
//通过I2C写入命令    
void WriteCmd(void)
{
	uint8_t i = 0;
	for(i=0; i<27; i++)
	{
		HAL_I2C_Mem_Write(&hi2c2 ,0x78,0x00,I2C_MEMADD_SIZE_8BIT,CMD_Data+i,1,0x100);
	}
}
//向设备写控制命令
void OLED_WR_CMD(uint8_t cmd)
{
	HAL_I2C_Mem_Write(&hi2c2 ,0x78,0x00,I2C_MEMADD_SIZE_8BIT,&cmd,1,0x100);    
}
//向设备写数据
void OLED_WR_DATA(uint8_t data)
{
	HAL_I2C_Mem_Write(&hi2c2 ,0x78,0x40,I2C_MEMADD_SIZE_8BIT,&data,1,0x100);
}
//初始化oled屏幕
void OLED_Init(void)
{ 	
	HAL_Delay(200);
 
	WriteCmd();
}
//清屏
void OLED_Clear(void)
{
	uint8_t i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_CMD(0xb0+i);
		OLED_WR_CMD (0x00); 
		OLED_WR_CMD (0x10); 
		for(n=0;n<128;n++)
			OLED_WR_DATA(0);
	} 
}
//开启OLED显示    
void OLED_Display_On(void)
{
	OLED_WR_CMD(0X8D);  //SET DCDC命令
	OLED_WR_CMD(0X14);  //DCDC ON
	OLED_WR_CMD(0XAF);  //DISPLAY ON
}
//关闭OLED显示     
void OLED_Display_Off(void)
{
	OLED_WR_CMD(0X8D);  //SET DCDC命令
	OLED_WR_CMD(0X10);  //DCDC OFF
	OLED_WR_CMD(0XAE);  //DISPLAY OFF
}		   			 
void OLED_Set_Pos(uint8_t x, uint8_t y) 
{ 	
	OLED_WR_CMD(0xb0+y);
	OLED_WR_CMD(((x&0xf0)>>4)|0x10);
	OLED_WR_CMD(x&0x0f);
} 
 
void OLED_On(void)  
{  
	uint8_t i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_CMD(0xb0+i);    //设置页地址（0~7）
		OLED_WR_CMD(0x00);      //设置显示位置—列低地址
		OLED_WR_CMD(0x10);      //设置显示位置—列高地址   
		for(n=0;n<128;n++)
			OLED_WR_DATA(1); 
	} //更新显示
}
unsigned int oled_pow(uint8_t m,uint8_t n)
{
	unsigned int result=1;	 
	while(n--)result*=m;    
	return result;
}
//显示2个数字
//x,y :起点坐标	 
//len :数字的位数
//size:字体大小
//mode:模式	0,填充模式;1,叠加模式
//num:数值(0~4294967295);	 		  
void OLED_ShowNum(uint8_t x,uint8_t y,unsigned int num,uint8_t len,uint8_t size2)
{         	
	uint8_t t,temp;
	uint8_t enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size2/2)*t,y,' ',size2);
				continue;
			}else enshow=1; 
		 	 
		}
	 	OLED_ShowChar(x+(size2/2)*t,y,temp+'0',size2); 
	}
} 
//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63
//mode:0,反白显示;1,正常显示				 
//size:选择字体 16/12 
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t Char_Size)
{      	
	unsigned char c=0,i=0;	
		c=chr-' ';//得到偏移后的值			
		if(x>128-1){x=0;y=y+2;}
		if(Char_Size ==16)
			{
			OLED_Set_Pos(x,y);	
			for(i=0;i<8;i++)
			OLED_WR_DATA(F8X16[c*16+i]);
			OLED_Set_Pos(x,y+1);
			for(i=0;i<8;i++)
			OLED_WR_DATA(F8X16[c*16+i+8]);
			}
			else {	
				OLED_Set_Pos(x,y);
				for(i=0;i<6;i++)
				OLED_WR_DATA(F6x8[c][i]);
				
			}
}
 
//显示一个字符号串
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t Char_Size)
{
	unsigned char j=0;
	while (chr[j]!='\0')
	{		OLED_ShowChar(x,y,chr[j],Char_Size);
			x+=8;
		if(x>120){x=0;y+=2;}
			j++;
	}
}
//显示汉字
//hzk 用取模软件得出的数组
void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no)
{      			    
	uint8_t t,adder=0;
	OLED_Set_Pos(x,y);	
    for(t=0;t<16;t++)
		{
				OLED_WR_DATA(Hzk[2*no][t]);
				adder+=1;
     }	
		OLED_Set_Pos(x,y+1);	
    for(t=0;t<16;t++)
			{	
				OLED_WR_DATA(Hzk[2*no+1][t]);
				adder+=1;
      }					
}

/*
	@brief			显示图片
	@param			x0：起始列地址
					y0：起始页地址
					x1：终止列地址
					y1：终止页地址
					BMP[]：存放图片代码的数组
	@retval			无
 */
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1)
{ 	
 	unsigned int j=0; //定义变量
 	unsigned char x,y; //定义变量
  
 	if(y1%8==0) y=y1/8;   //判断终止页是否为8的整数倍
 	 else y=y1/8+1;

		for(y=y0;y<y1;y++) //从起始页开始，画到终止页
		{
			OLED_Set_Pos(x0,y); //在页的起始列开始画
   			for(x=x0;x<x1;x++) //画x1 - x0 列
	    		{
	    			OLED_WR_DATA(BMP[j++]);	//画图片的点    	
	    		}
		}
} 

/*
	@brief			显示动图
	@param			x0：起始列地址
				y0：起始页地址
				x1：终止列地址
				y1：终止页地址
				k: 帧个数
				m: 单帧数组大小
				BMP[][m]：存放动图代码的数组
	@retval			无
 */
void OLED_DrawGIF(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1, unsigned char k, int m)
{
	unsigned int j=0; //定义变量
 	unsigned char x,y,i; //定义变量
  
 	if(y1%8==0) y=y1/8;   //判断终止页是否为8的整数倍
 	 else y=y1/8+1;
	for (i=0;i<k;i++) //从第一帧开始画
	{
		j = 0;
		for(y=y0;y<y1;y++) //从起始页开始，画到终止页
		{
			OLED_Set_Pos(x0,y); //在页的起始列开始画
   			
			for(x=x0;x<x1;x++) //画x1 - x0 列
	    		{
						
	    			OLED_WR_DATA(GIF[i][j++]);	//画图片的点    	
	    		}
		}
		//delay_ms(80);//人为制造卡顿？？？

	}
}
