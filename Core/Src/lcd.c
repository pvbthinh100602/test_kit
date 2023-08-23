/*
 * lcd.c
 *
 *  Created on: Aug 9, 2023
 *      Author: phamv
 */
#include "lcd.h"
#include "stdlib.h"
#include <string.h>
#include "lcdfont.h"

unsigned char s[50];







//¹ÜÀíLCDÖØÒª²ÎÊý
//Ä¬ÈÏÎªÊúÆÁ
_lcd_dev lcddev;

//Ð´¼Ä´æÆ÷º¯Êý
//regval:¼Ä´æÆ÷Öµ
void LCD_WR_REG(uint16_t reg)
{
	LCD->LCD_REG=reg;//Ð´ÈëÒªÐ´µÄ¼Ä´æÆ÷ÐòºÅ
}
//Ð´LCDÊý¾Ý
//data:ÒªÐ´ÈëµÄÖµ
void LCD_WR_DATA(uint16_t data)
{
	LCD->LCD_RAM=data;
}
//¶ÁLCDÊý¾Ý
//·µ»ØÖµ:¶Áµ½µÄÖµ
uint16_t LCD_RD_DATA(void)
{
	__IO uint16_t ram;			//·ÀÖ¹±»ÓÅ»¯
	ram=LCD->LCD_RAM;
	return ram;
}


/******************************************************************************
      º¯ÊýËµÃ÷£ºÉèÖÃÆðÊ¼ºÍ½áÊøµØÖ·
      Èë¿ÚÊý¾Ý£ºx1,x2 ÉèÖÃÁÐµÄÆðÊ¼ºÍ½áÊøµØÖ·
                y1,y2 ÉèÖÃÐÐµÄÆðÊ¼ºÍ½áÊøµØÖ·
      ·µ»ØÖµ£º  ÎÞ
******************************************************************************/
void lcd_AddressSet(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
		LCD_WR_REG(0x2a);//ÁÐµØÖ·ÉèÖÃ
		LCD_WR_DATA(x1>>8);
		LCD_WR_DATA(x1&0xff);
		LCD_WR_DATA(x2>>8);
		LCD_WR_DATA(x2&0xff);
		LCD_WR_REG(0x2b);//ÐÐµØÖ·ÉèÖÃ
		LCD_WR_DATA(y1>>8);
		LCD_WR_DATA(y1&0xff);
		LCD_WR_DATA(y2>>8);
		LCD_WR_DATA(y2&0xff);
		LCD_WR_REG(0x2c);//´¢´æÆ÷Ð´
}

/******************************************************************************
      º¯ÊýËµÃ÷£ºÉèÖÃ¹â±êÎ»ÖÃ
      Èë¿ÚÊý¾Ý£ºx,y ¹â±êÎ»ÖÃ
      ·µ»ØÖµ£º  ÎÞ
******************************************************************************/
void lcd_SetCursor(uint16_t x,uint16_t y)
{
	LCD_WR_REG(0x2a);//ÁÐµØÖ·ÉèÖÃ
	LCD_WR_DATA(x>>8);
	LCD_WR_DATA(x&0xff);
	LCD_WR_REG(0x2b);//ÐÐµØÖ·ÉèÖÃ
	LCD_WR_DATA(y>>8);
	LCD_WR_DATA(y&0xff);
}

//LCD¿ªÆôÏÔÊ¾
void lcd_DisplayOn(void)
{
	LCD_WR_REG(0X29);	//¿ªÆôÏÔÊ¾
}
//LCD¹Ø±ÕÏÔÊ¾
void lcd_DisplayOff(void)
{
	LCD_WR_REG(0X28);	//¹Ø±ÕÏÔÊ¾
}


//¶ÁÈ¡¸öÄ³µãµÄÑÕÉ«Öµ
//x,y:×ø±ê
//·µ»ØÖµ:´ËµãµÄÑÕÉ«
uint16_t lcd_ReadPoint(uint16_t x,uint16_t y)
{
 	uint16_t r=0,g=0,b=0;
	lcd_SetCursor(x,y);
	LCD_WR_REG(0X2E);
	r=LCD_RD_DATA();								//dummy Read
	r=LCD_RD_DATA();  		  				//Êµ¼Ê×ø±êÑÕÉ«
	b=LCD_RD_DATA();
	g=r&0XFF;
	g<<=8;
	return (((r>>11)<<11)|((g>>10)<<5)|(b>>11));
}


void lcd_Clear(uint16_t color)
{
	uint16_t i,j;
	lcd_AddressSet(0,0,lcddev.width-1,lcddev.height-1);//ÉèÖÃÏÔÊ¾·¶Î§
	for(i=0;i<lcddev.width;i++)
	{
		for(j=0;j<lcddev.height;j++)
		{
			LCD_WR_DATA(color);
		}
	}
}

/******************************************************************************
      º¯ÊýËµÃ÷£ºÔÚÖ¸¶¨ÇøÓòÌî³äÑÕÉ«
      Èë¿ÚÊý¾Ý£ºxsta,ysta   ÆðÊ¼×ø±ê
                xend,yend   ÖÕÖ¹×ø±ê
								color       ÒªÌî³äµÄÑÕÉ«
      ·µ»ØÖµ£º  ÎÞ
******************************************************************************/
void lcd_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color)
{
	uint16_t i,j;
	lcd_AddressSet(xsta,ysta,xend-1,yend-1);//ÉèÖÃÏÔÊ¾·¶Î§
	for(i=ysta;i<yend;i++)
	{
		for(j=xsta;j<xend;j++)
		{
			LCD_WR_DATA(color);
		}
	}
}

/******************************************************************************
      º¯ÊýËµÃ÷£ºÔÚÖ¸¶¨Î»ÖÃ»­µã
      Èë¿ÚÊý¾Ý£ºx,y »­µã×ø±ê
                color µãµÄÑÕÉ«
      ·µ»ØÖµ£º  ÎÞ
******************************************************************************/
void lcd_DrawPoint(uint16_t x,uint16_t y,uint16_t color)
{
	lcd_AddressSet(x,y,x,y);//ÉèÖÃ¹â±êÎ»ÖÃ
	LCD_WR_DATA(color);
}


/******************************************************************************
      º¯ÊýËµÃ÷£º»­Ïß
      Èë¿ÚÊý¾Ý£ºx1,y1   ÆðÊ¼×ø±ê
                x2,y2   ÖÕÖ¹×ø±ê
                color   ÏßµÄÑÕÉ«
      ·µ»ØÖµ£º  ÎÞ
******************************************************************************/
void lcd_DrawLine(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint16_t color)
{
	uint16_t t;
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;
	delta_x=x2-x1; //¼ÆËã×ø±êÔöÁ¿
	delta_y=y2-y1;
	uRow=x1;//»­ÏßÆðµã×ø±ê
	uCol=y1;
	if(delta_x>0)incx=1; //ÉèÖÃµ¥²½·½Ïò
	else if (delta_x==0)incx=0;//´¹Ö±Ïß
	else {incx=-1;delta_x=-delta_x;}
	if(delta_y>0)incy=1;
	else if (delta_y==0)incy=0;//Ë®Æ½Ïß
	else {incy=-1;delta_y=-delta_y;}
	if(delta_x>delta_y)distance=delta_x; //Ñ¡È¡»ù±¾ÔöÁ¿×ø±êÖá
	else distance=delta_y;
	for(t=0;t<distance+1;t++)
	{
		lcd_DrawPoint(uRow,uCol,color);//»­µã
		xerr+=delta_x;
		yerr+=delta_y;
		if(xerr>distance)
		{
			xerr-=distance;
			uRow+=incx;
		}
		if(yerr>distance)
		{
			yerr-=distance;
			uCol+=incy;
		}
	}
}


/******************************************************************************
      º¯ÊýËµÃ÷£º»­¾ØÐÎ
      Èë¿ÚÊý¾Ý£ºx1,y1   ÆðÊ¼×ø±ê
                x2,y2   ÖÕÖ¹×ø±ê
                color   ¾ØÐÎµÄÑÕÉ«
      ·µ»ØÖµ£º  ÎÞ
******************************************************************************/
void lcd_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t color)
{
	lcd_DrawLine(x1,y1,x2,y1,color);
	lcd_DrawLine(x1,y1,x1,y2,color);
	lcd_DrawLine(x1,y2,x2,y2,color);
	lcd_DrawLine(x2,y1,x2,y2,color);
}


/******************************************************************************
      º¯ÊýËµÃ÷£º»­Ô²
      Èë¿ÚÊý¾Ý£ºx0,y0   Ô²ÐÄ×ø±ê
                r       °ë¾¶
                color   Ô²µÄÑÕÉ«
      ·µ»ØÖµ£º  ÎÞ
******************************************************************************/
void lcd_DrawCircle(uint16_t x0,uint16_t y0,uint8_t r,uint16_t color)
{
	int a,b;
	a=0;b=r;
	while(a<=b)
	{
		lcd_DrawPoint(x0-b,y0-a,color);             //3
		lcd_DrawPoint(x0+b,y0-a,color);             //0
		lcd_DrawPoint(x0-a,y0+b,color);             //1
		lcd_DrawPoint(x0-a,y0-b,color);             //2
		lcd_DrawPoint(x0+b,y0+a,color);             //4
		lcd_DrawPoint(x0+a,y0-b,color);             //5
		lcd_DrawPoint(x0+a,y0+b,color);             //6
		lcd_DrawPoint(x0-b,y0+a,color);             //7
		a++;
		if((a*a+b*b)>(r*r))//ÅÐ¶ÏÒª»­µÄµãÊÇ·ñ¹ýÔ¶
		{
			b--;
		}
	}
}

/******************************************************************************
      º¯ÊýËµÃ÷£ºÏÔÊ¾µ¥¸ö×Ö·û
      Èë¿ÚÊý¾Ý£ºx,yÏÔÊ¾×ø±ê
                num ÒªÏÔÊ¾µÄ×Ö·û
                fc ×ÖµÄÑÕÉ«
                bc ×ÖµÄ±³¾°É«
                sizey ×ÖºÅ
                mode:  0·Çµþ¼ÓÄ£Ê½  1µþ¼ÓÄ£Ê½
      ·µ»ØÖµ£º  ÎÞ
******************************************************************************/
void lcd_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t temp,sizex,t,m=0;
	uint16_t i,TypefaceNum;//Ò»¸ö×Ö·ûËùÕ¼×Ö½Ú´óÐ¡
	uint16_t x0=x;
	sizex=sizey/2;
	TypefaceNum=(sizex/8+((sizex%8)?1:0))*sizey;
	num=num-' ';    //µÃµ½Æ«ÒÆºóµÄÖµ
	lcd_AddressSet(x,y,x+sizex-1,y+sizey-1);  //ÉèÖÃ¹â±êÎ»ÖÃ
	for(i=0;i<TypefaceNum;i++)
	{
		if(sizey==12)temp=ascii_1206[num][i];		       //µ÷ÓÃ6x12×ÖÌå
		else if(sizey==16)temp=ascii_1608[num][i];		 //µ÷ÓÃ8x16×ÖÌå
		else if(sizey==24)temp=ascii_2412[num][i];		 //µ÷ÓÃ12x24×ÖÌå
		else if(sizey==32)temp=ascii_3216[num][i];		 //µ÷ÓÃ16x32×ÖÌå
		else return;
		for(t=0;t<8;t++)
		{
			if(!mode)//·Çµþ¼ÓÄ£Ê½
			{
				if(temp&(0x01<<t))LCD_WR_DATA(fc);
				else LCD_WR_DATA(bc);
				m++;
				if(m%sizex==0)
				{
					m=0;
					break;
				}
			}
			else//µþ¼ÓÄ£Ê½
			{
				if(temp&(0x01<<t))lcd_DrawPoint(x,y,fc);//»­Ò»¸öµã
				x++;
				if((x-x0)==sizex)
				{
					x=x0;
					y++;
					break;
				}
			}
		}
	}
}


/******************************************************************************
      º¯ÊýËµÃ÷£ºÏÔÊ¾×Ö·û´®
      Èë¿ÚÊý¾Ý£ºx,yÏÔÊ¾×ø±ê
                *p ÒªÏÔÊ¾µÄ×Ö·û´®
                fc ×ÖµÄÑÕÉ«
                bc ×ÖµÄ±³¾°É«
                sizey ×ÖºÅ
                mode:  0·Çµþ¼ÓÄ£Ê½  1µþ¼ÓÄ£Ê½
      ·µ»ØÖµ£º  ÎÞ
******************************************************************************/
void lcd_ShowString(uint16_t x,uint16_t y,const uint8_t *p,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	while(*p!='\0')
	{
		lcd_ShowChar(x,y,*p,fc,bc,sizey,mode);
		x+=sizey/2;
		p++;
	}
}


/******************************************************************************
      º¯ÊýËµÃ÷£ºÏÔÊ¾Êý×Ö
      Èë¿ÚÊý¾Ý£ºmµ×Êý£¬nÖ¸Êý
      ·µ»ØÖµ£º  ÎÞ
******************************************************************************/
uint32_t mypow(uint8_t m,uint8_t n)
{
	uint32_t result=1;
	while(n--)result*=m;
	return result;
}


/******************************************************************************
      º¯ÊýËµÃ÷£ºÏÔÊ¾ÕûÊý±äÁ¿
      Èë¿ÚÊý¾Ý£ºx,yÏÔÊ¾×ø±ê
                num ÒªÏÔÊ¾ÕûÊý±äÁ¿
                len ÒªÏÔÊ¾µÄÎ»Êý
                fc ×ÖµÄÑÕÉ«
                bc ×ÖµÄ±³¾°É«
                sizey ×ÖºÅ
      ·µ»ØÖµ£º  ÎÞ
******************************************************************************/
void lcd_ShowIntNum(uint16_t x,uint16_t y,uint16_t num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey)
{
	uint8_t t,temp;
	uint8_t enshow=0;
	uint8_t sizex=sizey/2;
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				lcd_ShowChar(x+t*sizex,y,' ',fc,bc,sizey,0);
				continue;
			}else enshow=1;

		}
	 	lcd_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
}


/******************************************************************************
      º¯ÊýËµÃ÷£ºÏÔÊ¾Á½Î»Ð¡Êý±äÁ¿
      Èë¿ÚÊý¾Ý£ºx,yÏÔÊ¾×ø±ê
                num ÒªÏÔÊ¾Ð¡Êý±äÁ¿
                len ÒªÏÔÊ¾µÄÎ»Êý
                fc ×ÖµÄÑÕÉ«
                bc ×ÖµÄ±³¾°É«
                sizey ×ÖºÅ
      ·µ»ØÖµ£º  ÎÞ
******************************************************************************/
void lcd_ShowFloatNum1(uint16_t x,uint16_t y,float num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey)
{
	uint8_t t,temp,sizex;
	uint16_t num1;
	sizex=sizey/2;
	num1=num*100;
	for(t=0;t<len;t++)
	{
		temp=(num1/mypow(10,len-t-1))%10;
		if(t==(len-2))
		{
			lcd_ShowChar(x+(len-2)*sizex,y,'.',fc,bc,sizey,0);
			t++;
			len+=1;
		}
	 	lcd_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
}


/******************************************************************************
      º¯ÊýËµÃ÷£ºÏÔÊ¾Í¼Æ¬
      Èë¿ÚÊý¾Ý£ºx,yÆðµã×ø±ê
                length Í¼Æ¬³¤¶È
                width  Í¼Æ¬¿í¶È
                pic[]  Í¼Æ¬Êý×é
      ·µ»ØÖµ£º  ÎÞ
******************************************************************************/
void lcd_ShowPicture(uint16_t x,uint16_t y,uint16_t length,uint16_t width,const uint8_t pic[])
{
	uint8_t picH,picL;
	uint16_t i,j;
	uint32_t k=0;
	lcd_AddressSet(x,y,x+length-1,y+width-1);
	for(i=0;i<length;i++)
	{
		for(j=0;j<width;j++)
		{
			picH=pic[k*2];
			picL=pic[k*2+1];
			LCD_WR_DATA(picH<<8|picL);
			k++;
		}
	}
}


void lcd_SetDir(uint8_t dir)
{
	if((dir>>4)%4)
	{
		lcddev.width=320;
		lcddev.height=240;
	}else
	{
		lcddev.width=240;
		lcddev.height=320;
	}
}



//³õÊ¼»¯lcd
void lcd_init(void)
{
	HAL_GPIO_WritePin(FSMC_RES_GPIO_Port, FSMC_RES_Pin, GPIO_PIN_RESET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(FSMC_RES_GPIO_Port, FSMC_RES_Pin, GPIO_PIN_SET);
	HAL_Delay(500);
//	Set_Dir(DFT_SCAN_DIR);
	lcd_SetDir(L2R_U2D);
	LCD_WR_REG(0XD3);
	lcddev.id=LCD_RD_DATA();	//dummy read
	lcddev.id=LCD_RD_DATA();	//¶Áµ½0X00
	lcddev.id=LCD_RD_DATA();   	//¶ÁÈ¡93
	lcddev.id<<=8;
	lcddev.id|=LCD_RD_DATA();  	//¶ÁÈ¡41

	LCD_WR_REG(0xCF);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0xC1);
	LCD_WR_DATA(0X30);
	LCD_WR_REG(0xED);
	LCD_WR_DATA(0x64);
	LCD_WR_DATA(0x03);
	LCD_WR_DATA(0X12);
	LCD_WR_DATA(0X81);
	LCD_WR_REG(0xE8);
	LCD_WR_DATA(0x85);
	LCD_WR_DATA(0x10);
	LCD_WR_DATA(0x7A);
	LCD_WR_REG(0xCB);
	LCD_WR_DATA(0x39);
	LCD_WR_DATA(0x2C);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x34);
	LCD_WR_DATA(0x02);
	LCD_WR_REG(0xF7);
	LCD_WR_DATA(0x20);
	LCD_WR_REG(0xEA);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_REG(0xC0);    //Power control
	LCD_WR_DATA(0x1B);   //VRH[5:0]
	LCD_WR_REG(0xC1);    //Power control
	LCD_WR_DATA(0x01);   //SAP[2:0];BT[3:0]
	LCD_WR_REG(0xC5);    //VCM control
	LCD_WR_DATA(0x30); 	 //3F
	LCD_WR_DATA(0x30); 	 //3C
	LCD_WR_REG(0xC7);    //VCM control2
	LCD_WR_DATA(0XB7);
	LCD_WR_REG(0x36);    // Memory Access Control
	LCD_WR_DATA(0x08|L2R_U2D);
	LCD_WR_REG(0x3A);
	LCD_WR_DATA(0x55);
	LCD_WR_REG(0xB1);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x1A);
	LCD_WR_REG(0xB6);    // Display Function Control
	LCD_WR_DATA(0x0A);
	LCD_WR_DATA(0xA2);
	LCD_WR_REG(0xF2);    // 3Gamma Function Disable
	LCD_WR_DATA(0x00);
	LCD_WR_REG(0x26);    //Gamma curve selected
	LCD_WR_DATA(0x01);
	LCD_WR_REG(0xE0);    //Set Gamma
	LCD_WR_DATA(0x0F);
	LCD_WR_DATA(0x2A);
	LCD_WR_DATA(0x28);
	LCD_WR_DATA(0x08);
	LCD_WR_DATA(0x0E);
	LCD_WR_DATA(0x08);
	LCD_WR_DATA(0x54);
	LCD_WR_DATA(0XA9);
	LCD_WR_DATA(0x43);
	LCD_WR_DATA(0x0A);
	LCD_WR_DATA(0x0F);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_REG(0XE1);    //Set Gamma
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x15);
	LCD_WR_DATA(0x17);
	LCD_WR_DATA(0x07);
	LCD_WR_DATA(0x11);
	LCD_WR_DATA(0x06);
	LCD_WR_DATA(0x2B);
	LCD_WR_DATA(0x56);
	LCD_WR_DATA(0x3C);
	LCD_WR_DATA(0x05);
	LCD_WR_DATA(0x10);
	LCD_WR_DATA(0x0F);
	LCD_WR_DATA(0x3F);
	LCD_WR_DATA(0x3F);
	LCD_WR_DATA(0x0F);
	LCD_WR_REG(0x2B);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x01);
	LCD_WR_DATA(0x3f);
	LCD_WR_REG(0x2A);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0xef);
	LCD_WR_REG(0x11); //Exit Sleep
	HAL_Delay(120);
	LCD_WR_REG(0x29); //display on
	HAL_GPIO_WritePin(FSMC_BLK_GPIO_Port, FSMC_BLK_Pin, 1);
}
/******************************************************************************
      º¯ÊýËµÃ÷£º8¶Ô³ÆÐÔ»­Ô²Ëã·¨(ÄÚ²¿µ÷ÓÃ)
      Èë¿ÚÊý¾Ý£º(xc,yc) :Ô²ÖÐÐÄ×ø±ê
                (x,y):¹â±êÏà¶ÔÓÚÔ²ÐÄµÄ×ø±ê
                c µãµÄÑÕÉ«

      ·µ»ØÖµ£º  ÎÞ
******************************************************************************/
void _draw_circle_8(int xc, int yc, int x, int y, uint16_t c)
{
	lcd_DrawPoint(xc + x, yc + y, c);

	lcd_DrawPoint(xc - x, yc + y, c);

	lcd_DrawPoint(xc + x, yc - y, c);

	lcd_DrawPoint(xc - x, yc - y, c);

	lcd_DrawPoint(xc + y, yc + x, c);

	lcd_DrawPoint(xc - y, yc + x, c);

	lcd_DrawPoint(xc + y, yc - x, c);

	lcd_DrawPoint(xc - y, yc - x, c);
}

/******************************************************************************
      º¯ÊýËµÃ÷£ºÔÚÖ¸¶¨Î»ÖÃ»­Ò»¸öÖ¸¶¨´óÐ¡µÄÔ²
      Èë¿ÚÊý¾Ý£º(xc,yc) :Ô²ÖÐÐÄ×ø±ê
                c:Ìî³äµÄÑÕÉ«
                r:Ô²°ë¾¶
                fill:Ìî³äÅÐ¶Ï±êÖ¾£¬1-Ìî³ä£¬0-²»Ìî³ä
      ·µ»ØÖµ£º  ÎÞ
******************************************************************************/
void gui_circle(int xc, int yc,uint16_t c,int r, int fill)
{
	int x = 0, y = r, yi, d;

	d = 3 - 2 * r;


	if (fill)
	{
		// Èç¹ûÌî³ä£¨»­ÊµÐÄÔ²£©
		while (x <= y) {
			for (yi = x; yi <= y; yi++)
				_draw_circle_8(xc, yc, x, yi, c);

			if (d < 0) {
				d = d + 4 * x + 6;
			} else {
				d = d + 4 * (x - y) + 10;
				y--;
			}
			x++;
		}
	} else
	{
		// Èç¹û²»Ìî³ä£¨»­¿ÕÐÄÔ²£©
		while (x <= y) {
			_draw_circle_8(xc, yc, x, y, c);
			if (d < 0) {
				d = d + 4 * x + 6;
			} else {
				d = d + 4 * (x - y) + 10;
				y--;
			}
			x++;
		}
	}
}

/******************************************************************************
      º¯ÊýËµÃ÷£ºÏÔÊ¾Ò»¸ö×Ö·û´®,°üº¬ÖÐÓ¢ÎÄÏÔÊ¾
      Èë¿ÚÊý¾Ý£ºx,y :Æðµã×ø±ê
                c:Ìî³äµÄÑÕÉ«
								fc:Ç°ÖÃ»­±ÊÑÕÉ«
								bc:±³¾°ÑÕÉ«
								str :×Ö·û´®
								size:×ÖÌå´óÐ¡
								mode:Ä£Ê½	0,Ìî³äÄ£Ê½;1,µþ¼ÓÄ£Ê½
      ·µ»ØÖµ£º  ÎÞ
******************************************************************************/
void lcd_ShowStr(uint16_t x, uint16_t y,uint8_t *str,uint16_t fc, uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint16_t x0=x;
  uint8_t bHz=0;     //×Ö·û»òÕßÖÐÎÄ
	while(*str!=0)//Êý¾ÝÎ´½áÊø
	{
		if(!bHz)
		{
			if(x>(lcddev.width-sizey/2)||y>(lcddev.height-sizey)) return;
			if(*str>0x80)bHz=1;//ÖÐÎÄ
			else              //×Ö·û
			{
				if(*str==0x0D)//»»ÐÐ·ûºÅ
				{
					y+=sizey;
					x=x0;
					str++;
				}else
				{
					lcd_ShowChar(x,y,*str,fc,bc,sizey,mode);
					x+=sizey/2; //×Ö·û,ÎªÈ«×ÖµÄÒ»°ë
				}
			  str++;
			}
		}
	}
}

/******************************************************************************
      º¯ÊýËµÃ÷£º¾ÓÖÐÏÔÊ¾Ò»¸ö×Ö·û´®,°üº¬ÖÐÓ¢ÎÄÏÔÊ¾
      Èë¿ÚÊý¾Ý£ºx,y :Æðµã×ø±ê
                c:Ìî³äµÄÑÕÉ«
								fc:Ç°ÖÃ»­±ÊÑÕÉ«
								bc:±³¾°ÑÕÉ«
								str :×Ö·û´®
								size:×ÖÌå´óÐ¡
								mode:Ä£Ê½	0,Ìî³äÄ£Ê½;1,µþ¼ÓÄ£Ê½
      ·µ»ØÖµ£º  ÎÞ
******************************************************************************/
void lcd_StrCenter(uint16_t x, uint16_t y,uint8_t *str,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint16_t len=strlen((const char *)str);
	uint16_t x1=(lcddev.width-len*8)/2;
	lcd_ShowStr(x+x1,y,str,fc,bc,sizey,mode);
}

void lcd_ShowOne(){
	  DrawTestPage("Arm Kit");
	  lcd_StrCenter(0,30,"BKIT HARDWARE CLUB",BRRED,RED,16,1);
	  lcd_ShowPicture(70,170,97,100,gImage_bk);
//	  LCD_ShowPicture(0,150,60,60,gImage_dino);
}

uint8_t shift_index = 0;
uint8_t test_count = 0;

void lcd_Test(){
	lcd_Fill(0, 120, lcddev.width, 140, WHITE);
	lcd_ShowString(shift_index,120,"Welcome",BLUE,BLUE,16,1);
	shift_index = (shift_index+1)%lcddev.width;

	test_count = (test_count+1)%20;
	if(test_count == 0){
		sprintf(s, "%02d:%02d:%02d", rtc_GetHour(), rtc_GetMin(), rtc_GetSec());
		lcd_Fill(0, 60, lcddev.width, 80, BLACK);
		lcd_StrCenter(0,60,(uint8_t*)s,WHITE,BLUE,16,1);
		sprintf(s, "Light: %d, Res: %d", adc_GetLight(), adc_GetVarResistor());
		lcd_Fill(0, 80, lcddev.width, 100, BLACK);
		lcd_StrCenter(0,80,(uint8_t*)s,WHITE,BLUE,16,1);
	}
}
