/*
 * gui.c
 *
 *  Created on: Aug 14, 2023
 *      Author: phamv
 */

#include "lcd.h"
//#include "string.h"
#include "gui.h"

/******************************************************************************
      º¯ÊýËµÃ÷£ºÔÚÖ¸¶¨Î»ÖÃ»­µã
      Èë¿ÚÊý¾Ý£ºx,y »­µã×ø±ê
                color µãµÄÑÕÉ«
      ·µ»ØÖµ£º  ÎÞ
******************************************************************************/
void GUI_DrawPoint(uint16_t x,uint16_t y,uint16_t color)
{
	LCD_Address_Set(x,y,x,y);//ÉèÖÃ¹â±êÎ»ÖÃ
	LCD_WR_DATA(color);
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
	GUI_DrawPoint(xc + x, yc + y, c);

	GUI_DrawPoint(xc - x, yc + y, c);

	GUI_DrawPoint(xc + x, yc - y, c);

	GUI_DrawPoint(xc - x, yc - y, c);

	GUI_DrawPoint(xc + y, yc + x, c);

	GUI_DrawPoint(xc - y, yc + x, c);

	GUI_DrawPoint(xc + y, yc - x, c);

	GUI_DrawPoint(xc - y, yc - x, c);
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
void Show_Str(uint16_t x, uint16_t y,uint8_t *str,uint16_t fc, uint16_t bc,uint8_t sizey,uint8_t mode)
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
					LCD_ShowChar(x,y,*str,fc,bc,sizey,mode);
					x+=sizey/2; //×Ö·û,ÎªÈ«×ÖµÄÒ»°ë
				}
			  str++;
			}
		}else//ÖÐÎÄ
		{
			if(x>(lcddev.width-sizey)||y>(lcddev.height-sizey)) return;
			bHz=0;//ÓÐºº×Ö¿â
			if(sizey==32)
				LCD_ShowChinese32x32(x,y,str,fc,bc,sizey,mode);
			else if(sizey==24)
				LCD_ShowChinese24x24(x,y,str,fc,bc,sizey,mode);
			else if(sizey==16)
			  LCD_ShowChinese16x16(x,y,str,fc,bc,sizey,mode);
			else
			  LCD_ShowChinese12x12(x,y,str,fc,bc,sizey,mode);
				str+=2;
				x+=sizey;//ÏÂÒ»¸öºº×ÖÆ«ÒÆ
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
void Gui_StrCenter(uint16_t x, uint16_t y,uint8_t *str,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint16_t len=strlen((const char *)str);
	uint16_t x1=(lcddev.width-len*8)/2;
	Show_Str(x+x1,y,str,fc,bc,sizey,mode);
}

