/*
 * lcd.h
 *
 *  Created on: Aug 9, 2023
 *      Author: phamv
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "global.h"

#define DFT_SCAN_DIR  U2D_R2L  //Ĭ�ϵ�ɨ�跽��

//ɨ�跽����
#define L2R_U2D  0x00 //������,���ϵ��£�����
#define L2R_D2U  0x80 //������,���µ���
#define R2L_U2D  0x40 //���ҵ���,���ϵ���
#define R2L_D2U  0xc0 //���ҵ���,���µ��ϣ���ת180�ȣ�

#define U2D_L2R  0x20 //���ϵ���,������
#define U2D_R2L  0x60 //���ϵ���,���ҵ�����ת90�ȣ�
#define D2U_L2R  0xa0 //���µ���,�����ң���ת270�ȣ�
#define D2U_R2L  0xe0 //���µ���,���ҵ���

//LCD��Ҫ������
typedef struct
{
	uint16_t width;			//LCD ����
	uint16_t height;			//LCD �߶�
	uint16_t id;				  //LCD ID
}_lcd_dev;

//LCD����
extern _lcd_dev lcddev;	//����LCD��Ҫ����

//////////////////////////////////////////////////////////////////////////////////
//-----------------LCD�˿ڶ���----------------
//#define	LCD_LED PBout(15) //LCD����  PB15
//LCD��ַ�ṹ��
typedef struct
{
	__IO uint16_t LCD_REG;
	__IO uint16_t LCD_RAM;
} LCD_TypeDef;
//ʹ��NOR/SRAM�� Bank1.sector4,��ַλHADDR[27,26]=11 A10��Ϊ��������������
//ע������ʱSTM32�ڲ�������һλ����!
#define LCD_BASE        ((uint32_t)(0x60000000 | 0x000ffffe))
#define LCD             ((LCD_TypeDef *) LCD_BASE)
//////////////////////////////////////////////////////////////////////////////////

//������ɫ
#define WHITE         	 0xFFFF
#define BLACK         	 0x0000
#define BLUE         	 0x001F
#define BRED             0XF81F
#define GRED 			 0XFFE0
#define GBLUE			 0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			 0XBC40 //��ɫ
#define BRRED 			 0XFC07 //�غ�ɫ
#define GRAY  			 0X8430 //��ɫ
//GUI��ɫ

#define DARKBLUE      	 0X01CF	//����ɫ
#define LIGHTBLUE      	 0X7D7C	//ǳ��ɫ
#define GRAYBLUE       	 0X5458 //����ɫ
//������ɫΪPANEL����ɫ

#define LIGHTGREEN     	 0X841F //ǳ��ɫ
//#define LIGHTGRAY        0XEF5B //ǳ��ɫ(PANNEL)
#define LGRAY 			 0XC618 //ǳ��ɫ(PANNEL),���屳��ɫ

#define LGRAYBLUE        0XA651 //ǳ����ɫ(�м����ɫ)
#define LBBLUE           0X2B12 //ǳ����ɫ(ѡ����Ŀ�ķ�ɫ)

void lcd_SetCursor(uint16_t x,uint16_t y);//���ù��λ��
void lcd_AddressSet(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);//�������꺯��
void lcd_DisplayOn(void);//����ʾ
void lcd_DisplayOff(void);//����ʾ
uint16_t lcd_ReadPoint(uint16_t x,uint16_t y);//���㺯��
void lcd_Clear(uint16_t color);//��������

void lcd_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color);//ָ�����������ɫ
void lcd_DrawPoint(uint16_t x,uint16_t y,uint16_t color);//��ָ��λ�û�һ����
void lcd_DrawLine(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint16_t color);//��ָ��λ�û�һ����
void lcd_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t color);//��ָ��λ�û�һ������
void lcd_DrawCircle(uint16_t x0,uint16_t y0,uint8_t r,uint16_t color);//��ָ��λ�û�һ��Բ

void lcd_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);//��ʾһ���ַ�
void lcd_ShowString(uint16_t x,uint16_t y,const uint8_t *p,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);//��ʾ�ַ���
uint32_t mypow(uint8_t m,uint8_t n);//����
void lcd_ShowIntNum(uint16_t x,uint16_t y,uint16_t num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey);//��ʾ��������
void lcd_ShowFloatNum1(uint16_t x,uint16_t y,float num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey);//��ʾ��λС������

void lcd_ShowPicture(uint16_t x,uint16_t y,uint16_t length,uint16_t width,const uint8_t pic[]);//��ʾͼƬ

void lcd_SetDir(uint8_t dir);
void lcd_init(void);													   	//��ʼ��

void _draw_circle_8(int xc, int yc, int x, int y, uint16_t c);
void gui_circle(int xc, int yc,uint16_t c,int r, int fill);
void lcd_ShowStr(uint16_t x, uint16_t y,uint8_t *str,uint16_t fc, uint16_t bc,uint8_t sizey,uint8_t mode);
void lcd_StrCenter(uint16_t x, uint16_t y,uint8_t *str,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);

void lcd_ShowOne();

void lcd_Test();
#endif /* INC_LCD_H_ */
