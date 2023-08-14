/*
 * gui.h
 *
 *  Created on: Aug 14, 2023
 *      Author: phamv
 */

#ifndef INC_GUI_H_
#define INC_GUI_H_

void GUI_DrawPoint(uint16_t x,uint16_t y,uint16_t color);
void _draw_circle_8(int xc, int yc, int x, int y, uint16_t c);
void gui_circle(int xc, int yc,uint16_t c,int r, int fill);
void Show_Str(uint16_t x, uint16_t y,uint8_t *str,uint16_t fc, uint16_t bc,uint8_t sizey,uint8_t mode);
void Gui_StrCenter(uint16_t x, uint16_t y,uint8_t *str,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);

#endif /* INC_GUI_H_ */
