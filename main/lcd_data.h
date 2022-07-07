
#include "font_data.h"

#ifndef INLINE
#define INLINE static inline
#endif

extern const font_data_t font_data[];
extern const int font_data_size;

static int _sort_(const void*a1,const void*a2){
    font_data_t *d1 = (font_data_t*)a1;
    font_data_t *d2 = (font_data_t*)a2;
    if(d1->code==0&&d2->code==0) return d1-d2;
    if(d1->code==0)return 1;
    if(d2->code==0)return -1;
    return d1->code-d2->code;
}

INLINE int lcd_pixel_wchar_ex(uint16_t*pixel,wchar_t ch,uint16_t color,int xoffset,int yoffset,int width,int height,int bpp){
    font_data_t find;
    find.code = ch;
    font_data_t *d = (font_data_t*)bsearch(&find,font_data,font_data_size/sizeof(font_data[0]),sizeof(font_data[0]),_sort_);
    int pitch;
    int mask = 0;
    if(bpp==8)pitch=width;
    else if(bpp==16)pitch=width*2;
    else if(bpp==24)pitch=width*3;
    else if(bpp==32)pitch=width*4;
    else{
        pitch = width*8;
        pitch += pitch%bpp;
        pitch /= bpp;       //8对齐
        mask = (1<<bpp) - 1;
    }
    //printf("%p\n",d);
    if(d){
        //绘制文字
        yoffset += d->y;
        xoffset += d->x;
        int x,y;
        for(y=0;y<d->h;y++){
            int screen_y = y+yoffset;
            if(screen_y<height){
                //屏幕y应该小于高度
                /*16bit的宽度*/
                uint16_t *p = (uint16_t*)(y * 2 + (uint8_t*)d->data);
                uint16_t bcolor = *p;
                uint16_t *line = (uint16_t*)((screen_y*pitch) + (uint8_t*)pixel);
                for(x=0;x<d->w;x++){
                    #if 1||defined(SSD1306)
                        /*
                        1306的问题是,
                        每个字节会从上到下8位8行,下一个字节是下一列 ,01表示第0个字节,第0位,1270表示第127字节,第0位

                        显示方式

                        ------|00|10|20|...|1270|
                        PAGE00|01|11|21|...|1271|
                        PAGE00|02|12|22|...|1272|
                        PAGE00|03|13|23|...|1273|
                        PAGE00|04|14|24|...|1274|
                        PAGE00|05|15|25|...|1275|
                        PAGE00|06|16|26|...|1276|
                        ------|07|17|27|...|1277|
                        PAGE01
                        ...

                        */
                        int screen_x = x+xoffset;

                        uint8_t *p = (uint8_t*)pixel;
                        int bit_offset = screen_y % 4;
                        int page_offset = (screen_y / 4) * 128; //每一页为128个字节
                        p += page_offset;

                        p += screen_x;                  //一列的位移为1字节

                        *p &= ~(3<<bit_offset*2);           //每一行需要偏移1bit
                        if(bcolor&1){
                            *p |= (color<<bit_offset*2);
                        }
                    #else
                        if(bcolor&1){
                            //显示颜色
                            int screen_x = x+xoffset;
                            if(screen_x<width){
                                if(pitch==1)
                                    line[screen_x] = color;
                                else if(pitch==2)
                                    ((uint8_t*)line)[screen_x] = color;
                                else if(pitch==3){
                                    ((uint8_t*)line)[screen_x*3 + 0] = color&0xff;
                                    ((uint8_t*)line)[screen_x*3 + 1] = color>>8;
                                    ((uint8_t*)line)[screen_x*3 + 2] = color>>16;
                                }
                                else if(pitch==4)
                                    ((uint32_t*)line)[screen_x] = color;
                                else{
                                    int offset = (screen_x * bpp) / 8;          //字节偏移
                                    int bit_offset = (screen_x * bpp) %8;       //位偏移
                                    int bit_mask = mask<<bit_offset;            //位掩码
                                    uint16_t*p = (uint16_t*)((uint8_t*)line + offset);
                                    *p &= ~bit_mask;                    //保留其他颜色
                                    *p |= (color&mask)<<bit_offset;     //设置颜色
                                }
                            }
                        }
                    #endif
                    bcolor >>= 1;
                }
            }
        }
        return d->w;
    }
    return 0;
}

INLINE int lcd_pixel_textw_ex(uint16_t*pixel,const wchar_t *text,uint16_t color,int xoffset,int yoffset,int width,int height,int bpp){
    int i = 0;
    int xoff = xoffset;
    while(text[i]){
        if(text[i]==' '){
            xoff += 10;
        }
        else{
            xoff += lcd_pixel_wchar_ex(pixel,text[i],color,xoff,yoffset,width,height,bpp) +2;
        }
        //printf("off:%d\n",xoff);
        i++;
    }
    return xoff - xoffset;
}

INLINE int lcd_pixel_wchar(uint16_t*pixel,wchar_t ch,uint16_t color,int xoffset,int yoffset,int width,int height){
    return lcd_pixel_wchar_ex(pixel,ch,color,xoffset,yoffset,width,height,16);
}
