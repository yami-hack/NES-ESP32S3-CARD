//input font :SourceHanSansCN-Light.otf
//output size:16
#include <stdint.h>
typedef struct{
  uint16_t code;
  int8_t x,y;
  int8_t w,h;
  uint8_t data[0x20];
}font_data_t;
