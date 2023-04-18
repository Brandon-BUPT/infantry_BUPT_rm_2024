#include "struct_typedef.h"
//第一版本测试串口通信和自瞄
//摄像机图像640*480
typedef struct 
{
    int16_t x_offset;       //识别出的装甲板中心相对于画面中心的x距离
                            //装甲板在左边，为负；装甲板在右边，为正
    int16_t y_offset;       //识别出的装甲板中心相对于画面中心的y距离。
                            //装甲板在下边，为负；装甲板在上边，为正
    uint16_t max_x_size;    //装甲板的轮廓的最大x距离
    uint16_t max_y_size;    //装甲板的轮廓的最大y距离
} toSTM32_naive_t;
