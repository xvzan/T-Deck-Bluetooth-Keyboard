#ifndef KEYMAP_H
#define KEYMAP_H
#include <Arduino.h>

// 键定义结构体
struct Key
{
    uint8_t hid;
    uint8_t mod;
    char label;
};

// 矩阵大小
const int rowCount = 7;
const int colCount = 5;

// 普通键映射
extern const Key keymap_flat[35] PROGMEM;

// 符号键映射
extern const Key keymap_symbol_flat[35] PROGMEM;
#endif // KEYMAP_H