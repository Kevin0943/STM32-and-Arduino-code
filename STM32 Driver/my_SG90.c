#include "my_library.h"
#include "my_SG90.h"


/**
===== 簡述 =====
SG90 Library
需搭配my_library.c使用


===== 使用說明 =====
使用前需先執行SG90_Init()進行初始化

SG90由50Hz的PWM控制 (週期 20 ms)
PWM高電位時間對應角度：
0.5ms—————0度；
1.0ms—————45度；
1.5ms—————90度；
2.0ms—————135度；
2.5ms—————180度；

 */


volatile unsigned int HighTime = 1500;

// ===== SG90 Function =====
// --- SG90 初始化 ---
void SG90_Init(unsigned int angle) { // 輸入 angle：初始角度
    if(angle > 180) { angle = 180;} // 限制角度範圍
    angle = 180 - angle; // 反轉角度，使SG90隨angle增加順時鐘轉動

    // 計算高電平時間 (0.5ms ~ 2.5ms) (500us ~ 2500us)
    HighTime = 500 + ((2000 * angle) / 180); // 單位: us

    // PWM1 初始化，週期 PWM_Period us，頻率 50 Hz，高電位時間 HighTime us
    PWM1_Init(PWM_Period, HighTime);
    PWM1_Enable();
}

void SG90_ChangeAngle(unsigned int angle) { // 輸入 angle：改變角度
    if(angle > 180) { angle = 180;} // 限制角度範圍
    angle = 180 - angle; // 反轉角度，使SG90隨angle增加順時鐘轉動

    // 計算高電平時間 (0.5ms ~ 2.5ms) (500us ~ 2500us)
    HighTime = 500 + ((2000 * angle) / 180); // 單位: us

    // 設定PWM高電位時間為 HighTime us
    PWM1_ChangeHighTime(HighTime);
}
