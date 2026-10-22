#include "app_mpu6050.h"
#include "math.h"
extern I2C_HandleTypeDef hi2c1;
static void reg_write(uint8_t reg, uint8_t data);
float ax,ay,az,gx,gy,gz,temperature;
static int16_t ax_raw,ay_raw,az_raw;
static float yaw,pitch,roll;

static void reg_write(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c1,0xd0,reg,1,&data,1,100);
}
void App_Mpu6050_Init(void)
{
    reg_write(0x6b,0x80);
    HAL_Delay(100);
    reg_write(0x6b,0x00);
    reg_write(0x1b,0x18);
    reg_write(0x1c,0x00);
}
void App_Mpu6050_Update(void)
{
    /* 修复(10)：原来是 14 次独立的单字节 I2C 读（7 个寄存器 × 高低字节）。
       400kHz 下每次事务约 0.2~0.3ms，14 次加起来 ≈4ms，正好把 Proc 里 nxt+=5
       的 5ms 周期吃光；而且每次超时给的是 100ms，MPU6050 一旦没接好，
       最坏 14×100ms = 1.4 秒卡死，1kHz 的 PID 直接废掉。
       0x3B~0x48 这 14 个字节地址是连续的，MPU6050 支持读时地址自增，
       所以一次连读就够，超时也收到 10ms。 */
    uint8_t b[14];
    if (HAL_I2C_Mem_Read(&hi2c1, 0xd0, 0x3b, 1, b, 14, 10) != HAL_OK)
    {
        return;   /* 读失败就保留上一帧数据，别让半截的乱值污染角度 */
    }

    // raw值改为文件级静态变量，Proc里直接用raw算角度，省掉三次浮点缩放乘法
    ax_raw = (int16_t) ((b[0]  << 8) | b[1]);
    ay_raw = (int16_t) ((b[2]  << 8) | b[3]);
    az_raw = (int16_t) ((b[4]  << 8) | b[5]);
    int16_t temperature_raw = (int16_t) ((b[6]  << 8) | b[7]);
    int16_t gx_raw = (int16_t) ((b[8]  << 8) | b[9]);
    int16_t gy_raw = (int16_t) ((b[10] << 8) | b[11]);
    int16_t gz_raw = (int16_t) ((b[12] << 8) | b[13]);
    ax = ax_raw * 6.1035e-5;
    ay = ay_raw * 6.1035e-5;
    az = az_raw * 6.1035e-5;
    temperature = temperature_raw / 333.87f + 21.0f;
    gx = gx_raw * 6.1035e-2;
    gy = gy_raw * 6.1035e-2;
    gz = gz_raw * 6.1035e-2;
}   
float App_Mpu6050_GetAx(void)
{
    return ax;
}
float App_Mpu6050_GetAy(void)
{
    return ay;
}
float App_Mpu6050_GetAz(void)
{
    return az;
}
float App_Mpu6050_GetGx(void)
{
    return gx;
}
float App_Mpu6050_GetGy(void)
{
    return gy;
}
float App_Mpu6050_GetGz(void)
{
    return gz;
}
float App_Mpu6050_GetTemperature(void)
{
    return temperature;
}

float App_MPU6050_GetYaw(void)
{
    return yaw;
}
float App_MPU6050_GetPitch(void)
{
    return pitch;
}
float App_MPU6050_GetRoll(void)
{
    return roll;
}




// 快速atan2，直接返回角度制，误差约±0.2°，比库函数atan2快约10倍
// 原理：|y|<=|x|时用多项式 atan(z) ≈ 45z + 15.64z(1-|z|) 近似（z=y/x）
//       |y|>|x|时先算atan(x/y)，再按象限折算成 90°-角度
static float fast_atan2_deg(float y, float x)
{
    float deg, r;
    if (fabsf(x) >= fabsf(y)) {
        if (x == 0.0f) return 0.0f;                 // x、y都为0时角度无定义
        r = y / x;                                  // |r| <= 1
        deg = r * (45.0f + 15.64f * (1.0f - fabsf(r)));
    } else {
        r = x / y;                                  // |r| <= 1
        deg = r * (45.0f + 15.64f * (1.0f - fabsf(r)));
        deg = (y > 0.0f) ? (90.0f - deg) : (-90.0f - deg);
    }
    return deg;
}

void App_MPU6050_Proc(void)
{
    static uint32_t nxt = 0;
    if(HAL_GetTick() < nxt)return;
    App_Mpu6050_Update();
    float yaw_g = yaw + gz * 0.005;
    float pitch_g = pitch + gy * 0.005;
    float roll_g = roll - gx * 0.005;
    
    // 用raw原始值算角度：三轴缩放系数相同，比值不变，结果和用ax/ay/az一致
    // int16_t转float是精确的（16位有效值 < float的24位尾数），不损失精度
    float pitch_a = fast_atan2_deg((float)ay_raw, (float)az_raw);
    float roll_a = fast_atan2_deg((float)ax_raw, (float)az_raw);
    yaw = yaw_g;
    pitch = pitch_g*0.95238 + (1-0.95238)*pitch_a;
    roll = roll_g*0.95238 + (1-0.95238)*roll_a;
    nxt += 5;
    
    
}
