#include "app_control.h"
#include "PID.h"
#include "app_mpu6050.h"
#include "arm_math.h"
#include "app_encoder.h"
extern float fast_atan2_deg(float y, float x);
static PID_TypeDef PID_veiocity;
extern uint32_t GetMicros(void); 
static PID_TypeDef PID_theta;
static PID_TypeDef PID_theta_dot;
static const float g = 9.8f;
static const float lp = 0.062;
static const float rw = 0.032;
extern void PID_change_SP(PID_TypeDef *PID, float SP);
extern float App_MPU6050_GetPitch(void);
static float omega_ref = 0.0f;
static uint64_t last_time = 0;
static float theta_offset = 0.0f;
static float gx_offset = 0.0f;
void App_Control_Init(void)
{
	PID_Init(&PID_veiocity, 10.0f, 1.0f, 0.0f);
    PID_LimitConfig(&PID_veiocity, 0.5*g, -0.5*g);
    PID_Init(&PID_theta, 4.0f, 0.0f, 0.0f);
    PID_LimitConfig(&PID_theta, 12.57f, -12.57f);
    PID_Init(&PID_theta_dot, 10.0f, 10.0f, 0.0f);
    PID_LimitConfig(&PID_theta_dot, 125.7f, -125.7f);
}

/* 修复(13)：按键启动时清零平衡状态并标定直立角。
   原因：omega_ref = ∫(ẍ/rw)dt 从上电就在跑。上电时车是躺着的（θ≈90°，
   cosθ≈0），ẍ = (g·sinθ − θ̈·l)/cosθ 会算出 ±inf，omega_ref 几毫秒内
   被顶到天文数字，按键后轮子满占空比只朝一个方向猛转、倾斜也改变不了。
   把按键瞬间的角度和 gx 记为零点：MPU 装歪几度、陀螺仪有零偏都能抵消。
   注意：按键时车必须扶稳、接近竖直。 */
void App_Control_Zero(void)
{
    theta_offset = App_MPU6050_GetPitch();
    gx_offset = App_Mpu6050_GetGx();
    omega_ref = 0.0f;
    last_time = 0;
    PID_reset(&PID_theta);
    PID_reset(&PID_theta_dot);
    PID_reset(&PID_veiocity);   /* 修复(19)：速度环积分也要在按键时清掉 */
}

void App_Control_Proc(void)
{
    static uint32_t nxt = 0;
    if(HAL_GetTick() < nxt)return;
    uint64_t now = GetMicros();
    float delta_t = (now - last_time)*1.0e-6f;
    /* 修复(12)：上电第一拍 last_time=0，delta_t 会是开机以来的秒数，
       把 omega_ref 一步顶成天文数字，按键后轮子猛冲；中途卡顿同理。
       间隔超过 5ms 一律按 1ms 算，防止积分跳变。 */
    if (delta_t > 0.005f)
    {
        delta_t = 0.001f;
    }
	PID_change_SP(&PID_veiocity, 0.0f);
	float omega = 0.5*(App_Encoder_GetSpeed_L()+App_Encoder_GetSpeed_R());
	float theta = (App_MPU6050_GetPitch() - theta_offset)*0.0174533f;
	float theta_dot = (App_Mpu6050_GetGx() - gx_offset)*0.0174533f;
	float omega2 = -theta_dot * (lp+rw)/rw;
	float omega1 = omega -omega2;
	float x_dot = omega1 * rw;
	
	/* 修复(19)：速度环原来没接上——theta_ref 送进了角速度环的 SP，
	   但两行后就被 theta_dot_ref 覆盖，角度环 SP 还写死 0，
	   速度环输出等于被完全丢弃（车照样漂）。
	   正确级联是 速度环→角度环：速度环输出 u（期望加速度）→
	   期望倾角 atan(u/g) → 角度环 SP。
	   顺带修三处：fast_atan2_deg(g,u) 参数反了（u=0 时算出 90°），应为 (u,g)；
	   fast_atan2_deg 返回角度制而 theta 是弧度，要乘 0.0174533；
	   PID_Compute 原来一周期调两次，第一次结果丢弃还多推一步积分状态，改一次。 */
	float u_vel = PID_Compute(&PID_veiocity, x_dot);
	float theta_ref = fast_atan2_deg(u_vel, g) * 0.0174533f;
    PID_change_SP(&PID_theta, theta_ref);
    /* 修复(18)：阻尼用 +gx，符号以串口实测为准（前倾 gx 为正，
       pitch 也用 +gx 积分，所以 θ̇ = +gx，两边符号一致）。
       修复16 的 -(gx) 符号反了，是反阻尼（往前倒反而往前推，立刻倒）；
       修复17 的 +gy 是横滚轴，俯仰时恒为 0，阻尼失效。 */


    float theta_dot_ref = PID_Compute(&PID_theta, theta);
    PID_change_SP(&PID_theta_dot, theta_dot_ref);

    float theta_dot_dot_ref = PID_Compute(&PID_theta_dot, theta_dot);
	

    float sin_theta, cos_theta;
    sin_theta = arm_sin_f32(theta);
    cos_theta = arm_cos_f32(theta);
    float x_dot_dot_ref = (g * sin_theta - theta_dot_dot_ref*lp)/cos_theta;
    /* 修复(13)：|θ|接近 90° 时 cosθ≈0，上式会除出 ±inf，必须夹住，
       否则 omega_ref 又被顶飞。±20 m/s² 超出正常平衡需求两倍以上，正常时碰不到。 */
    if (x_dot_dot_ref >  20.0f) x_dot_dot_ref =  20.0f;
    if (x_dot_dot_ref < -20.0f) x_dot_dot_ref = -20.0f;
	if(last_time != 0)
	{
		omega_ref += 1.0f/ rw * x_dot_dot_ref*delta_t;
	}
    
    /* 轮子空载顶速约 12 rad/s，指令超出 ±20 rad/s 没有意义，夹住防止累积。 */
    if (omega_ref >  20.0f) omega_ref =  20.0f;
    if (omega_ref < -20.0f) omega_ref = -20.0f;

    App_Motor_SetOmega_L(omega_ref);
    App_Motor_SetOmega_R(omega_ref);



    last_time = now;
    nxt += 1;
}
void app_rest(void)
{
	last_time = 0;
	omega_ref = 0;
	PID_reset(&PID_veiocity);
	PID_reset(&PID_theta);
	PID_reset(&PID_theta_dot);
}
