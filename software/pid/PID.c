#include <rtthread.h>
#include "board.h"
#include "math.h"
#include "dmp.h"

#ifdef RT_USING_HWTIMER
#include <drivers/hwtimer.h>
#endif 

//定义最大与最小脉冲频率
#define MAX_PWM_DUTY 70    //  Hz

static struct pid_initstruct
	{
		int16_t Set;
		int16_t Actual;
		int16_t err;
		int16_t err_last;
		int16_t err_next;
		volatile int16_t Kp,Ki,Kd;
		int16_t integral;

	} pid;

//函数部分

int timer_device_init(void)
{
		rt_device_t dev;
		rt_hwtimer_mode_t rt_hwtimer_mode_t_tmp = HWTIMER_MODE_PERIOD;
		rt_uint32_t freq_tmp = 100000;
	
		/*get timer device handle*/
    dev = rt_device_find("timer0");
		/*open device*/
		rt_device_open(dev, RT_DEVICE_OFLAG_RDWR);
		/*set timer mode*/
		rt_device_control(dev, HWTIMER_CTRL_MODE_SET, &rt_hwtimer_mode_t_tmp);
		/*set timer period*/
		rt_device_control(dev, HWTIMER_CTRL_FREQ_SET, &freq_tmp);
	
	return 0;
}	
	
static int pid_init(int16_t Kp,int16_t Ki,int16_t Kd)
{
	pid.Set=0;
	pid.Actual=0;
	pid.err=0;
	pid.err_last=0;
	pid.err_next=0;
	pid.Kp=Kp;
	pid.Ki=Ki;
	pid.Kd=Kd;
	pid.integral=0;
	
	return 0;
}

int PID_Ctrl(int set,int actual)
{
	int32_t pwm_tmp=0;
	
	pid.Actual=actual;
	pid.Set=set;
	
	pid.err=pid.Set-pid.Actual;    //角偏量
	pid.integral+=pid.err;

  pwm_tmp=pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);
	
	pid.err_last=pid.err;
	
	return pwm_tmp;
}

/*
int Timer6_init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	
	NVIC_InitStruct.NVIC_IRQChannel=TIM6_DAC_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=3;
	NVIC_Init(&NVIC_InitStruct);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
	
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;		 //设置定时器频率CK_INT频率与采样时钟频率比,一般不需要
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Prescaler=84-1; 											//分频银因子确定
	TIM_TimeBaseInitStruct.TIM_Period=100000; 											//100ms   									  
//	TIM_TimeBaseInitStruct.TIM_RepetitionCounter=0;   						//重复
	
	TIM_TimeBaseInit(TIM6,&TIM_TimeBaseInitStruct);
	
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM6,ENABLE);
	
	return 0;
}

void TIM6_DAC_IRQHandler(void)
{
	struct euler_angle el;
	
	// enter interrupt 
    rt_interrupt_enter();
		
	
	 TIM_ClearFlag(TIM6,TIM_FLAG_Update);
		
	// leave interrupt 
    rt_interrupt_leave();
}

*/

	

