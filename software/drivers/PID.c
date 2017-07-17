#include <rtthread.h>
#include "board.h"
#include "math.h"

int cnt = 0;


//定义最大与最小脉冲频率
#define MAX_FREQ 1500    //  Hz
#define MIN_FREQ 150      // Hz
#define PER_PUS  400
#define DELTA    50

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

extern volatile int16_t Motor_freq;
extern void Time_Pwm_Out_Mode(void);

//函数部分

static int pid_init(int16_t Kp,int16_t Ki,int16_t Kd)
{
	pid.Set=PER_PUS/2-2;
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

int pid_gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_5;
  GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;
	
	GPIO_Init(GPIOA,&GPIO_InitStruct);                    //PA5_init  --->方向控制
	GPIO_ResetBits(GPIOA,GPIO_Pin_5);  
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_3;
	GPIO_Init(GPIOA,&GPIO_InitStruct);                   //PA3_init   --->使能控制
	GPIO_ResetBits(GPIOA,GPIO_Pin_3);                    //使能电机
  
//	GPIOA->ODR|=((1<<5)|(1<<3));
	 pid_init(18,0,2);
	
	return 0;
}

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
	TIM_TimeBaseInitStruct.TIM_Period=20000-1;    									  
//	TIM_TimeBaseInitStruct.TIM_RepetitionCounter=0;   						//重复
	
	TIM_TimeBaseInit(TIM6,&TIM_TimeBaseInitStruct);
	
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM6,ENABLE);
	
	return 0;
}

///////////////
INIT_BOARD_EXPORT(pid_gpio_init);
INIT_DEVICE_EXPORT(Timer6_init);
///////////////



int PID_Ctrl(void)
{
	int32_t Motor_freq_tmp=0;
	
//	pid.Kp=Kp;
//	pid.Ki=Ki;
//	pid.Kd=Kd;
	GPIO_ResetBits(GPIOA,GPIO_Pin_3);
	
	pid.Actual=cnt;
	
	pid.err=pid.Set-pid.Actual;    //角偏量
	pid.integral+=pid.err;
	
	
	/*
	Motor_freq_tmp=pid.Kp*(pid.err-pid.err_next)+pid.Ki*pid.err+pid.Kd*(pid.err-2*pid.err_next+pid.err_last); //速度增量
	
//	Motor_freq=TIM14->ARR;
//	if(GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_5)) pid.Actual=-Motor_freq;
//  else pid.Actual=Motor_freq;	                                           //当前速度量
	
//	pid.Actual=Motor_freq_tmp;     //速度偏量
	
	
	if(Motor_freq_tmp > 0) GPIO_ResetBits(GPIOA,GPIO_Pin_5); 
	else GPIO_SetBits(GPIOA,GPIO_Pin_5); 
 
//  Motor_freq_tmp=pid.Actual;
	
//	rt_kprintf("Kp:%d Ki:%d Kd:%d err:%d err_next:%d err_last:%d detla:%d freq:%d Cnt:%d\n",(int)pid.Kp,(int)pid.Ki,(int)pid.Kd,(int)pid.err
//		,(int)pid.err_last,(int)pid.err_next,(int)(pid.err-pid.err_next),(int)Motor_freq,(int)(TIM1->CNT)%PER_PUS);
	
	pid.err_last=pid.err_next; 
	pid.err_next=pid.err; */    //增量式

  Motor_freq=pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);
	
	if(Motor_freq>0) GPIO_ResetBits(GPIOA,GPIO_Pin_5); 
	else GPIO_SetBits(GPIOA,GPIO_Pin_5); 
	pid.err_last=pid.err;
	

	
	Motor_freq=abs(Motor_freq);
	
   rt_kprintf("Kp:%d Ki:%d Kd:%d err:%d freq:%d Cnt:%d\n",(int)pid.Kp,(int)pid.Ki,(int)pid.Kd,(int)pid.err,(int)Motor_freq,(int)(TIM1->CNT)%PER_PUS);
	if(Motor_freq>MAX_FREQ) Motor_freq=MAX_FREQ;
	
	if(Motor_freq <= 50 && pid.err <= 2) return 1; //调整完成
	TIM_SetAutoreload(TIM14,(1000000/Motor_freq-1)/2);     //ARR--周期
//  TIM_SetCompare1(TIM14,(1000000/Motor_freq-1)/2);   //CRR
//  
	return 0;
}

void TIM6_DAC_IRQHandler(void)
{
//	static uint16_t tmp=0;
	
	/* enter interrupt */
    rt_interrupt_enter();
	
	 cnt=(TIM1->CNT)%PER_PUS;
	GPIO_SetBits(GPIOA,GPIO_Pin_3); 
//	if(tmp!=(TIM1->CNT))              //test脉冲个数
//	{
//		tmp=(int)TIM1->CNT;
//	  rt_kprintf("CNT:%d\n",tmp);
//		
//	}
	if( (cnt > (PER_PUS/2)-DELTA) && cnt < ((PER_PUS/2)+DELTA))
	{
//		   PID_Ctrl();
		if(PID_Ctrl())GPIO_SetBits(GPIOA,GPIO_Pin_3);    //锁机
	}
	TIM_ClearFlag(TIM6,TIM_FLAG_Update);
		
	/* leave interrupt */
    rt_interrupt_leave();
}

int Clear_cnt(void)
{
	rt_kprintf("CNT:%d\n",(int)TIM1->CNT);
	TIM1->CNT=0;
	
	return (int)TIM1->CNT;
}
	
#ifdef RT_USING_FINSH
#include <finsh.h>
FINSH_FUNCTION_EXPORT_ALIAS(pid_init,pid,set kp ki kd);
FINSH_FUNCTION_EXPORT_ALIAS(Clear_cnt,clear,clear cnt register);
FINSH_VAR_EXPORT(cnt,finsh_type_int,tim1 cnt registers);
#endif /* RT_USING_FINSH */
