#include <rtthread.h>
#include "board.h"
#include "finsh.h"

volatile int16_t Motor_freq=25000;       //电机脉冲频率  

#define MAX_PWM_DUTY 80

static void Timer_Gpio_Init(void);      //定时器引脚初始化
static void Timer_Pwm_Out_Mode(void);   //频率脉冲发生
static void Timer_Encode_Mode(void);     //编码器初始化
static void Zero_Clear(void);
static void Timer4_CH1to4_Config(void);

int stm32_hw_timer_init(void);   

int PWM1_Set(uint8_t num)
{
	if(num<=MAX_PWM_DUTY)
		TIM_SetCompare1(TIM4,(int)((1000000/Motor_freq-1)*(num*0.01)));
	else rt_kprintf("input invalid!");
	return 0;
}
int PWM2_Set(uint8_t num)
{
	if(num<=MAX_PWM_DUTY)
		TIM_SetCompare2(TIM4,(int)((1000000/Motor_freq-1)*(num*0.01)));
	else rt_kprintf("input invalid!");
	return 0;
}
int PWM3_Set(uint8_t num)
{
	if(num<=MAX_PWM_DUTY)
		TIM_SetCompare3(TIM4,(int)((1000000/Motor_freq-1)*(num*0.01)));
	else rt_kprintf("input invalid!");
	return 0;
}
int PWM4_Set(uint8_t num)
{
	if(num<=MAX_PWM_DUTY)
		TIM_SetCompare4(TIM4,(int)((1000000/Motor_freq-1)*(num*0.01)));
	else rt_kprintf("input invalid!");
	return 0;
}
  

INIT_BOARD_EXPORT(stm32_hw_timer_init);    
FINSH_FUNCTION_EXPORT_ALIAS(PWM1_Set,pwm1,set pwm1 duty percent);
FINSH_FUNCTION_EXPORT_ALIAS(PWM2_Set,pwm2,set pwm2 duty percent);
FINSH_FUNCTION_EXPORT_ALIAS(PWM3_Set,pwm3,set pwm3 duty percent);
FINSH_FUNCTION_EXPORT_ALIAS(PWM4_Set,pwm4,set pwm3 duty percent);

int stm32_hw_timer_init(void)
{
	Timer_Gpio_Init();
	Timer4_CH1to4_Config();
//	Timer_Pwm_Out_Mode();
//	Timer_Encode_Mode();
//	Zero_Clear();
	
	return 0;
}

/*中断服务函数*/

void TIM8_TRG_COM_TIM14_IRQHandler(void)
{	
		/* enter interrupt */
    rt_interrupt_enter();
	
	  GPIO_ToggleBits(GPIOA,GPIO_Pin_7);

		
	
	  TIM_ClearFlag(TIM14,TIM_FLAG_Update);
	
		/* leave interrupt */
    rt_interrupt_leave();
}

void EXTI0_IRQHandler(void)
{
	/* enter interrupt */
    rt_interrupt_enter();

	GPIO_ToggleBits(GPIOD,GPIO_Pin_12);
	
	TIM1->CNT=0;
	EXTI_ClearITPendingBit(EXTI_Line0);
 

	/* leave interrupt */
    rt_interrupt_leave();
}

static void Timer_Gpio_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	
	
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1);      //TIM1_CH2--->PE11
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1);       //TIM1_CH1 --->PE9
	
	//TIM14_init
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_7;   										//PWM  TIM14
  GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;       						//    <--电平翻转要修改为输出模式
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM14);  
	
	 //LED_init
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_12;                  
	GPIO_Init(GPIOD,&GPIO_InitStruct);
	GPIO_ResetBits(GPIOD,GPIO_Pin_12);
	
	//TIM4_CH1~4_init
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;   
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;
	
	GPIO_Init(GPIOD,&GPIO_InitStruct);
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);    //PD12 --> CH1
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);    //PD13 --> CH2
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4);    //PD14 --> CH3
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_TIM4);    //PD15 --> CH4
	
	///////编码器////////
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_11;
  GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;
//	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_UP;                //必须上拉
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;
//	
//	GPIOE->ODR|=((1<<9)|(1<<11));
	
	GPIO_Init(GPIOE,&GPIO_InitStruct);                    //PE11_init
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_9;                 //PE9_init
	GPIO_Init(GPIOE,&GPIO_InitStruct);       
	/////////////////////////

  GPIO_InitStruct.GPIO_Pin=GPIO_Pin_0;                 //PE7_init    -->清零控制
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN;
	GPIO_Init(GPIOB,&GPIO_InitStruct);       
	
}
/*
外部中断配置注意事项：
1.打开SYSCFG时钟源，把引脚接入外部中断
2.外部中断引脚是分组的，如：PA0、PB0...对应EXTI_line0
3.PB2引脚为BOOT1引脚，不适合作为输入引脚
4.系统时钟源SYSCFG不开的话，无法对EXTICR寄存器进行操作，默认以PA端口为中断引脚(寄存器默认值为0)
*/


static void Zero_Clear(void)
{
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);   //使能系统时钟
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource0);   //Selects the PE7 used as EXIT Line;
//	SYSCFG->EXTICR[0]=1<<8;
	
	EXTI_InitStruct.EXTI_Line=EXTI_Line0;   
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt; 
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Falling;
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;
	
	EXTI_Init(&EXTI_InitStruct);
		
	NVIC_InitStruct.NVIC_IRQChannel=EXTI0_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=2;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	
	
}
/*
  //中断引脚电平翻转产生PWM
static void Timer_Pwm_Out_Mode2(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
//	TIM_OCInitTypeDef TIM_OCInitStruct;
	
	NVIC_InitStruct.NVIC_IRQChannel=TIM8_TRG_COM_TIM14_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=2;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14,ENABLE);
	
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;		 //设置定时器频率CK_INT频率与采样时钟频率比,一般不需要
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Prescaler=83; 											//分频银因子确定
	TIM_TimeBaseInitStruct.TIM_Period=(1000000/Motor_freq-1)/2;       //1K
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter=0;   						//重复
	
	TIM_TimeBaseInit(TIM14,&TIM_TimeBaseInitStruct);
	
	TIM_ITConfig(TIM14,TIM_IT_Update,ENABLE);                    //使能中断
	
	TIM_Cmd(TIM14,ENABLE);
}*/

   //硬件产生PWM
static void Timer_Pwm_Out_Mode(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14,ENABLE);
	
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;		 //设置定时器频率CK_INT频率与采样时钟频率比,一般不需要
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Prescaler=83; 											//分频银因子确定
	TIM_TimeBaseInitStruct.TIM_Period=1000000/Motor_freq-1;       //1K
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter=0;   						//重复
	
	TIM_TimeBaseInit(TIM14,&TIM_TimeBaseInitStruct);
	
	
	TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High;   //指定有效电平
	TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;
   TIM_OCInitStruct.TIM_Pulse=(1000000/Motor_freq-1)/2;
//	TIM_OCInitStruct.TIM_OCIdleState=TIM_OCIdleState_Reset;   //制定空闲状态，TIM1/8有效
//	TIM_OCInitStruct.TIM_OCNIdleState=TIM_OCNIdleState_Set;   //制定空闲状态，TIM1/8有效
//	TIM_OCInitStruct.TIM_OCNPolarity=TIM_OCNPolarity_High;   
//	TIM_OCInitStruct.TIM_OutputNState=TIM_OutputNState_Disable ; //TIM1/8有效
//	
	TIM_OC1Init(TIM14,&TIM_OCInitStruct);
	
	TIM_OC1PreloadConfig(TIM14,TIM_OCPreload_Enable);    //时能CCR比较通道，重点！！！
	TIM_ARRPreloadConfig(TIM14,ENABLE);
	
	TIM_Cmd(TIM14,ENABLE);

}
static void Timer4_CH1to4_Config(void)
{
		TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
		TIM_OCInitTypeDef TIM_OCInitStruct;
		

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
		
		TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;		 //设置定时器频率CK_INT频率与采样时钟频率比,一般不需要
		TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
		TIM_TimeBaseInitStruct.TIM_Prescaler=83; 											//分频银因子确定
		TIM_TimeBaseInitStruct.TIM_Period=1000000/Motor_freq-1;       //1K
		TIM_TimeBaseInitStruct.TIM_RepetitionCounter=0;   						//重复
		
		TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStruct);
		
		TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
		TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High;   //指定有效电平
		TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;
		TIM_OCInitStruct.TIM_Pulse=(1000000/Motor_freq-1)*0;

		TIM_OC1Init(TIM4,&TIM_OCInitStruct);
		TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable);    //时能CCR比较通道，重点！！！
		TIM_OC2Init(TIM4,&TIM_OCInitStruct);
		TIM_OC2PreloadConfig(TIM4,TIM_OCPreload_Enable);    //时能CCR比较通道，重点！！！
		TIM_OC3Init(TIM4,&TIM_OCInitStruct);
		TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Enable);    //时能CCR比较通道，重点！！！
		TIM_OC4Init(TIM4,&TIM_OCInitStruct);
		TIM_OC4PreloadConfig(TIM4,TIM_OCPreload_Enable);    //时能CCR比较通道，重点！！！
		
		TIM_ARRPreloadConfig(TIM4,ENABLE);
		
		TIM_Cmd(TIM4,ENABLE);
}

static void Timer_Encode_Mode(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_ICInitTypeDef TIM_ICInitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;		 //设置定时器频率CK_INT频率与采样时钟频率比,一般不需要
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Prescaler=9; 											//分频银因子确定
	TIM_TimeBaseInitStruct.TIM_Period=40000;       //1K
//	TIM_TimeBaseInitStruct.TIM_RepetitionCounter=0;   						//重复
	
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStruct);
	
	TIM_ICInitStruct.TIM_Channel=TIM_Channel_1;
	TIM_ICInitStruct.TIM_ICFilter=0;      
	TIM_ICInitStruct.TIM_ICPolarity=TIM_ICPolarity_Rising;
	TIM_ICInitStruct.TIM_ICPrescaler=TIM_ICPSC_DIV1;
	TIM_ICInitStruct.TIM_ICSelection=TIM_ICSelection_DirectTI;
	
	TIM_ICInit(TIM1,&TIM_ICInitStruct);              //初始化输入CH1通道
	
	TIM_ICInitStruct.TIM_Channel=TIM_Channel_2;      //初始化输入CH2通道
	
	TIM_ICInit(TIM1,&TIM_ICInitStruct);
	TIM1->SMCR |= 3<<0;	
//	TIM_EncoderInterfaceConfig(TIM1,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
	
	TIM_Cmd(TIM1,ENABLE);
}