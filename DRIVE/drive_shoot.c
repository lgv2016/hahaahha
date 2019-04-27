#include <drive_shoot.h>
#include <drive_control.h>
#include <motor_cradle_head.h>
#include <robotstatus.h>
#include <drive_rc.h>


#define PRESS_LONG_TIME     (350/SHOOT_CONTROL_CYCLE)    //�������ʱ��
#define NO_PRESS_LONG_TIME  (150/SHOOT_CONTROL_CYCLE)    //���δ��ʱ��
#define BLOCK_LONG_TIME     (2000/SHOOT_CONTROL_CYCLE)   //��תʱ��
#define KEY_E_TIME          (100/SHOOT_CONTROL_CYCLE)    //E������ʱ��



#define RC_SW_UP   ((uint16_t)1)
#define RC_SW_MID  ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)


#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s)  (s == RC_SW_MID)
#define switch_is_up(s)   (s == RC_SW_UP)


static shoot_motor_t s_shoot_motor;

static void SHOOT_Set_Mode(void);


void SHOOT_Init()
{
	g_data_2006.angle_reset_flag=1;
	
	robot_status.shoot_mode=SHOOT_NO;
	robot_status.firc_status=FRIC_OFF;
}
/*
 *�� �� ��: SHOOT_Set_Mode
 *����˵��: ���״̬����
 *��    ��: void
 *�� �� ֵ: void
 */
static void SHOOT_Set_Mode()
{
	static u8 last_s=RC_SW_UP;
	
		
	 //�ϲ��жϣ� һ�ε���
	if((switch_is_up(g_rc_control.rc.s2)&&!switch_is_up(last_s))||(s_shoot_motor.press_l&&!s_shoot_motor.last_press_l))
	{
		if(robot_status.firc_status==FRIC_OFF&&s_shoot_motor.key_time==KEY_E_TIME)	
	    {
			robot_status.shoot_mode=SHOOT_READY;
			robot_status.firc_status=FRIC_ON_START;
			
	    }
		if(robot_status.firc_status==FRIC_ON)
		{
			robot_status.shoot_mode=SHOOT_AWM;
			g_data_2006.angle_reset_flag=1;
	        g_data_2006.count=0;
		}
	}
	
	//����
	else if((s_shoot_motor.press_l_time==PRESS_LONG_TIME)||(switch_is_down(g_rc_control.rc.s2)))
	{
		if(robot_status.firc_status==FRIC_OFF&&s_shoot_motor.key_time==KEY_E_TIME)
	    {
			robot_status.shoot_mode=SHOOT_READY;
			robot_status.firc_status=FRIC_ON_START;
	    }
		
		if(robot_status.firc_status==FRIC_ON)
		{
			robot_status.shoot_mode=SHOOT_AK47;
		}
	}
	
	else if((switch_is_mid(g_rc_control.rc.s2))&&s_shoot_motor.no_press_l_time==NO_PRESS_LONG_TIME)
	{
		if((robot_status.firc_status==FRIC_ON)&&(s_shoot_motor.key_time==KEY_E_TIME))
		{
			//��ʼ�ر�Ħ���� ֹͣ���
			//1 ���� 2 �ر�
			Cmd_GIMBAL_ESC(0,2);
			robot_status.firc_status=FRIC_OFF_START;
		}
		robot_status.shoot_mode=SHOOT_STOP;
	}
	
	//�����̨û��У׼���
    if(robot_status.gimbal_data!=GIMBAL_MOTOR_GYRO)
	{
		robot_status.shoot_mode=SHOOT_STOP;
	}
	last_s=g_rc_control.rc.s2;
}


/*
 *�� �� ��: SHOOT_Data_Update
 *����˵��: ����������ݸ���
 *��    ��: void
 *�� �� ֵ: void
 */

static void SHOOT_Data_Update()
{
	s_shoot_motor.last_press_l=s_shoot_motor.press_l;
	s_shoot_motor.press_l=g_rc_control.mouse.press_l;
	
	if(s_shoot_motor.press_l)
	{
		if(s_shoot_motor.press_l_time<PRESS_LONG_TIME)
		{
			s_shoot_motor.press_l_time++;
		}
		s_shoot_motor.no_press_l_time=0;
	}
	else
	{
		if(s_shoot_motor.no_press_l_time<NO_PRESS_LONG_TIME)
		{
			s_shoot_motor.no_press_l_time++;
		}
		s_shoot_motor.press_l_time=0;
	}
	
	if(g_rc_control.key.k[E])
	{
		if(s_shoot_motor.key_time<KEY_E_TIME)
		{
			s_shoot_motor.key_time++;
		}
	}
	
	else
	{
		s_shoot_motor.key_time=0;
	}
}
/*
 *�� �� ��: SHOOT_Stop_Control
 *����˵��: ʹ2006����
 *��    ��: void
 *�� �� ֵ: void
 */
static void SHOOT_Stop_Control()
{
	Cmd_2006_ESC(0);
}
/*
 *�� �� ��: SHOOT_AWM_Control
 *����˵��: �������������
 *��    ��: �����Ƕ�
 *�� �� ֵ: void
 */
static void SHOOT_AWM_Control(float set_angle)
{	
	g_angle_target.shoot=set_angle;
	Angle_2006_Control(g_angle_target);
}
/*
 *�� �� ��: Shoot_AK47_Control
 *����˵��: �����������
 *��    ��: �����ٶ�
 *�� �� ֵ: void
 */
static void SHOOT_AK47_Control(int16_t  fire_rate)
{
	g_speed_target.shoot=fire_rate;
	Speed_2006_Control(g_speed_target);	
}
/*
 *�� �� ��: Shoot_Ready_Control
 *����˵��: ���׼�����ƣ�����̨���ʹ�Ħ���ֵ�����
 *��    ��: void
 *�� �� ֵ: void
 */
static void SHOOT_Ready_Control()
{
	//��ʼ����Ħ����
	Cmd_GIMBAL_ESC(0,1);
}

/*
 *�� �� ��: SHOOT_Loop_Control
 *����˵��: ���ѭ������
 *��    ��: void
 *�� �� ֵ: void
 */
void SHOOT_Loop_Control()
{
	SHOOT_Set_Mode();
	SHOOT_Data_Update();
	
	if(robot_status.shoot_mode==SHOOT_AWM)
	{
		SHOOT_AWM_Control(60);
	}
	else if(robot_status.shoot_mode==SHOOT_AK47)
	{
		SHOOT_AK47_Control(1500);
	}
	else if(robot_status.shoot_mode==SHOOT_STOP)
	{
		SHOOT_Stop_Control();
	}
	else if(robot_status.shoot_mode==SHOOT_READY)
	{
		SHOOT_Ready_Control();
	}
}
/*
 *�� �� ��: GET_209_Data
 *����˵��: ��ȡ209����
 *��    ��: CAN���ݽṹ��
 *�� �� ֵ: void
 */
void GET_209_Data(CanRxMsg rx_message)
{
	if(rx_message.StdId==0x210)
	{
		if(rx_message.Data[0]==0x01)
			robot_status.firc_status=FRIC_ON;
		if(rx_message.Data[0]==0x02)
			robot_status.firc_status=FRIC_OFF;
	}
}

