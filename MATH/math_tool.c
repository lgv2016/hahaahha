#include <math_tool.h>

union FH_t
{
	char H[4];
	float F;
};
/**********************************************************************************************************
*函 数 名: constrainfloat
*功能说明: 浮点型限幅
*形    参: 输入值 限幅下限 限幅上限
*返 回 值: 限幅后的值
**********************************************************************************************************/
float Constrainfloat(float amt, float low, float high)
{
    if (isnan(amt))
    {
        return (low+high)*0.5f;
    }
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

/**********************************************************************************************************
*函 数 名: ApplyDeadbandfloat
*功能说明: 误差死区控制
*形    参: 输入值 死区
*返 回 值: 误差值
**********************************************************************************************************/

float ApplyDeadbandfloat(float value, float deadband)
{
    if (abs(value) < deadband)
        value = 0;
    return value;
}

/**********************************************************************************************************
*函 数 名: ApplyDeadbandfloat
*功能说明: 误差死区控制
*形    参: 输入值 死区
*返 回 值: 误差值
**********************************************************************************************************/

int32_t ApplyDeadbandInt(int32_t value, int32_t deadband)
{
    if (abs(value) < deadband)
        value = 0;
    return value;
}




/**********************************************************************************************************
*函 数 名: Radians
*功能说明: 角度转弧度
*形    参: 角度值
*返 回 值: 弧度值
**********************************************************************************************************/
float Radians(float deg)
{
    return deg * DEG_TO_RAD;
}

/**********************************************************************************************************
*函 数 名: Radians
*功能说明: 弧度转角度
*形    参: 弧度值
*返 回 值: 角度值
**********************************************************************************************************/
float Degrees(float rad)
{
    return rad * RAD_TO_DEG;
}

/****************************
* 功能    ：计算校验和
* 输入参数：pchMessage 数据数组
*           dwLength 数据长度
*  
* 输出参数：ucSUM 值
****************************/
u8 Get_Check_SUM(u8* pchMessage, u16 dwLength)
{
  u8 ucSUM=0;
  while (dwLength--)
  {
	 ucSUM+=(*pchMessage++);
  }
  return (ucSUM);
}
/**************************************
* 功能    ：SUM校验
* 输入参数：pchMessage 数据数组
*           dwLength 数据长度
*     
* 输出参数：1:正确 0：错误
***************************************/
u8 Verify_Check_SUM(u8* pchMessage, u16 dwLength)
{
  u8 ucExpected = 0;
  if ((pchMessage == 0) || (dwLength <= 2))
    return 0;
  ucExpected = Get_Check_SUM(pchMessage, dwLength - 1);
  return (ucExpected == pchMessage[dwLength - 1]);
}

/******************************
* 功能    ： 将校验和值放在数组的末尾
* 输入参数：pchMessage 数据数组
*           dwLength 数据长度+1（sum值存放在这里）
*******************************/
void Append_Check_SUM(u8* pchMessage, u16 dwLength)
{
  u8 ucSUM = 0;
  if ((pchMessage == 0) || (dwLength <= 2))
    return;
  ucSUM                    = Get_Check_SUM((u8*)pchMessage, dwLength - 1);
  pchMessage[dwLength - 1] = ucSUM;
}

/**************************************
* 功能    ：16进制转浮点数
* 输入参数：pchMessage 16进制数组
*          
*     
* 输出参数：转化后的浮点数
***************************************/
float HEX_TO_float(u8 * pchMessage)
{
	union FH_t FH;
	FH.H[0]=*(pchMessage+0);
	FH.H[1]=*(pchMessage+1);
	FH.H[2]=*(pchMessage+2);
    FH.H[3]=*(pchMessage+3);
	return FH.F;
}
/**************************************
* 功能    ：浮点数转16进制
* 输入参数：pchMessage 16进制数组
*           fdata      浮点数
*     
* 输出参数：无
***************************************/
void float_TO_Hex(u8 * pchMessage,float fdata)
{
	union FH_t FH;
	FH.F=fdata;
    *(pchMessage+0)= FH.H[0];
	*(pchMessage+1)= FH.H[1];
	*(pchMessage+2)= FH.H[2];
	*(pchMessage+3)= FH.H[3];
}

/**
  * @brief          一阶低通滤波初始化
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      滤波参数
  * @retval         返回空
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
  * @brief          一阶低通滤波计算
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @retval         返回空
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
        first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}


