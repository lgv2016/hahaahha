#include <math_tool.h>

union FH_t
{
	char H[4];
	float F;
};
/**********************************************************************************************************
*�� �� ��: constrainfloat
*����˵��: �������޷�
*��    ��: ����ֵ �޷����� �޷�����
*�� �� ֵ: �޷����ֵ
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
*�� �� ��: ApplyDeadbandfloat
*����˵��: �����������
*��    ��: ����ֵ ����
*�� �� ֵ: ���ֵ
**********************************************************************************************************/

float ApplyDeadbandfloat(float value, float deadband)
{
    if (abs(value) < deadband)
        value = 0;
    return value;
}

/**********************************************************************************************************
*�� �� ��: ApplyDeadbandfloat
*����˵��: �����������
*��    ��: ����ֵ ����
*�� �� ֵ: ���ֵ
**********************************************************************************************************/

int32_t ApplyDeadbandInt(int32_t value, int32_t deadband)
{
    if (abs(value) < deadband)
        value = 0;
    return value;
}




/**********************************************************************************************************
*�� �� ��: Radians
*����˵��: �Ƕ�ת����
*��    ��: �Ƕ�ֵ
*�� �� ֵ: ����ֵ
**********************************************************************************************************/
float Radians(float deg)
{
    return deg * DEG_TO_RAD;
}

/**********************************************************************************************************
*�� �� ��: Radians
*����˵��: ����ת�Ƕ�
*��    ��: ����ֵ
*�� �� ֵ: �Ƕ�ֵ
**********************************************************************************************************/
float Degrees(float rad)
{
    return rad * RAD_TO_DEG;
}

/****************************
* ����    ������У���
* ���������pchMessage ��������
*           dwLength ���ݳ���
*  
* ���������ucSUM ֵ
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
* ����    ��SUMУ��
* ���������pchMessage ��������
*           dwLength ���ݳ���
*     
* ���������1:��ȷ 0������
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
* ����    �� ��У���ֵ���������ĩβ
* ���������pchMessage ��������
*           dwLength ���ݳ���+1��sumֵ��������
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
* ����    ��16����ת������
* ���������pchMessage 16��������
*          
*     
* ���������ת����ĸ�����
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
* ����    ��������ת16����
* ���������pchMessage 16��������
*           fdata      ������
*     
* �����������
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
  * @brief          һ�׵�ͨ�˲���ʼ��
  * @author         RM
  * @param[in]      һ�׵�ͨ�˲��ṹ��
  * @param[in]      �����ʱ�䣬��λ s
  * @param[in]      �˲�����
  * @retval         ���ؿ�
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
  * @brief          һ�׵�ͨ�˲�����
  * @author         RM
  * @param[in]      һ�׵�ͨ�˲��ṹ��
  * @param[in]      �����ʱ�䣬��λ s
  * @retval         ���ؿ�
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
        first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}


