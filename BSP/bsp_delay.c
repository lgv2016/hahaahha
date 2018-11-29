#include <bsp_delay.h>

/*********************************************************************************************************
*	函 数 名: bsp_InitDWT
*	功能说明: 初始化DWT
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************/
void BSP_DWT_Init()
{
	DEM_CR         |= (unsigned int)DEM_CR_TRCENA;   /* Enable Cortex-M4's DWT CYCCNT reg.  */
	DWT_CYCCNT      = (unsigned int)0u;
	DWT_CR         |= (unsigned int)DWT_CR_CYCCNTENA;
}
