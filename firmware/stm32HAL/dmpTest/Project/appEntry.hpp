/*
 * appEntry.hpp
 *
 *  Created on: Dec 7, 2021
 *      Author: alan
 */

#ifndef INC_APPENTRY_HPP_
#define INC_APPENTRY_HPP_



#ifdef __cplusplus
	extern "C"
	{
#endif

	void app_entry (void);
	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#ifdef __cplusplus
	}
#endif





#endif /* INC_APPENTRY_HPP_ */
