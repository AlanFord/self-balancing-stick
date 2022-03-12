/*
 * appEntry.hpp
 *
 *  Created on: Dec 7, 2021
 *      Author: alan
 */

#ifndef INC_APPENTRY_HPP_
#define INC_APPENTRY_HPP_

/*
 * IMPORTANT:
 * If changing mcs's or clock speeds, change the TIM5 prescaler.  The prescaler value of 71
 * is appropriate only for a clock speed of 72 MHz.
 */



#ifdef __cplusplus
extern "C"
{
#endif

void app_entry (void);

#ifdef __cplusplus
}
#endif




#endif /* INC_APPENTRY_HPP_ */
