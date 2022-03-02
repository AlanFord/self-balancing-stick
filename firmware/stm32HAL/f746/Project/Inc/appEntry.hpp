/*
 * appEntry.hpp
 *
 *  Created on: Dec 7, 2021
 *      Author: alan
 */

#ifndef INC_APPENTRY_HPP_
#define INC_APPENTRY_HPP_

//#include "hardware.hpp"
#include "controller.hpp"

Controller *rightController_ptr;
Controller *leftController_ptr;



#ifdef __cplusplus
extern "C"
{
#endif

void app_entry (void);

#ifdef __cplusplus
}
#endif




#endif /* INC_APPENTRY_HPP_ */
