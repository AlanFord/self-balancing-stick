/*
 * filters.hpp
 *
 *  Created on: Feb 26, 2022
 *      Author: alan
 */

#ifndef INC_FILTERS_HPP_
#define INC_FILTERS_HPP_

class ExponentialFilter {
	float alphaPrime;
	float filteredValue;
public:
	float filter(float dataPoint);
	ExponentialFilter(float alphaPrime, float initialValue = 0.0);
	void initialize(float initialValue);
};



#endif /* INC_FILTERS_HPP_ */
