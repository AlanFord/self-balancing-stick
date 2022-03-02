/*
 * filters.cpp
 *
 *  Created on: Feb 26, 2022
 *      Author: alan
 */

#include "filters.hpp"

	ExponentialFilter::ExponentialFilter(float alphaPrime, float initialValue) {
		this->alphaPrime = alphaPrime;
		filteredValue = 0.0;
	}

	void ExponentialFilter::initialize(float initialValue = 0.0){
		filteredValue = initialValue;
	}

	float ExponentialFilter::filter(float dataPoint){
		filteredValue = (1 - alphaPrime) * dataPoint
				+ alphaPrime * filteredValue;
		return filteredValue;
	}




