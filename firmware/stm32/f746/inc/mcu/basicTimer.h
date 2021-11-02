
#include <common.h>
#include <nodate.h>

#include <functional>

//TODO: TIM5 is missing from rcc.h
enum BasicTimerDevice_t {
	TIMER_1,
	TIMER_2,
	TIMER_3,
	TIMER_4,
	TIMER_5,
	TIMER_6,
	TIMER_7,
	TIMER_8
};

struct Basic_timer_device {
	bool active = false;
	TIM_TypeDef* regs;
	RccPeripheral per;
	std::function<void(uint8_t)> callback;
}

class BasicTimer {
	
public:
	static bool BasicTimer::initialize(BasicTimerDevice_t device);
	static bool BasicTimer::start(BasicTimerDevice_t device);
	static bool BasicTimer::stop(BasicTimerDevice_t device);
	static bool BasicTimer::setInterrupt(BasicTimerDevice_t device, std::function<void(uint8_t)> callback);
	static bool BasicTimer::clearInterrupt(BasicTimerDevice_t device);
	static uint32_t BasicTimer::get_counter(BasicTimerDevice_t device);
	static bool BasicTimer::reset_counter(BasicTimerDevice_t device);
private:
}
