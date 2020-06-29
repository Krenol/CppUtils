#include "waiter.hpp"
#include <thread>


utils::Waiter::Waiter() {}

utils::Waiter::~Waiter() {}

void utils::Waiter::SleepNanos(const std::chrono::nanoseconds nanos)
{
	std::thread p([&, nanos] {std::this_thread::sleep_for(nanos); });
	p.join();
}

void utils::Waiter::SleepNanos(const long nanos)
{
	std::thread p([&, nanos] {std::this_thread::sleep_for(std::chrono::nanoseconds(nanos)); });
	p.join();
}

void utils::Waiter::SleepMillis(const std::chrono::milliseconds millis)
{
	std::thread p([&, millis] {std::this_thread::sleep_for(millis); });
	p.join();
}

void utils::Waiter::SleepMillis(const long millis)
{
	std::thread p([&, millis] {std::this_thread::sleep_for(std::chrono::milliseconds(millis)); });
	p.join();
}

void utils::Waiter::SleepSecs(const std::chrono::seconds secs)
{
	std::thread p([&, secs] {std::this_thread::sleep_for(secs); });
	p.join();
}

void utils::Waiter::SleepSecs(const long secs)
{
	std::thread p([&, secs] {std::this_thread::sleep_for(std::chrono::seconds(secs)); });
	p.join();
}
