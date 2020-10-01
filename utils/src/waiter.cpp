#include "waiter.hpp"
#include <thread>

namespace utils
{
	void Waiter::SleepMicros(const std::chrono::milliseconds &micros)
	{
		std::thread p([&, micros] { std::this_thread::sleep_for(micros); });
		p.join();
	}

	void Waiter::SleepMicros(long micros)
	{
		std::thread p([&, micros] { std::this_thread::sleep_for(std::chrono::microseconds(micros)); });
		p.join();
	}

	Waiter::Waiter() {}

	Waiter::~Waiter() {}

	void Waiter::SleepNanos(const std::chrono::nanoseconds& nanos)
	{
		std::thread p([&, nanos] { std::this_thread::sleep_for(nanos); });
		p.join();
	}

	void Waiter::SleepNanos(long nanos)
	{
		std::thread p([&, nanos] { std::this_thread::sleep_for(std::chrono::nanoseconds(nanos)); });
		p.join();
	}

	void Waiter::SleepMillis(const std::chrono::milliseconds& millis)
	{
		std::thread p([&, millis] { std::this_thread::sleep_for(millis); });
		p.join();
	}

	void Waiter::SleepMillis(long millis)
	{
		std::thread p([&, millis] { std::this_thread::sleep_for(std::chrono::milliseconds(millis)); });
		p.join();
	}

	void Waiter::SleepSecs(const std::chrono::seconds& secs)
	{
		std::thread p([&, secs] { std::this_thread::sleep_for(secs); });
		p.join();
	}

	void Waiter::SleepSecs(long secs)
	{
		std::thread p([&, secs] { std::this_thread::sleep_for(std::chrono::seconds(secs)); });
		p.join();
	}
} // namespace utils