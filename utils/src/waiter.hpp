#include <chrono>


#ifndef UTILS_WAITER_H
#define UTILS_WAITER_H

namespace utils {

	class Waiter {

	public:
		/**
		* Method that lets a new thread sleep for a certain amount of nanoseconds
		* @param nanos: the amount of nanoseconds for the thread sleep
		*/
		static void SleepNanos(const std::chrono::nanoseconds nanos);

		/**
		* Method that lets a new thread sleep for a certain amount of nanoseconds
		* @param nanos: the amount of nanoseconds for the thread sleep
		*/
		static void SleepNanos(const long nanos);

		/**
		* Method that lets a new thread sleep for a certain amount of milliseconds
		* @param millis: the amount of milliseconds for the thread sleep
		*/
		static void SleepMillis(const std::chrono::milliseconds millis);

		/**
		* Method that lets a new thread sleep for a certain amount of milliseconds
		* @param millis: the amount of milliseconds for the thread sleep
		*/
		static void SleepMillis(const long millis);

		/**
		* Method that lets a new thread sleep for a certain amount of seconds
		* @param secs: the amount of seconds for the thread sleep
		*/
		static void SleepSecs(const std::chrono::seconds secs);

		/**
		* Method that lets a new thread sleep for a certain amount of seconds
		* @param secs: the amount of seconds for the thread sleep
		*/
		static void SleepSecs(const long secs);

	private:
		/**
		 * Constructor; static class by private Constructor & Deconstructor
		 */
		Waiter();
		~Waiter();
	};
}


#endif