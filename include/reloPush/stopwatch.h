#include <chrono>
#include <string>

class stopWatch
{
	public:
		stopWatch(std::string name_in = "")
		{
			name = name_in;
			start = std::chrono::high_resolution_clock::now();
		}

		void stop()
		{
			end = std::chrono::high_resolution_clock::now();
		}

		size_t print_us()
		{
			auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

			std::cout << name << ": " <<  duration << " us" << std::endl;

			if (duration > 0)
				return duration;
			else
				return 0;
		}

		size_t get_measurement()
		{
			auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

			if (duration > 0)
				return duration;
			else
				return 0;		
		}

		size_t stop_and_get_us()
		{
			stop();
			return print_us();
		}

		std::string get_name()
		{
			return name;
		}

	private:
		std::string name;
		std::chrono::high_resolution_clock::time_point start;
		std::chrono::high_resolution_clock::time_point end;
};