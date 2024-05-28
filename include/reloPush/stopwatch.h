#include <chrono>
#include <string>
#include <vector>

enum measurement_type {graphPlan, pathPlan, relocatePlan, assign, allPlan};

class stopWatch
{
	public:
		stopWatch(std::string name_in = "", measurement_type type_in = measurement_type::allPlan)
		{
			name = name_in;
			type = type_in;
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

		measurement_type get_type()
		{
			return type;
		}

	private:
		std::string name;
		measurement_type type;
		std::chrono::high_resolution_clock::time_point start;
		std::chrono::high_resolution_clock::time_point end;
};

class stopWatchSet
{
	public:
		std::vector<stopWatch> watches;
		void print()
		{
			for(auto& it : watches)
			{
				std::cout << it.get_name() << ": " << it.get_measurement() << " us" << std::endl;
			}
		}
};