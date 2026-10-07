// mrs_port2021: converts a 2021 E-puck task set (.dat) to a timeline (.mrsl) for UAVs (spec 09 §7).
// Usage: mrs_port2021 <TaskSetN.dat> [altitude_m]   The timeline goes to standard output.
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include "mrs/algorithms/TaskIssuer.h"

int main(int argc, char** argv) {
	if (argc < 2) {
		std::cerr << "usage: mrs_port2021 <TaskSetN.dat> [altitude_m]\n";
		return 2;
	}
	std::ifstream in(argv[1], std::ios::binary);
	if (!in) {
		std::cerr << "cannot open " << argv[1] << "\n";
		return 2;
	}
	std::stringstream text;
	text << in.rdbuf();
	MRS::Algorithms::Port2021Config c;
	if (argc > 2) c.altitude = std::atof(argv[2]);
	try {
		std::cout << MRS::Algorithms::Port2021TaskSet(text.str(), c);
	} catch (const std::exception& e) {
		std::cerr << e.what() << "\n";
		return 1;
	}
	return 0;
}
