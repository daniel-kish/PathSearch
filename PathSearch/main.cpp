#include <iostream>
#include "Obstacles.h"
#include <fstream>

int main(int argc, char** argv)
try{
	using namespace std;
	using namespace std::literals;
	using Node = Graph::Node;
	
	if (argc < 6) {
		std::cerr << "generator: usage: 'gen' W H Nx Ny file\n";
		std::exit(1);
	}

	double wid = std::stod(argv[1]);
	if (wid < 0.0) {
		std::cerr << "generator: error: W < 0\n";
		std::exit(1);
	}
	double height = std::stod(argv[2]);
	if (height < 0.0) {
		std::cerr << "generator: error: H < 0\n";
		std::exit(1);
	}

	Rect r({wid,height});

	int nx = std::stoi(argv[3]);
	if (nx < 0) {
		std::cerr << "generator: error: Nx < 0\n";
		std::exit(1);
	}
	int ny = std::stoi(argv[4]);
	if (ny < 0) {
		std::cerr << "generator: error: Ny < 0\n";
		std::exit(1);
	}

	std::ofstream os(argv[5]);
	if (!os.is_open()) {
		std::cerr << "generator: error: can't open file: " << argv[5] << '\n';
		std::exit(1);
	}

	ObstacleGrower grower(r, nx, ny, 0.7f);
	
	std::cout << "current max_tree_lvl: " << grower.max_tree_lvl << '\n';
	std::cout << "current probability balance: " << grower.probability_balance << '\n';

	while (true) {
		char c;
		std::cin >> c;
		if (c == 'e') // enough
		{
			std::vector<Poly> polys;
			for (int i = 0; i < grower.sets.size(); ++i)
				polys.push_back(grower.getPoly(i));
			writePoly(os, polys);
			return 0;
		}
		else if (c == 'c') {
			int lvl; float prob;
			std::cin >> lvl >> prob;
			grower.max_tree_lvl = lvl;
			grower.probability_balance = prob;
		}
		grower.growObstacle();
		std::cout << grower.density() << '\n';
	}
}
catch (...)
{
	std::cerr << "Something went wrong. Try changing the parameters slightly\n";
}