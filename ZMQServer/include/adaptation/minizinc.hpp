#pragma once

#include <string>
#include <vector>

namespace miron {

class Minizinc {

public:

typedef struct DataPair
{
	std::string id;
	std::string value;
};

typedef struct Solution
{
	std::vector<DataPair> output;
	std::string msg;
};

private:
	std::string mzn_path_;
	std::string dzn_path_;

protected:

    /**
     * This method parses the output string resulted from a Minizinc solver
     * 
     * @param outputs parsing results
     */
	void ParseSolution(Solution* outputs);

public:

    /**
     * Construct a new Reasoner object. It instantiates a reasoner based 
     * on the Minizinc solver
     * 
     * @param mzn_path indicates the path of the *.mzn file
     */
	Minizinc(const std::string mzn_path);

    /**
     * Construct a new Reasoner object. It instantiates a reasoner based 
     * on the Minizinc solver
     * 
     * @param mzn_path indicates the path of the *.mzn file
     * @param dzn_path indicates the path of the *.dzn file
     */
	Minizinc(const std::string mzn_path, const std::string dzn_path);
    
    /**
     * @brief Destroy the Reasoner object
     */
	virtual ~Minizinc();

    /**
     * It runs the solver for a given input
     * 
     * @param inputs Parameters for the Minizinc solver
     * @param outputs Gives the configuration for variation points
     * @return int 0 when the execution is successful, otherwise -1
     */
	int Run(const std::vector<DataPair> inputs, Solution* outputs);

    /**
     * It runs the solver for a given input
     * 
     * @param inputs Parameters for the Minizinc solver
     * @param outputs Gives the configuration for variation points
     * @param display_msgs The results are display in the console when true
     * @return int 0 when the execution is successful, otherwise -1
     */
	int Run(const std::vector<DataPair> inputs, Solution* outputs, bool display_msgs);
};

}
