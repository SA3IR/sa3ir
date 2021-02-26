#include "adaptation/minizinc.hpp"
#include <iostream>

namespace miron {


Minizinc::Minizinc(const std::string mzn_path) : Minizinc(mzn_path, NULL) 
{}


Minizinc::Minizinc(const std::string mzn_path, const std::string dzn_path) 
{
	mzn_path_ = mzn_path;
	dzn_path_ = dzn_path;

	FILE *fp;
	char output[1035];
	bool error = false;
	std::string command;

	// Checking minizinc command
	command = "$MINIZINC_DIR/minizinc --version 2>&1 1>/dev/null";
	fp = popen(command.c_str(),"r");

	if (fp == NULL) {
		std::cerr << "[Minizinc]" << std::endl;
		std::cerr << "\tAn error has been encountered while checking the minizinc command" << std::endl;
		error = true;
	}
	else {
		while (fgets(output, sizeof(output)-1, fp) != NULL) {
			printf("[Minizinc] \t%s", output);
			error = true;
		}
		pclose(fp);
	}
	if(error)
		exit(EXIT_FAILURE);
}


Minizinc::~Minizinc() 
{}


void Minizinc::ParseSolution(Solution* outputs)
{
	std::string msg = outputs->msg;
	size_t pos1 = 0;
	size_t pos2 = 0;
	std::string assignment_token;
	std::string assignment_delimiter = ";";
	std::string param_delimiter = "=";
	DataPair varpoint;

	// Removes the spaces in the string
	msg.erase(remove_if(msg.begin(), msg.end(), isspace), msg.end());

	// Tokinizes the string
	while ((pos1 = msg.find(assignment_delimiter)) != std::string::npos) {
		assignment_token = msg.substr(0, pos1);

		if((pos2 = assignment_token.find(param_delimiter)) != std::string::npos) {
			varpoint.id = assignment_token.substr(0, pos2);
			varpoint.value = assignment_token.substr(pos2+1, assignment_token.length());
			outputs->output.push_back(varpoint);
		}
	    msg.erase(0, pos1 + assignment_delimiter.length());
	}
}


int Minizinc::Run(const std::vector<DataPair> inputs, Solution* outputs)
{
	return Run(inputs, outputs, false);
}


int Minizinc::Run(const std::vector<DataPair> inputs, Solution* outputs, bool display_msgs)
{
	FILE *fp;
	char output_str[1035];
	std::string command;
	std::string context_str;
	std::string solution_str;
	int result = 0;

	// Getting the parameters as string
	context_str = "";
	for(unsigned int i=0; i<inputs.size(); i++) {
		context_str.append(inputs[i].id.c_str());
		context_str.append("=");
		context_str.append(inputs[i].value.c_str());
		context_str.append(";");
	}

	// Running minizinc command to execute the optimization
	if(result == 0) {
		command = "$MINIZINC_DIR/minizinc -b mip -G linear -D \""
			+ context_str + "\" " + mzn_path_ + " " + dzn_path_
			+ " --mzn2fzn-cmd \"mzn2fzn --output-ozn-to-file "
			+ mzn_path_.substr(0,mzn_path_.size()-3) + "ozn --output-to-file "
			+ mzn_path_.substr(0,mzn_path_.size()-3) + "fzn\"";
		fp = popen(command.c_str(), "r");
		if (fp == NULL) {
			std::cerr << "[Minizinc.run]" << std::endl;
			std::cerr << "\tAn error has been encountered while executing minizinc" << std::endl;
			result = -1;
		}
		else {
			// Getting the solution
			while (fgets(output_str, sizeof(output_str)-1, fp) != NULL) {
				solution_str.append(output_str);
				if(display_msgs)
					printf("[Minizinc.run] \t%s", output_str);
			}
			outputs->msg = solution_str;
			ParseSolution(outputs);
			pclose(fp);
		}
	}
	return result;
}

}
