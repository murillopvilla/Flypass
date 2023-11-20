#ifndef READ_INPUT_H
#define READ_INPUT_H


#include <vector>
#include <string>
#include <sstream>
#include <fstream>

#include "solution.h"


// ---------- Read text file ----------

Target readManeuverFile(std::string filename);

std::vector<Target> readManeuverFile2(std::string filename);

#endif // READ_INPUT_H
