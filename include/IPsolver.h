#ifndef IPSOLVER_H
#define IPSOLVER_H

#include <vector>


extern bool finished_solve;
extern std::vector<std::vector<bool>> choosen_path_record;

// Precompute the length of each path
void CalculateEveryPathLength();

bool SolveNetwork();




#endif