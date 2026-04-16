#ifndef LOADDATA_H
#define LOADDATA_H 

#include "initial/common.h"

// Save PathDataset to a binary file
bool SavePathDataset(const PathDataset& dataset, const std::string& filename);

// Load PathDataset from a binary file
bool LoadPathDataset(PathDataset& dataset, const std::string& filename) ;

#endif
