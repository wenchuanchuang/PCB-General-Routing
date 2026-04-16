#ifndef LOADYAML_H 
#define LOADYAML_H 

#include "initial/common.h"
#include "initial/config.h"


void PutComponenetOnBoard(
    const std::string& comps_yaml, const std::string& boardcomps_yaml, const std::string& boardNets_yaml,
    const std::unordered_set<std::string>& ref /*Set of component references to be placed on the board*/,
    const RefToPos& ref_pos /*Mapping from reference to placement information (position + rotation)*/,
    std::vector<std::vector<std::vector<uint8_t>>>& pin_map,
    std::vector<std::pair<Coord, Coord>>& path_endpoints/*start and end coordinates for routing (pin pairs)*/,
    std::vector<int>& electrical_net
);


#endif
