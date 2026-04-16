/*
Reload saved candidate paths into the viewer (not critical)
*/
#include <fstream>
#include <iostream>
#include <vector>
#include <stdexcept>

# include <loadData.h>

bool SavePathDataset(const PathDataset& dataset, const std::string& filename) {

    std::ofstream out(filename, std::ios::binary);
    if (!out) {
        std::cerr << "Error: Cannot open file for writing: " << filename << std::endl;
        return false;
    }

    size_t num_groups = dataset.size();
    out.write(reinterpret_cast<const char*>(&num_groups), sizeof(num_groups));

    for (size_t g = 0; g < num_groups; ++g) {
        const auto& group = dataset[g];
        
        size_t num_paths = group.size();
        out.write(reinterpret_cast<const char*>(&num_paths), sizeof(num_paths));

        for (size_t p = 0; p < num_paths; ++p) {
            const auto& path_info = group[p];
            
            out.write(reinterpret_cast<const char*>(&path_info.first), sizeof(path_info.first));

            size_t num_coords = path_info.second.size();
            out.write(reinterpret_cast<const char*>(&num_coords), sizeof(num_coords));

            if (num_coords > 0) {
                out.write(reinterpret_cast<const char*>(path_info.second.data()), num_coords * sizeof(Coord));
            }
        }
    }

    out.close();
    std::cout << "[OK] PathDataset saved successfully to " << filename << std::endl;
    return true;
}


bool LoadPathDataset(PathDataset& dataset, const std::string& filename) {

    std::ifstream in(filename, std::ios::binary);
    if (!in) {
        std::cerr << "Error: Cannot open file for reading: " << filename << std::endl;
        return false;
    }

    dataset.clear();

    size_t num_groups = 0;
    in.read(reinterpret_cast<char*>(&num_groups), sizeof(num_groups));
    dataset.resize(num_groups);

    for (size_t g = 0; g < num_groups; ++g) {

        size_t num_paths = 0;
        in.read(reinterpret_cast<char*>(&num_paths), sizeof(num_paths));
        dataset[g].resize(num_paths);

        for (size_t p = 0; p < num_paths; ++p) {
            auto& path_info = dataset[g][p];

            in.read(reinterpret_cast<char*>(&path_info.first), sizeof(path_info.first));

            size_t num_coords = 0;
            in.read(reinterpret_cast<char*>(&num_coords), sizeof(num_coords));

            path_info.second.resize(num_coords);
            if (num_coords > 0) {
                in.read(reinterpret_cast<char*>(path_info.second.data()), num_coords * sizeof(Coord));
            }
        }
    }

    in.close();
    std::cout << "[OK] PathDataset loaded successfully from " << filename 
              << " (Groups: " << num_groups << ")" << std::endl;
    return true;
}