// This is a demonstration file to load and print information from a saved finalSequence file.

#include <iostream>
#include <vector>
#include <string>
#include <SerializeFinalSequence.h>    // Assume this is the header with loadFinalSequenceFromFile
#include <ReloPush/TaskAllocation.hpp> // For FinalAllocation and printFinalSequence
#include <ReloPush/base64.h>
#include <ReloPush/config.h>

// Function to load finalSequence from a file
std::vector<FinalAllocation> loadFinalSequenceFromFile(const std::string &filename)
{
    std::ifstream inFile(filename);
    if (!inFile)
    {
        std::cerr << "Error: Could not open file " << filename << " for reading." << std::endl;
        return {};
    }
    std::string base64Data((std::istreambuf_iterator<char>(inFile)), std::istreambuf_iterator<char>());
    inFile.close();
    std::string binaryData = base64_decode(base64Data);
    return deserializeFinalSequence(binaryData);
}

int main(int argc, char *argv[])
{

    std::string filename = "";

    if (argc < 2)
    {
        // std::cerr << "Usage: " << argv[0] << " <filename.b64>" << std::endl;
        // return 1;
        filename = std::string(CMAKE_SOURCE_DIR) + "/results/relopush-out/result_seq_ReloPush-BOSS_13_objects.txt_ind2.b64";
    }
    else
    {
        filename = argv[1];
    }

    std::vector<FinalAllocation> loadedSequence = loadFinalSequenceFromFile(filename);

    if (loadedSequence.empty())
    {
        std::cerr << "Failed to load finalSequence from " << filename << std::endl;
        return 1;
    }

    std::cout << "Loaded " << loadedSequence.size() << " allocations from " << filename << std::endl;

    // Print the loaded sequence using the existing print function
    // printFinalSequence(loadedSequence);

    return 0;
}
