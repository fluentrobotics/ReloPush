#ifndef HEXNFLOAT_H
#define HEXNFLOAT_H


#include <string>
#include <memory>
#include <vector>
#include <any>
//for hex
#include <sstream>
#include <bitset>
#include <iomanip>
#include <iostream>

using namespace std;

namespace jeeho{
    static const std::string group_delim = "!";
    static const std::string data_delim = ";";
    static const std::string elem_delim = ",";

    /**
     * @brief Split a string by a deliminator
     * @return Splited string in std vector
     */
    std::vector<std::string> split(std::string s, std::string delimiter);

    /**
     * @brief Join strings in a vector by a deliminator
     * @return Joined string
     */
    std::string join(const vector<string>& vec, string delim);

    std::string join(const std::vector<std::string>& vec, char delim);

    double hexstr2double(const std::string& hexstr);

    std::string double2hexstr(double x);

    float hexstr2float(std::string hexstr);

    std::string float2hexstr(float x);

    void remove0x(std::string& hex);

    std::shared_ptr<std::vector<std::string>> floatList_to_hexListPtr(std::vector<float>& list_in);

    std::shared_ptr<std::vector<float>> hexList_to_floatListPtr(std::vector<std::string>& list_in);

    /*
    template<typename T>
    std::string any_to_string(std::any& to_cast, std::string list_delim=",")
    {
        try {
            T casted = std::any_cast<T>(to_cast);

            //string, vector
            if constexpr(is_same_v<T,float>)
            {
                return float2hexstr(casted);
            }
            else if constexpr(is_same_v<T,std::vector<float>>)
            {
                return jeeho::join(*floatList_to_hexListPtr(casted), list_delim);
            }
            else if constexpr (is_same_v<T, std::vector<std::string>>)
            {
                return jeeho::join(casted, list_delim);
            }
            else
            {
                return casted;
            }

        }
        catch (...)
        {
            return "";
        }
    }
    */
}

#endif