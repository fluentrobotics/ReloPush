#ifndef HEXNDOUBLE_H
#define HEXNDOUBLE_H

#include <string>
#include <memory>
#include <vector>
#include <any>
//for hex
#include <sstream>
#include <bitset>
#include <iomanip>
#include <type_traits>
#include <iostream>
#include <cstring> // For memcpy

using namespace std;

double hexstr2double(const std::string& hexstr);

std::string double2hexstr(double x);

float hexstr2float(std::string hexstr);

std::string float2hexstr(float x);

void remove0x(std::string& hex);

std::shared_ptr<std::vector<std::string>> floatList_to_hexListPtr(std::vector<float>& list_in);

std::shared_ptr<std::vector<float>> hexList_to_floatListPtr(std::vector<std::string>& list_in);

std::string join(const vector<string>& vec, string delim);

template<typename T>
std::string any_to_string(std::any& to_cast, std::string list_delim=",")
{
    try {
        T casted = std::any_cast<T>(to_cast);

        //string, vector
        if constexpr(std::is_same_v<T,float>)
        {
            return float2hexstr(casted);
        }
        else if constexpr(is_same_v<T,std::vector<float>>)
        {
            return join(*floatList_to_hexListPtr(casted), list_delim);
        }
        else if constexpr (is_same_v<T, std::vector<std::string>>)
        {
            return join(casted, list_delim);
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

std::string float2binarystr(float f_in);

float binarystr2float(std::string str_in);

std::string bool2binarystr(bool b_in);

bool binarystr2bool(std::string str_in);

#endif // HEXNDOUBLE_H
