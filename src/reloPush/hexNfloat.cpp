#include <reloPush/hexNfloat.h>

using namespace std;

namespace jeeho
{
    /// Split a string by a delimiter
    std::vector<std::string> split(std::string s, std::string delimiter) {
        size_t pos_start = 0, pos_end, delim_len = delimiter.length();
        std::string token;
        std::vector<std::string> res;

        while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos) {
        token = s.substr(pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back(token);
        }

        res.push_back(s.substr(pos_start));
        return res;
    }

    std::string join(const std::vector<std::string>& vec, char delim) {

        //std::string s;
        //s.clear();

        return join(vec,std::string(1,delim));
    }

    std::string join(const vector<string>& vec, string delim)
    {
        size_t last_ind = vec.size() - 1;
        std::string out_str = "";
        for (size_t n = 0; n < vec.size(); n++)
        {
        out_str += vec[n];
        if (n != last_ind)
            out_str += delim;
        }
        return out_str;
    }

    double hexstr2double(const std::string& hexstr)
    {
        double d = 0.0;

        try {
            *reinterpret_cast<unsigned long long*>(&d) = std::stoull(hexstr, nullptr, 16);
        }
        catch (...) {}

        return d;
    }


    std::string double2hexstr(double x) {

        union
        {
            long long i;
            double    d;
        } value;

        value.d = x;

        char buf[17];

        snprintf(buf, sizeof(buf), "%016llx", value.i);
        buf[16] = 0; //make sure it is null terminated.

        return std::string(buf);

    }

    float hexstr2float(std::string hexstr)
    {
        float f = 0.0f;

        try {
        //*reinterpret_cast<unsigned long*>(&f) = std::stoul(hexstr, nullptr, 8);
        union ulf
        {
            unsigned long ul;
            float f;
        } value;

        //remove 0x
        remove0x(hexstr);
        stringstream ss(hexstr);
        ss >> hex >> value.ul;
        //std::cout <<value.f << std::endl;
        f = value.f;
        }
        catch (...) {
        std::cout << "hexstr2float error" << std::endl;
        }

        return f;

    }

    std::string float2hexstr(float x)
    {
    /*
        union
        {
            unsigned long hex;
            float f;
        } value;

        value.f = x;
        char buf[9];

        snprintf(buf, sizeof(buf), "%08lx", value.hex);
        buf[8] = 0; //make sure it is null terminated.

        return std::string(buf);
        */

    uint32_t intVal = *reinterpret_cast<uint32_t*>(&x);
    std::stringstream ss;
    ss << std::hex << std::setw(8) << std::setfill('0') << intVal;
    std::string hexadecimal = ss.str();

    return hexadecimal;
    }

    void remove0x(std::string& hex)
    {
        if (hex.substr(0, 2) == "0x")
        {
            hex.erase(0, 2);
        }
    }

    std::shared_ptr<std::vector<std::string>> floatList_to_hexListPtr(std::vector<float>& list_in)
    {
        std::shared_ptr<std::vector<std::string>> out_vec(new std::vector<std::string>);
        out_vec->resize(list_in.size());

        if(out_vec->size()>0)
        out_vec->at(0) = float2hexstr(0); // to bypass a weird bug

        //convert n to n
        for (size_t n = 0; n < list_in.size(); n++)
        {
            out_vec->at(n) = float2hexstr(list_in[n]);
        }

        return out_vec;
    }

    std::shared_ptr<std::vector<float>> hexList_to_floatListPtr(std::vector<std::string>& list_in)
    {
    std::shared_ptr<std::vector<float>> out_vec(new std::vector<float>);
    out_vec->resize(list_in.size());

    if(out_vec->size()>0)
        out_vec->at(0) = hexstr2float("0x0"); // to bypass a weird bug

    //convert n to n
    for (size_t n = 0; n < list_in.size(); n++)
    {
        out_vec->at(n) = hexstr2float(list_in[n]);
    }

    return out_vec;
    }
}