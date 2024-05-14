#ifndef COLOR_PRINT_H
#define COLOR_PRINT_H

#include <ostream>
#include <string>
#include <iostream>
namespace Color {
    enum Code {
        RED      = 31,
        GREEN    = 32,
        BLUE     = 34,
        CYAN     = 96,
        YELLOW   = 33,
        BLACK    = 30,
        MAGENTA  = 35,
        HEADER   = 95,
        WARNING = 93,
        FAIL = 91,
        LBLUE = 94,
        UNDERLINE = 4,
        BOLD = 1, 
        DEFAULT  = 39,
        BG_RED      = 41,
        BG_GREEN    = 42,
        BG_YELLOW   = 43,
        BG_BLACK    = 40,
        BG_BLUE     = 44,
        BG_MAGENTA  = 45,
        BG_DEFAULT  = 49
    };
/*
    enum CodePreset {
        TITLE = "\x1b[6;30;42m",
        GC = "\x1b[5;30;43m",
        DC = "\x1b[6;30;41m",
        NC = "\x1b[0;30;47m",
        HEADER = "\033[95m",
        OKBLUE = "\033[94m",
        OKCYAN = "\033[96m",
        OKGREEN = "\033[92m",
        WARNING = "\033[93m",
        FAIL = "\033[91m",
        ENDC = "\033[0m",
        BOLD = "\033[1m",
        UNDERLINE = "\033[4m",
        VOL = "\x1b[6;31;47m"
    };
    */
    class Modifier {
        Code code;
    public:
        Modifier(Code pCode) : code(pCode) {}
        friend std::ostream&
        operator<<(std::ostream& os, const Modifier& mod) {
            return os << "\033[" << mod.code << "m";
        }
    };

    void print(std::string msg, Code color, Code bg_color = BG_DEFAULT);
    void println(std::string msg, Code color, Code bg_color = BG_DEFAULT);
}

#endif