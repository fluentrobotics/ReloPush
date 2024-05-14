#include <reloPush/color_print.h>

namespace Color {
    void print(std::string msg, Code color, Code bg_color)
    {
        Color::Modifier c(color);
        Color::Modifier d(DEFAULT);
        Color::Modifier bg(bg_color);
        Color::Modifier bgd(BG_DEFAULT);
        std::cout << c << bg << msg << d << bgd; 
    }

    void println(std::string msg, Code color, Code bg_color)
    {
        print(msg,color,bg_color);
        std::cout << std::endl;
    }
}