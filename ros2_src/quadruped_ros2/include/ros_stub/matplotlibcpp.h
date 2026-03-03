#ifndef MATPLOTLIBCPP_H_STUB
#define MATPLOTLIBCPP_H_STUB

#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <stdexcept>

namespace matplotlibcpp {
    template<typename T>
    void plot(const std::vector<T>& x, const std::vector<T>& y, const std::map<std::string, std::string>& keywords = {}) {}
    
    template<typename T>
    void scatter(const std::vector<T>& x, const std::vector<T>& y, float s = 1.0, const std::map<std::string, std::string>& keywords = {}) {}

    inline void figure() {}
    inline void show() {}
    inline void grid(bool b) {}
    inline void legend() {}
    inline void title(const std::string& s) {}
    inline void xlabel(const std::string& s) {}
    inline void ylabel(const std::string& s) {}
    inline void save(const std::string& s) {}
    inline void clf() {}
    inline void close() {}
    inline void xticks(const std::vector<float>& v) {}
}

#endif
