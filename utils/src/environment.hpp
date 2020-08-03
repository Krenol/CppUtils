#include <string>

#ifndef UTILS_ENVIRONMENT_H
#define UTILS_ENVIRONMENT_H

namespace utils {

    class Environment {
        public:
            static std::string getVar(const std::string& varName, const std::string& defaultValue = "");
            static void getVar(const std::string& varName, std::string& outValue, const std::string& defaultValue);
    };
}

#endif