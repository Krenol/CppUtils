#include <string>

#ifndef UTILS_ENV_H
#define UTILS_ENV_H

namespace utils {

    class Env {
        public:
            static std::string getEnvVar(const std::string& varName, const std::string& defaultValue = "");
            static void getEnvVar(const std::string& varName, std::string& outValue, const std::string& defaultValue);
    };
}

#endif