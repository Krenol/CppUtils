#include "env.hpp"
#include <cstdlib>

namespace utils
{
    std::string Env::getEnvVar(const std::string &varName, const std::string &defaultValue)
    {
        std::string out;
        getEnvVar(varName, out, defaultValue);
        return out;
    }

    void Env::getEnvVar(const std::string &varName, std::string &outValue, const std::string &defaultValue)
    {
        auto env_var = std::getenv(varName.c_str());
        try
        {
            if (env_var)
            {
                outValue = std::string(env_var);
            }
            else
            {
                outValue = defaultValue;
            }
        }
        catch (...)
        {
            outValue = defaultValue;
        }
    }
} // namespace utils