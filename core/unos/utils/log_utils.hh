#ifndef UNOS_UTILS_LOG_UTILS_HH
#define UNOS_UTILS_LOG_UTILS_HH

#include <string>

std::string cutParenthesesNTail(std::string &&pretty_func);

#define __STR_FUNCTION__ \
  cutParenthesesNTail(std::string(__PRETTY_FUNCTION__))

#endif // UNOS_UTILS_LOG_UTILS_HH
