#ifndef UNOS_LOG_UTILS_H__
#define UNOS_LOG_UTILS_H__

#include <string>

std::string cutParenthesesNTail(std::string &&pretty_func) {
  size_t pos = pretty_func.find('(');
  if (pos != std::string::npos) {
    pretty_func.erase(pretty_func.begin() + pos, pretty_func.end());
  }

  pos = pretty_func.find(' ');
  if (pos != std::string::npos) {
    pretty_func.erase(pretty_func.begin(), pretty_func.begin() + pos + 1);
  }

  pretty_func = "[" + pretty_func + "]";
  return pretty_func;
}

#define __STR_FUNCTION__ \
  cutParenthesesNTail(std::string(__PRETTY_FUNCTION__))

#endif  // LOG_UTILS_HH
