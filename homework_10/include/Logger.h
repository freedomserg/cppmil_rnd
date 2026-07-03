#pragma once

#include <iostream>

#ifndef ENABLE_LOG
#  define ENABLE_LOG 1
#endif

#ifndef ENABLE_DEBUG
#  define ENABLE_DEBUG 1
#endif

#if ENABLE_LOG
#  define LOG(msg) std::cout << "[LOG] " << msg << std::endl
#else
#  define LOG(msg) do {} while (0)
#endif

#if ENABLE_DEBUG
#  define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
#  define DEBUG(msg) do {} while (0)
#endif
