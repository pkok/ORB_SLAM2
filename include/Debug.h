#pragma once

#include <iostream>

#define DEBUG_BUILD

#define DIM_BLACK    "\033[0;30m"
#define DIM_RED      "\033[0;31m"
#define DIM_GREEN    "\033[0;32m"
#define DIM_YELLOW   "\033[0;33m"
#define DIM_BLUE     "\033[0;34m"
#define DIM_MAGENTA  "\033[0;35m"
#define DIM_CYAN     "\033[0;36m"
#define DIM_WHITE    "\033[0;37m"
#define BRIGHT_BLACK    "\033[1;30m"
#define BRIGHT_RED      "\033[1;31m"
#define BRIGHT_GREEN    "\033[1;32m"
#define BRIGHT_YELLOW   "\033[1;33m"
#define BRIGHT_BLUE     "\033[1;34m"
#define BRIGHT_MAGENTA  "\033[1;35m"
#define BRIGHT_CYAN     "\033[1;36m"
#define BRIGHT_WHITE    "\033[1;37m"
#define COLOR_RESET "\033[0;0m"

#ifdef DEBUG_BUILD
#define D(x) std::cout << BRIGHT_GREEN << __func__ << ": " << DIM_YELLOW << x << COLOR_RESET << std::endl;
#define DC(x) do { std::cout << BRIGHT_GREEN << __FILE__ << " l." << __LINE__ << ": " << #x << ": " << DIM_YELLOW << x << COLOR_RESET << std::endl; } while (false)
#define DL(x) do { std::cout << BRIGHT_GREEN << __FILE__ << " l." << BRIGHT_MAGENTA << __LINE__ << ": " << DIM_YELLOW << x << COLOR_RESET << std::endl; } while (false)
#else
#define D(x)
#define DC(x)
#define DL(x)
#endif
