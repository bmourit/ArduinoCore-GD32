#if defined(GD32F10x)
#include "gd32f10x_gpio.h"
#elif defined(GD32F1x0)
#include "gd32f1x0_gpio.h"
#elif defined(GD32F20x)
#include "gd32f20x_gpio.h"
#elif defined(GD32F3x0)
#include "gd32f3x0_gpio.h"
#elif defined(GD32F30x)
#include "gd32f30x_gpio.h"
#elif defined(GD32F4xx)
#include "gd32f4xx_gpio.h"
#elif defined(GD32F403)
#include "gd32f403_gpio.h"
#elif defined(GD32E10X)
#include "gd32e10x_gpio.h"
#elif defined(GD32E23x)
#include "gd32e23x_gpio.h"
#elif defined(GD32E50X)
#include "gd32e50x_gpio.h"
#else
#error "Unknown chip series"
#endif