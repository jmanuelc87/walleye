#include "drivers/utils.hpp"


float lmap(float x, float x1, float x2, float y1, float y2) {
    return (((x - x1) / (x2 - x1)) * (y2 - y1)) + y1;
}
