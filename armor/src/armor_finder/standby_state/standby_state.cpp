/**
 * @file standby_state.cpp
 * @brief 没有应用的模式
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2022-05-16
 * 
 */

#include <armor_finder/armor_finder.h>

bool ArmorFinder::stateStandBy() {
    state = SEARCHING_STATE;
    return true;
}

