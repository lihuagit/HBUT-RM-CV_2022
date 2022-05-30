/**
 * @file standby_state.cpp
 * @brief 没有应用的模式
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2022-05-16
 * 
 */

#include <armor_finder/armor_finder.h>
#include <show_images/show_images.h>
#include <options.h>
#include <log.h>

bool ArmorFinder::stateStandBy(cv::Mat &src) {
    static int last_num=0;
    target_boxes.clear();
    if (findArmorBoxes(src, target_boxes)) { // 在原图中寻找目标，并返回是否找到
        // 是否存在上次击打的装甲板
        ArmorBoxes last_armors;
        for(auto box : target_boxes)
            if(box.id==last_box.id)
                last_armors.push_back( box );
        if(last_armors.empty()){
            // 若不存在上次击打装甲板 则选取优先级最高的装甲板
            sort(target_boxes.begin(), target_boxes.end());
            target_box = target_boxes[0];
        }else{
            // 若只有一个装甲板 追踪此装甲板
            if(last_armors.size()==1) target_box = last_armors[0], last_num=1;
            else {
                sort(last_armors.begin(), last_armors.end(), [&]( ArmorBox a, ArmorBox b ){
                    // 反陀螺则选取最左边装甲板 否则选取面积最大的装甲板
                    if(is_anti_top)
                        return a.rect.x < b.rect.x;
                    else return a.rect.area() > b.rect.area();
                });
                target_box = last_armors[0];

                // 击打前哨站策略 装甲板第一次出现视为有效
                if(is_anti_top){
                    if(last_num==1) sendData.mode = 'Y';
                    else if(last_num==2) sendData.mode = 'N';
                }

                last_num=2;
            }
        }
        if(show_armor_boxes){
            showArmorBoxesClass("Boxes", src, target_boxes);
        }
        return true;
    } else {
        target_box = ArmorBox();
        anti_switch_cnt++;
        return false;
    }
}

