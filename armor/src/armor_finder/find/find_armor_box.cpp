/**
 * @file find_armor_box.cpp
 * @brief 装甲板的识别函数实现
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2022-05-16
 * 
 */
#include <armor_finder/armor_finder.h>
#include <show_images/show_images.h>
#include <options.h>
#include <opencv2/highgui.hpp>

#define DO_NOT_CNT_TIME

#include <log.h>

// 判断两个灯条的角度差
static bool angelJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
    float angle_i = light_blob_i.rect.size.width > light_blob_i.rect.size.height ? light_blob_i.rect.angle :
                    light_blob_i.rect.angle - 90;
    float angle_j = light_blob_j.rect.size.width > light_blob_j.rect.size.height ? light_blob_j.rect.angle :
                    light_blob_j.rect.angle - 90;
    return abs(angle_i - angle_j) < 20;
}
// 判断两个灯条的高度差
static bool heightJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
    cv::Point2f centers = light_blob_i.rect.center - light_blob_j.rect.center;
    return abs(centers.y) < 30;
}
// 判断两个灯条的间距
static bool lengthJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
    double side_length;
    cv::Point2f centers = light_blob_i.rect.center - light_blob_j.rect.center;
    side_length = sqrt(centers.ddot(centers));
    return (side_length / light_blob_i.length < 10 && side_length / light_blob_i.length > 0.5);
}
// 判断两个灯条的长度比
static bool lengthRatioJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
    return (light_blob_i.length / light_blob_j.length < 2.5
            && light_blob_i.length / light_blob_j.length > 0.4);
}

/* 判断两个灯条的错位度，不知道英文是什么！！！ */
static bool CuoWeiDuJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
    float angle_i = light_blob_i.rect.size.width > light_blob_i.rect.size.height ? light_blob_i.rect.angle :
                    light_blob_i.rect.angle - 90;
    float angle_j = light_blob_j.rect.size.width > light_blob_j.rect.size.height ? light_blob_j.rect.angle :
                    light_blob_j.rect.angle - 90;
    float angle = (angle_i + angle_j) / 2.0 / 180.0 * 3.14159265459;
    if (abs(angle_i - angle_j) > 90) {
        angle += 3.14159265459 / 2;
    }
    Vector2f orientation(cos(angle), sin(angle));
    Vector2f p2p(light_blob_j.rect.center.x - light_blob_i.rect.center.x,
                 light_blob_j.rect.center.y - light_blob_i.rect.center.y);
    return abs(orientation.dot(p2p)) < 25;
}
// 判断装甲板方向
static bool boxAngleJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
    float angle_i = light_blob_i.rect.size.width > light_blob_i.rect.size.height ? light_blob_i.rect.angle :
                    light_blob_i.rect.angle - 90;
    float angle_j = light_blob_j.rect.size.width > light_blob_j.rect.size.height ? light_blob_j.rect.angle :
                    light_blob_j.rect.angle - 90;
    float angle = (angle_i + angle_j) / 2.0;
    if (abs(angle_i - angle_j) > 90) {
        angle += 90.0;
    }
    return (-120.0 < angle && angle < -60.0) || (60.0 < angle && angle < 120.0);
}
// 判断两个灯条是否可以匹配
static bool isCoupleLight(const LightBlob &light_blob_i, const LightBlob &light_blob_j, uint8_t enemy_color) {
    return light_blob_i.blob_color == enemy_color &&
           light_blob_j.blob_color == enemy_color &&
           lengthRatioJudge(light_blob_i, light_blob_j) &&
           lengthJudge(light_blob_i, light_blob_j) &&
           //           heightJudge(light_blob_i, light_blob_j) &&
           angelJudge(light_blob_i, light_blob_j) &&
           boxAngleJudge(light_blob_i, light_blob_j) &&
           CuoWeiDuJudge(light_blob_i, light_blob_j);

}
// 匹配所有灯条，得出装甲板候选区
bool ArmorFinder::matchArmorBoxes(const cv::Mat &src, const LightBlobs &light_blobs, ArmorBoxes &armor_boxes) {
    armor_boxes.clear();
    for (int i = 0; i < light_blobs.size() - 1; ++i) {
        for (int j = i + 1; j < light_blobs.size(); ++j) {
            if (!isCoupleLight(light_blobs.at(i), light_blobs.at(j), enemy_color)) {
                continue;
            }
            cv::Rect2d rect_left = light_blobs.at(static_cast<unsigned long>(i)).rect.boundingRect();
            cv::Rect2d rect_right = light_blobs.at(static_cast<unsigned long>(j)).rect.boundingRect();
            double min_x, min_y, max_x, max_y;
            min_x = fmin(rect_left.x, rect_right.x) - 4;
            max_x = fmax(rect_left.x + rect_left.width, rect_right.x + rect_right.width) + 4;
            min_y = fmin(rect_left.y, rect_right.y) - 0.5 * (rect_left.height + rect_right.height) / 2.0;
            max_y = fmax(rect_left.y + rect_left.height, rect_right.y + rect_right.height) +
                    0.5 * (rect_left.height + rect_right.height) / 2.0;
            if (min_x < 0 || max_x > src.cols || min_y < 0 || max_y > src.rows) {
                continue;
            }
            if (state == SEARCHING_STATE && (max_y + min_y) / 2 < 120) continue;
            if ((max_x - min_x) / (max_y - min_y) < 0.8) continue;
            LightBlobs pair_blobs = {light_blobs.at(i), light_blobs.at(j)};
            armor_boxes.emplace_back(
                    cv::Rect2d(min_x, min_y, max_x - min_x, max_y - min_y),
                    pair_blobs,
                    enemy_color
            );
            ArmorBox &box=armor_boxes.back();

            // 更精确的框选装甲板
            auto blob=light_blobs[i];
            cv::Point2f cen=blob.rect.center;
            std::vector<cv::Point2f> ans1;
            double ang;
            if(blob.rect.size.width>blob.rect.size.height){
                ang=blob.rect.angle;
            }
            else ang=-blob.rect.angle;
            double heigh=max(blob.rect.size.width,blob.rect.size.height);
            // heigh=max( heigh,(double)max(blob.rect.size.width,blob.rect.size.height) );
            double h=sin((ang)*M_PI/180)*heigh;
            h/=2;
            double w=cos((ang)*M_PI/180)*heigh;
            w/=2;
            if(h<w) swap(w,h);
            ans1.push_back({cen.x+w,cen.y+h});
            ans1.push_back({cen.x-w,cen.y-h});
            sort(ans1.begin(),ans1.end(),[](cv::Point2f a,cv::Point2f b){ return a.y<b.y; });
            
            blob=light_blobs[j];
            cen=blob.rect.center;
            std::vector<cv::Point2f> ans2;
            ang;
            if(blob.rect.size.width>blob.rect.size.height){
                ang=blob.rect.angle;
            }
            else ang=-blob.rect.angle;
            heigh=max(blob.rect.size.width,blob.rect.size.height);
            h=sin((ang)*M_PI/180)*heigh;
            h/=2;
            w=cos((ang)*M_PI/180)*heigh;
            w/=2;
            if(h<w) swap(w,h);
            ans2.push_back({cen.x+w,cen.y+h});
            ans2.push_back({cen.x-w,cen.y-h});
            sort(ans2.begin(),ans2.end(),[](cv::Point2f a,cv::Point2f b){ return a.y<b.y; });

            if(ans1[0].x>ans2[0].x){
                swap(ans1[0],ans2[0]);
                swap(ans1[1],ans2[1]);
            }
            box.pts[0]=ans1[0];
            box.pts[1]=ans1[1];
            box.pts[2]=ans2[1];
            box.pts[3]=ans2[0];
        }
    }
    return !armor_boxes.empty();
}
// 在给定的图像上寻找装甲板
bool ArmorFinder::findArmorBox(const cv::Mat &src, ArmorBox &box) {
    LightBlobs light_blobs; // 存储所有可能的灯条
    ArmorBoxes armor_boxes; // 装甲板候选区

    box.rect = cv::Rect2d(0, 0, 0, 0);
    box.id = -1;
// 寻找所有可能的灯条
    CNT_TIME("blob", {
        if (!findLightBlobs(src, light_blobs)) {
            return false;
        }
    });
    if (show_light_blobs && state==SEARCHING_STATE) {
        showLightBlobs("light_blobs", src, light_blobs);
        cv::waitKey(1);
    }
// 对灯条进行匹配得出装甲板候选区
    CNT_TIME("boxes", {
        if (!matchArmorBoxes(src, light_blobs, armor_boxes)) {
            return false;
        }
    });
    if (show_armor_boxes && state==SEARCHING_STATE) {
        showArmorBoxes("boxes", src, armor_boxes);
        cv::waitKey(1);
    }
// 如果分类器可用，则使用分类器对装甲板候选区进行筛选
    if (classifier) {
        CNT_TIME("classify: %d", {
            for (auto &armor_box : armor_boxes) {
                cv::Mat roi = src(armor_box.rect).clone();
                cv::resize(roi, roi, cv::Size(48, 36));
                int c = classifier(roi);
                armor_box.id = c;
            }
        }, armor_boxes.size());
// 按照优先级对装甲板进行排序
        if(is_anti_top){
            sort(armor_boxes.begin(), armor_boxes.end(), [&](const ArmorBox &a, const ArmorBox &b) {
                return a.rect.x < b.rect.y;
            });
        }
        else{
            sort(armor_boxes.begin(), armor_boxes.end(), [&](const ArmorBox &a, const ArmorBox &b) {
                if (last_box.rect != cv::Rect2d()) {
                    return getPointLength(a.getCenter() - last_box.getCenter()) <
                        getPointLength(b.getCenter() - last_box.getCenter());
                } else {
                    return a < b;
                }
            });
        }
        for (auto &one_box : armor_boxes) {
            if (one_box.id != 0) {
                box = one_box;
                break;
            }
        }
        if (save_labelled_boxes) {
            for (const auto &one_box : armor_boxes) {
                char filename[100];
                sprintf(filename, PROJECT_DIR"/armor_box_photo/%s_%d.jpg", id2name[one_box.id].data(),
                        time(nullptr) + clock());
                auto box_roi = src(one_box.rect);
                cv::resize(box_roi, box_roi, cv::Size(48, 36));
                cv::imwrite(filename, box_roi);
            }
        }
        if (box.rect == cv::Rect2d(0, 0, 0, 0)) {
            return false;
        }
        if (show_armor_boxes && state==SEARCHING_STATE) {
            showArmorBoxesClass("class", src, armor_boxes);
        }
    } else { // 如果分类器不可用，则直接选取候选区中的第一个区域作为目标(往往会误识别)
        box = armor_boxes[0];
    }
    return true;
}

// 在给定的图像上寻找装甲板
bool ArmorFinder::findArmorBoxes(const cv::Mat &src, ArmorBoxes &boxes) {
    LightBlobs light_blobs; // 存储所有可能的灯条
    ArmorBoxes armor_boxes; // 装甲板候选区

// 寻找所有可能的灯条
    CNT_TIME("blob", {
        if (!findLightBlobs(src, light_blobs)) {
            return false;
        }
    });
    if (show_light_blobs && state==SEARCHING_STATE) {
        showLightBlobs("light_blobs", src, light_blobs);
        cv::waitKey(1);
    }
// 对灯条进行匹配得出装甲板候选区
    CNT_TIME("boxes", {
        if (!matchArmorBoxes(src, light_blobs, armor_boxes)) {
            return false;
        }
    });
    if (show_armor_boxes && state==SEARCHING_STATE) {
        showArmorBoxes("boxes", src, armor_boxes);
        cv::waitKey(1);
    }
// 如果分类器可用，则使用分类器对装甲板候选区进行筛选
    if (classifier) {
        CNT_TIME("classify: %d", {
            for (auto &armor_box : armor_boxes) {
                cv::Mat roi = src(armor_box.rect).clone();
                cv::resize(roi, roi, cv::Size(48, 36));
                int c = classifier(roi);
                armor_box.id = c;
            }
        }, armor_boxes.size());
// // 按照优先级对装甲板进行排序
//         sort(armor_boxes.begin(), armor_boxes.end(), [&](const ArmorBox &a, const ArmorBox &b) {
//             if (last_box.rect != cv::Rect2d()) {
//                 return getPointLength(a.getCenter() - last_box.getCenter()) <
//                        getPointLength(b.getCenter() - last_box.getCenter());
//             } else {
//                 return a < b;
//             }
//         });
        for (auto &one_box : armor_boxes) {
            if (one_box.id != 0) {
                boxes.push_back(one_box);
                // break;
            }
        }
        if (save_labelled_boxes) {
            for (const auto &one_box : armor_boxes) {
                char filename[100];
                sprintf(filename, PROJECT_DIR"/armor_box_photo/%s_%d.jpg", id2name[one_box.id].data(),
                        time(nullptr) + clock());
                auto box_roi = src(one_box.rect);
                cv::resize(box_roi, box_roi, cv::Size(48, 36));
                cv::imwrite(filename, box_roi);
            }
        }
        if (boxes.size()==0) {
            return false;
        }
        if (show_armor_boxes && state==SEARCHING_STATE) {
            showArmorBoxesClass("class", src, armor_boxes);
        }
    } else { // 如果分类器不可用，则直接选取候选区中的第一个区域作为目标(往往会误识别)
        boxes.push_back(armor_boxes[0]);
    }
    return true;
}
