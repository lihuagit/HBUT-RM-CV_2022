#include <show_images/show_images.h>
#include <opencv2/highgui.hpp>
#include <options.h>
#include <log.h>

using namespace cv;


void drawLightBlobs(cv::Mat &src, const LightBlobs &blobs){
    for (const auto &blob:blobs) {
        Scalar color(0,255,0);
        if (blob.blob_color == BLOB_RED)
            color = Scalar(0, 0, 255);
        else if (blob.blob_color == BLOB_BLUE)
            color = Scalar(255, 0, 0);
        cv::Point2f vertices[4];
        blob.rect.points(vertices);
        for (int j = 0; j < 4; j++) {
            cv::line(src, vertices[j], vertices[(j + 1) % 4], color, 2);
        }
    }
}

/**************************
 *     显示多个装甲板区域    *
 **************************/
void showArmorBoxes(std::string windows_name, const cv::Mat &src, const ArmorBoxes &armor_boxes) {
    static Mat image2show;
    if (src.type() == CV_8UC1) {// 黑白图像
        cvtColor(src, image2show, COLOR_GRAY2RGB);
    } else if (src.type() == CV_8UC3) { //RGB 彩色
        image2show = src.clone();
    }

    for (auto &box:armor_boxes) {
        if(box.box_color == BOX_BLUE) {
            rectangle(image2show, box.rect, Scalar(0, 255, 0), 1);
            drawLightBlobs(image2show, box.light_blobs);
        }else if(box.box_color == BOX_RED){
            rectangle(image2show, box.rect, Scalar(0, 255, 0), 1);
            drawLightBlobs(image2show, box.light_blobs);
        }

    }
    imshow(windows_name, image2show);
}

/**************************
 * 显示多个装甲板区域及其类别 *
 **************************/
void showArmorBoxesClass(std::string window_names, const cv::Mat &src, const ArmorBoxes &boxes) {
    static Mat image2show;
    if (src.type() == CV_8UC1) { // 黑白图像
        cvtColor(src, image2show, COLOR_GRAY2RGB);
    } else if (src.type() == CV_8UC3) { //RGB 彩色
        image2show = src.clone();
    }
    for (const auto &box : boxes) {
        if(box.id != 0) {
            rectangle(image2show, box.rect, Scalar(0, 255, 0), 1);
            drawLightBlobs(image2show, box.light_blobs);
            if (box.id == -1)
                putText(image2show, id2name[box.id], Point(box.rect.x + 2, box.rect.y + 2), cv::FONT_HERSHEY_TRIPLEX, 1,
                        Scalar(0, 255, 0));
            else if (1 <= box.id && box.id < 8)
                putText(image2show, id2name[box.id], Point(box.rect.x + 2, box.rect.y + 2), cv::FONT_HERSHEY_TRIPLEX, 1,
                        Scalar(255, 0, 0));
            else if (8 <= box.id && box.id < 15)
                putText(image2show, id2name[box.id], Point(box.rect.x + 2, box.rect.y + 2), cv::FONT_HERSHEY_TRIPLEX, 1,
                        Scalar(0, 0, 255));
            else if (box.id != 0)
                LOGE_INFO("Invalid box id:%d!", box.id);
        }
    }
    imshow(window_names, image2show);

}

/**************************
 * 显示单个装甲板区域及其类别 *
 **************************/
void showArmorBox(std::string windows_name, const cv::Mat &src, const ArmorBox &box) {
    static Mat image2show;
    if(box.rect == cv::Rect2d()){
        imshow(windows_name, src);
    }
    if (src.type() == CV_8UC1) { // 黑白图像
        cvtColor(src, image2show, COLOR_GRAY2RGB);
    } else if (src.type() == CV_8UC3) { //RGB 彩色
        image2show = src.clone();
    }
    drawLightBlobs(image2show, box.light_blobs);
//    static FILE *fp = fopen(PROJECT_DIR"/ratio.txt", "w");
//    if(box.light_blobs.size() == 2)
//        fprintf(fp, "%lf %lf %lf\n", box.light_blobs[0].length, box.light_blobs[1].length, box.getBlobsDistance())
//    cout << box.lengthDistanceRatio() << endl;

    if(box.getOrientation() == ArmorBox::FRONT){
        rectangle(image2show, box.rect, Scalar(0, 255, 0), 3);
    }else{
        rectangle(image2show, box.rect, Scalar(0, 255, 0), 1);
    }
    int w=image2show.size().width;
    int h=image2show.size().height;
    line(image2show,Point2d(0,h/2),Point2d(w,h/2),Scalar(0,0,255),1);
    line(image2show,Point2d(w/2,0),Point2d(w/2,h),Scalar(0,0,255),1);

    char dist[10];
    sprintf(dist, "%.1f", box.getBoxDistance());
    if (box.id == -1)
        putText(image2show, id2name[box.id]+" "+dist, Point(box.rect.x + 2, box.rect.y + 2), cv::FONT_HERSHEY_TRIPLEX, 1,
                Scalar(0, 255, 0));
    else if (1 <= box.id && box.id < 8)
        putText(image2show, id2name[box.id]+" "+dist, Point(box.rect.x + 2, box.rect.y + 2), cv::FONT_HERSHEY_TRIPLEX, 1,
                Scalar(255, 0, 0));
    else if (8 <= box.id && box.id < 15)
        putText(image2show, id2name[box.id]+" "+dist, Point(box.rect.x + 2, box.rect.y + 2), cv::FONT_HERSHEY_TRIPLEX, 1,
                Scalar(0, 0, 255));
    else if (box.id != 0)
        LOGE_INFO("Invalid box id:%d!", box.id);
    if(is_kalman) kalman_run(image2show,box);
    imshow(windows_name, image2show);
}

/**************************
 *      显示多个灯条区域     *
 **************************/
void showLightBlobs(std::string windows_name, const cv::Mat &src, const LightBlobs &light_blobs) {
    static Mat image2show;

    if (src.type() == CV_8UC1) { // 黑白图像
        cvtColor(src, image2show, COLOR_GRAY2RGB);
    } else if (src.type() == CV_8UC3) { //RGB 彩色
        image2show = src.clone();
    }

    for (const auto &light_blob:light_blobs) {
        Scalar color(0, 255, 0);
        if (light_blob.blob_color == BLOB_RED)
            color = Scalar(0, 0, 255);
        else if (light_blob.blob_color == BLOB_BLUE)
            color = Scalar(255, 0, 0);
        cv::Point2f vertices[4];
        light_blob.rect.points(vertices);
        for (int j = 0; j < 4; j++) {
            cv::line(image2show, vertices[j], vertices[(j + 1) % 4], color, 2);
        }
    }
    imshow(windows_name, image2show);
}


void showTrackSearchingPos(std::string window_names, const cv::Mat &src, const cv::Rect2d pos){
    static Mat image2show;
    if (src.type() == CV_8UC1) { // 黑白图像
        cvtColor(src, image2show, COLOR_GRAY2RGB);
    } else if (src.type() == CV_8UC3) { //RGB 彩色
        image2show = src.clone();
    }
    rectangle(image2show, pos, Scalar(0, 255, 0), 1);
    imshow(window_names, image2show);
}

void showArmorBox(std::string windows_name, const cv::Mat &src, const ArmorBox &box, const cv::Rect2d &kal_box){
    static Mat image2show;
    if(box.rect == cv::Rect2d()){
        imshow(windows_name, src);
        return ;
    }
    if (src.type() == CV_8UC1) { // 黑白图像
        cvtColor(src, image2show, COLOR_GRAY2RGB);
    } else if (src.type() == CV_8UC3) { //RGB 彩色
        image2show = src.clone();
    }
    drawLightBlobs(image2show, box.light_blobs);
//    static FILE *fp = fopen(PROJECT_DIR"/ratio.txt", "w");
//    if(box.light_blobs.size() == 2)
//        fprintf(fp, "%lf %lf %lf\n", box.light_blobs[0].length, box.light_blobs[1].length, box.getBlobsDistance())
//    cout << box.lengthDistanceRatio() << endl;

    if(box.getOrientation() == ArmorBox::FRONT){
        rectangle(image2show, box.rect, Scalar(0, 255, 0), 3);
    }else{
        rectangle(image2show, box.rect, Scalar(0, 255, 0), 1);
    }
    int w=image2show.size().width;
    int h=image2show.size().height;
    line(image2show,Point2d(0,h/2),Point2d(w,h/2),Scalar(0,0,255),1);
    line(image2show,Point2d(w/2,0),Point2d(w/2,h),Scalar(0,0,255),1);
    if(is_kalman) rectangle(image2show, kal_box, Scalar(0, 0, 255), 4);

    char dist[10];
    sprintf(dist, "%.1f", box.getBoxDistance());
    if (box.id == -1)
        putText(image2show, id2name[box.id]+" "+dist, Point(box.rect.x + 2, box.rect.y + 2), cv::FONT_HERSHEY_TRIPLEX, 1,
                Scalar(0, 255, 0));
    else if (1 <= box.id && box.id < 8)
        putText(image2show, id2name[box.id]+" "+dist, Point(box.rect.x + 2, box.rect.y + 2), cv::FONT_HERSHEY_TRIPLEX, 1,
                Scalar(255, 0, 0));
    else if (8 <= box.id && box.id < 15)
        putText(image2show, id2name[box.id]+" "+dist, Point(box.rect.x + 2, box.rect.y + 2), cv::FONT_HERSHEY_TRIPLEX, 1,
                Scalar(0, 0, 255));
    else if (box.id != 0)
        LOGE_INFO("Invalid box id:%d!", box.id);
    if (save_video) saveVideos(image2show);//保存视频
    imshow(windows_name, image2show);
}


void kalman_run(cv::Mat& src,const ArmorBox& box){
    static kal_test kalx;
    static kal_test kaly;
    
    static cv::Mat srcxxx( 640, 1080, CV_8UC3, cv::Scalar(255,255,255) );
    static cv::Mat srcyyy( 640, 1080, CV_8UC3, cv::Scalar(255,255,255) );
    static cv::Scalar Color_blue(255,0,0);
    static cv::Scalar Color_red(0,0,255);
    static int iii=0;

    cv::Point2f temp=box.getCenter();
    double newx,newy;
    // std::cout<<"now_time"<<std::endl;
    // std::cout<<now_time<<std::endl<<std::endl;
    newx=kalx.slove(temp.x,0);
    newy=kaly.slove(temp.y,0);
    int w=box.rect.width;
    int h=box.rect.height;
    cv::Rect2f r2f(newx-(w/2),newy-(h/2),w,h);
    rectangle(src, r2f, cv::Scalar(0, 0, 255), 2);
    circle(src, cv::Point2f(newx,newy) , 5, cv::Scalar(0, 0, 255),-1);  // -1 表示圆被填充，正数表示线条粗细

    if(is_kalman_map)
    {
        if(iii==1080){
            srcxxx=cv::Mat( 640, 1080, CV_8UC3, cv::Scalar(255,255,255) );
            srcyyy=cv::Mat( 640, 1080, CV_8UC3, cv::Scalar(255,255,255) );
            iii=0;
        }
        cv::Point2d pos(iii++,temp.x);
        cv::circle(srcxxx,pos,1,Color_blue,1);
        pos.y=newx;
        cv::circle(srcxxx,pos,1,Color_red,1);

        pos.y=temp.y;
        cv::circle(srcyyy,pos,1,Color_blue,1);
        pos.y=newy;
        cv::circle(srcyyy,pos,1,Color_red,1);

        cv::imshow("srcxxx",srcxxx);
        cv::imshow("srcyyy",srcyyy);
    }
}