#include <cmath>
#include <cstdio>

#include "lidar_defs.h"
#include "mtime.h"
#include "logger.h"

using namespace cv;
using namespace std;

LOG_DEFINE(loggerLidarDefs, "Lidar");

#ifndef WIN32
static const string outputFName = "out/lidar.json";
#else
static const string outputFName = "out\\lidar.json";
#endif

int lidar_process(const lidar_data_t *data, const int& data_cnt, DegreeMap& dmap, int &stop)
{
    double t = timeBegin();

    dmap.init();
    float dd = 1;
    int stop_cnt = 0;
    for(int i = 0; i < data_cnt; i++) {
        float angle1 = data[i].angle;
        // low quality => no obstacle
        int val = 1;
        int dist = -1;
        if ((data[i].distance >= LIDAR_DISTANCE_MIN * 10) && (data[i].distance <= LIDAR_DISTANCE_MAX * 10) &&
            (data[i].quality >= LIDAR_QUALITY_MIN) && (data[i].quality < LIDAR_QUALITY_MAX)) {
            // distance above threshold => no obstacle
            val = (data[i].distance >= LIDAR_DISTANCE_DMAP * 10) || (data[i].quality < LIDAR_QUALITY_DMAP);
            if (data[i].quality >= LIDAR_QUALITY_DMAP) {    // set distance to dmap on if quality is OK
                dist = trunc(data[i].distance / 10);        // distance in centimeters
            }
            if ((angle1 >= LIDAR_STOP_ANGLE_MIN) && (angle1 <= LIDAR_STOP_ANGLE_MAX) && (data[i].distance < LIDAR_DISTANCE_STOP * 10) && (data[i].quality >= LIDAR_QUALITY_STOP)) {
                stop_cnt++;
            }
        }
        // for the last one we use old value of dd
        if (i + 1 < data_cnt) {
            float angle2 = data[i+1].angle;
            dd = (angle2 - angle1) / 2;
        }
        // in dmap zero degrees is on the right side of the robot (counterclockwise)
        dmap.fill(180 - angle1 - dd, 180 - angle1 + dd, val, dist, LIDAR_DISTANCE_MAX);
    }
    dmap.finish();

    stop = (stop_cnt >= LIDAR_STOP_COUNT);

    timeEnd("lidar_process", t);

    return 0;
}

int lidar_draw_output(const lidar_data_t *data, const int& data_cnt, Mat& img, int stop, int angle_min, int angle_max,
    int process_angle, int process_angle_min, int process_angle_max, long image_number)
{
    double t = timeBegin();
    char outgingData[30] = "";

    if (data_cnt > 0) {
        FILE * pFile;
        pFile = fopen(outputFName.c_str(), "a");
        if (pFile!=NULL) {
            fprintf(pFile, "{\"image_number\":%d,\"lidar_data\":[\n", (int)image_number);
            for (int pos = 0; pos < data_cnt - 1; pos++) {
                fprintf(pFile, "[%d,%.2f,%.2f,%d],",
                    data[pos].sync, data[pos].angle, data[pos].distance, data[pos].quality);
                if (pos%4 == 3) fputs("\n", pFile);
            }
            int pos = data_cnt - 1;
            fprintf(pFile, "[%d,%.2f,%.2f,%d]]},\n",
                data[pos].sync, data[pos].angle, data[pos].distance, data[pos].quality);
            fclose(pFile);
        }
    }

    img.create(480,640,CV_8UC3);
    img.setTo(Scalar(20,20,20));
    line(img, Point(0,320), Point(640,320), Scalar(128,128,128), 1, 8, 0);
    line(img, Point(320,0), Point(320,480), Scalar(128,128,128), 1, 8, 0);

    const int x=320;

    if (process_angle >= 0) {
        float fi = (-process_angle)*M_PI/180.0;
        int xn=320+x*cos(fi);
        int yn=320+x*sin(fi);
        line(img,Point(320,320),Point(xn,yn),Scalar(0,255,0),1,8,0);
    }

    if (process_angle_min >= 0) {
        float fi = (-process_angle_min)*M_PI/180.0;
        int xn=320+x*cos(fi);
        int yn=320+x*sin(fi);
        line(img,Point(320,320),Point(xn,yn),Scalar(0,255,255),1,8,0);
    }

    if (process_angle_max >= 0) {
        float fi = (-process_angle_max)*M_PI/180.0;
        int xn=320+x*cos(fi);
        int yn=320+x*sin(fi);
        line(img,Point(320,320),Point(xn,yn),Scalar(0,255,255),1,8,0);
    }

    for (int pos = 0; pos < data_cnt; ++pos) {
        // mapping 0..3,2 meters to 10..320 pixels
        float x = data[pos].distance / 10;
        // set minimum drawing length to correctly show color
        if (x < 10) x = 20;
        if (x > 320) x = 320;

        int   qq = data[pos].quality;
        float dd = data[pos].distance;
        float fiact = data[pos].angle;

        float fi = (180.0+fiact)*M_PI/180.0;
        int xn = 320 + x*cos(fi);
        int yn = 320 + x*sin(fi);

        int angle = trunc(180 - fiact);
        if (angle < 0) {
            angle += 360;
        }
        int angle_ok = (angle >= angle_min) && (angle <= angle_max);

        int dist_ok = 0;
        int dist_dmap = 1;
        int dist_stop = 0;
        int qual_ok = 0;
        int qual_dmap = 0;
        int qual_stop = 0;

        qual_ok = (qq >= LIDAR_QUALITY_MIN) && (qq < LIDAR_QUALITY_MAX);
        dist_ok = (dd >= LIDAR_DISTANCE_MIN * 10) && qual_ok;
        if (dist_ok) {
            qual_dmap = qq >= LIDAR_QUALITY_DMAP;
            dist_dmap = (dd >= LIDAR_DISTANCE_DMAP * 10) || (!qual_dmap);
            qual_stop = (qq >= LIDAR_QUALITY_STOP);
            dist_stop = ((fiact >= LIDAR_STOP_ANGLE_MIN) && (fiact <= LIDAR_STOP_ANGLE_MAX) && (dd < LIDAR_DISTANCE_STOP * 10) && qual_stop);
        }

        Scalar color1 = Scalar(200, 200, 200);
        Scalar color2 = Scalar(255, 128,   0);
        if (angle_ok) {
            color1 = Scalar(0, 200, 0);
        }

        if (dist_ok) {
            if (!dist_dmap) {
                color1 = Scalar(0, 0, 128);
                color2 = Scalar(255, 255, 0);
            }
        } else {
            if (angle_ok) {
                color1 = Scalar(0, 100, 0);
                color2 = Scalar(0, 100, 0);
            } else {
                color1 = Scalar(150, 150, 150);
                color2 = Scalar(150, 150, 150);
            }
        }

        if (dist_stop) {
            color1 = Scalar(250, 0, 250);
        }

        line(img,Point(320,320),Point(xn,yn),color1,1,8,0);
        line(img,Point(xn,yn),Point(xn+1,yn),color2,1,8,0);
        // fixme: causes segmentation fault
        //        img.at<Vec3b>(yn,xn)[0] = color2.val[0];
        //        img.at<Vec3b>(yn,xn)[1] = color2.val[1];
        //        img.at<Vec3b>(yn,xn)[2] = color2.val[2];

        if ((fiact >= 0) && (fiact < 180)) {
            int x1 = 50 + (int)(fiact * 3) - 1;
            int x2 = x1 + 2;
            int y0 = 370;       // draw frame top, frame width is 540 pixels, frame height is 101 pixels
            int y1 = y0 + 60;
            int y2 = y0 + 68;
            int y3 = y2 + 1;        // max: y0 + 100

            float dy;
            if (dd >= LIDAR_DISTANCE_MIN * 10) {
                if (dd < LIDAR_DISTANCE_STOP * 10) {
                    dy = 20.0 / (LIDAR_DISTANCE_STOP * 10 - LIDAR_DISTANCE_MIN * 10);
                    y1 = y0 + 60 - (dd - LIDAR_DISTANCE_MIN * 10) * dy;   // 40..60
                } else
                if (dd < LIDAR_DISTANCE_DMAP * 10) {
                    dy = 20.0 / (LIDAR_DISTANCE_DMAP * 10 - LIDAR_DISTANCE_STOP * 10);
                    y1 = y0 + 40 - (dd - LIDAR_DISTANCE_STOP * 10) * dy;  // 20..40
                } else
                if (dd < LIDAR_DISTANCE_MAXD * 10) {
                    dy = 20.0 / (LIDAR_DISTANCE_MAXD * 10 - LIDAR_DISTANCE_DMAP * 10);
                    y1 = y0 + 20 - (dd - LIDAR_DISTANCE_DMAP * 10) * dy;  // 0..20
                } else {
                    y1 = y0 + 0;
                }
            }

            color2 = Scalar(150, 150, 150);
            if (qq >= LIDAR_QUALITY_MIN) {
                if (qq < LIDAR_QUALITY_DMAP) {
                    dy = 8.0 / (LIDAR_QUALITY_DMAP - LIDAR_QUALITY_MIN);
                    y3 = y2 + (qq - LIDAR_QUALITY_MIN) * dy;   // 0..8
                    if (y3 < y2 + 1) {
                        y3 = y2 + 1;  // minimum height to draw
                    }
                } else
                if (qq < LIDAR_QUALITY_STOP) {
                    dy = 8.0 / (LIDAR_QUALITY_STOP - LIDAR_QUALITY_DMAP);
                    y3 = y2 + 8 + (qq - LIDAR_QUALITY_DMAP) * dy;  // 8..16
                    color2 = Scalar(0, 0, 128);
                } else
                if (qq < LIDAR_QUALITY_MAX) {
                    dy = 8.0 / (LIDAR_QUALITY_MAX - LIDAR_QUALITY_STOP);
                    y3 = y2 + 16 + (qq - LIDAR_QUALITY_STOP) * dy;  // 16..24
                    color2 = Scalar(250, 0, 250);
                } else
                if (qq < LIDAR_QUALITY_MAXD) {
                    dy = 8.0 / (LIDAR_QUALITY_MAXD - LIDAR_QUALITY_MAX);
                    y3 = y2 + 24 + (qq - LIDAR_QUALITY_MAX) * dy;  // 32..32
                    color2 = Scalar(0, 255, 255);
                } else {
                    y3 = y2 + 32;
                    color2 = Scalar(0, 255, 255);
                }
            }

            rectangle(img, Point(x1, y1), Point(x2, y2 - 2), color1, cv::FILLED);
            rectangle(img, Point(x1, y2), Point(x2, y3), color2, cv::FILLED);
        }
    }

    sprintf(outgingData,"process_angle: %d", process_angle);
    putText(img, outgingData, Point(30,60), cv::FONT_HERSHEY_DUPLEX, 0.8, Scalar(0, 255,0));
    sprintf(outgingData,"process_angle_min: %d", process_angle_min);
    putText(img, outgingData, Point(30,90), cv::FONT_HERSHEY_DUPLEX, 0.8, Scalar(0, 255,0));
    sprintf(outgingData,"process_angle_max: %d", process_angle_max);
    putText(img, outgingData, Point(30,120), cv::FONT_HERSHEY_DUPLEX, 0.8, Scalar(0, 255,0));
    sprintf(outgingData,"stop: %d", stop);
    putText(img, outgingData, Point(30,150), cv::FONT_HERSHEY_DUPLEX, 0.8, Scalar(0, 255,0));

#ifdef ISTRO_GUI
    LOGM_TRACE(loggerLidarDefs, "drawOutput", "before imshow()...");
    imshow("Lidar", img);
#endif

    timeEnd("Lidar::drawOutput", t);

    return 0;
}
