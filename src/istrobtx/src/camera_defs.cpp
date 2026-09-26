#include <cstdio>

#include "camera_defs.h"

using namespace cv;
using namespace std;

#ifndef WIN32
static const string DEPTH_DATA_FNAME = "out/camera_depth.json";
#else
static const string DEPTH_DATA_FNAME = "out\\camera_depth.json";
#endif

void camera_draw_depth_frame(const Mat &frame, Mat &image, long image_number, int device_id)
{
    FILE * pFile;
    pFile = fopen(DEPTH_DATA_FNAME.c_str(), "a");
    if (pFile!=NULL) {
        fprintf(pFile, "{\"dev_id\":%d,\"image_number\":%d,\"cdepth_data\":[\n", device_id, (int)image_number);

        int sep = 0;
        int dy = frame.rows / 5;
        int dx = frame.cols / 4;

        for (int y = 0; y < frame.rows; y++) {
            if ((y == 0) || (y % dy) != 0) continue;

            if (sep) fputs(",\n", pFile);
            fprintf(pFile, "{\"row\":%d,\"d\":[", y);
            for (int x = 0; x < frame.cols - 1; x++) {
                fprintf(pFile, "%d,", (int)frame.at<ushort>(y, x));
            }
            int x = frame.cols - 1;
            fprintf(pFile, "%d]}", (int)frame.at<ushort>(y, x));
            sep = 1;
        }

        for (int x = 0; x < frame.cols; x++) {
            if ((x == 0) || (x % dx) != 0) continue;

            if (sep) fputs(",\n", pFile);
            fprintf(pFile, "{\"col\":%d,\"d\":[", x);
            for (int y = 0; y < frame.rows - 1; y++) {
                fprintf(pFile, "%d,", (int)frame.at<ushort>(y, x));
            }
            int y = frame.rows - 1;
            fprintf(pFile, "%d]}", (int)frame.at<ushort>(y, x));
            sep = 1;
        }

        if (sep) fputs("]},\n", pFile);
        fclose(pFile);
    }

    frame.convertTo(image, CV_8U, 1 / 256.0);
    equalizeHist(image, image);
    applyColorMap(image, image, COLORMAP_RAINBOW);  //COLORMAP_JET);
}
