//
// Created by redwan on 4/30/22.
//

#ifndef MPCHRVO_MAP_PARSER_H
#define MPCHRVO_MAP_PARSER_H
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ostream>


using namespace std;




class MapParser{
    using PWP = vector<pair<cv::Point, cv::Point>>; // piecewise path
    using WP = vector<vector<float>>;
public:

    MapParser(const string& filename, double low, double high){
        // read image in gray scale
        img_ = cv::imread(filename, 0);
        this->low_ = low;
        this->high_ = high;
    }
    /**
     * convert coordinate value from workspace to image domain
     * @param X scalar coordinate value in workspace e.g., x
     * @param img_max maximum axis in image domain
     * @return coordinate in image domain
     */
    int image_scale(float X, float img_max){
        float img_min = 0;
        float X_std = (X - low_) / (high_ - low_);
        int X_scaled = X_std * (img_max - img_min) + img_min;
        return X_scaled;
    }


    /**
     * obtain field intensity for a particular point
     * @param rx robot x coordinate
     * @param ry robot y coordinate
     * @return z measured value from the image i.e., filed intensity
     */
    int get_reading(float rx, float ry){
        if( (low_ <= rx && rx < high_) && (low_ <= ry && ry < high_))
        {
            int height = img_.size[0]; // y_axis
            int width = img_.size[1]; // x_axis
            int x = image_scale(rx, width - 1);
            int y = height - image_scale(ry, height - 1);
            int pixelValue = 0;
            if( (0 <= x && x < width) && (0 <= y && y < height))
                pixelValue = (int)img_.at<uchar>(x, y);
            return pixelValue;
        }
        return 0;
    }



private:
    cv::Mat img_;;
    double low_, high_;
};



#endif //MPCHRVO_MAP_PARSER_H
