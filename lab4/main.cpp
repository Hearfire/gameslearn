#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    int n=control_points.size();
    if(n<1)return cv::Point2f();
    if(n==1)return control_points[0];

    std::vector<cv::Point2f> next_control_points;
    for(int i=0;i<n-1;++i){
        auto &p_0 = control_points[i];
        auto &p_1 = control_points[i+1];
        cv::Point2f p=(1-t)*p_0+t*p_1;
        next_control_points.push_back(p);
    }
    return recursive_bezier(next_control_points,t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    //float t=0;
    //int N=100;

    for(double t = 0.0; t <= 1.0; t += 0.001){
        cv::Point2f point=recursive_bezier(control_points,t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
        float center_x=round(point.x),center_y=round(point.y);
        float pixel_bias[5]={-0.5,0.5,0.5,-0.5,-0.5};

        float mindist=pow(center_x-point.x,2)+pow(center_y-point.y,2);
        for(int i=0;i<4;++i){
            float nowx=center_x+pixel_bias[i],nowy=center_y+pixel_bias[i+1];
            float weight=mindist/((nowx-point.x,2)+pow(nowy-point.y,2));
            window.at<cv::Vec3b>(nowy, nowx)[1] = std::max(255*weight,(float)window.at<cv::Vec3b>(nowy, nowx)[1]);
        }

    }

}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            // naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
