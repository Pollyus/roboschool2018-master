#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <car_msgs/MotorsControl.h>
#include <stdint.h>

void image_callback(const sensor_msgs::Image::ConstPtr& msg);
void motors(int16_t left, int16_t right);
int16_t truncate(int16_t pwm);

const uint16_t MAX_PWM = 255;

ros::Publisher pub;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "line_follower");
    ros::NodeHandle nh;

    auto sub = nh.subscribe("/car_gazebo/camera1/image_raw", 5, image_callback);
    pub = nh.advertise<car_msgs::MotorsControl>("/motors_commands", 10);

    ros::spin();
    return 0;
}

void image_callback(const sensor_msgs::ImageConstPtr& msg)
{        
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;

    // TODO: control the motors
    // motors(50,50);
    cv::Mat hsvImage;
    cv:cvtColor(image, hsvImage, CV_BGR2HSV);

    // Threshold the HSV image, keep only the red pixels
    cv::Mat mask;
    cv::Scalar lower_red(115, 110, 20);
    cv::Scalar upper_red(195, 255, 140);
    cv::inRange(hsvImage, lower_red, upper_red, mask);

    // TODO: control the motors
    //motors(50,50);

    int width = mask.cols;
    int height = mask.rows;

    int search_top = height / 2;
    int search_bottom = search_top + 150;

    // Zero out pixels outside the desired region
    for (int y = 0; y < height - 2; y++) {
        if (y < search_top || y > search_bottom) {
            for (int x = 0; x < width; x++) {
                mask.at<cv::Vec3b>(y, x)[0] = 0;
                mask.at<cv::Vec3b>(y, x)[1] = 0;
                mask.at<cv::Vec3b>(y, x)[2] = 0;
            }
        }
    }

    cv::Moments M = cv::moments(mask);

    if (M.m00 > 0) {
        int cx = int(M.m10 / M.m00);
        int cy = int(M.m01 / M.m00);
        cv::circle(image, cv::Point(cx, cy), 20, CV_RGB(255, 0, 0), -1);
        //int err = cx - width / 2;
        // Move the robot in proportion to the error signal
        int err = cx - width / 2;
        // geometry_msgs::Twist cmd;
        // cmd.linear.x = 0.1;
        // cmd.angular.z = -(float)err / 500;
        // cmdVelPublisher.publish(cmd);

        if (err<cx)
        // {
            motors(10+err/65,10);
        motor(10,10)
        // }
        // else
        // {
        //     motors(10,10+err/100);
        // }
    }
    else 
    {
        motors(2,0);
    }
    
    // Update the GUI window
    cv::resize(mask, mask, cv::Size(width, height));
    cv::imshow("img", mask);
    cv::waitKey(3);
    // cv::imshow("img", image);
    // cv::waitKey(1);
}

void motors(int16_t left, int16_t right)
{
    car_msgs::MotorsControl msg;
    msg.left = truncate(left);
    msg.right = truncate(right);
    pub.publish(msg);
}

int16_t truncate(int16_t pwm)
{
    if(pwm < -MAX_PWM)
        return -MAX_PWM;
    if(pwm > MAX_PWM)
        return MAX_PWM;
    return pwm;
}