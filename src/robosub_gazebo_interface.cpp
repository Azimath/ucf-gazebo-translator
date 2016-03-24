#include <gate_search.hpp>

static const char* OPENCV_WINDOW = "Image window";

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gate_finder");
    ImageConverter ic;
    ros::spin();
    return 0;
}


ImageConverter::ImageConverter()
    : it_(nh_)
{

    //Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_color",1,
                               &ImageConverter::imageCb,this);
    image_pub_ = it_.advertise("/image_converter/output_video",1);
    target_pub = nh_.advertise<geometry_msgs::Vector3>("targetPos", 1);

    cv::namedWindow(OPENCV_WINDOW);

    cvCreateTrackbar("LowH", OPENCV_WINDOW, &iLowH, 179);
    cvCreateTrackbar("HighH", OPENCV_WINDOW, &iHighH, 179);

    cvCreateTrackbar("LowS", OPENCV_WINDOW, &iLowS, 255);
    cvCreateTrackbar("HighS", OPENCV_WINDOW, &iHighS, 255);

    cvCreateTrackbar("LowV", OPENCV_WINDOW, &iLowV, 255);
    cvCreateTrackbar("HighV", OPENCV_WINDOW, &iHighV, 255);
}

ImageConverter::~ImageConverter()
{
    cv::destroyWindow(OPENCV_WINDOW);
}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr; //Somewhere to keep the image from ROS
    cv::Mat imageGray; //A place to store the incoming image as a greyscale for canny

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //Converts from ROS message to cv image
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what()); //Something went wrong, no idea what
        return;
    }

    cv::Mat hsvImage;
    cv::cvtColor(cv_ptr->image, hsvImage, cv::COLOR_BGR2HSV);

    cv::Mat objectMask;
    cv::inRange(hsvImage, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), objectMask);

    cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(8, 8));
    cv::erode(objectMask, objectMask, erodeElement);
    cv::dilate(objectMask, objectMask, dilateElement);

    cv::Mat andMask;
    if(!prevObjectMask.empty())
    {
        cv::bitwise_and(objectMask, prevObjectMask, andMask);
        prevObjectMask = objectMask;
    }
    else
    {
        andMask = objectMask;
    }

    cv::Mat drawing = cv::Mat::zeros( andMask.size(), CV_8UC3 );

    if(andMask != 0)
    {
        cv::Rect bounds = cv::boundingRect(andMask);

        cv::cvtColor(objectMask, drawing, cv::COLOR_GRAY2RGB);
        cv::rectangle(drawing, bounds, cv::Scalar(255,0,0),1,8,0);
        cv::Point centerPoint = cv::Point(bounds.x+bounds.width/2,bounds.y+bounds.height/2);
        cv::circle(drawing, centerPoint,3, cv::Scalar(0,255,0));
        std::stringstream pointText;
        pointText << "(" << centerPoint.x << ", " << centerPoint.y << ") " << "Ratio: " << (double)bounds.width/(double)bounds.height;
        cv::putText(drawing, pointText.str(), cv::Point(0,drawing.rows*9/10), 0, 1, cv::Scalar(0,0,255));
        cv::resize(drawing, drawing, cv::Size(0, 0), 0.5, 0.5);

        geometry_msgs::Vector3 targetData;
        targetData.x = centerPoint.x-objectMask.cols/2;
        targetData.y = objectMask.rows/2-centerPoint.y;
        targetData.z = (double)bounds.width/(double)bounds.height;

        target_pub.publish(targetData);
        cv::imshow(OPENCV_WINDOW, drawing); //display the incoming image to the user
        cv::waitKey(3);
    }
    else
    {
        ROS_INFO("No target found");
    }

    //cv::Mat drawing



    image_pub_.publish(cv_ptr->toImageMsg());
}
