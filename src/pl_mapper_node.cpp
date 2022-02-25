/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "pl_mapper_node.h"

/*****************************************************************************/
// Implementation
/*****************************************************************************/

//InvertImageNode::InvertImageNode(const std::string & node_name, const std::string & node_namespace) 
//    : Node(node_name, node_namespace)
//{
//    rclcpp::QoS video_qos(10);
//    video_qos.keep_last(10);
//    video_qos.reliable();
//    video_qos.durability_volatile();
//
//    subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
//            "/image", video_qos, std::bind(&InvertImageNode::imageRecvCallback, this, std::placeholders::_1));
//
//    char inv_img_topic[100];
//    sprintf(inv_img_topic, "%s/inv_image", this->get_namespace());
//
//    inv_img_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
//            inv_img_topic, video_qos);
//
//    char org_img_topic[100];
//    sprintf(org_img_topic, "%s/org_image", this->get_namespace());
//
//    org_img_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
//            org_img_topic, video_qos);
//
//    int success = XInvert_image_Initialize(&x_inv_img_, "invert_image");
//
//	if (success == XST_DEVICE_NOT_FOUND)
//	{
//		RCLCPP_FATAL(this->get_logger(), "Device invert_image not found");
//		
//        rclcpp::shutdown();
//	}
//
//	if (success == XST_OPEN_DEVICE_FAILED)
//	{
//		RCLCPP_FATAL(this->get_logger(), "Open device invert_image failed");
//		
//        rclcpp::shutdown();
//	}
//
//    if (success != XST_SUCCESS)
//    {
//        RCLCPP_FATAL(this->get_logger(), "Component invert_image initialization failed ");
//    
//        rclcpp::shutdown();
//    }        
//    else{
//        RCLCPP_INFO(this->get_logger(), "Component invert_image initialization successful");
//    }
//}
//
//InvertImageNode::~InvertImageNode()
//{
//    XInvert_image_Release(&x_inv_img_);
//}
//
//void InvertImageNode::imageRecvCallback(const sensor_msgs::msg::Image::SharedPtr msg)
//{
//    RCLCPP_DEBUG(this->get_logger(), "Image received");
//
//    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
//
//    cv::Mat img_grey;
//    cv::cvtColor(cv_ptr->image, img_grey, CV_BGR2GRAY);
//
//    cv::Mat img_grey_out;
//
//    int status = invertImage(img_grey, &img_grey_out);
//
//    if (!status)
//    {
//        RCLCPP_ERROR(this->get_logger(), "Failed inverting image");
//
//        return;
//    }
//
//    RCLCPP_DEBUG(this->get_logger(), "Successfully inverted image");
//
//    cv_bridge::CvImage inv_img_out;
//    inv_img_out.header.set__frame_id(msg->header.frame_id);
//    inv_img_out.header.set__stamp(rclcpp::Clock().now());
//    inv_img_out.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
//    inv_img_out.image = img_grey_out;
//
//    cv_bridge::CvImage org_img_out;
//    org_img_out.header.set__frame_id(msg->header.frame_id);
//    org_img_out.header.set__stamp(rclcpp::Clock().now());
//    org_img_out.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
//    org_img_out.image = img_grey;
//
//    sensor_msgs::msg::Image::SharedPtr inv_msg_out = inv_img_out.toImageMsg();
//    sensor_msgs::msg::Image::SharedPtr org_msg_out = org_img_out.toImageMsg();
//    
//    publishInvertedImage(*inv_msg_out, *org_msg_out);
//}
//
//
//void InvertImageNode::publishInvertedImage(const sensor_msgs::msg::Image inv_img_msg, const sensor_msgs::msg::Image org_img_msg)
//{
//    RCLCPP_DEBUG(this->get_logger(), "Publishing inverted image");
//
//    inv_img_publisher_->publish(inv_img_msg);
//    org_img_publisher_->publish(org_img_msg);
//}
//
//int InvertImageNode::invertImage(const cv::Mat img_grey, cv::Mat *ptr_inv_img_grey)
//{
//    RCLCPP_DEBUG(this->get_logger(), "Calling invert_image");
//
//    if (img_grey.total() != SIZE)
//    {
//        RCLCPP_ERROR(this->get_logger(), "Expected image of size %d, got size %d, aborting", SIZE, img_grey.total());
//
//        return 0;
//    }
//
//    std::vector<uint8_t> img_vec_in;
//
//    img_vec_in.assign(img_grey.data, img_grey.data + img_grey.total());
//
//    uint8_t img_out_arr[SIZE];
//
//    int success = callIP(&img_vec_in[0], &img_out_arr[0]);
//
//    if (!success)
//    {
//        RCLCPP_ERROR(this->get_logger(), "Unsuccessful call to IP");
//
//        return 0;
//    }
//
//    RCLCPP_DEBUG(this->get_logger(), "Successful call to IP");
//
//    *ptr_inv_img_grey = cv::Mat(IMAGE_X, IMAGE_Y, CV_8UC1, &img_out_arr);
//
//    return 1;
//}
//
//int InvertImageNode::callIP(const uint8_t *ptr_img_data_in, const uint8_t *ptr_img_data_out)
//{
//    int length;
//
//    RCLCPP_DEBUG(this->get_logger(), "Polling for invert_image IP ready");
//
//    while(!XInvert_image_IsReady(&x_inv_img_));
//
//    length = XInvert_image_Write_image_in_Bytes(&x_inv_img_, 0, (char *)ptr_img_data_in, SIZE);
//
//    if(length == SIZE)
//    {
//        RCLCPP_DEBUG(this->get_logger(), "Wrote data to invert_image IP");
//    } else
//    {
//        RCLCPP_ERROR(this->get_logger(), "Could not write data to invert_image IP");
//        
//        return 0;
//    }
//
//    RCLCPP_DEBUG(this->get_logger(), "Polling for invert_image IP idle");
//
//    while(!XInvert_image_IsIdle(&x_inv_img_));
//
//    RCLCPP_DEBUG(this->get_logger(), "Starting invert_image IP");
//
//    XInvert_image_Start(&x_inv_img_);
//
//    RCLCPP_DEBUG(this->get_logger(), "Started invert_image IP, polling for IP done");
//
//    while(!XInvert_image_IsDone(&x_inv_img_));
//
//    RCLCPP_DEBUG(this->get_logger(), "Reading data form invert_image IP");
//
//    length = XInvert_image_Read_image_out_Bytes(&x_inv_img_, 0, (char *)ptr_img_data_out, SIZE);
//
//    if(length == SIZE)
//    {
//        RCLCPP_DEBUG(this->get_logger(), "Read data from invert_image IP");
//    } else
//    {
//        RCLCPP_ERROR(this->get_logger(), "Could not read data from invert_image IP");
//        
//        return 0;
//    }
//
//    return 1;
//}

int main(int argc, char *argv[])
{
    //rclcpp::init(argc, argv);

    //rclcpp::executors::SingleThreadedExecutor executor;

    //auto node = std::make_shared<InvertImageNode>();

    //executor.add_node(node);

    //executor.spin();

    //rclcpp::shutdown();

    return 0;
}