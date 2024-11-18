#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp> // Include this for cv::dnn::blobFromImage
#include <onnxruntime/core/session/onnxruntime_cxx_api.h>
#include <iostream>
#include <vector>
#include <string>

const float CONFIDENCE_THRESHOLD = 0.5;
const std::vector<std::string> CLASS_NAMES = { "ground", "pallet" };

class PalletDetectionNode : public rclcpp::Node
{
public:
    PalletDetectionNode() : Node("pallet_detection_onnx")
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/rgb/image_rect_color", 10, std::bind(&PalletDetectionNode::imageCallback, this, std::placeholders::_1));

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/modified_image", 10);

        // Initialize ONNX runtime
        env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "PalletDetection");
        session_options_ = std::make_unique<Ort::SessionOptions>();
        session_ = std::make_unique<Ort::Session>(*env_, "/home/peer_ws/src/pallet_detection_RT/src/best.onnx", *session_options_);
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat image = cv_ptr->image;

        // Preprocess the image and run it through the ONNX model
        std::vector<float> input_tensor_values = preprocessImage(image);
        std::vector<int64_t> input_shape = {1, 3, image.rows, image.cols};
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(memory_info, input_tensor_values.data(), input_tensor_values.size(), input_shape.data(), input_shape.size());

        // Run the model
        auto output_tensors = session_->Run(Ort::RunOptions{nullptr}, &input_node_names_[0], &input_tensor, 1, &output_node_names_[0], 1);
        auto &output_tensor = output_tensors.front();

        // Postprocess the output and draw bounding boxes
        postprocessAndDrawBoundingBoxes(image, output_tensor);

        // Convert the image back to ROS message and publish
        sensor_msgs::msg::Image::SharedPtr output_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
        image_pub_->publish(*output_msg);
    }

    std::vector<float> preprocessImage(const cv::Mat &image)
    {
        cv::Mat blob;
        cv::dnn::blobFromImage(image, blob, 1.0 / 255.0, cv::Size(image.cols, image.rows), cv::Scalar(), true, false);
        std::vector<float> input_tensor_values(blob.begin<float>(), blob.end<float>());
        return input_tensor_values;
    }

    void postprocessAndDrawBoundingBoxes(cv::Mat &image, const Ort::Value &output_tensor)
    {

        const float* data = output_tensor.GetTensorData<float>();
        size_t numDetections = output_tensor.GetTensorTypeAndShapeInfo().GetShape()[1];

        for (size_t i = 0; i < numDetections; i++)
        {
            int classId = static_cast<int>(data[i * 6 + 1]);
            float confidence = data[i * 6 + 2];
            if (confidence > CONFIDENCE_THRESHOLD)
            {
                int left = static_cast<int>(data[i * 6 + 3] * image.cols);
                int top = static_cast<int>(data[i * 6 + 4] * image.rows);
                int right = static_cast<int>(data[i * 6 + 5] * image.cols);
                int bottom = static_cast<int>(data[i * 6 + 6] * image.rows);
                cv::Rect box(left, top, right - left, bottom - top);
                if (classId == 1){
                    cv::rectangle(image, box, cv::Scalar(0, 255, 0), 2);
                    cv::putText(image, CLASS_NAMES[classId], cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
                }else{
                    cv::rectangle(image, box, cv::Scalar(0, 0, 255), 2);
                    cv::putText(image, CLASS_NAMES[classId], cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
                }
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::SessionOptions> session_options_;
    std::unique_ptr<Ort::Session> session_;
    std::vector<const char *> input_node_names_ = {"input"};
    std::vector<const char *> output_node_names_ = {"output"};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PalletDetectionNode>());
    rclcpp::shutdown();
    return 0;
}