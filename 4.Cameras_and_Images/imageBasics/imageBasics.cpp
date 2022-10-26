#include <iostream>
#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

int main(int argc, char** argv) {
    // Read the image in argv[1]
    cv::Mat image = cv::imread(argv[1], cv::IMREAD_ANYCOLOR);

    // Check the data is correctly loaded
    if (image.data == nullptr) {        // file does not exist
        std::cerr << "file " << argv[1] << " not exist." << std::endl;
        return 1;
    }

    // Print some basic information
    std::cout << "Image cols: " << image.cols
              << " , rows: " << image.rows
              << " , channels: " << image.channels() << std::endl;

    // Check hte pixels
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    for (size_t y = 0; y < image.rows; ++y) {
        // use cv::Mat::ptr to get the pointer of each row
        uint8_t *row_ptr = image.ptr<uint8_t>(y);                   // row_ptr is the pointer to y-th rows
        
        for (size_t x = 0; x < image.cols; ++x) {
            // read the pixel on (x,y), x=column, y=row
            uint8_t *data_ptr = &row_ptr[x * image.channels()];     // data_ptr is the pointer to (x, y)

            // visit the pixel in each channel
            for (int c = 0; c != image.channels(); ++c) {
                uint8_t data = data_ptr[c];                         // data should be pixel of I(x,y) in c-th channel
            }
        }
    }

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "time used: " << time_used.count() * 1000 << " ms." << std::endl;

    // Copying cv::Mat
    // Operator = will not copy the image data, but the reference
    cv::Mat image_anothor = image;

    // Changing the image_another will also change image
    image_anothor(cv::Rect(0, 0, 100, 100)).setTo(0);               // set top−left 100∗100 block to zero
    cv::imshow("image", image);
    cv::waitKey(0);

    // Use cv::Mat::clone to actually clone the data
    cv::Mat image_clone = image.clone();
    cv::imshow("image", image);
    cv::imshow("image_clone", image_clone);
    cv::waitKey(0);

    return 0;
}