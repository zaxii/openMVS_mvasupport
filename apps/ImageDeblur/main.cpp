#include <iostream>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui.hpp>
#include <spdlog/spdlog.h>
#include "ghc/filesystem.hpp"

#include <omp.h>

namespace fs=ghc::filesystem;

using namespace cv;
using namespace std;

void calcPSF(Mat& outputImg, Size filterSize, int R);
void fftshift(const Mat& inputImg, Mat& outputImg);
void filter2DFreq(const Mat& inputImg, Mat& outputImg, const Mat& H);
void calcWnrFilter(const Mat& input_h_PSF, Mat& output_G, double nsr);

#include <cxxopts.hpp>

cxxopts::Options parseOptions() {
    cxxopts::Options options("InterfaceMSP", "fuse");
    options.add_options()
            ("i,input", "input image", cxxopts::value<std::string>())
            ("o,output", "output image", cxxopts::value<std::string>())
            ("r,radius", "input las file list", cxxopts::value<int>()->default_value("1"))
            ("s,snr", "signal to noise ratio", cxxopts::value<float>()->default_value("100"))
            ("h,help", "Print usage of program");
    options.parse_positional({"input", "output"});
    options.positional_help("input_image output_image or input_image_list output_image_directory [ ... options ]");
    return options;
}

int process(std::string input, std::string output, int radius, float snr)
{
    Mat imgInBGR=imread(input);
    if (imgInBGR.empty()) //check whether the image is loaded or not
    {
        spdlog::warn("Image is empty: {0}", input);
        return 0;
    }
    Mat imgOutBGR;
    Mat bgr[3], outBgr[3];
    cv::split(imgInBGR, bgr);
    for(int i=0;i<3;++i)
    {
        cv::Mat& imgIn = bgr[i];
        Mat& imgOut = outBgr[i];
        // it needs to process even image only
        Rect roi = Rect(0, 0, imgIn.cols & -2, imgIn.rows & -2);
        //Hw calculation (start)
        Mat Hw, h;
        calcPSF(h, roi.size(), radius);
        calcWnrFilter(h, Hw, 1.0 / double(snr));
        //Hw calculation (stop)
        // filtering (start)
        filter2DFreq(imgIn(roi), imgOut, Hw);
        // filtering (stop)
        imgOut.convertTo(imgOut, CV_8U);
        normalize(imgOut, imgOut, 0, 255, NORM_MINMAX);
    }
    cv::merge(outBgr, 3, imgOutBGR);
    cv::imwrite(output, imgOutBGR);
    return 0;
}
int main(int argc, char *argv[])
{
    spdlog::set_level(spdlog::level::info);

    auto options = parseOptions();
    auto result = options.parse(argc, argv);
    if (result.count("help")) {
        std::cout << options.help() << std::endl;
        exit(0);
    }
    bool batch_mode = false;
    if(result.count("input")==0 || result.count("output")==0)
    {
        std::cout << options.help() << std::endl;
        exit(0);
    }
    fs::path input_path (result["input"].as<std::string>()),
        output_path(result["output"].as<std::string>());
    if(!fs::exists(input_path))
    {
        spdlog::error("Input not exists: {0}",input_path.string());
        std::cout << options.help() << std::endl;
        exit(1);
    }
    spdlog::info("Input Path: {0}",input_path.string());
    spdlog::info("Output Path: {0}", output_path.string());

    std::vector<std::string> input_image_paths;
    std::vector<std::string> output_image_paths;
    if(input_path.extension().string()==".txt")
    {
        batch_mode=true;
        if(!fs::exists(output_path)) fs::create_directories(output_path);
        spdlog::info("Batch Mode");
        ifstream ifs(input_path.string());
        if(!ifs.is_open())
        {
            spdlog::error("Input cannot been opened: {0}",input_path.string());
            std::cout << options.help() << std::endl;
            exit(1);
        }
        while(ifs.good())
        {
            std::string line;
            ifs>>line;
            if(!line.empty()){
                fs::path imgpath(line);
                fs::path outimgpath = output_path / imgpath.filename();
                if(!fs::exists(imgpath))
                {
                    spdlog::error("Image file not found: {0}",imgpath.string());
                    exit(1);
                }
                input_image_paths.push_back(imgpath.string());
                output_image_paths.push_back(outimgpath.string());
            }
        }
        spdlog::info("Found {0} images.",input_image_paths.size());
    }else
    {
        batch_mode=false;
        if(!fs::exists(output_path.parent_path())) fs::create_directories(output_path.parent_path());
        spdlog::info("One Shot Mode");
        input_image_paths.push_back(input_path.string());
        output_image_paths.push_back(output_path.string());
    }


    int radius = result["radius"].as<int>();
    float snr = result["snr"].as<float>();
    spdlog::info("Radius: {0}    SNR: {1}",radius, snr);

    size_t num_imgs = input_image_paths.size();

    int finished = 0;
#pragma omp parallel for
    for(int i=0;i<num_imgs;++i)
    {
        process(input_image_paths[i],output_image_paths[i], radius, snr);
        if (batch_mode)
        {
#pragma omp critical
            finished += 1;
            spdlog::info("Processed {0}/{1}...", finished, num_imgs);
        }
    }
    spdlog::info("Done.");
    return 0;
}
void help()
{
    cout << "2018-07-12" << endl;
    cout << "DeBlur_v8" << endl;
    cout << "You will learn how to recover an out-of-focus image by Wiener filter" << endl;
}

void gaussianPSF(Mat& outputImg, Size filterSize, int R)
{
    Mat h(filterSize, CV_32F, Scalar(0));
    Point point(filterSize.width / 2, filterSize.height / 2);
    circle(h, point, 1, 255, -1, 8);
    GaussianBlur(h,h,cv::Size(5,5),R);
    Scalar summa = sum(h);
    outputImg = h / summa[0];
}

void calcPSF(Mat& outputImg, Size filterSize, int R)
{
    Mat h(filterSize, CV_32F, Scalar(0));
    Point point(filterSize.width / 2, filterSize.height / 2);
    circle(h, point, R, 255, -1, 8);
    Scalar summa = sum(h);
    outputImg = h / summa[0];
}

void fftshift(const Mat& inputImg, Mat& outputImg)
{
    outputImg = inputImg.clone();
    int cx = outputImg.cols / 2;
    int cy = outputImg.rows / 2;
    Mat q0(outputImg, Rect(0, 0, cx, cy));
    Mat q1(outputImg, Rect(cx, 0, cx, cy));
    Mat q2(outputImg, Rect(0, cy, cx, cy));
    Mat q3(outputImg, Rect(cx, cy, cx, cy));
    Mat tmp;
    q0.copyTo(tmp);
    q3.copyTo(q0);
    tmp.copyTo(q3);
    q1.copyTo(tmp);
    q2.copyTo(q1);
    tmp.copyTo(q2);
}
void filter2DFreq(const Mat& inputImg, Mat& outputImg, const Mat& H)
{
    Mat planes[2] = { Mat_<float>(inputImg.clone()), Mat::zeros(inputImg.size(), CV_32F) };
    Mat complexI;
    merge(planes, 2, complexI);
    dft(complexI, complexI, DFT_SCALE);
    Mat planesH[2] = { Mat_<float>(H.clone()), Mat::zeros(H.size(), CV_32F) };
    Mat complexH;
    merge(planesH, 2, complexH);
    Mat complexIH;
    mulSpectrums(complexI, complexH, complexIH, 0);
    idft(complexIH, complexIH);
    split(complexIH, planes);
    outputImg = planes[0];
}
void calcWnrFilter(const Mat& input_h_PSF, Mat& output_G, double nsr)
{
    Mat h_PSF_shifted;
    fftshift(input_h_PSF, h_PSF_shifted);
    Mat planes[2] = { Mat_<float>(h_PSF_shifted.clone()), Mat::zeros(h_PSF_shifted.size(), CV_32F) };
    Mat complexI;
    merge(planes, 2, complexI);
    dft(complexI, complexI);
    split(complexI, planes);
    Mat denom;
    pow(abs(planes[0]), 2, denom);
    denom += nsr;
    divide(planes[0], denom, output_G);
}