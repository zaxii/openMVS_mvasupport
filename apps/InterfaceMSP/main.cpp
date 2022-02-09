// Copyright 2022 Shaun Song <sxsong1207@qq.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <spdlog/spdlog.h>
#include <string>
#include <fstream>
#include <iostream>

#include <lasreader.hpp>
#include "ghc/filesystem.hpp"
#include <Eigen/Eigen>
#include <unordered_map>
#include "MVArchive/ARInterface.h"
using namespace std;
namespace fs = ghc::filesystem;
struct QinPose{
    string imgname;
    double f;
    double cx;
    double cy;
    int dummy1;
    int dummy2;
    int width;
    int height;
    double x,y,z;
    double omega, phi, kappa;

    Eigen::Matrix3d GetK() const
    {
        Eigen::Matrix3d K;
        K<<f,0,double(width-1)/2.0+cx,0,f,double(height-1)/2.0+cy,0,0,1;
        return K;
    }

    /**
     Implicitly convert Photogrammetry CS to ComputerVision CS
      [ cos(kappa)*cos(phi), cos(omega)*sin(kappa) + cos(kappa)*sin(omega)*sin(phi),   sin(kappa)*sin(omega) - cos(kappa)*cos(omega)*sin(phi)]
      [ cos(phi)*sin(kappa), sin(kappa)*sin(omega)*sin(phi) - cos(kappa)*cos(omega), - cos(kappa)*sin(omega) - cos(omega)*sin(kappa)*sin(phi)]
      [           -sin(phi),                                    cos(phi)*sin(omega),                                     -cos(omega)*cos(phi)]
     * @return
     */
    Eigen::Matrix3d GetR() const {
        Eigen::Matrix3d Rph;
        Rph << cos(kappa)*cos(phi), cos(omega)*sin(kappa) + cos(kappa)*sin(omega)*sin(phi),   sin(kappa)*sin(omega) - cos(kappa)*cos(omega)*sin(phi),
                cos(phi)*sin(kappa), sin(kappa)*sin(omega)*sin(phi) - cos(kappa)*cos(omega), - cos(kappa)*sin(omega) - cos(omega)*sin(kappa)*sin(phi),
                -sin(phi), cos(phi)*sin(omega), -cos(omega)*cos(phi);
        return Rph;
    }

    Eigen::Vector3d GetC() const{
        Eigen::Vector3d C(x,y,z);
        return C;
    }
};
istream& operator>>(istream& is, QinPose &qin)
{
    is>>qin.imgname>>qin.f>>qin.cx>>qin.cy>>
      qin.dummy1>>qin.dummy2>>qin.width>>qin.height>>
      qin.x>>qin.y>>qin.z>>qin.omega>>qin.phi>>qin.kappa;
    return is;
}

ostream& operator<<(ostream& os, const QinPose &qin)
{
    os<<qin.imgname<<" "<<qin.f<<" "<<qin.cx<<" "<<qin.cy<<
      " 1 1 "<<qin.width<<" "<<qin.height<<
      qin.x<<" "<<qin.y<<" "<<qin.z<<" "<<qin.omega<<" "<<qin.phi<<" "<<qin.kappa<<"\n";
    return os;
}

inline MVSA::Interface::Mat33d Eigen_to_MVSA_mat3(const Eigen::Matrix3d mat)
{
    return MVSA::Interface::Mat33d(mat(0,0),mat(0,1),mat(0,2),
                                   mat(1,0),mat(1,1),mat(1,2),
                                   mat(2,0),mat(2,1),mat(2,2));
}

inline MVSA::Interface::Pos3d Eigen_to_MVSA_vec3(const Eigen::Vector3d vec){
    return MVSA::Interface::Pos3d(vec(0),vec(1),vec(2));
}
#include <cxxopts.hpp>

cxxopts::Options parseOptions() {
    cxxopts::Options options("InterfaceMSP", "fuse");
    options.add_options()
            ("outputMVA", "output .mva", cxxopts::value<std::string>())
            ("inputQin", "input qin file", cxxopts::value<std::string>())
            ("f,inputLasFiles", "input las files", cxxopts::value<std::vector<std::string>>())
            ("l,inputLasList", "input las file list", cxxopts::value<std::string>())
            ("s,step", "include point per steps", cxxopts::value<int>()->default_value("1"))
            ("m,mode", "id mode of las file", cxxopts::value<std::string>()->default_value("GLOBAL"))
            ("h,help", "Print usage of program");
    options.parse_positional({"outputMVA", "inputQin", "inputLasFiles"});
    options.positional_help("outputMVA inputQin [ -f inputLas#1 inputLas#2 ... | -l inputLasLists.txt ]");
    return options;
}

int main(int argc, char** argv)
{
    spdlog::set_level(spdlog::level::info);

    auto options = parseOptions();
    auto result = options.parse(argc, argv);
    if (result.count("help")) {
        std::cout << options.help() << std::endl;
        exit(0);
    }
    if(result.count("outputMVA")==0 || result.count("inputQin")==0 ||
        (result.count("inputLasFiles")==0 && result.count("inputLasList")==0)){
        std::cout << options.help() << std::endl;
        exit(0);
    }

    string output_mva = result["outputMVA"].as<std::string>();
    string input_qin = result["inputQin"].as<std::string>();
    vector<string> input_lases;

    if(result.count("inputLasList")!=0)
    {
        std::ifstream ifs(result["inputLasList"].as<std::string>());
        if(!ifs.is_open())
        {
            spdlog::error("Input Las list not found: {0}", result["inputLasList"].as<std::string>());
            return 1;
        }

        while(ifs.good())
        {
            std::string line;
            ifs>>line;
            if(line.size()>3)
                input_lases.push_back(line);
        }
        ifs.close();
    }
    else
        input_lases = result["inputLasFiles"].as<std::vector<std::string>>();

    int sample_per_steps = result["step"].as<int>();
    bool isGlobalId = true;
    if(result["mode"].as<std::string>() != "GLOBAL")
        isGlobalId = false;

    spdlog::info("MVA: {0}", output_mva);
    spdlog::info("Qin: {0}", input_qin);
    spdlog::info("Las files: {0}", input_lases.size());
    spdlog::info("sample per steps: {0}", sample_per_steps);
    spdlog::info("image ID encoder: {0}", isGlobalId? "Global" : "Bit");
    for(auto& i: input_lases)
        spdlog::debug("Las: {0}", i);

    if(!fs::exists(fs::path(input_qin)))
    {
        spdlog::error("input Qin file not exists: {0}", input_qin);
        return 1;
    }
    if(!fs::exists(fs::path(output_mva).parent_path()))
        fs::create_directories(fs::path(output_mva).parent_path());
    for(auto& i: input_lases)
    {
        spdlog::info("{0}",i);
        if(!fs::exists(fs::path(i))){
            spdlog::error("input las file not exists: {0}", i);
            return 1;
        }
    }

    ifstream qinifs(input_qin);
    size_t num_pose;
    qinifs>>num_pose;
    spdlog::info("Read Qin file: {0} images", num_pose);
    std::vector<QinPose> qinposes(num_pose);
    std::unordered_map<string, size_t> name_to_idx;
    for(size_t i=0;i<num_pose;++i)
    {
        qinifs>>qinposes[i];
        string basename = fs::path(qinposes[i].imgname).filename().stem().string();
        name_to_idx[basename] = i;
    }
    spdlog::info("Process poses...");
    /// Inject Camera poses
    MVSA::Interface obj;
    for(size_t i=0;i<qinposes.size();++i)
    {
        const QinPose& qinpose = qinposes[i];
        MVSA::Interface::Platform platform;
        MVSA::Interface::Platform::Camera cameraint;
        MVSA::Interface::Platform::Pose pose;
        MVSA::Interface::Image img;

        cameraint.width = qinpose.width;
        cameraint.height = qinpose.height;
        cameraint.name = qinpose.imgname;
        cameraint.C = Eigen_to_MVSA_vec3(Eigen::Vector3d::Zero());
        cameraint.R = Eigen_to_MVSA_mat3(Eigen::Matrix3d::Identity());
        auto K = qinpose.GetK();
        double nzScale = MVSA::Interface::Platform::Camera::GetNormalizationScale(qinpose.width,qinpose.height);
        K.topRows<2>(0) /= nzScale;
        cameraint.K = Eigen_to_MVSA_mat3(K);
        img.ID = i;
        img.platformID = i;
        img.cameraID = 0;
        img.poseID = 0;
        img.name = qinpose.imgname;
        if(!img.IsValid()) spdlog::warn("Image {0} is invalid!", qinpose.imgname);
        pose.C = Eigen_to_MVSA_vec3(qinpose.GetC());
        pose.R = Eigen_to_MVSA_mat3(qinpose.GetR());
        platform.cameras.push_back(cameraint);
        platform.poses.push_back(pose);
        obj.platforms.push_back(platform);
        obj.images.push_back(img);
    }

    // Inject point cloud and visibility
    spdlog::info("Process LAS...");
    LASreadOpener lasreadopener;
    size_t total_num_points = 0;
    for(auto input_las: input_lases)
    {
        lasreadopener.set_file_name(input_las.c_str());
        if (!lasreadopener.active())
        {
            spdlog::error("No input specified");
        }
        LASreader* lasreader = lasreadopener.open();
        LASheader* lasheader = &lasreader->header;
        total_num_points += lasheader->number_of_point_records;
        lasreader->close();
    }
    spdlog::info("Total {0} are points found from {1} las files.",
                 total_num_points, input_lases.size());
    if(sample_per_steps>0)
    {
        obj.vertices.reserve(total_num_points/sample_per_steps);
        obj.verticesColor.reserve(total_num_points/sample_per_steps);
    }else{
        obj.vertices.reserve(total_num_points);
        obj.verticesColor.reserve(total_num_points);
    }

    size_t progress = 0;
    for(auto input_las: input_lases)
    {
        lasreadopener.set_file_name(input_las.c_str());
        if (!lasreadopener.active())
        {
            spdlog::error("No input specified");
        }
        LASreader* lasreader = lasreadopener.open();
        LASheader* lasheader = &lasreader->header;
        size_t num_attributes = lasheader->number_attributes;
        size_t step_attribute = 2;
        size_t pt_cnt = 0;
        while(lasreader->read_point())
        {
            pt_cnt++;
            if(pt_cnt % sample_per_steps != 0 && sample_per_steps>0) continue;

            MVSA::Interface::Vertex vtx;
            MVSA::Interface::Vertex::ViewArr  varr;
            MVSA::Interface::Color col;
            vtx.X.x = lasreader->point.get_x();
            vtx.X.y = lasreader->point.get_y();
            vtx.X.z = lasreader->point.get_z();
            for(int vi=0;vi<num_attributes;++vi) {
                uint16_t visId;
                lasreader->point.get_attribute(vi*step_attribute, visId);
                if(visId == 65535) continue;
                MVSA::Interface::Vertex::View vw;
                vw.imageID = visId;
                vw.confidence = 1.f;
                varr.push_back(vw);
            }
            vtx.views = varr;
            col.c.x = lasreader->point.get_R();
            col.c.y = lasreader->point.get_G();
            col.c.z = lasreader->point.get_B();
            obj.vertices.push_back(vtx);
            obj.verticesColor.push_back(col);
        }
        lasreader->close();
        progress ++;
        spdlog::info("Process LAS {0}/{1}",progress,input_lases.size());
    }

    spdlog::info("Save to Disk: {0}", output_mva);
    MVSA::MVArchive::SerializeSave(obj,output_mva);
    return 0;
}