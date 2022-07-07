#include "util.h"

#include <sstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <cassert>

#include <opencv2/imgproc/imgproc.hpp>

images_sequence::images_sequence(const std::string& seq_dir_path, const std::string& img_left_folder_name, const std::string& img_right_folder_name) {
    // load timestamps
    const std::string timestamp_file_path = seq_dir_path + "times.txt";
    ifs_timestamp.open(timestamp_file_path.c_str());
    cv::glob(seq_dir_path + "/image_0/*.jpg", img_path_vector);//or image_1
    //std::cout<<img_path_vector.size()<<std::endl;
    if (ifs_timestamp) {
        //throw std::runtime_error("Could not load a timestamp file from " + timestamp_file_path);
        assert(timestamps_.size() == img_path_vector.size());
        timestamps_.clear();
				while (!ifs_timestamp.eof()) {
				    std::string s;
				    getline(ifs_timestamp, s);
				    if (!s.empty()) {
				        std::stringstream ss;
				        ss << s;
				        double timestamp;
				        ss >> timestamp;
				        timestamps_.push_back(timestamp);
				    }
				}

				ifs_timestamp.close();
    }
    else
    {
    	std::cout<<"Could not load a timestamp file from " + timestamp_file_path<<", "<<"using frames ids"<<std::endl;
    }

    // load image file paths
    const std::string left_img_dir_path = seq_dir_path + img_left_folder_name;
    const std::string right_img_dir_path = seq_dir_path + img_right_folder_name;
    
    std::cout<<left_img_dir_path<<" "<<right_img_dir_path<<std::endl;

    left_img_file_paths_.clear();
    right_img_file_paths_.clear();
    for (unsigned int i = 0; i < img_path_vector.size(); ++i)
    {
        left_img_file_paths_.push_back(img_path_vector[i]);
        right_img_file_paths_.push_back(img_path_vector[i]);
        //std::cout<<left_img_dir_path + ss.str() + ".jpg"<<std::endl;
    }
}

std::vector<images_sequence::frame> images_sequence::get_frames() const {
    std::vector<frame> frames;
    if (ifs_timestamp){
		  assert(timestamps_.size() == left_img_file_paths_.size());
		  assert(timestamps_.size() == right_img_file_paths_.size());
    }
    assert(left_img_file_paths_.size() == right_img_file_paths_.size());
    for (unsigned int i = 0; i < img_path_vector.size(); ++i) {
    		if (ifs_timestamp)
	        frames.emplace_back(frame{left_img_file_paths_.at(i), right_img_file_paths_.at(i), timestamps_.at(i)});
        else
	        frames.emplace_back(frame{left_img_file_paths_.at(i), right_img_file_paths_.at(i), i});
    }
    return frames;
}
