#include <iostream>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <opencv/highgui.h>
#include <string>
int main(int argc, char* argv[] ) {
    int number = 0;
    char key;

    std::string file_name = argv[1];
    std::string output_names = "Outputs";

    rs2::pipeline pipe;
    rs2::config cfg;

    cfg.enable_device_from_file(file_name.c_str());
    pipe.start(cfg);

    
    ///////////////////////////////////////////////////////////////////////////////
    rs2::decimation_filter dec_filter;
    dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 3);
    rs2::disparity_transform depth_to_disparity(true);
    rs2::disparity_transform disparity_to_depth(false);
    rs2::spatial_filter spat_filter;
    spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5);
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
    spat_filter.set_option(RS2_OPTION_HOLES_FILL, 0);
    rs2::temporal_filter temp_filter;
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4);
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
    temp_filter.set_option(RS2_OPTION_HOLES_FILL, 3);
    rs2::hole_filling_filter hf_filter;
    hf_filter.set_option(RS2_OPTION_HOLES_FILL, 1);
    //////////////////////////////////////////////////////////////////////////////

    //rs2::device device = pipe.get_active_profile().get_device();
    //rs2::playback playback = device.as<rs2::playback>();
    rs2::frameset frames;
    rs2::points points;
    rs2::pointcloud pc;

    //cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
   // cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_ANY, 30);
    while(true)
    {
        if(pipe.poll_for_frames(&frames))
        {
            auto frame = pipe.wait_for_frames();
            auto depth_frame = frame.get_depth_frame();
            auto color_frame = frame.get_color_frame();
            //rs2::frameset frames=pipe.wait_for_frames();


            cv::Mat color_mat = cv::Mat(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3, const_cast<void *>( color_frame.get_data()));
            cv::Mat depth(cv::Size(depth_frame.get_width(), depth_frame.get_height()), CV_16SC1, (void *) depth_frame.get_data(), cv::Mat::AUTO_STEP);

            //cv::cvtColor(color_mat,color_mat,CV_RGB2BGR);
            // cv::Mat color_mat = cv::Mat(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3, const_cast<void *>( color_frame.get_data()),cv::Mat::AUTO_STEP);
            //cv::Mat depth(cv::Size(depth_frame.get_width(), depth_frame.get_height()), CV_16SC1, (void *) depth_frame.get_data(), cv::Mat::AUTO_STEP);
            //cv::Mat color_mat(cv::Size(1280, 720), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            //cv::Mat depth(cv::Size(1280, 720), CV_8UC3, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

            // filter
            depth_frame = dec_filter.process(depth_frame);
            depth_frame = depth_to_disparity.process(depth_frame);
            depth_frame = spat_filter.process(depth_frame);
            depth_frame = temp_filter.process(depth_frame);
            depth_frame = disparity_to_depth.process(depth_frame);
            depth_frame = hf_filter.process(depth_frame);


            points=pc.calculate(depth_frame);
            pc.map_to(color_frame);
            points.export_to_ply("test.ply",color_frame);
            
           // cv::cvtColor(color_mat,color_mat,CV_RGB2BGR);
            depth.convertTo(depth, CV_8U, -255.0/10000.0, 255.0);
            cv::equalizeHist(depth, depth);

            cv::namedWindow("color", cv::WINDOW_AUTOSIZE);
            cv::imshow("color", color_mat);
            cv::namedWindow("depth", cv::WINDOW_AUTOSIZE);
            cv::imshow("depth", depth);


            key = cv::waitKey(1);

            if(key=='s'){

                output_names = std::to_string(number)+".ply";
                points.export_to_ply(output_names,color_frame);
                number++;
                std::cout << "save ply_file" << std::endl;
            }


        }
        
        
    }
    return 0;
}
