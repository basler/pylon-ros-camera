/*
 * hdr_generator.cpp
 *
 *  Created on: Jul 13, 2015
 *      Author: md
 */

#include <pylon_camera/hdr_generator.h>

namespace pylon_camera {

HDRGenerator::HDRGenerator()
{
    // TODO Auto-generated constructor stub

}

HDRGenerator::~HDRGenerator()
{
    // TODO Auto-generated destructor stub
}

bool HDRGenerator::mergeMertens(vector<Mat> img_sequence, vector<float> exp_times, Mat &hdr_img)
{
//    cout << "CHECK 01" << endl;
//
    imwrite("00input.png", img_sequence.at(0));
    imwrite("01input.png", img_sequence.at(1));
    imwrite("02input.png", img_sequence.at(2));
//
//    cout << "CHECK 01a" << endl;
//    cout << "exp times size = " << exp_times.size() << endl;
//    cout << exp_times.at(0) << endl;
//    cout << "CHECK 01b" << endl;

//    Mat response;
//    Ptr<CalibrateDebevec> calibrate = createCalibrateDebevec();
//    cout << "CHECK 02" << endl;
//    calibrate->process(img_sequence, response, exp_times);
//    cout << "CHECK 03" << endl;
//    Mat hdr;
//    Ptr<MergeDebevec> merge_debevec = createMergeDebevec();
//    cout << "CHECK 04" << endl;
//    merge_debevec->process(img_sequence, hdr, exp_times, response);
//    Mat ldr;
//    cout << "CHECK 05" << endl;
//    Ptr<TonemapDurand> tonemap = createTonemapDurand(2.2f);
//    cout << "CHECK 06" << endl;
//    tonemap->process(hdr, ldr);
//    Mat fusion;
//    cout << "CHECK 07" << endl;
//    Ptr<MergeMertens> merge_mertens = createMergeMertens();
//    cout << "CHECK 08" << endl;
//    merge_mertens->process(img_sequence, fusion);
//    imwrite("fusion.png", fusion * 255);

    Mat fusion;
    Ptr<MergeMertens> merge_mertens = createMergeMertens();
    merge_mertens->process(img_sequence, fusion);

    double min, max;
    Point min_loc, max_loc;
    minMaxLoc(fusion, &min, &max, &min_loc, &max_loc);

//    cout << "min = " << min << ", max = " << max << endl;
    fusion.convertTo(hdr_img,CV_8U,255.0/(max-min),-255.0*min/(max-min));
//    cout << hdr_img.type() << endl;
//    hdr_img *= 255;

//    imwrite("fusion.png", hdr_img);
//    imwrite("ldr.png", ldr * 255);
//    imwrite("hdr.hdr", hdr);
    return true;
}

} /* namespace pylon_camera */
