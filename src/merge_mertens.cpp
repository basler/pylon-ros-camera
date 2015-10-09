#include <pylon_camera/merge_mertens.h>
#include <opencv2/imgproc.hpp>

using namespace cv;

namespace pylon_camera
{


class Parallel_computeWeights: public cv::ParallelLoopBody
{
private:
    const std::vector<Mat>& images_;
    std::vector<Mat>& float_images_;
    std::vector<Mat>& weights_;
    Mat& weight_sum_;
    Mutex& weight_sum_mutex_;
    const float& wcon_;
    const float& wexp_;

public:
    Parallel_computeWeights(const std::vector<Mat>& images,
                            std::vector<Mat>& float_images,
                            std::vector<Mat>& weights,
                            Mat& weight_sum,
                            Mutex weight_sum_mutex,
                            const float& contrast_weight,
                            const float& exposure_weight) :
        images_(images),
        float_images_(float_images),
        weights_(weights),
        weight_sum_(weight_sum),
        weight_sum_mutex_(weight_sum_mutex),
        wcon_(contrast_weight),
        wexp_(exposure_weight)
    {
    }

    virtual ~Parallel_computeWeights()
    {}

    virtual void operator() (const cv::Range& range) const
    {
        Mat contrast, downscaled, weights;
        for (int i = range.start; i < range.end; ++i)
        {
            // Compute weights not on fullscale image. We will upscale later
            images_[i].convertTo(float_images_[i], CV_32F, 1.0f/255.0f);
            pyrDown(float_images_[i], downscaled);

            Laplacian(downscaled, contrast, CV_32F);

            contrast = abs(contrast);

            if (wexp_ != 0.0)
            {
                weights = downscaled - 0.5f;
                multiply(weights, weights, weights, -1./0.08);
                if (wexp_ != 1.0)
                {
                    pow(weights, wexp_, weights);
                }
            }

            if (wcon_ != 1.0)
            {
                pow(contrast, wcon_, contrast);
            }

            if (wexp_ != 0.0)
            {
                weights *= contrast + 1e-12f;
            }
            else
            {
                weights = contrast + 1e-12f;
            }

            pyrUp(weights, weights_[i], images_[i].size());
            //weight_sum_mutex_.lock();
            weight_sum_ += weights_[i];
            //weight_sum_mutex_.unlock();
        }
    }
};


class Parallel_computeImagePyramid : public cv::ParallelLoopBody
{
private:
    const std::vector<Mat>& images_;
    const std::vector<Mat>& weights_;
    const Mat& weight_sum_;
    std::vector<Mat>& res_pyr_;
    std::vector<Mutex>& mutexes_;

public:
    Parallel_computeImagePyramid(const std::vector<Mat>& images,
                                 const std::vector<Mat>& weights,
                                 const Mat& weight_sum,
                                 std::vector<Mat>& res_pyr,
                                 std::vector<Mutex>& mutexes) :
        images_(images),
        weights_(weights),
        weight_sum_(weight_sum),
        res_pyr_(res_pyr),
        mutexes_(mutexes)
    {
    }

    virtual ~Parallel_computeImagePyramid()
    {}

    virtual void operator() (const cv::Range& range) const
    {
        const int maxlevel = res_pyr_.size() - 1;
        Mat contrast;
        Mat float_img;
        std::vector<Mat> img_pyr, weight_pyr;
        Mat up;

        for (int i = range.start; i < range.end; ++i)
        {
            buildPyramid(images_[i], img_pyr, maxlevel);
            buildPyramid(weights_[i] / weight_sum_, weight_pyr, maxlevel);

            for (int lvl = 0; lvl < maxlevel; ++lvl)
            {
                pyrUp(img_pyr[lvl+1], up, img_pyr[lvl].size());
                up = img_pyr[lvl] - up;

                mutexes_[lvl].lock();
                if(res_pyr_[lvl].empty())
                {
                    res_pyr_[lvl] = up.mul(weight_pyr[lvl]);
                    mutexes_[lvl].unlock();
                }
                else
                {
                    // we can unlock here as matrix operations are thread safe
                    mutexes_[lvl].unlock();
                    accumulateProduct(up, weight_pyr[lvl], res_pyr_[lvl]);
                }
            }

            // Uppest level is without substraction of upscaled image.
            mutexes_[maxlevel].lock();
            if(res_pyr_[maxlevel].empty())
            {
                res_pyr_[maxlevel] = img_pyr[maxlevel].mul(weight_pyr[maxlevel]);
                mutexes_[maxlevel].unlock();
            }
            else
            {
                // we can unlock here as matrix operations are thread safe
                mutexes_[maxlevel].unlock();
                accumulateProduct(img_pyr[maxlevel], weight_pyr[maxlevel], res_pyr_[maxlevel]);
            }
        }
    }
};


MergeMertensC1::MergeMertensC1(const float& contrast_weight, const float& exposure_weight) :
    wcon_(contrast_weight),
    wexp_(exposure_weight)
{
}

MergeMertensC1::~MergeMertensC1()
{
}

void MergeMertensC1::process(InputArrayOfArrays src, OutputArray dst,
                             InputArray, InputArray)
{
    process(src, dst);
}

void MergeMertensC1::process(InputArrayOfArrays src, OutputArray dst)
{
    std::vector<Mat> images;
    src.getMatVector(images);

    const Size size = images[0].size();

    const int maxlevel = static_cast<int>(logf(static_cast<float>(min(size.width, size.height))) / logf(2.0f));
    std::vector<Mat> res_pyr(maxlevel + 1);
    std::vector<Mutex> mutexes(maxlevel + 1);
    std::vector<Mat> weights(images.size());
    std::vector<Mat> float_images(images.size());
    Mat weight_sum = Mat::zeros(images[0].size(), CV_32F);
    Mutex weight_sum_mutex;

    parallel_for_(Range(0, images.size()), Parallel_computeWeights(images, float_images, weights, weight_sum, weight_sum_mutex, wcon_, wexp_));


    parallel_for_(Range(0, images.size()), Parallel_computeImagePyramid(float_images, weights, weight_sum, res_pyr, mutexes));

    for (int lvl = maxlevel; lvl > 0; --lvl)
    {
        Mat up;
        pyrUp(res_pyr[lvl], up, res_pyr[lvl - 1].size());
        res_pyr[lvl - 1] += up;
    }
    dst.create(size, CV_32FC1);
    res_pyr[0].copyTo(dst.getMat());
}

float MergeMertensC1::getContrastWeight() const
{
    return wcon_;
}

void MergeMertensC1::setContrastWeight(float contrast_weiht)
{
    wcon_ = contrast_weiht;
}

float MergeMertensC1::getExposureWeight() const
{
    return wexp_;
}
void MergeMertensC1::setExposureWeight(float exposure_weight)
{
    wexp_ = exposure_weight;
}

}
