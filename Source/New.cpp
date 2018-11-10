#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <cstdio>
#include "cv_basic.h"
#include "Math/Vector.h"

using namespace std;

const int thres_min_R = 190;
const int thres_max_G = 160;
const int thres_max_B = 160;
inline auto R(cv::Vec3b i) noexcept { return i[2]; }
inline auto G(cv::Vec3b i) noexcept { return i[1]; }
inline auto B(cv::Vec3b i) noexcept { return i[0]; }

namespace {
    std::mutex mutex_res;
    std::condition_variable cond;
    std::chrono::steady_clock::time_point last, last_update;
    int count_ch[3];
    char result = 0;
}

char get_result() {
    char this_res = 0;
    {
        std::unique_lock<std::mutex> lk{mutex_res};
        while (result == 0) {
            cond.wait(lk);
        }
        this_res = result;
        result = 0;
    }
    return this_res;
}

void uart_set_result(char res) {
    std::unique_lock<std::mutex> lk{mutex_res};
    if (std::chrono::steady_clock::now() - last_update > std::chrono::milliseconds(1500)) {
        count_ch[0] = count_ch[1] = count_ch[2] = 0;
    }
    count_ch[res - 1]++;
    if (std::chrono::steady_clock::now() - last > std::chrono::seconds(3) && ((count_ch[0] + count_ch[1] +count_ch[2]) > 5)) {
        char _this = 0;
        if (count_ch[0] >= count_ch[1] && count_ch[0] >= count_ch[2])
            _this = 1;

        if (count_ch[1] >= count_ch[0] && count_ch[1] >= count_ch[2])
            _this = 2;

        if (count_ch[2] >= count_ch[1] && count_ch[2] >= count_ch[0])
            _this = 3;
        result = _this;
        count_ch[0] = count_ch[1] = count_ch[2] = 0;
        last = std::chrono::steady_clock::now();
        cond.notify_all();
    }
    last_update = std::chrono::steady_clock::now();
}

cv::Mat Tubeidentify(const cv::Mat& input)
{
    cv::Mat cx_src= cv::Mat(input, cv::Range(static_cast<int>(double(input.rows) * 0.18), static_cast<int>(double(input.rows) * 0.52)),
            cv::Range(static_cast<int>(double(input.cols) * 0.43), static_cast<int>(double(input.cols) * 0.59))), dump;
    dump = cx_src;
    int result_shumaguan = 0;
    auto view = cx_src.begin<cv::Vec3b>();
    auto _masked = cv::Mat1b(cx_src.rows, cx_src.cols);
    for (auto&& x : _masked) {
        auto pixel = *(view++);
        x = static_cast<uchar>((R(pixel)>thres_min_R && G(pixel)<thres_max_G && B(pixel)<thres_max_B) * 255);
    }
    //Mat element=getStructuringElement(MORPH_RECT,Size(3,3));
    //morphologyEx(cx_src,cx_src,MORPH_DILATE,element);
    //morphologyEx(cx_src,cx_src,MORPH_ERODE,element);
    //morphologyEx(cx_src,cx_src,MORPH_ERODE,element);
    cx_src = _masked;
    cv::Mat row1,row2,col1;
    row1 = cx_src.rowRange(cx_src.rows/3,cx_src.rows/3+1);
    row2 = cx_src.rowRange(2*cx_src.rows/3,2*cx_src.rows/3+1);
    col1 = cx_src.colRange(cx_src.cols/2,cx_src.cols/2+1);
    int flag_row1=0;
    int flag_row2=0;
    int flag_col1=0;
    int point_row1[100],point_row2[100],point_col1[100];
    for(int i=0;i<row1.cols-1;i++)
    {
        if(abs(row1.at<uchar>(0,i)-row1.at<uchar>(0,i+1))==255)
        {
            point_row1[flag_row1]=i;
            flag_row1++;
        }
        if(abs(row2.at<uchar>(0,i)-row2.at<uchar>(0,i+1))==255)
        {
            point_row2[flag_row2]=i;
            flag_row2++;
        }
    }
    for(int j=0;j<col1.rows-1;j++)
    {
        if(abs(col1.at<uchar>(j,0)-col1.at<uchar>(j+1,0))==255)
        {
            point_col1[flag_col1]=j;
            flag_col1++;
        }
    }
    if(flag_row1==2&&flag_row2==2&&flag_col1==2)
    {
        result_shumaguan=1;
    }
    if(flag_row1==2&&flag_row2==2&&flag_col1==6)
    {
        if(point_row1[0]>row1.cols/2)
        {
            if(point_row2[0]>row2.cols/2)
            {
                result_shumaguan=3;
            }
            else
                result_shumaguan=2;
        }
        else
            result_shumaguan=1;

    }
    if(flag_row1==4&&flag_row2==4&&flag_col1==6)
    {
        result_shumaguan=1;
    }
    if(flag_row1==2&&flag_row2==4&&flag_col1==6)
    {
        result_shumaguan=1;
    }
    if(flag_row1==4&&flag_row2==2&&flag_col1==6)
    {
        result_shumaguan=1;
    }
    if(flag_row1==2&&flag_row2==2&&point_row2[1]>row2.cols/2&&point_row1[1]>row1.cols/2&&flag_col1<6)
    {
        result_shumaguan=1;
    }
    if (result_shumaguan != 0)
        uart_set_result(result_shumaguan);
    //cout<<result_shumaguan<<endl;
    return dump;
}

#include <iostream>
#include <deque>
#include <utility>

using namespace std;
namespace {
    const int sdirx[12] = {1, 0, -1, 0, 2, 1, 0, -1, -2, -1, 0, 1};
    const int sdiry[12] = {0, 1, 0, -1, 0, -1, -2, -1, 0, 1, 2, 1};
    const int thres_min_R = 190;
    const int thres_max_G = 160;
    const int thres_max_B = 160;
    const int dis_max_R = 100;
    const int dis_max_G = 100;
    const int dis_max_B = 100;
    const int const_catch = 256;
    const int const_sample = 128;
    bool vis[const_catch][const_catch];

    int dsize, w, h;

    struct Rect {
        Vec2i min;
        Vec2i max;
    };

    void clip_rect(Rect& op, const Rect& bound) {
        if (op.min.x<bound.min.x) op.min.x = bound.min.x;
        if (op.min.y<bound.min.y) op.min.y = bound.min.y;
        if (op.max.x>bound.max.x) op.max.x = bound.max.x;
        if (op.max.y>bound.max.y) op.max.y = bound.max.y;
    }

    auto R(cv::Vec3b i) noexcept { return i[2]; }
    auto G(cv::Vec3b i) noexcept { return i[1]; }
    auto B(cv::Vec3b i) noexcept { return i[0]; }

    class selector {
    public:
        explicit selector(const cv::Mat& input)
                :_input(input), _masked(input.rows, input.cols), _scaled() {
            auto view = input.begin<cv::Vec3b>();
            for (auto&& x : _masked) {
                auto pixel = *(view++);
                x = static_cast<uchar>((R(pixel)>thres_min_R && G(pixel)<thres_max_G && B(pixel)<thres_max_B) * 255);
            }
            dsize = std::max(input.rows, input.cols)/const_catch+1;
            auto scale = 1.0 / dsize;
            cv::resize(_masked, _scaled, cv::Size(), scale, scale, cv::InterpolationFlags::INTER_NEAREST);
            h = _scaled.rows;
            w = _scaled.cols;
        }

        std::pair<Rect, bool> extract_digit_rect(const Vec2i& base_point) {
            static constexpr int threshold = 20;
            int possible_point_count = 0;
            Rect ret{base_point, base_point};
            std::deque<Vec2i> q{base_point};
            while (!q.empty()) {
                auto u = q.front();
                q.pop_front();
                ++possible_point_count;
                for (int k = 0; k<12; k++) {
                    int vx = u.x+sdirx[k];
                    int vy = u.y+sdiry[k];
                    if (in_sample(vx, vy) && !vis[vx][vy] && _scaled(vy, vx)) {
                        vis[vx][vy] = true;
                        ret.min = {std::min(ret.min.x, vx), std::min(ret.min.y, vy)};
                        ret.max = {std::max(ret.max.x, vx), std::max(ret.max.y, vy)};
                        q.emplace_back(vx, vy);
                    }
                }
                if (q.empty()) {
                    for (int y = ret.min.y; y<ret.max.y; y++)
                        for (int x = ret.min.x; x<ret.max.x; x++)
                            if (!vis[x][y]) {
                                vis[x][y] = true;
                                if (_scaled(y, x))
                                    q.emplace_back(x, y);
                            }
                }
            }
            return {ret, possible_point_count>threshold};
        }

        bool evaluate_background_is_dark(const Rect& rect) {
            cv::Mat3b reduce_row, reduce_pix;
            auto area = cv::Mat3b(_input, cv::Range(rect.min.y, rect.max.y), cv::Range(rect.min.x, rect.max.x));
            cv::reduce(area, reduce_row, 0, cv::REDUCE_AVG);
            cv::reduce(reduce_row, reduce_pix, 1, cv::REDUCE_AVG);
            auto& pix = *reduce_pix.begin();
            return R(pix)<dis_max_R && G(pix)<dis_max_G && B(pix)<dis_max_B;
        }

        void compress_sample(const Rect& area) {
            auto size = area.max-area.min;
            auto step = 1.0/(std::max(size.x, size.y)/const_sample+1);
            cv::Mat1b compressed;
            cv::resize(cv::Mat1b(_masked, cv::Range(area.min.y, area.max.y), cv::Range(area.min.x, area.max.x)),
                    compressed, cv::Size(), step, step);
            cv::imshow(cv::String("this"), compressed);
            Tubeidentify(compressed);
        }

        bool get_sample(const Vec2i& base_point) {
            vis[base_point.x][base_point.y] = true;
            // try to get the largest sample area
            Rect area {{0,0}, {0,0}};
            auto&&[rect, success] = extract_digit_rect(base_point);
            if (!success) return false;
            //if (!evaluate_background_is_dark(rect)) return false;
            auto sample_area = Rect{(rect.min-Vec2i{1, 1})*dsize, (rect.max+Vec2i{1, 1})*dsize};
            clip_rect(sample_area, Rect{Vec2i(0, 0), Vec2i(_input.cols, _input.rows)});
            compress_sample(sample_area);
            return true;
        }

        bool test_sample_centre(int x, int y) noexcept { return !vis[x][y] && _scaled(y, x); }

        bool in_sample(int x, int y) noexcept { return x>=0 && x<_scaled.cols && y>=0 && y<_scaled.rows; }
        const cv::Mat& _input;
        cv::Mat1b _masked, _scaled;
    };

}

cv::Mat ProcessFrame(const cv::Mat& input) {
    memset(vis, 0, sizeof(vis));
    selector select {input};
    for (int j = 0; j<h; j++)
        for (int i = 0; i<w; i++)
            if (select.test_sample_centre(i, j)) {
                if (select.get_sample({i, j})) {
                    return select._scaled;
                }
            }
    return select._scaled;// cv::Mat1b(1, 1);
}
