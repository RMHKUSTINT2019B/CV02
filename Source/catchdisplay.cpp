#include <iostream>
#include <deque>
#include <utility>
#include "Math/Vector.h"
#include "FrameProcessor.h"
#include "7Segment.h"
#include <opencv2/imgproc.hpp>
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
    const int Diff = 70;
    const int const_s=60;

    const int min_catch_size=3;

    const int min_V=200;
    const int Hred01=60;
    const int Hred02=300;
    const double min_S=0.2;
    Colour sample[const_catch][const_catch];
    bool visit[const_catch][const_catch];
    Colour average_colour;

    int dsize, w, h;
    const Vec2i sspacev[sspace_size]=
            {{0,-5},{-1,-4},{0,-4},{1,-4},{-2,-3},{-1,-3},{0,-3},{1,-3},{2,-3},{-3,-2},{-2,-2},{-1,-2},
             {0,-2},{1,-2},{2,-2},{3,-2},{-4,-1},{-3,-1},{-2,-1},{-1,-1},{0,-1},{1,-1},{2,-1},{3,-1},
             {4,-1},{-5,0},{-4,0},{-3,0},{-2,0},{-1,0},{1,0},{2,0},{3,0},{4,0},{5,0},{-4,1},{-3,1},
             {-2,1},{-1,1},{0,1},{1,1},{2,1},{3,1},{4,1},{-3,2},{-2,2},{-1,2},{0,2},{1,2},{2,2},{3,2},
             {-2,3},{-1,3},{0,3},{1,3},{2,3},{-1,4},{0,4},{1,4},{0,5}};

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
                Colour c;
                c.set(R(pixel),G(pixel),B(pixel));
                x = static_cast<uchar>(((!different_colour(c,average_colour))) * 255);
            }
            dsize = std::max(input.rows, input.cols)/const_catch+1;
            auto scale = 1.0 / dsize;
            cv::resize(_masked, _scaled, cv::Size(), scale, scale, cv::InterpolationFlags::INTER_NEAREST);
            h = _scaled.rows;
            w = _scaled.cols;
        }

        std::pair<Rect, bool> extract_digit_rect(const Vec2i& base_point) {
            static constexpr int threshold = 20;
            Colour total_colour;
            int total=1;
            Rect ret{base_point, base_point};
            std::deque<Vec2i> q{base_point};
            while (!q.empty()) {
                auto u = q.front();
                q.pop_front();
                for(int k=0;k<sspace_size;k++){
                    auto v=u+sspacev[k];
                    if(!in_sample(v)||visit[v.x][v.y]||!_scaled(v.y, v.x))continue;
                    visit[v.x][v.y]=true;
                    ret.min = {std::min(ret.min.x, v.x), std::min(ret.min.y, v.y)};
                    ret.max = {std::max(ret.max.x, v.x), std::max(ret.max.y, v.y)};
                    q.emplace_back(v.x,v.y);
                    simple_add(total_colour,sample[v.x][v.y]);
                    ++total;
                    get_average_colour(average_colour,total_colour,total);
                }
            }
            bool success = total>min_catch_size&&is_red(get_HSV(average_colour));
            return {ret, success};
        }

        bool evaluate_background_is_dark(const Rect& rect) {
            cv::Mat3b reduce_row, reduce_pix;
            auto area = cv::Mat3b(_input, cv::Range(rect.min.y, rect.max.y), cv::Range(rect.min.x, rect.max.x));
            cv::reduce(area, reduce_row, 0, cv::REDUCE_AVG);
            cv::reduce(reduce_row, reduce_pix, 1, cv::REDUCE_AVG);
            auto& pix = *reduce_pix.begin();
            return R(pix)<dis_max_R && G(pix)<dis_max_G && B(pix)<dis_max_B;
        }

        void compress_sample(Img_segment7* s, const Rect& area) {
            auto size = area.max-area.min;
            auto step = 1.0/(std::max(size.x, size.y)/const_sample+1);
            cv::Mat1b compressed;
            cv::resize(cv::Mat1b(_masked, cv::Range(area.min.y, area.max.y), cv::Range(area.min.x, area.max.x)),
                    compressed, cv::Size(), step, step);
            s->input_image(compressed);
        }

        bool different_colour(Colour A,Colour B)
        {
            return A.R>B.R+Diff||A.R<B.R-Diff||A.G>B.G+Diff||A.G<B.G-Diff||A.B>B.B+Diff||A.B<B.B-Diff;
        }
        void simple_add(Colour &A,Colour B)
        {
            A.R+=B.R;
            A.G+=B.G;
            A.B+=B.B;
        }
        void get_average_colour(Colour &A,Colour total,int n)
        {
            A.R=total.R/n;
            A.G=total.G/n;
            A.B=total.B/n;
        }
        bool is_red(HSV A)
        {
            return (A.H<Hred01||A.H>Hred02)&&A.V>min_V&&A.S>min_S;
        }
        bool point_is_red(int i,int j)
        {
            return is_red(get_HSV(sample[i][j]));
        }

        void cover_colour(int x,int y,Colour &average_colour,Vec2i &top_left,Vec2i &bottom_right,int &catch_size)
        {
            Colour total_colour;
            int total=1;
            //average colour = total colour / total
            Vec2i u(x,y),v;
            visit[x][y]=true;
            queue<Vec2i >q;
            top_left.x=x;
            top_left.y=y;
            bottom_right.x=x;
            bottom_right.y=y;
            q.push(u);
            total_colour=average_colour=sample[x][y];
            catch_size=0;
            while(!q.empty()){
                u=q.front();
                q.pop();
                ++catch_size;
                if(u.x<top_left.x)top_left.x=u.x;
                if(u.y<top_left.y)top_left.y=u.y;
                if(u.x>bottom_right.x)bottom_right.x=u.x;
                if(u.y>bottom_right.y)bottom_right.y=u.y;
                for(int k=0;k<sspace_size;k++){
                    v=u+sspacev[k];
                    if(!in_sample(v)||visit[v.x][v.y]||different_colour(average_colour,sample[v.x][v.y]))continue;
                    q.push(v);
                    visit[v.x][v.y]=true;
                    simple_add(total_colour,sample[v.x][v.y]);
                    ++total;
                    get_average_colour(average_colour,total_colour,total);
                }
            }
        }

        bool get_sample(const Vec2i& base_point, Img_segment7* s) {
            visit[base_point.x][base_point.y] = true;
            // try to get the largest sample area
            Rect area {{0,0}, {0,0}};
            auto&&[rect, success] = extract_digit_rect(base_point);
            if (!success) return false;
            //if (!evaluate_background_is_dark(rect)) return false;
            auto sample_area = Rect{(rect.min-Vec2i{1, 1})*dsize, (rect.max+Vec2i{1, 1})*dsize};
            clip_rect(sample_area, Rect{Vec2i(0, 0), Vec2i(_input.cols, _input.rows)});
            compress_sample(s, sample_area);
            return true;
        }

        bool test_sample_centre(int x, int y) noexcept { return !visit[x][y] && _scaled(y, x); }

        bool in_sample(int x, int y) noexcept { return x>=0 && x<_scaled.cols && y>=0 && y<_scaled.rows; }

        bool in_sample(Vec2i a) noexcept { return a.x>=0 && a.x<_scaled.cols && a.y>=0 && a.y<_scaled.rows; }

        const cv::Mat& _input;
        cv::Mat1b _masked, _scaled;
    };

}

cv::Mat ProcessFrame(const cv::Mat& input) {
    memset(visit, 0, sizeof(visit));
    Img_segment7 s{};
    selector select {input};
    s.init();
    for (int j = 0; j<h; j++)
        for (int i = 0; i<w; i++) {
            if (select.point_is_red(i,j)) {
                if (select.get_sample({i, j}, &s)) {
                    s.cv05_identify();
                    s.output_little_display();
                    std::cout << s.get_result() << std::endl;
                    return s.img;
                }
            }
        }
    return select._scaled;// cv::Mat1b(1, 1);
}
