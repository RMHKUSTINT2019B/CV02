#pragma once

#include <algorithm>
#include <cstring>
#include <deque>
#include <vector>
#include "Math/Vector.h"
#include <opencv2/core/mat.hpp>
#include <iostream>
#include "cv_basic.h"
#include <queue>
#include <set>

#define sqr(a) ((a)*(a))

namespace {
    constexpr Vec2i directions[4] {{1, 0}, {0, 1}, {-1, 0}, {0, -1}}; //4 directions up,down,left,right
    const int block_count_W_log2 = 3;//sizeW,L=width,height>>block_size
    const int block_count_H_log2 = 4;//
    constexpr Vec2i co {1, 2};//block_count_H_log2 is 2*block_count_W_log2
    const int block_sqrLim = 36;//shoule be adjusted along with block_size & co.x,y
    const int min_dense = 2;//the min density for a block to sprawl
    const int verti_limit = 6;
    const int horiz_limit = 8;
    const int block_sizeW = 3;
    const int block_sizeL = 4;
    const double joint_ratio = 1.6;
    //consistent with dirx,y
    int basic_direction(Vec2i v) {
        //consider line y=x y=-x and the four parts they cut out
        if (v.x>=v.y && v.x>-v.y)return 0;
        if (v.x>v.y && v.x<=-v.y)return 3;
        if (v.x<=v.y && v.x<-v.y)return 2;
        return 1;
    }
    
    struct Block {
        pl_vector location{0, 0};//location bottomleft

        pl_vector size {0, 0};
        pl_vector centre {0, 0};//mass centre
        pl_vector d[4] {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
        int dx,dy;
        int density = 0;//density

    };
    typedef struct {
        pl_vector A,B;
        std::set<int>joint;
    }Segment_display;


    class Img_segment7 {
    public:
        void init() {
            height = width = 0;
            result = -1;
            bx = by = 0;
        }
        //notice that the form of input image is:x-axis right,y-axis down
        void input_image(cv::Mat1b matrix) {
            height = matrix.rows;
            width = matrix.cols;
            img = std::move(matrix);
        }
        
       void cv05_identify()
       {
            Block_init();
            Segment_identify();
       }

        int get_result() {
            return result;
        }
        cv::Mat1b img;
    private:
        int height, width;
        int result;
        Block b[32][32];
        int bx, by;
        int L,R,T,B;
        int LBx,LBy;
        int Wth,Lth;
        int dx,dy;
        int display[7];
        //end illustration
        bool in_img(int x, int y) {
            return x>=0 && x<width && y>=0 && y<height;
        }

        //measure the actual distance that function "sprawl" travels when there is coefficient co.x,y
        int sqr_block_dis(Vec2i a, Vec2i b, int cox, int coy) {
            return sqr(coy)*sqr(a.x-b.x)+sqr(cox)*sqr(a.y-b.y);
        }
        //get basic info of the block
        void block_process(Block* b) {
            pl_vector v {0, 0}, location = b->location;
            const auto bound = b->location + b->size;
            for (; location.x < bound.x; ++location.x)
                for (; location.y < bound.y; ++location.y)
                    if (in_img(location.x, location.y) && img(location.y, location.x)) {
                        ++b->density;
                        v += location - b->location;
                    }
            if (b->density)
                b->centre = b->location+v/b->density;
        }
        //b[i][j] is in process area
        bool block_exist(int i, int j) {
            return i>=0 && i<bx && j>=0 && j<by;
        }
        bool block_exist(pl_vector a)
        {
            return a.x>=0&&a.x<bx&&a.y>=0&&a.y<by;
        }
        //get the vectors, say, direction of a block considering its adjacent blocks
        //each segment of display has a direction, which is also the direction of blocks in it
        void block_sprawl(int i, int j, int sqrLim) {
            if (b[i][j].density<=min_dense)return;
            bool vis[32][32];
            memset(vis, 0, sizeof(vis));
            Vec2i p = {i, j};
            vis[i][j] = true;
            std::deque<Vec2i> q {p};
            while (!q.empty()) {//BFS/Flood Fill
                const auto u = q.front();
                q.pop_front();
                for (const auto x : directions) {//how about space.x,y
                    const auto v = u + x;
                    int sqrd = sqr_block_dis(p, v, co.x, co.y);
                    if (block_exist(v.x, v.y) && b[v.x][v.y].density && !vis[v.x][v.y] && sqrd<=sqrLim) {
                        q.push_back(v);
                        vis[v.x][v.y] = true;
                        const auto vec = b[v.x][v.y].density * (b[v.x][v.y].centre - b[i][j].centre);
                        b[i][j].d[vec.basic_direction()] += vec;
                    }
                }
            }
        }

        void Block_init()
        {
            //L,R,T,B for the range of the process in the image
            L=width-1;R=0;T=0;B=height-1;
            int i,j;
            //get range
            for(i=0;i<width;i++)
                for(j=0;j<height;j++)if(img[i][j]){
                        if(i<L)L=i;
                        if(i>R)R=i;
                        if(j<B)B=j;
                        if(j>T)T=j;
                    }
            if(L>R||B>T){
                result=-1;
                return;
            }
            //Wth,Lth for the width & height of the process area
            Wth=R-L;Lth=T-B;
            //LBx,LBy location bottonleft
            LBx=L;LBy=B;
            //A Stick
            //dx,dy for the size of each block
            //divide the image into small blocks for process
            //Notice that in cv03 the dx=2*dy because the display is such size
            dx=Wth>>block_sizeW;dy=Lth>>block_sizeL;
            //bx,by for the size of b[][], blocks
            bx=Wth/dx+1;by=Lth/dy+1;
            //process b[][], blocks
            for(i=0;i<bx;i++)
                for(j=0;j<by;j++){
                    b[i][j].location=make_vec(i*dx+LBx,j*dy+LBy);
                    b[i][j].dx=dx;
                    b[i][j].dy=dy;
                    block_process(&b[i][j]);
                }
            for(i=0;i<bx;i++)
                for(j=0;j<by;j++){
                    block_sprawl(i,j,block_sqrLim);
                }
        }
        void print_direction(int** direction)
        {
            int i,j;
            for(j=by-1;j>=0;j--){
                for(i=0;i<bx;i++)switch(direction[i][j]){
                        case 0:std::cout<<" ";break;
                        case 2:std::cout<<"-";break;
                        case 3:std::cout<<"|";break;
                        case 1:std::cout<<"+";break;
                    }
                std::cout<<std::endl;
            }
        }
        void Block_direction_process(int** direction,pl_vector &horizontal_direction,pl_vector &vertical_direction)
        {
            int i,j;
            double modu[4];
            for(i=0;i<bx;i++)
                for(j=0;j<by;j++){
                    direction[i][j]=0;
                    if(b[i][j].density<=min_dense)continue;
                    int mk=0,k;//mk be the max modu's subscript
                    for(k=0;k<4;k++){
                        modu[k]=b[i][j].d[k].modulus();
                        if(modu[k]>modu[mk])mk=k;
                    }
                    //if one vecter(direction) is signifcantly larger than another
                    if(modu[mk]>joint_ratio*modu[(mk+1)&3]&&modu[mk]>joint_ratio*modu[(mk+3)&3]){
                        //measure normal direction, &3 = %4
                        if(mk&1){
                            direction[i][j]=3;//vertical (11)2
                            vertical_direction+=b[i][j].d[1]-b[i][j].d[3];
                        }
                        else{
                            direction[i][j]=2;//horizontal (10)2
                            horizontal_direction+=b[i][j].d[0]-b[i][j].d[2];
                        }
                    }
                    else direction[i][j]=1;//Not, view it as a joint (01)2
                }
        }
        Segment_display Create_segment(int** direction,int x,int y,int dir_value,int No)
        //dir_value:2:hori,3:verti
        {
            No=(No<<1)|(dir_value&1);
            /*Note
			direction[x][y] binary form:[No(2)][0:hori/1:verti][01:joint/10:hori/11:verti]
			*/
            std::queue<pl_vector>q;
            pl_vector Start=make_vec(x,y);
            Segment_display segment;
            segment.joint.clear();
            segment.A=segment.B=Start;
            q.push(Start);
            while(!q.empty()){
                pl_vector u=q.front();
                q.pop();
                for(int k=0;k<espace_size;k++){
                    pl_vector v=u+espacev[k];
                    if(!block_exist(v))continue;
                    if(direction[v.x][v.y]==1||direction[v.x][v.y]==dir_value||
                       ((direction[v.x][v.y]&3)==1&&(direction[v.x][v.y]>>2)!=No)){
                        direction[v.x][v.y]=(No<<2)|(direction[v.x][v.y]&3);
                        q.push(v);
                        segment.A.x=std::min(segment.A.x,v.x);
                        segment.A.y=std::min(segment.A.y,v.y);
                        segment.B.x=std::max(segment.B.x,v.x);
                        segment.B.y=std::max(segment.B.y,v.y);
                    }
                    else if((direction[v.x][v.y]&3)==(dir_value^1)&&direction[v.x][v.y]>>3){
                        segment.joint.insert(direction[v.x][v.y]>>3);
                    }
                }
            }
            return segment;
        }
        bool find_joint(pl_vector A,std::vector<Segment_display> vec)
        {
            std::vector<Segment_display>::iterator it;
            for(int k=0;k<9;k++){
                pl_vector joint_search=A+spacev[k];
                if(!block_exist(joint_search))continue;
                for(it=vec.begin();it!=vec.end();it++){
                    if(joint_search==it->A||joint_search==it->B)return true;
                }
            }
            return false;
        }
        bool Left_vertical(Segment_display seg)
        {
            return seg.A.x+seg.B.x<bx;
        }
        void Segment_process_123(std::vector<Segment_display> &horizontal,std::vector<Segment_display> &vertical,
                                 pl_vector horizontal_direction,pl_vector vertical_direction)
        {
            //std::cout<<horizontal_direction<<std::endl<<vertical_direction<<std::endl;
            if(horizontal.size()==0){
                result=1;
                display[2]=display[5]=1;
                return;
            }
            std::vector<Segment_display>::iterator it;
            if(horizontal_direction.y<0){
                for(it=horizontal.begin();it!=horizontal.end();it++){
                    std::swap(it->A.y,it->B.y);
                }
            }
            if(vertical_direction.x<0){
                for(it=vertical.begin();it!=vertical.end();it++){
                    std::swap(it->A.x,it->B.x);
                }
            }
            if(horizontal.size()>=3){
                display[0]=display[3]=display[6]=1;
                //sort horizontal[0,1,2]
                if(horizontal[0].A.y<horizontal[1].A.y)std::swap(horizontal[0],horizontal[1]);
                if(horizontal[0].A.y<horizontal[2].A.y)std::swap(horizontal[0],horizontal[2]);
                if(horizontal[1].A.y<horizontal[2].A.y)std::swap(horizontal[1],horizontal[2]);
                //end sort
                if(!horizontal[0].joint.empty()){
                    std::set<int>::iterator iti;
                    for(iti=horizontal[0].joint.begin();iti!=horizontal[0].joint.end();iti++){
                        if(Left_vertical(vertical[*iti-1]))display[1]=1;
                        else display[2]=1;
                    }
                }
                if(!horizontal[2].joint.empty()){
                    std::set<int>::iterator iti;
                    for(iti=horizontal[2].joint.begin();iti!=horizontal[2].joint.end();iti++){
                        if(Left_vertical(vertical[*iti-1]))display[4]=1;
                        else display[5]=1;
                    }
                }
                if(display[4])result=2;
                else result=3;
            }
        }
        void Segment_identify()
        {
            pl_vector horizontal_direction,vertical_direction;
            horizontal_direction.init();
            vertical_direction.init();
            int i,j;
            int** direction=new int*[bx];
            for(i=0;i<bx;i++){
                direction[i]=new int[by];
            }
            Block_direction_process(direction,horizontal_direction,vertical_direction);

            print_direction(direction);

            std::vector<Segment_display>horizontal;
            std::vector<Segment_display>vertical;
            int No_hori=0,No_verti=0;
            for(i=0;i<bx;i++)
                for(j=0;j<by;j++){
                    if(direction[i][j]==2){
                        horizontal.push_back(Create_segment(direction,i,j,2,++No_hori));
                    }
                    if(direction[i][j]==3){
                        vertical.push_back(Create_segment(direction,i,j,3,++No_verti));
                    }
                }
            std::vector<Segment_display>::iterator it;
            std::set<int>::iterator itj;
            for(it=horizontal.begin(),i=1;it!=horizontal.end();it++,i++){
                for(itj=it->joint.begin();itj!=it->joint.end();itj++){
                    vertical[*itj-1].joint.insert(i);
                }
            }
            for(it=vertical.begin(),i=1;it!=vertical.end();it++,i++){
                for(itj=it->joint.begin();itj!=it->joint.end();itj++){
                    horizontal[*itj-1].joint.insert(i);
                }
            }

            for(i=0;i<bx;i++){
                delete[] direction[i];
            }
            delete []direction;
            Segment_process_123(horizontal,vertical,horizontal_direction,vertical_direction);
        }
    };


}
