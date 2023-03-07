/******************************************************************************

   REBVO: RealTime Edge Based Visual Odometry For a Monocular Camera.
   Copyright (C) 2016  Juan José Tarrio
   
   Jose Tarrio, J., & Pedre, S. (2015). Realtime Edge-Based Visual Odometry
   for a Monocular Camera. In Proceedings of the IEEE International Conference
   on Computer Vision (pp. 702-710).
   
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA

 *******************************************************************************/
#include "mtracklib/edge_finder.h"
#include <iostream>
#include <TooN/TooN.h>
#include <math.h>

using namespace TooN;
namespace  rebvo{
edge_finder::edge_finder(cam_model &cam, float max_i_value, int kl_num_max)		//valor maximo de keypoints
    :cam_mod(cam),fsz(cam_mod.sz),max_img_value(max_i_value),
     img_mask_kl(fsz),kl_size(kl_num_max),kn(0)
{
    if(kl_num_max>0)
        kl=new KeyLine[kl_num_max];
    else
        kl=nullptr;
    img_mask_kl.Reset(-1);
}

edge_finder::edge_finder(const edge_finder &ef)
    :cam_mod(ef.cam_mod),fsz(ef.cam_mod.sz),max_img_value(ef.max_img_value),
     img_mask_kl(fsz),kl_size(ef.KNum()),kl(new KeyLine[kl_size]),kn(kl_size)
{
    img_mask_kl=ef.img_mask_kl;
//    for(int i=0;i<kn;i++)
  //      kl[i]=ef.kl[i];
    memcpy(kl,ef.kl,kn*sizeof(KeyLine));
}

edge_finder::~edge_finder()		//valor maximo de keypoints
{

    if(kl)
        delete[] kl;

}
//********************************************************************
// build_mask(): detect KeyLines using DoG and Gradient
//********************************************************************
void edge_finder::build_mask(Image<DetectorImgType> &originImg,
                             sspace *ss,        //State space containing image data
                             int kl_max,        //Maximun number of KeyLines to search for
                             int win_s,         //Windopw size for plane fitting
                             float per_hist,    //Fast threshold on the maximun
                             float grad_thesh,  //Fast threshold on the gradioent
                             float dog_thesh    //Threshold on the DoG
                             )
{
    //Matrix used for plane fitting
    Matrix <Dynamic,Dynamic> Y((win_s*2+1)*(win_s*2+1),1);
    Vector <3> theta;

    static bool PhiReCalc=true;
    static Matrix <Dynamic,Dynamic> PInv(3,(win_s*2+1)*(win_s*2+1));

    if(PhiReCalc){

        Matrix <Dynamic,Dynamic,double> Phi((win_s*2+1)*(win_s*2+1),3);
        for(int i=-win_s,k=0;i<=win_s;i++){
            for(int j=-win_s;j<=win_s;j++,k++){
                Phi(k,0)=j;
                Phi(k,1)=i;
                Phi(k,2)=1;
            }
        }

        PInv=util::Matrix3x3Inv(Phi.T()*Phi)*Phi.T();
        PhiReCalc=false;
    }

    if(kl_max>kl_size)
        kl_max=kl_size;

    kn=0;

    int mag = 5;
    float fDesc[6][2] =
    {
    {-2, 2}, {0, 2}, {2, 2},
    {2,-2}, {0,-2}, {-2,-2}
    };

    
    //Start searching...
    for(int y=win_s;y<fsz.h-win_s;y++){
        for(int x=win_s;x<fsz.w-win_s;x++){

            int img_inx=img_mask_kl.GetIndex(x,y);  //Catch the image index

            img_mask_kl[img_inx]=-1;                //Default to no Keyline

            float n2gI=util::norm2(ss->ImgDx()[img_inx],ss->ImgDy()[img_inx]);

            if(n2gI<util::square(grad_thesh*max_img_value))        //Fisrt test on the 2-norm squared of the gradient
                continue;                                          //The theshhold must be proporcional to the
                                                                   //intensity scaling of the image
            int pn=0;

            for(int i=-win_s,k=0;i<=win_s;i++){
                for(int j=-win_s;j<=win_s;j++,k++){             //For each pixel in a (2*win_sz+1)*(2*win_sz+1) window
                    Y(k,0)=ss->ImgDOG()[(y+i)*fsz.w+x+j];       //Store DoG value on Y
                    if(ss->ImgDOG()[(y+i)*fsz.w+x+j]>0)         //
                        pn++;                                   //Increment pn if the DoG is positive
                    else
                        pn--;                                   //Decrement pn if the DoG is negative
                }
            }

            if(fabs(pn)>((float)((2.0*win_s+1.0)*(2.0*win_s+1.0)))*per_hist)        //If the point is a local root of the DoG
                continue;                                                           //Then the number of neighbour Positive values
                                                                                    //must be similar to the Negative values.
                                                                                    //So, apply a threshold on the difference
                                                                                    //(for non-max suppesion)

            theta=(PInv*Y).T()[0];                                                  //Fit a plane to the DoG

            float xs=-theta[0]*theta[2]/(theta[0]*theta[0]+theta[1]*theta[1]);      //(xs,ys) is the point belonging to the line of zero
            float ys=-theta[1]*theta[2]/(theta[0]*theta[0]+theta[1]*theta[1]);      //crossing of the plane, that is closest to (0,0)

            if(fabs(xs)>0.5 || fabs(ys)>0.5)                             //if the zero crossing is outside pixel area, then is not an edge
                continue;

            Point2DF mn={(float)theta[0],(float)theta[1]};              //doG gradient


            float n2_m=util::norm2(mn.x,mn.y);                          //sqare norm of the gradient

            if(n2_m<util::square(grad_thesh*max_img_value*dog_thesh))       //Apply a treshold on the DoG gradient
                continue;                                                   //This threshold is proporcional to the pure gradient one

            //Al test passed, anotate keyline...
            //经过所有的测试，该点为边缘点
            kl[kn].p_inx=img_inx;//y*size.w+x 一个点在一维图像向量上的索引

            kl[kn].m_m=mn;                          //梯度向量
            kl[kn].n_m=sqrt(n2_m);                  //梯度模
            kl[kn].u_m.x=kl[kn].m_m.x/kl[kn].n_m;   //梯度x/norm
            kl[kn].u_m.y=kl[kn].m_m.y/kl[kn].n_m;   //梯度y/norm

            //kl[kn].c_p={x+xs,y+ys};                 //边缘点的亚像素位置
            kl[kn].c_p={x,y};                 //边缘点的亚像素位置
            kl[kn].p_m=cam_mod.Img2Hom(kl[kn].c_p); //齐次位置
            kl[kn].p_m_0=kl[kn].p_m;                //以图像中心为0点的坐标

            kl[kn].rho=RhoInit;                     //逆深度初始化
            kl[kn].s_rho=RHO_MAX;                   //逆深度估计不确定性，给最大
            kl[kn].rho0=RhoInit;                    //逆深度初始化（EKF）
            kl[kn].s_rho0=RHO_MAX;                  //预测的逆深度不确定性  
            kl[kn].rho_nr=RhoInit;                  //逆深度估计的非正则化
            kl[kn].s_rho_nr=RHO_MAX;                //逆深度估计不确定性的非正则化

            kl[kn].m_num=0;                         //连续匹配上了多少次
            kl[kn].n_id=-1;                         //下一个KL的索引
            kl[kn].p_id=-1;                         //上一个KL的索引
            kl[kn].net_id=-1;                       //边缘点网络ID

            kl[kn].m_id=-1;                         //匹配点的ID
            kl[kn].m_id_f=-1;                       //前向匹配点的ID
            kl[kn].m_id_kf=-1;                      //和最近关键帧匹配上的ID

            kl[kn].depth_valid = -1;                //激光深度有效性

            //计算四邻域描述子
            if(x < mag || x > (fsz.w-mag) || y < mag || y > (fsz.h-mag))
            {
                
                for (int j = 0; j < 6; j++)
                {
                kl[kn].des[j] = 0.0; 
                }
                kl[kn].desValid=false;
            }
            else
            {

                int sumD = 0.00001;
                int img_inx=originImg.GetIndex(x,y);
                float I0 = originImg[img_inx];
                for (int j = 0; j < 6; j++)
                {
                    int x1 = x - kl[kn].u_m.y*fDesc[j][0] + kl[kn].u_m.x*fDesc[j][1] + 0.5;
                    int y1 = y - kl[kn].u_m.x*fDesc[j][0] + kl[kn].u_m.y*fDesc[j][1] + 0.5;
                    img_inx=originImg.GetIndex(x1,y1);
                    int d = originImg[img_inx]-I0;
                    kl[kn].des[j] = d;
                    sumD += d*d;
                }
                sumD = std::sqrt(sumD);
                for (int j = 0; j < 6; j++)
                {
                kl[kn].des[j] = kl[kn].des[j]/sumD; 
                }
                kl[kn].desValid=true;
            }

            img_mask_kl[img_inx]=kn;    //Set the index of the keyline in the mask

            if(++kn>=kl_max){

                for(++img_inx;img_inx<fsz.w*fsz.h;img_inx++){
                    img_mask_kl[img_inx]=-1;
                }
                return;
            }
        }
    }

}

//********************************************************************
// NextPoint(): Look for a consecutive KeyLine neighbour
//********************************************************************
inline int NextPoint(const int &x, const int &y,    //Keyline coordinate
                     const Point2DF &m,             //KeyLine grad
                     Image<int>&img_mask)            //Keyline Image Mask
{
    const auto tx=-m.y;
    const auto ty=m.x;  //KL Tangent Direction

    //Look for the 3 posible points in cuadrant of the tangeng direction

    int kl_inx;

    if(ty>0){
        if(tx>0){

            if((kl_inx=img_mask(x+1,y+0))>=0){
                return kl_inx;                  //If presense, return KL index
            }


            if((kl_inx=img_mask(x+0,y+1))>=0){
                return kl_inx;
            }


            if((kl_inx=img_mask(x+1,y+1))>=0){
                return kl_inx;
            }

        }else{
            if((kl_inx=img_mask(x-1,y+0))>=0){
                return kl_inx;
            }

            if((kl_inx=img_mask(x+0,y+1))>=0){
                return kl_inx;
            }

            if((kl_inx=img_mask(x-1,y+1))>=0){
                return kl_inx;
            }

        }
    }else{

        if(tx<0){

            if((kl_inx=img_mask(x-1,y+0))>=0){
                return kl_inx;
            }

            if((kl_inx=img_mask(x+0,y-1))>=0){
                return kl_inx;
            }

            if((kl_inx=img_mask(x-1,y-1))>=0){
                return kl_inx;
            }

        }else{

            if((kl_inx=img_mask(x+1,y+0))>=0){
                return kl_inx;
            }

            if((kl_inx=img_mask(x+0,y-1))>=0){
                return kl_inx;
            }

            if((kl_inx=img_mask(x+1,y-1))>=0){
                return kl_inx;
            }

        }
    }
    return -1;  //-1 if no match
}

//********************************************************************
// join_edges(): Join consecutive edge points
//********************************************************************
void edge_finder::join_edges(){

    for(int ikl=0;ikl<kn;ikl++){

        int x=util::round2int_positive(kl[ikl].c_p.x);
        int y=util::round2int_positive(kl[ikl].c_p.y);  //KeyLine image coordinate

        int ikl2;
        if((ikl2=NextPoint(x,y,kl[ikl].m_m,img_mask_kl))<0)    //Look for an edge
            continue;

        kl[ikl2].p_id=ikl;      //asign previous and next index
        kl[ikl].n_id=ikl2;
    }
}

//********************************************************************
// UpdateThresh(): perform auto-threshold with a Proporcional-Law
//********************************************************************
inline void UpdateThresh(double &tresh,const int& l_kl_num, const int& kl_ref,const double& gain,const double& thresh_max,const double& thresh_min)
{
    tresh-=gain*(double)(kl_ref-l_kl_num);
    tresh=util::Constrain(tresh,thresh_min,thresh_max);
}

//********************************************************************
// detect(): Full edge detect
//********************************************************************
void edge_finder::detect(Image<DetectorImgType> &originImg,
                         sspace *ss,
                         int plane_fit_size,    //Size of window for plane fitting
                         double pos_neg_thresh, //DoG Pos-Neg max difference
                         double dog_thresh,     //DoG gradient thresh (proporcional to &tresh)
                         int kl_max,            //Maximun number of KeyLines
                         double &tresh,         //Current threshold
                         int &l_kl_num,         //Number of KLs on the last edgemap
                         int kl_ref,            //Requested number of KLs (for auto-thresh)
                         double gain,           //Auto-thresh gain
                         double thresh_max,     //Maximun allowed thresh
                         double thresh_min      //Minimun allowed thresh
                         )
{
    if(gain>0)
        UpdateThresh(tresh,l_kl_num,kl_ref,gain,thresh_max,thresh_min);     //Autogain threshold

    build_mask(originImg,ss,kl_max,plane_fit_size,pos_neg_thresh,tresh,dog_thresh);   //Detect & Anotate KeyLines

    join_edges();   //Join consecutive KeyLines
    l_kl_num=kn;    //Save the number of effective KL detected
}


//********************************************************************************
// EstimateQuantile() uses histograms to estimate an uncertainty threshold
// for a certain quatile
//********************************************************************************

int edge_finder::reEstimateThresh(int  knum,int n){               //Number of bins on the histo


    float max_dog=kl[0].n_m;
    float min_dog=kl[0].n_m;

    for(int ikl=1;ikl<kn;ikl++){
        util::keep_max(max_dog,kl[ikl].n_m);
        util::keep_min(min_dog,kl[ikl].n_m);
    }

    int histo[n];


    for(int i=0;i<n;i++){
        histo[i]=0;
    }

    for(int ikl=0;ikl<kn;ikl++){
        int i=n*(max_dog-kl[ikl].n_m)/(max_dog-min_dog);    //histogram position

        i=i>n-1?n-1:i;
        i=i<0?0:i;

        histo[i]++;     //count
    }

    int i=0;
    for(int a=0;i<n && a<knum;i++,a+=histo[i]);

    return reTunedThresh=max_dog-(float)i*(max_dog-min_dog)/(float)n;

}


//************************************************************************
// dumpToBinaryFile(): Print keyline list to file
//************************************************************************
void edge_finder::dumpToBinaryFile(std::ofstream &file)
{

    file.write((const char *)&kn,sizeof(kn));
    file.write((const char *)kl,sizeof(KeyLine)*kn);

    std::cout <<"\nDumpped "<<kn<<" Keylines, "<<sizeof(kn)+sizeof(KeyLine)*kn<<" bytes";

}

//************************************************************************
// dumpToBinaryFile(): Read keyline list from file
//************************************************************************
void edge_finder::readFromBinaryFile(std::ifstream &file)
{

    file.read((char *)&kn,sizeof(kn));

    if(kl!=nullptr)
        delete [] kl;
    kl=new KeyLine[kn];
    kl_size=kn;

    file.read((char *)kl,sizeof(KeyLine)*kn);

}
}
