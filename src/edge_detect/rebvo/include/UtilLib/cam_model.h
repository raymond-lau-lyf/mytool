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

#ifndef CAM_MODEL_H
#define CAM_MODEL_H

#include "VideoLib/video_io.h"
#include "VideoLib/common_types.h"
#include "util.h"
#include <math.h>
namespace  rebvo{

struct cam_model{

public:

    typedef struct{
        double Kc2;
        double Kc4;
        double Kc6;
        double P1;
        double P2;
    }rad_tan_distortion;

    Point2DF pp;
    Point2DF zf;
    double zfm;
    rad_tan_distortion Kc;
    Size2D sz;

    cam_model(Point2DF prin_point,Point2DF focal_dist,rad_tan_distortion DistKc,Size2D ImageSize)
        :pp(prin_point),zf(focal_dist),zfm((focal_dist.x+focal_dist.y)/2),Kc(DistKc),sz(ImageSize){ }

    cam_model(){}

    template <typename PointType>
    void undistortHom2Hom(PointType &p,int newton_it)  const{

        double rn,rd;

        rn=rd=util::norm((p.x/zf.x),(p.y/zf.y));

        for(int i=0;i<newton_it;i++){

            rn = rn - (rn*(1+rn*rn*(Kc.Kc2+Kc.Kc4*rn*rn))-rd)/(1+rn*rn*(3*Kc.Kc2+5*Kc.Kc4*rn*rn));

        }
        p.x*=rn/rd*zfm/zf.x;
        p.y*=rn/rd*zfm/zf.y;
    }

    template <typename PointType>
    void distortHom2Hom(PointType &p)  const{

        double xp=p.x/zfm,yp=p.y/zfm;
        double r2=util::norm2(xp,yp);

        double xpp=xp*(1+r2*(Kc.Kc2+r2*(Kc.Kc4+r2*Kc.Kc6)))+2*Kc.P1*xp*yp       +  Kc.P2*(r2+2*xp*xp);
        double ypp=yp*(1+r2*(Kc.Kc2+r2*(Kc.Kc4+r2*Kc.Kc6)))+  Kc.P1*(r2+2*yp*yp)+2*Kc.P2*xp*yp;

        p.x=xpp*zf.x;
        p.y=ypp*zf.y;
    }

    template <typename PointType>
    inline PointType Hom2Img(const PointType &ph) const{
        return {ph.x+pp.x,ph.y+pp.y};
    }

    template <typename PointType>
    inline PointType Img2Hom(const PointType &pi) const{
        return {pi.x-pp.x,pi.y-pp.y};
    }

    template <class T>
    void Hom2Img(T &ix,T &iy,const T hx, const T hy) const{
        ix=hx+pp.x;
        iy=hy+pp.y;
    }

    template <class T>
    void Img2Hom(T &hx,T &hy,const T ix, const T iy) const{
        hx=ix-pp.x;
        hy=iy-pp.y;
    }

    template <typename PointType>
    inline PointType projectHomCord(const PointType &p3d) const{
        float &&x=p3d.x/p3d.z*zfm;
        float &&y=p3d.y/p3d.z*zfm;
        float &&r=1/p3d.z;
        return {x,y,r};
    }
    template <typename PointType>
    inline PointType projectImgCord(const PointType &p3d) const{
        float &&x=p3d.x/p3d.z*zfm+pp.x;
        float &&y=p3d.y/p3d.z*zfm+pp.y;
        float &&r=1/p3d.z;
        return {x,y,r};
    }

    template <typename PointType>
    inline PointType unprojectHomCord(const PointType &pIm) const{
        float &&x=pIm.x/pIm.z/zfm;
        float &&y=pIm.y/pIm.z/zfm;
        float &&z=1.0/pIm.z;
        return {x,y,z};
    }


    template <typename PointType>
    inline PointType unprojectImgCord(const PointType &pIm) const{
        float &&x=(pIm.x-pp.x)/pIm.z/zfm;
        float &&y=(pIm.y-pp.y)/pIm.z/zfm;
        float &&z=1.0/pIm.z;
        return {x,y,z};
    }


    template <typename VectorType>
    inline VectorType projectHomCordVec(const VectorType &p3d) const{
        VectorType out;
        out[0]=p3d[0]/p3d[2]*zfm;
        out[1]=p3d[1]/p3d[2]*zfm;
        out[2]=1/p3d[2];
        return out;
    }
    template <typename VectorType>
    inline VectorType projectImgCordVec(const VectorType &p3d) const{
        VectorType out;
        out[0]=p3d[0]/p3d[2]*zfm+pp.x;
        out[1]=p3d[1]/p3d[2]*zfm+pp.y;
        out[2]=1/p3d[2];
        return out;
    }

    template <typename VectorType>
    inline VectorType unprojectHomCordVec(const VectorType &pIm) const{
        VectorType out;
        out[0]=pIm[0]/pIm[2]/zfm;
        out[1]=pIm[1]/pIm[2]/zfm;
        out[2]=1.0/pIm[2];
        return out;
    }
    template <typename VectorType>
    inline VectorType unprojectImgCordVec(const VectorType &pIm) const{
        VectorType out;
        out[0]=(pIm[0]-pp.x)/pIm[2]/zfm;
        out[1]=(pIm[1]-pp.y)/pIm[2]/zfm;
        out[2]=1.0/pIm[2];
        return out;
    }

};
}
#endif // CAM_MODEL_H
