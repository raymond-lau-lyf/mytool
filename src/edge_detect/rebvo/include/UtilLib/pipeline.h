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

#ifndef PIPELINE_H
#define PIPELINE_H
#include <exception>
#include <pthread.h>

#include <mutex>
#include <thread>
#include <iostream>
namespace  rebvo{
template <typename OPipe>
class Pipeline{

    const uint CircSize;        //Size of the circular buffer
    OPipe *CircBuff;            //Circular buffer
    uint *CircPlayer;           //Player state of the buffer

    const uint PlayerNum;       //Player num and position in the buffer
    uint *PlayerPos;

    std::mutex mut;             //


    //Mutex protected check of the circular buffer. If the previous player has done
    //playing with the next buffer it will be released to the current player
    uint RequestBufferNonBlock(const int PlayerId){

        std::lock_guard<std::mutex> locker(mut);

        const uint next_inx=(PlayerPos[PlayerId]+1) % CircSize;
        // if(PlayerId==0&&CircSize==4)
        // {
        //     //    std::cout<<"next_inx id: "<<next_inx<<std::endl;
        //         // std::cout<<"====:"<<PlayerId<<","<<PlayerNum<<std::endl;
        //         std::cout<<"CircPlayer[next_inx] id: "<<CircPlayer[next_inx]<<"=="<<(PlayerId+PlayerNum-1)%PlayerNum<<std::endl;
        //     // std::cout<<"CircPlayer:"<<CircPlayer[0]<<","<<CircPlayer[1]<<","<<CircPlayer[2]<<","<<CircPlayer[3]<<std::endl;
        // }
     
        if(CircPlayer[next_inx] == (PlayerId+PlayerNum-1)%PlayerNum )    //Previus player done and the buffer is ready
            {
                return PlayerPos[PlayerId]=next_inx; //Update Id and return
            }
        else
            return -1;                          //Buffer not ready
    }

public:

    Pipeline(const uint CircularSize,const uint PlayerNumber)
        : CircSize(CircularSize), CircBuff(new OPipe[CircularSize]), CircPlayer(new uint[CircularSize]),
          PlayerNum(PlayerNumber),PlayerPos(new uint[PlayerNumber])
    {
        std::fill_n(CircPlayer,CircularSize,PlayerNum-1);
        // if(CircularSize==4)
            // std::cout<<"origion:CircPlayer:"<<CircPlayer[0]<<","<<CircPlayer[1]<<","<<CircPlayer[2]<<","<<CircPlayer[3]<<std::endl;

        std::fill_n(PlayerPos,PlayerNumber,0);
    }

    ~Pipeline(){
        delete [] CircBuff;
        delete [] CircPlayer;
        delete [] PlayerPos;
    }



    //Notification of the current player done playing with the current buffer
    void ReleaseBuffer(const uint PlayerId){

        std::lock_guard<std::mutex> locker(mut);
        // if((0==PlayerId||1==PlayerId)&&CircSize==4)
            // std::cout<<"before release:"<<CircPlayer[0]<<","<<CircPlayer[1]<<","<<CircPlayer[2]<<","<<CircPlayer[3]<<std::endl;

        CircPlayer[PlayerPos[PlayerId]] = PlayerId;
        //  if((0==PlayerId||1==PlayerId)&&CircSize==4)
            // std::cout<<"after release:"<<CircPlayer[0]<<","<<CircPlayer[1]<<","<<CircPlayer[2]<<","<<CircPlayer[3]<<std::endl;

    }

    //Busy test of the buffer to acquire the next position
    //Sleep secs is the number of second betwen each test
    OPipe &RequestBuffer(const int PlayerId,const double sleep_secs=1e-4){
        uint id;
        while((id=RequestBufferNonBlock(PlayerId)) == -1)
            std::this_thread::sleep_for(std::chrono::duration<double>(sleep_secs)); //If buffer not ready sleep...
        return CircBuff[id];
    }

    //Busy test of the buffer to acquire the next position
    //Sleep secs is the number of second betwen each test
    //Timeout is timeout
    OPipe* RequestBufferTimeoutable(const int PlayerId,const double timeout_secs=0,const double sleep_secs=1e-4){
        uint id;
        uint try_num=timeout_secs/sleep_secs+1;
        while((id=RequestBufferNonBlock(PlayerId)) == -1){
            if(--try_num<=0){
                return nullptr;
            }
            std::this_thread::sleep_for(std::chrono::duration<double>(sleep_secs)); //If buffer not ready sleep...
        }
        return &CircBuff[id];
    }

    //Operations for accesing the pipe buffer like a regular arrar
    //Only for construction, not safe during multithreading

    OPipe & operator[](uint inx){
        if(inx>=CircSize) throw std::out_of_range("Circular size out of range");
        return CircBuff[inx];
    }

    uint Size(){return CircSize;}

    typedef OPipe * iterator;
    typedef const OPipe * const_iterator;

    iterator begin(){return &CircBuff[0];}
    iterator end(){return &CircBuff[CircSize];}

};
}
#endif // PIPELINE_H
