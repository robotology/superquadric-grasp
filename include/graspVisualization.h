/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Giulia Vezzani
 * email:  giulia.vezzani@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef __VISUALIZATION_H__
#define __VISUALIZATION_H__

#include <string>
#include <deque>
#include <map>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

#include <opencv2/opencv.hpp>

/*******************************************************************************/
class GraspVisualization : public yarp::os::RateThread
{
protected:
    std::string eye;
    yarp::dev::PolyDriver GazeCtrl;
    yarp::dev::IGazeControl *igaze;
    yarp::sig::Matrix K,H;

    yarp::sig::Vector poseR, poseL, solR, solL;
    yarp::sig::Vector hand_in_poseL, hand_in_poseR;
    std::deque<yarp::sig::Vector> trajectory_right;
    std::deque<yarp::sig::Vector> trajectory_left;
    yarp::sig::Vector point2D, point, point1, superq;
    std::deque<yarp::sig::Vector> trajectory;
    yarp::sig::Matrix R;


    double &quality_right1, &quality_left1;
    double &quality_right2, &quality_left2;

public:

    double t_vis;
    bool show_hand;
    bool look_object;
    bool show_only_pose;
    std::string left_or_right;
    yarp::os::Mutex mutex;
    const yarp::sig::Vector &object;
    const std::deque<yarp::sig::Vector> &obstacles;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *imgIn;

    yarp::os::Property vis_par;
    yarp::sig::Vector &hand, &hand1;
    const std::deque<yarp::os::Property> &complete_sol;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > portImgIn;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > portImgOut;

    /***********************************************************************/
    GraspVisualization(int rate, const std::string &_eye, yarp::dev::IGazeControl *_igaze,
                       const yarp::sig::Matrix _K,  std::string left_or_right, const std::deque<yarp::os::Property> &complete_sol,
                       const yarp::sig::Vector &_object, const std::deque<yarp::sig::Vector> &_obstacles, yarp::sig::Vector &hand,
                       yarp::sig::Vector &hand1, yarp::os::Property &vis_par, double &quality_right, double &quality_left, double &quality_right2, double &quality_left2);

    /***********************************************************************/
    void addSuperq(const yarp::sig::Vector &x, yarp::sig::ImageOf<yarp::sig::PixelRgb> &imgOut,const int &col);

    /***********************************************************************/
    yarp::sig::Vector from3Dto2D(const yarp::sig::Vector &point3D);

    /***********************************************************************/
    bool showTrajectory(const std::string &hand);

    /***********************************************************************/
    virtual bool threadInit();

    /***********************************************************************/
    virtual void run();

    /***********************************************************************/
    virtual void threadRelease();

    /***********************************************************************/
    double getTime();

    /***********************************************************************/
    void getPoses(const std::deque<yarp::os::Property> &poses);

    /***********************************************************************/
    void setPar(const yarp::os::Property &newOptions, bool first_time);

    /***********************************************************************/
    yarp::os::Property getPar();
};

#endif
