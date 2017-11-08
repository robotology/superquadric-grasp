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

/**
  * This class shows the computed grasping pose and trajectory overlapped
  * to the camera images.
  */
/*******************************************************************************/
class GraspVisualization : public yarp::os::RateThread
{
protected:
    // Gaze variables
    std::string eye;
    yarp::dev::PolyDriver GazeCtrl;
    yarp::dev::IGazeControl *igaze;
    yarp::sig::Matrix K,H;

    // pose and trajectory variables
    yarp::sig::Vector poseR, poseL, solR, solL;
    yarp::sig::Vector hand_in_poseL, hand_in_poseR;
    std::deque<yarp::sig::Vector> trajectory_right;
    std::deque<yarp::sig::Vector> trajectory_left;
    yarp::sig::Vector point2D, point, point1, superq;
    std::deque<yarp::sig::Vector> trajectory;
    yarp::sig::Matrix R;

    double &quality_right, &quality_left;

public:

    /** Visualization time*/
    double t_vis;
    /** Boolean variable for set if to show hand or not*/
    bool show_hand;
    /** Boolean variable for set if to look object or not*/
    bool look_object;
    /** Boolean variable for set if to show trajectory or not*/
    bool show_only_pose;
    /** Which hand is involved*/
    std::string left_or_right;

    yarp::os::Mutex mutex;
    /** Object superquadric*/
    const yarp::sig::Vector &object;
    /** Input image*/
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *imgIn;

    /** Visualization parameters*/
    yarp::os::Property vis_par;
     /** One hand ellipsoid*/
    yarp::sig::Vector &hand;
    /** One hand ellipsoid*/
    yarp::sig::Vector &hand1;
    /** Complete solution */
    const yarp::os::Property &complete_sol;

    bool &executed;
    bool stop_fixate;

    /** Buffered port of input image*/
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > portImgIn;
    /** Buffered port of output image*/
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > portImgOut;

    /***********************************************************************/
    GraspVisualization(int rate, const std::string &_eye, yarp::dev::IGazeControl *_igaze, bool &executed,
                       const yarp::sig::Matrix _K,  std::string left_or_right, const yarp::os::Property &complete_sol,
                       const yarp::sig::Vector &_object,  yarp::sig::Vector &hand,  yarp::sig::Vector &hand1, yarp::os::Property &vis_par, double &quality_right, double &quality_left);

    /** Overlap superquadric on the camera image
    * @param x is the superquadric to be visualization
    * @param imgOut is the image on which to overlpa the superquadroc
    * @param col is the color to be used for drawing
    */
    /***********************************************************************/
    void addSuperq(const yarp::sig::Vector &x, yarp::sig::ImageOf<yarp::sig::PixelRgb> &imgOut,const int &col);

    /** Convert 3D point to 3D point
    * @param point3D is the 3D point to be converted
    * @return a 2D vector representing the pixel
    */
    /***********************************************************************/
    yarp::sig::Vector from3Dto2D(const yarp::sig::Vector &point3D);

    /** Show the computed trajectory
    * @param hand is the name of the hand used
    * @return true
    */
    /***********************************************************************/
    bool showTrajectory(const std::string &hand);

    /** Init function of the thread */
    /***********************************************************************/
    virtual bool threadInit();

    /** Run function of the thread */
    /***********************************************************************/
    virtual void run();

     /** release function of the thread */
    /***********************************************************************/
    virtual void threadRelease();

     /** Get time required for showing the superquadrics and poses
     * @return the thread period
     */
    /***********************************************************************/
    double getTime();

     /** Acquire poses from other threads
    * @param poses contains the poses to be reached
    */
    /***********************************************************************/
    void getPoses(const yarp::os::Property &poses);

     /** Set parameters for visualization
     * @param newOptions are the options to be set
     * @param first_time takes into account if the options have been set once
     */
    /***********************************************************************/
    void setPar(const yarp::os::Property &newOptions, bool first_time);

    /** Set parameters used for visualization
    * @return a Property with the options set for visualization
    */
    /***********************************************************************/
    yarp::os::Property getPar();
};

#endif
