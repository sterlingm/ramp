/****************************************************************************
**
** Copyright (C) 2010 Nokia Corporation and/or its subsidiary(-ies).
** All rights reserved.
** Contact: Nokia Corporation (qt-info@nokia.com)
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** You may use this file under the terms of the BSD license as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of Nokia Corporation and its Subsidiary(-ies) nor
**     the names of its contributors may be used to endorse or promote
**     products derived from this software without specific prior written
**     permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
** $QT_END_LICENSE$
**
****************************************************************************/

#include "TrajectoryView.h"

#include <math.h>
#include <QtGui>
#include <stdio.h>

TrajectoryView::TrajectoryView(QWidget *parent)
    : QGraphicsView(parent)
{
    width_ = 240;
    height_ = 240;
    maxWidthMeters_ = 4.0f;
    maxHeightMeters_ = 4.0f;

    // Setup the scene
    QGraphicsScene *scene = new QGraphicsScene(this);
    scene->setItemIndexMethod(QGraphicsScene::NoIndex);
    scene->setSceneRect(0, 0, width_, height_);

    setScene(scene);
    setCacheMode(CacheBackground);
    setViewportUpdateMode(BoundingRectViewportUpdate);
    setRenderHint(QPainter::Antialiasing);
    setTransformationAnchor(AnchorUnderMouse);
    setWindowTitle(tr("Trajectory View"));


}




void TrajectoryView::size_changed()
// Change the scene size to the updated one when the user resizes the window
{

    width_ = this->parentWidget()->frameSize().width();
    height_ = this->parentWidget()->frameSize().height();
    std::cout<<"\nwidth: "<<width_;
    std::cout<<"\nheight: "<<height_<<"\n";

    this->resize(width_,height_);
    this->scene()->setSceneRect(0, -height_, width_-10, height_-10);// We need to make the scene a little smaller than the frame
}

void TrajectoryView::population(const ramp_msgs::Population& msg)
// Update the population and called the drawing function
{
  std::cout<<"\n\nReceived Population from robot "<<msg.robot_id<<"\n";

  populations_.clear();
  populations_.push_back(msg);

  /*if(populations_.size() < 2) {
    populations_.push_back(msg);
  }

  else if(populations_.size() == 2) {
    if((unsigned int)msg.robot_id == 0) {
      populations_.erase(populations_.begin());
      populations_.insert(populations_.begin(), msg);
    }
    else if((unsigned int)msg.robot_id == 1) {
      populations_.erase(populations_.begin()+1);
      populations_.insert(populations_.begin()+1, msg);
    }
  }

  if((unsigned int)msg.robot_id == 1) {
    ROS_INFO("Displaying pop for robot 1:");
    ROS_INFO("Best id: %i", msg.best_id);
    for(int i=0;i<msg.population.size();i++) {
      ROS_INFO("Trajectory %i: %s", i, u.toString(msg.population.at(i)).c_str());
    }
  }*/
  drawPopulation();
}

/** Draw the population of trajectories in the scene */
void TrajectoryView::drawPopulation() {

  // Clear the scene
  this->scene()->clear();

  // Initialize a QPen object 
  QPen pen = QPen( QColor(0,0,0,150) ); 
  QPen pen1 = QPen( QColor(0,255,0,150) ); 
  QPen pen2 = QPen( QColor(0,0,255,150) ); 
  
  /* Draw some grid lines */
    this->scene()->addLine(0, metersToPixels(3.5, false), width_-20, metersToPixels(3.5, false), pen);
    this->scene()->addLine(metersToPixels(3.5, true), 0, metersToPixels(3.5, true), metersToPixels(3.5, false), pen);
    
    this->scene()->addLine(0, metersToPixels(3, false), width_-20, metersToPixels(3, false), pen);
    this->scene()->addLine(metersToPixels(3, true), 0, metersToPixels(3, true), metersToPixels(3.5, false), pen);
    
    this->scene()->addLine(0, metersToPixels(2, false), width_-20, metersToPixels(2, false), pen);
    this->scene()->addLine(metersToPixels(2, true), 0, metersToPixels(2, true), metersToPixels(3.5, false), pen);
    
    this->scene()->addLine(0, metersToPixels(1, false), width_-20, metersToPixels(1, false), pen);
    this->scene()->addLine(metersToPixels(1, true), 0, metersToPixels(1, true), metersToPixels(3.5, false), pen);
  

  double radius = 0.275;
  int radiusPixels = metersToPixels(radius, true);

  QPen penTraj;
  // For each population
  for(unsigned int p=0;p<populations_.size();p++) {

    // Set i to the index of the best trajectory
    int i = populations_.at(p).best_id;

    // For each trajectory in the population
    for(unsigned int t=0;t<populations_.at(p).population.size();t++) {

      // Get the points for that trajectory
      std::vector<trajectory_msgs::JointTrajectoryPoint> points = populations_.at(p).population.at(t).trajectory.points;
      //std::cout<<"\npoints.size(): "<<points.size();
      //std::cout<<"\nrobot_id: "<<populations_.at(p).robot_id;
      //std::cout<<"\nfeasible: "<<(int)populations_.at(p).population.at(i).feasible;
      //std::cout<<"\npoints[0]: ("<<points.at(0).positions.at(0)<<", "<<points.at(0).positions.at(1)<<")\n";

      // If movingOn, set to black
      if(t == populations_.at(p).population.size()-1) 
      {
        penTraj = QPen( QColor(0, 0, 0, 255) );
      }
      // Green for robot 1 and feasible
      else if(populations_.at(p).robot_id == 0 && populations_.at(p).population.at(t).feasible) 
      {
        penTraj = QPen( QColor(0, 255, 0, 255) );
      }
      // Blue for robot 2 and feasible
      else if(populations_.at(p).robot_id == 1 && populations_.at(p).population.at(t).feasible) 
      {
        penTraj = QPen( QColor(0,0,255,255) );
      }
      
      // Else, if either are in collision, red
      else 
      {
        penTraj = QPen( QColor(255,0,0,150) );
      }

      // If the best trajectory (and not in collision), set to blue
      // Used for single robot traj viewing
      if(t == i && populations_.at(p).population.at(t).feasible) 
      {
        penTraj = QPen( QColor(0,0,255,255) );
      }

      if(points.size() == 1) {
        std::vector<float> p;
        p.push_back(points.at(0).positions.at(0));
        p.push_back(points.at(0).positions.at(1));

        this->scene()->addEllipse(metersToPixels(p.at(0), true)-(radiusPixels/2),
                                    metersToPixels(p.at(1), false)+(radiusPixels/2),
                                    metersToPixels(0.275f, true), metersToPixels(0.275f, false), pen);
      } //end if 1 point

      else if (points.size() > 0) {
        // For each point in the trajectory
        for(int j = 0 ; j < (points.size() -2 ) ; j+=2) {

          // If the first point
          if(j == 0) {
            // Set pen color
            if(populations_.at(p).robot_id == 0) 
            {
              pen = pen1;
            }
            else 
            {
              pen = pen2;
            }
            // Draw a circle
            this->scene()->addEllipse(metersToPixels(points.at(j).positions.at(0), true)-(radiusPixels/2),
                                      metersToPixels(points.at(j).positions.at(1), false)+(radiusPixels/2),
                                      metersToPixels(radius, true), metersToPixels(radius, false), pen);
            pen = penTraj;               
          } // end if first point

          // Draw a line to the next point
          this->scene()->addLine(metersToPixels(points.at(j).positions.at(0), true),
                         metersToPixels(points.at(j).positions.at(1), false),
                         metersToPixels(points.at(j+2).positions.at(0), true),
                         metersToPixels(points.at(j+2).positions.at(1), false),
                         pen);

        } //end for each point in the trajectory
      } //end if many points
    } //end for each trajectory
  } //end for each population 
} //End drawPopulation



int const TrajectoryView::metersToPixels(float value, bool isWidth)
//Calculate the pixel value of a distance. If width is true, treat the value has a x position, if false treat it as a y position.
{
    if (isWidth)
    {
        if (value > maxWidthMeters_)
            maxWidthMeters_ = value;

        //return value * width_ / (maxWidthMeters_ - 1);
        return value * width_ / (maxWidthMeters_);
    }

    if (!isWidth)
    {
        if (value > maxHeightMeters_)
            maxHeightMeters_ = value;
        value *= -1;

        //return value * height_ / (maxHeightMeters_ - 1);
        return value * height_ / (maxHeightMeters_);
    }
}


