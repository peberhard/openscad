/*
 *  OpenSCAD (www.openscad.org)
 *  Copyright (C) 2014 Pascal Eberhard <pascal.eberhard@38plumes.fr>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  As a special exception, you have permission to link this program
 *  with the CGAL library and distribute executables, as long as you
 *  follow the requirements of the GNU GPL in regard to all of the
 *  software in the executable aside from CGAL.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#pragma once

#include <ostream>
#include <string>
#include <map>
#include <boost/shared_ptr.hpp>

using boost::shared_ptr;

class CarvingTool
{
public:
  CarvingTool(int tool_number, const std::string &name, double diameter, double front_angle, double head_end,
      std::string description);
  ~CarvingTool();
  double getToolNumber() const;
  std::string getName() const;
  double getDiameter() const;
  double getRadius() const;
  double getFrontAngle() const;
  double getHeadEnd() const;
  std::string toString() const;

private:
// TODO Carving: required for map() to disable it?    GCodeTool();    // forbidden default constructor

  int tool_number;          // Gcode tool number (T1,T2,T3...). Must be the same as in LinuxCNC tool table
  std::string name;
  double diameter;          // in mm
  double front_angle;           // in degree
  double head_end;          // in mm
  std::string description;
};
std::ostream &operator<<(std::ostream &stream, const CarvingTool &o);

class CarvingToolSpeed
{
public:
  CarvingToolSpeed(const std::string &tool_name, const std::string &material_name, double spinde_speed, double feedrate,
      double step_down);
  ~CarvingToolSpeed();
  std::string getToolName() const;
  std::string getMaterialName() const;
  double getSpindleSpeed() const;
  double getFeedrate() const;
  double getStepDown() const;
  double getHorizontalFeedrate() const;
  double getVerticalFeedrate() const;
  double getArcFeedrate() const;
  std::string toString() const;

private:
  std::string tool_name;
  std::string material_name;
  double spindle_speed;     // in revolutions per minute
  double feedrate;          // in mm/min
  double step_down;         // in mm,
};
std::ostream &operator<<(std::ostream &stream, const CarvingToolSpeed &o);


class CarvingSettings
{
public:
  CarvingSettings(double tool_change_height, double clearance_height, double fn, double fs, double fa);
  ~CarvingSettings();
  double getClearanceHeight() const;
  double getToolChangeHeight() const;
  double getFn() const;
  double getFs() const;
  double getFa() const;
  bool isValid() const;
  std::string toString() const;
  const std::map<std::string, shared_ptr<const CarvingTool> > getTools() const;
  void setTool(shared_ptr<const CarvingTool> tool);
  bool isTool(const std::string &tool_name) const;
  shared_ptr<const CarvingTool> getTool(const std::string &tool_name) const;
  bool isMaterial(const std::string &material_name) const;
  void setToolSpeed(const shared_ptr<const CarvingToolSpeed> ts);
  shared_ptr<const CarvingToolSpeed> getToolSpeed(const std::string &tool_name, const std::string &material_name);

private:
  double tool_change_height;
  double clearance_height;
  double fn;
  double fs;
  double fa;
  std::map<std::string, shared_ptr<const class CarvingTool> > tools;
  std::map<std::pair<std::string, std::string>, shared_ptr<const class CarvingToolSpeed> > tool_speeds;
};
std::ostream &operator<<(std::ostream &stream, const CarvingSettings &o);
