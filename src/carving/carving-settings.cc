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

#include "printutils.h"
#include "carving/carving-settings.h"

#include <utility>	// make_pair
#include <boost/math/special_functions/fpclassify.hpp>
#include <boost/foreach.hpp>

using boost::math::isfinite;


CarvingTool::CarvingTool(int tool_number, const std::string &name, double diameter, double front_angle, double head_end,
    std::string description) :
    tool_number(tool_number), name(name), diameter(diameter), front_angle(front_angle), head_end(head_end), description(
        description)
{
  assert(tool_number > 0 && !name.empty() && isfinite(diameter) && diameter > 0);
}

CarvingTool::~CarvingTool()
{
}

double CarvingTool::getToolNumber() const
{
  return this->tool_number;
}

std::string CarvingTool::getName() const
{
  return this->name;
}

double CarvingTool::getDiameter() const
{
  return this->diameter;
}

double CarvingTool::getRadius() const
{
  return this->diameter / 2;
}

double CarvingTool::getFrontAngle() const
{
  return this->front_angle;
}

double CarvingTool::getHeadEnd() const
{
  return this->head_end;
}

std::string CarvingTool::toString() const
{
  std::stringstream stream;
  stream << "CarvingTool(tool_number=" << this->tool_number << ", name=" << this->name << ", diameter="
      << this->diameter << ", front_angle=" << this->front_angle << ", head_end=" << this->head_end << ")";
  return stream.str();
}

std::ostream &operator<<(std::ostream &stream, const CarvingTool &o)
{
  stream << o.toString();
  return stream;
}





CarvingToolSpeed::CarvingToolSpeed(const std::string &tool_name, const std::string &material_name, double spinde_speed,
    double feedrate, double step_down) :
    tool_name(tool_name), material_name(material_name), spindle_speed(spinde_speed), feedrate(feedrate), step_down(
        step_down)
{
  assert(isfinite(spindle_speed) && isfinite(feedrate) && isfinite(step_down) && spindle_speed > 0 && feedrate > 0
      && step_down > 0);
}

CarvingToolSpeed::~CarvingToolSpeed()
{
}

std::string CarvingToolSpeed::getToolName() const
{
  return this->tool_name;
}

std::string CarvingToolSpeed::getMaterialName() const
{
  return this->material_name;
}

double CarvingToolSpeed::getSpindleSpeed() const
{
  return this->spindle_speed;
}

double CarvingToolSpeed::getFeedrate() const
{
  return this->feedrate;
}

double CarvingToolSpeed::getStepDown() const
{
  return this->step_down;
}

double CarvingToolSpeed::getHorizontalFeedrate() const
{
  return this->feedrate;
}

double CarvingToolSpeed::getVerticalFeedrate() const
{
  return this->feedrate * 2/4;
}

double CarvingToolSpeed::getArcFeedrate() const
{
  return this->feedrate * 3/4;
}

std::string CarvingToolSpeed::toString() const
{
  std::stringstream stream;
  stream << "CarvingToolSpeed(tool=" << this->tool_name << ", material=" << this->material_name << ", spindle_speed="
      << this->spindle_speed << ", feedrate=" << this->feedrate << ", step_down=" << this->step_down << ")";
  return stream.str();
}

std::ostream &operator<<(std::ostream &stream, const CarvingToolSpeed &o)
{
  stream << o.toString();
  return stream;
}

/*
 * CarvingSettings
 */

CarvingSettings::CarvingSettings(double tool_change_height, double clearance_height, double fn, double fs, double fa) :
    tool_change_height(tool_change_height), clearance_height(clearance_height), fn(fn), fs(fs), fa(fa)
{
  assert(isfinite(this->tool_change_height) && this->tool_change_height > 0);
}

CarvingSettings::~CarvingSettings()
{
}

double CarvingSettings::getClearanceHeight() const
{
  return this->clearance_height;
}

double CarvingSettings::getToolChangeHeight() const
{
  return this->tool_change_height;
}

double CarvingSettings::getFn() const
{
  return this->fn;
}     // when >0, fs & fa are ignored, circle is rendered using this number of fragments.

double CarvingSettings::getFs() const
{
  return this->fs;
} // minimum size of a fragment

double CarvingSettings::getFa() const
{
  return this->fa;
} // minimum angle for a fragment

bool CarvingSettings::isValid() const
{
  bool valid = !this->tools.empty() && !this->tool_speeds.empty();
  if (!valid)
    PRINTDB("CarvingSettings::isValid %d s %s", valid % *this);
  return valid;
}

std::string CarvingSettings::toString() const
{
  std::stringstream stream;
  stream << "CarvingSettings(tool_change_height=" << this->tool_change_height << ", clearance_height="
      << this->clearance_height << ", fn=" << this->fn << ", fs=" << this->fs << ", fa=" << this->fa << ", tools.size="
      << this->tools.size() << ", tool_speeds.size=" << this->tool_speeds.size() << ")";
  return stream.str();
}

const std::map<std::string, shared_ptr<const CarvingTool> > CarvingSettings::getTools() const
{
  return this->tools;
}

void CarvingSettings::setTool(shared_ptr<const CarvingTool> tool)
{
  assert(tool);
  assert(this->tools.count(tool->getName()) == 0);
  this->tools[tool->getName()] = tool;
}

bool CarvingSettings::isTool(const std::string &tool_name) const
{
  return this->tools.count(tool_name) == 1;
}

shared_ptr<const CarvingTool> CarvingSettings::getTool(const std::string &tool_name) const
{
  std::map<std::string, shared_ptr<const CarvingTool> >::const_iterator iter;
  iter = this->tools.find(tool_name);
  if (iter == this->tools.end()) {
    PRINTB("ERROR: Carving: Unable to find tool %s", tool_name);
    assert(0);
    return shared_ptr<const CarvingTool>();
  }
  return iter->second;
}

bool CarvingSettings::isMaterial(const std::string &material_name) const
{
  typedef std::pair<std::pair<std::string, std::string>, shared_ptr<const CarvingToolSpeed> >  MapEntryType;

  BOOST_FOREACH(MapEntryType entry, this->tool_speeds) {
    if (material_name == entry.first.second) {
      return true;
    }
  }
  return false;
}

void CarvingSettings::setToolSpeed(const shared_ptr<const CarvingToolSpeed> ts)
{
  assert(ts);
  assert(this->tools.count(ts->getToolName()) == 1);
  std::pair<std::string, std::string> key = std::make_pair(ts->getToolName(), ts->getMaterialName());
  assert(this->tool_speeds.count(key) == 0);
  this->tool_speeds[key] = ts;
}

shared_ptr<const CarvingToolSpeed> CarvingSettings::getToolSpeed(const std::string &tool_name, const std::string &material_name)
{
  std::map<std::pair<std::string, std::string>, shared_ptr<const CarvingToolSpeed> >::const_iterator iter;
  iter = this->tool_speeds.find(std::make_pair(tool_name, material_name));
  if (iter == this->tool_speeds.end()) {
    PRINTB("ERROR: Carving: Unable to find tool_speed tool_name '%s', material '%s'", tool_name % material_name);
    assert(0);
    return shared_ptr<const CarvingToolSpeed>();
  }
  return iter->second;
}

std::ostream &operator<<(std::ostream &stream, const CarvingSettings &o)
{
  stream << o.toString();
  return stream;
}

