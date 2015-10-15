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

#include "linalg.h"
#include "carving/carving.h"
#include "carving/carving-settings.h"
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/math/special_functions/fpclassify.hpp>
#include <boost/format.hpp>

using boost::shared_ptr;
using boost::make_shared;
using boost::math::isfinite;

#define ROUND4D(f) (round(f*10000)/10000)

class GCode
{
public:
  /* class methods */
  static std::string escape(const std::string &s)
  {
    std::string tmp_str = s;
    boost::replace_all(tmp_str, "(", "{");
    boost::replace_all(tmp_str, ")", "}");
    return tmp_str;
  }
  static shared_ptr<GCode> lastMove(shared_ptr<GCode> gc);
  static shared_ptr<GCode> firstMove(shared_ptr<GCode> gc);

public:
  /* instance methods */
  virtual ~GCode()
  {
  }
  virtual std::string toStr() const = 0;
  virtual std::string toString() const;
  virtual void toNgc(std::ostream &stream) const;

  virtual shared_ptr<GCode> clone() const = 0;
  shared_ptr<GCode> cloneZ() const;

  double getX(void) const {
    return this->x;
  }
  double getY(void) const {
    return this->y;
  }
  double getZ(void) const {
    return this->z;
  }
  double getI(void) const {
    return this->i;
  }
  double getJ(void) const {
    return this->j;
  }
  double getP(void) const {
    return this->p;
  }
  std::string getComment() const {
    return this->comment;
  }

  mutable std::list< shared_ptr<GCode> > children; // FIXME Carving

protected: /* class methods & attributes */

  GCode() : x(nan("")), y(nan("")), z(nan("")), i(nan("")), j(nan("")),
  p(nan("")), s(nan("")), comment("") {}
  GCode(const GCode &o) : x(o.x), y(o.y), z(o.z), i(o.i), j(o.j),
  p(o.p), s(o.s), comment(o.comment) {}

  double x, y, z, i, j, p, s;
  std::string comment;
};

std::ostream &operator<<(std::ostream &stream, const GCode &ope);

class GContainer: public GCode
{
public:
  GContainer(const std::string &comment = "container") :
      GCode()
  {
    this->comment = escape(comment);
  }
  GContainer(const GContainer &o) :
      GCode(o)
  {
  }

  virtual shared_ptr<GCode> clone() const
  {
    shared_ptr<GCode> new_ptr = make_shared<GContainer>(*this);
    return new_ptr;
  }
  virtual std::string toStr() const
  {
    if (comment.empty()) {
      return "";
    }
    return (boost::format("%|40t|(%s)") % comment).str();
  }
};

class G0: public GCode
{ /* rapid motion */
public:
  G0(double z) :
      GCode()
  {
    assert(isfinite(z));
    this->z = z;
  }
  G0(double x, double y) :
      GCode()
  {
    assert(isfinite(x) && isfinite(y));
    this->x = x;
    this->y = y;
  }
  G0(const G0 &o) :
      GCode(o)
  {
  }
  virtual shared_ptr<GCode> clone() const
  {
    return make_shared<G0>(*this);
  }
  virtual std::string toStr() const
  {
    if (isfinite(z)) {
      return (boost::format("G0 Z%g %|40t|%s") % ROUND4D(z) % "(G0 rapid motion in Z)").str();
    } else {
      return (boost::format("G0 X%g Y%g %|40t|%s") % ROUND4D(x) % ROUND4D(y) % "(G0 rapid motion in XY)").str();
    }
  }
};

class G1: public GCode
{ /* linear motion */
public:
  G1(double z) : // Z ONLY
      GCode()
  {
    assert(isfinite(z));
    this->z = z;
  }
  G1(double z, double x, double y) : // Z ONLY TOO !!!
      GCode()
  {
    assert(isfinite(z));
    this->z = z;
    this->x = x;
    this->y = y;
  }
  G1(double x, double y) :
      GCode()
  {
    assert(isfinite(x) && isfinite(y));
    this->x = x;
    this->y = y;
  }
  G1(const G1 &o) :
      GCode(o)
  {
  }
  virtual shared_ptr<GCode> clone() const
  {
    return shared_ptr<G1>(new G1(*this));
  }
  virtual std::string toStr() const
  {
    if (isfinite(this->z)) {
      return (boost::format("G1 Z%g %|40t|%s") % ROUND4D(z) % "(G1 linear Z motion)").str();
    } else {
      return (boost::format("G1 X%g Y%g %|40t|%s") % ROUND4D(x) % ROUND4D(y) % "(G1 linear motion in XY)").str();
    }
  }
};

class G2: public GCode
{ /* arc motion in XY */
public:
  G2(double x, double y, double i, double j, double p) :
      GCode()
  {
    assert(isfinite(x) && isfinite(y) && isfinite(i) && isfinite(j) && isfinite(p));
    this->x = x;
    this->y = y;
    this->i = i;
    this->j = j;
    this->p = p;
  }
  G2(const G2 &o) :
      GCode(o)
  {
  }
  virtual shared_ptr<GCode> clone() const
  {
    return shared_ptr<G2>(new G2(*this));
  }
  virtual std::string toStr() const
  {
    return (boost::format("G2 X%g Y%g I%g J%g P%g %|40t|%s") % ROUND4D(x) % ROUND4D(y) % ROUND4D(i) % ROUND4D(j) % p
        % "(G2 arc motion in XY)").str();
  }
};

class G3: public GCode
{
public:
  G3(double x, double y, double i, double j, double p) :
      GCode()
  {
    assert(isfinite(x) && isfinite(y) && isfinite(i) && isfinite(j) && isfinite(p));
    this->x = x;
    this->y = y;
    this->i = i;
    this->j = j;
    this->p = p;
  }
  G3(const G3 &o) :
      GCode(o)
  {
  }
  virtual shared_ptr<GCode> clone() const
  {
    return shared_ptr<G3>(new G3(*this));
  }
  virtual std::string toStr() const
  {
    return (boost::format("G3 X%g Y%g I%g J%g P%g %|40t|%s") % ROUND4D(x) % ROUND4D(y) % ROUND4D(i) % ROUND4D(j) % p
        % "(G3 arc ccw in XY)").str();
  }
};

class G17: public GCode
{ /* select XY plane */
public:
  G17() :
      GCode()
  {
  }
  G17(const G17 &o) :
      GCode(o)
  {
  }
  virtual shared_ptr<GCode> clone() const
  {
    return shared_ptr<G17>(new G17(*this));
  }
  virtual std::string toStr() const
  {
    return (boost::format("G17        %|40t|%s") % "(G17 select XY plane)").str();
  }
};

class G21: public GCode
{ /* unit mm */
public:
  G21() :
      GCode()
  {
  }
  G21(const G21 &o) :
      GCode(o)
  {
  }
  virtual shared_ptr<GCode> clone() const
  {
    return shared_ptr<G21>(new G21(*this));
  }
  virtual std::string toStr() const
  {
    return (boost::format("G21    %|40t|%s") % "(G21 unit mm)").str();
  }
};

class G40: public GCode
{ /* turn cutter compensation off */
public:
  G40() :
      GCode()
  {
  }
  G40(const G40 &o) :
      GCode(o)
  {
  }
  virtual shared_ptr<GCode> clone() const
  {
    return shared_ptr<G40>(new G40(*this));
  }
  virtual std::string toStr() const
  {
    return (boost::format("G40    %|40t|%s") % "(G40 turn cutter compensation off)").str();
  }
};

class G43: public GCode
{ /* tool length compensation on */
public:
  G43() :
      GCode()
  {
  }
  G43(const G43 &o) :
      GCode(o)
  {
  }
  virtual shared_ptr<GCode> clone() const
  {
    return shared_ptr<G43>(new G43(*this));
  }
  virtual std::string toStr() const
  {
    return (boost::format("G43 %|40t|%s") % "(tool length compensation on)").str();
  }
};

class G49: public GCode
{ /* turn tool length compensation off */
public:
  G49() :
      GCode()
  {
  }
  G49(const G49 &o) :
      GCode(o)
  {
  }
  virtual shared_ptr<GCode> clone() const
  {
    return shared_ptr<G49>(new G49(*this));
  }
  virtual std::string toStr() const
  {
    return (boost::format("G49    %|40t|%s") % "(G49 turn tool length compensation off)").str();
  }
};

class G64: public GCode
{ /* continuous with optional path tolerance */
public:
  G64() :
      GCode()
  {
  }
  G64(const G64 &o) :
      GCode(o)
  {
  }
  virtual shared_ptr<GCode> clone() const
  {
    return shared_ptr<G64>(new G64(*this));
  }
  virtual std::string toStr() const
  {
    return (boost::format("G64    %|40t|%s") % "(G64 continuous with optional path tolerance)").str();
  }
};

class G90: public GCode
{ /* absolute distance mode */
public:
  G90() :
      GCode()
  {
  }
  G90(const G90 &o) :
      GCode(o)
  {
  }
  virtual shared_ptr<GCode> clone() const
  {
    return shared_ptr<G90>(new G90(*this));
  }
  virtual std::string toStr() const
  {
    return (boost::format("G90    %|40t|%s") % "(G90 absolute distance mode)").str();
  }
};

class G90_1: public GCode
{ /* absolute distance for G2/G3 I,J offsets */
public:
  G90_1() :
      GCode()
  {
  }
  G90_1(const G90_1 &o) :
      GCode(o)
  {
  }
  virtual shared_ptr<GCode> clone() const
  {
    return shared_ptr<G90_1>(new G90_1(*this));
  }
  virtual std::string toStr() const
  {
    return (boost::format("G90.1  %|40t|%s") % "(G90 absolute distance for G2/G3 I,J offsets)").str();
  }
};

class M2: public GCode
{ /* end program */
public:
  M2() :
      GCode()
  {
  }
  M2(const M2 &o) :
      GCode(o)
  {
  }
  virtual shared_ptr<GCode> clone() const
  {
    return shared_ptr<M2>(new M2(*this));
  }
  virtual std::string toStr() const
  {
    return (boost::format("M2 %|40t|%s") % "(M2 end program)").str();
  }
};

class M3: public GCode
{ /* turn spindle clockwise with specified speed */
public:
  M3(double s) :
      GCode()
  {
    this->s = s;
  }
  M3(const M3 &o) :
      GCode(o)
  {
  }
  virtual shared_ptr<GCode> clone() const
  {
    return shared_ptr<M3>(new M3(*this));
  }
  virtual std::string toStr() const
  {
    return (boost::format("M3 S%g %|40t|%s") % s % "(M3 turn spindle clockwise with specified speed)").str();
  }
};

class TM6: public GCode
{ /* change tool */
public:
  TM6(shared_ptr<const CarvingTool> tool) :
      GCode(), tool(tool)
  {
  }
  TM6(const TM6 &o) :
      GCode(o), tool(o.tool)
  {
  }
  virtual shared_ptr<GCode> clone() const
  {
    return shared_ptr<TM6>(new TM6(*this));
  }
  virtual std::string toStr() const
  {
    return (boost::format("T%d M6 %|40t|(change tool %s)") % this->tool->getToolNumber()
        % escape(this->tool->toString())).str();
  }
private:
  shared_ptr<const CarvingTool> tool;
};

class F: public GCode
{ /* change tool */
public:
  F(double tool_speed) :
      GCode(), tool_speed(tool_speed)
  {
  }
  F(const F &o) :
      GCode(o), tool_speed(o.tool_speed)
  {
  }
  virtual shared_ptr<GCode> clone() const
  {
    return shared_ptr<F>(new F(*this));
  }
  virtual std::string toStr() const
  {
    return (boost::format("F%d %|40t|(change tool speed)") % this->tool_speed).str();
  }
private:
  double tool_speed;
};


