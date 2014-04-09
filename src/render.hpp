/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/* 
 * Base code for CS 296 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * Instructor: Parag Chaudhuri
 */

#ifndef _RENDER_HPP_
#define _RENDER_HPP_

#include <Box2D/Box2D.h>

struct b2AABB;


//! This class implements debug drawing callbacks that are invoked
//! inside b2World::Step.
class debug_draw_t : public b2Draw
{
public:
	///The function for responsible for drawing Polygons using glut/glui
  void DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color);
  ///The function for responsible for drawing SolidPolygons using glut/glui
  void DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color);
  ///The function for responsible for drawing cicles using glut/glui
  void DrawCircle(const b2Vec2& center, float32 radius, const b2Color& color);
  ///The function for responsible for drawing solidcircles using glut/glui
  void DrawSolidCircle(const b2Vec2& center, float32 radius, const b2Vec2& axis, const b2Color& color);
  ///The function for responsible for drawing segments using glut/glui
  void DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color);
  ///The function for responsible for drawing transform using glut/glui
  void DrawTransform(const b2Transform& xf);
  ///The function for responsible for drawing points using glut/glui
  void DrawPoint(const b2Vec2& p, float32 size, const b2Color& color);
  ///The function for responsible for drawing string using glut/glui
  void DrawString(int x, int y, const char* string, ...); 
  ///The function for responsible for drawing AABB's using glut/glui
  void DrawAABB(b2AABB* aabb, const b2Color& color);
};


#endif
