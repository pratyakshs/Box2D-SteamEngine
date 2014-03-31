		/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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


#include "cs296_base.hpp"
#include "render.hpp"

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"

namespace cs296
{
  /**  The is the constructor 
   * This is the documentation block for the constructor.
   */ 
  
  dominos_t::dominos_t()
  {
    /// b2Body* b1: Brief pointer to the static body ground. \n
    /// Edge from (-90.0, 0.0) to (90.0, 0) \n
    /// b2BodyDef bd : ____
    /// b2EdgeShape shape: passed to b1->CreateFixture. Value set to (b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f) \n

    b2Body* b1;  
    {
      
      b2EdgeShape shape; 
      shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
      b2BodyDef bd; 
      b1 = m_world->CreateBody(&bd); 
      b1->CreateFixture(&shape, 0.0f);
    }
    
    {
      //The steam engine scaled and relatively positioned
      //The (xpos,ypos) denotes the bottom center of engine hull
      float scale=3;
      float xpos=0;
      float ypos=0;
      b2Body* sbody;
      b2Vec2 vertices[27];
      
      vertices[26].Set(xpos-5*scale,ypos+5*scale);
      vertices[25].Set(xpos-5*scale,ypos+0*scale);
      vertices[24].Set(xpos+5*scale,ypos+0*scale);
      vertices[23].Set(xpos+5*scale,ypos+5*scale);
      vertices[22].Set(xpos+4*scale,ypos+5*scale);
      vertices[21].Set(xpos+4*scale,ypos+7.5*scale);
      vertices[20].Set(xpos+1*scale,ypos+7.5*scale);
      vertices[19].Set(xpos+1*scale,ypos+8.5*scale);
      vertices[18].Set(xpos+0.5*scale,ypos+8.5*scale);
      vertices[17].Set(xpos+0.5*scale,ypos+7*scale);
      vertices[16].Set(xpos+3.5*scale,ypos+7*scale);
      vertices[15].Set(xpos+3.5*scale,ypos+5*scale);
      vertices[14].Set(xpos+2*scale,ypos+5*scale);
      vertices[13].Set(xpos+4*scale,ypos+3.5*scale);
      vertices[12].Set(xpos+4*scale,ypos+0.5*scale);
      vertices[11].Set(xpos-4*scale,ypos+0.5*scale);
      vertices[10].Set(xpos-4*scale,ypos+3.5*scale);
      vertices[9].Set(xpos-2*scale,ypos+5*scale);
      vertices[8].Set(xpos-3.5*scale,ypos+5*scale);
      vertices[7].Set(xpos-3.5*scale,ypos+7*scale);
      vertices[6].Set(xpos-0.5*scale,ypos+7*scale);
      vertices[5].Set(xpos-0.5*scale,ypos+8.5*scale);
      vertices[4].Set(xpos-1*scale,ypos+8.5*scale);
      vertices[3].Set(xpos-1*scale,ypos+7.5*scale);
      vertices[2].Set(xpos-4*scale,ypos+7.5*scale);
      vertices[1].Set(xpos-4*scale,ypos+5*scale);
      vertices[0].Set(xpos-5*scale,ypos+5*scale);
      b2ChainShape chain;
	  chain.CreateChain(vertices, 27);
	  //chain.CreateLoop(vertices, 8);
      b2FixtureDef wedgefd;
      wedgefd.shape = &chain;
      wedgefd.density = 10.0f;
      wedgefd.friction = 0.0f;
      wedgefd.restitution = 1.0f;
      b2BodyDef wedgebd;
      wedgebd.position.Set(0.0f, 0.0f);
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);
}
    {
      //The steam engine scaled and relatively positioned
      //The (xpos,ypos) denotes the bottom center 
      float scale=3;
      float xpos=0;
      float ypos=0;
      b2Body* sbody;
      b2Vec2 vertices[11];
      vertices[10].Set(xpos-3.25*scale,ypos+3*scale);
      vertices[9].Set(xpos+3.25*scale,ypos+3*scale);
      vertices[8].Set(xpos+1.25*scale,ypos+5*scale);
      vertices[7].Set(xpos+1*scale,ypos+5*scale);
      vertices[6].Set(xpos+1*scale,ypos+4.5*scale);
      vertices[5].Set(xpos+2.25*scale,ypos+3.25*scale);
      vertices[4].Set(xpos-2.25*scale,ypos+3.25*scale);
      vertices[3].Set(xpos-1*scale,ypos+4.5*scale);
      vertices[2].Set(xpos-1*scale,ypos+5*scale);
      vertices[1].Set(xpos-1.25*scale,ypos+5*scale);
      vertices[0].Set(xpos-3.25*scale,ypos+3*scale);
      b2ChainShape chain;
	  chain.CreateChain(vertices, 11);
	  //chain.CreateLoop(vertices, 8);
      b2FixtureDef wedgefd;
      wedgefd.shape = &chain;
      wedgefd.density = 10.0f;
      wedgefd.friction = 0.0f;
      wedgefd.restitution = 1.0f;
      b2BodyDef wedgebd;
      wedgebd.position.Set(0.0f, 0.0f);
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);
}
    {
	  float scale=3;
      float xpos=0;
      float ypos=0;
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(xpos,ypos);
      bd->fixedRotation = true;
      
      //The open box
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 0.1;
      fd1->friction = 0.5;
      fd1->restitution = 1.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(xpos+0.15*scale,ypos+1.25*scale, b2Vec2(xpos+0*scale,ypos+1.75*scale), 0);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 0.1;
      fd2->friction = 0.5;
      fd2->restitution = 0.f; 
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(xpos+1*scale,ypos+0.4*scale, b2Vec2(xpos-1.15*scale,ypos+1.75*scale), 0);
      fd2->shape = &bs2;
       
      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
}
	{
      b2PolygonShape shape;
      shape.SetAsBox(6.0f, 0.25f);
	
      b2BodyDef bd;
      bd.position.Set(0, 25.5f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }
{
	int num_balls=1000;
 for (int i = 0; i < num_balls; i++) {
      float angle = (i / (float)num_balls) * 2 *3.1416;
      b2Vec2 rayDir( sinf(angle), cosf(angle) );
	  
	  b2Vec2 center = b2Vec2(0,24);
	  int blastPower=100;
      b2BodyDef bd;
      bd.type = b2_dynamicBody;
      bd.fixedRotation = true; // rotation not necessary
      bd.bullet = true; // prevent tunneling at high speed
      bd.linearDamping = 0; // drag due to moving through air
      bd.gravityScale = 0; // ignore gravity
      bd.position = center; // start at blast center
      bd.linearVelocity = blastPower * rayDir;
      b2Body* body = m_world->CreateBody( &bd );
  
      b2CircleShape circleShape;
      circleShape.m_radius = 0.05; // very small
  
      b2FixtureDef fd;
      fd.shape = &circleShape;
      fd.density = 60 / (float)num_balls; // very high - shared across all particles
      fd.friction = 0; // friction not necessary
      fd.restitution = 1.f; // high restitution to reflect off obstacles
      fd.filter.groupIndex = -1; // particles should not collide with each other
      body->CreateFixture( &fd );
  }	
}
}
  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
