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
#include <stdio.h>

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"
   
  	float xpos = 10;///the x-ordinate of engine center. Center refers to bottom center of engine
	float scale = 3;///the scale for engine
	float ypos = 10;///the y-ordinate of engine center. Center refers to bottom center of engine
	bool accl = false;///This  varible controls the acceleration of steam enngine
	bool stop = false;///This controls the breaks on engine
	bool checker =false;


namespace cs296
{
  /**  The is the constructor 
   * This is the documentation block for the constructor.
   */ 

  //in FooTest constructor
  dominos_t::dominos_t()
  {
    /// b2Body* b1: Brief pointer to the static body ground. \n
    /// Edge from (-90.0, 0.0) to (90.0, 0) \n
    /// b2BodyDef bd : ____
    /// b2EdgeShape shape: passed to b1->CreateFixture. Value set to (b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f) \n
	      printf("hi");
	    b2Body* b1; ///The ground body 
    {
      
      b2EdgeShape shape; 
      shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
      b2BodyDef bd; 
      b1 = m_world->CreateBody(&bd); 
      b1->CreateFixture(&shape, 0.0f);
    }
    ///The rails
    /*    {
      b2EdgeShape shape; 
      shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
      b2BodyDef bd; 
      b1 = m_world->CreateBody(&bd); 
      b1->CreateFixture(&shape, 0.0f);
    }*/
    ///This part corresponds to exterior part of the engine
    {
      //The steam engine scaled and relatively positioned
      //The (xpos,ypos) denotes the bottom center of engine hull
      b2Vec2 vertices[18];
      vertices[17].Set(xpos-4*scale,ypos+0.5*scale);
      vertices[16].Set(xpos-4*scale,ypos+1.5*scale);
      vertices[15].Set(xpos-5*scale,ypos+1.5*scale);
      vertices[14].Set(xpos-5*scale,ypos+0*scale);
      vertices[13].Set(xpos+5*scale,ypos+0*scale);
      vertices[12].Set(xpos+5*scale,ypos+5*scale);
      vertices[11].Set(xpos+4*scale,ypos+5*scale);
      vertices[10].Set(xpos+4*scale,ypos+7.5*scale);
      vertices[9].Set(xpos+1*scale,ypos+7.5*scale);
      vertices[8].Set(xpos+1*scale,ypos+8.5*scale);
      vertices[7].Set(xpos+0.5*scale,ypos+8.5*scale);
      vertices[6].Set(xpos+0.5*scale,ypos+7*scale);
      vertices[5].Set(xpos+3.5*scale,ypos+7*scale);
      vertices[4].Set(xpos+3.5*scale,ypos+5*scale);
      vertices[3].Set(xpos+2*scale,ypos+5*scale);
      vertices[2].Set(xpos+4*scale,ypos+3.5*scale);
      vertices[1].Set(xpos+4*scale,ypos+0.5*scale);
      vertices[0].Set(xpos-4*scale,ypos+0.5*scale);
      
      b2ChainShape chain;
	  chain.CreateChain(vertices, 18);
	  //chain.CreateLoop(vertices, 8);
      b2FixtureDef fd1;
      fd1.shape = &chain;
      fd1.density = 10.0f;
      fd1.friction = 0.0f;
      fd1.restitution = 1.0f;
      
      
      b2Vec2 v2[10];
      b2Vec2 v1[9];
      v2[9].Set(xpos-5*scale,ypos+5*scale);
      v2[8].Set(xpos-5*scale,ypos+2*scale);
      v2[7].Set(xpos-4*scale,ypos+2*scale);
      v2[6].Set(xpos-4*scale,ypos+3.5*scale);
      v2[5].Set(xpos-2*scale,ypos+5*scale);
      v2[4].Set(xpos-3.5*scale,ypos+5*scale);
      v2[3].Set(xpos-3.5*scale,ypos+5.39*scale);
      v2[2].Set(xpos-4*scale,ypos+5.39*scale);
      v2[1].Set(xpos-4*scale,ypos+5*scale);
      v2[0].Set(xpos-5*scale,ypos+5*scale);
      b2ChainShape chain2;
	  chain2.CreateChain(v2, 10);
      v1[8].Set(xpos-4*scale,ypos+7.5*scale);
      v1[7].Set(xpos-4*scale,ypos+5.61*scale);
      v1[6].Set(xpos-3.5*scale,ypos+5.61*scale);
      v1[5].Set(xpos-3.5*scale,ypos+7*scale);
      v1[4].Set(xpos-0.5*scale,ypos+7*scale);
      v1[3].Set(xpos-0.5*scale,ypos+8.5*scale);
      v1[2].Set(xpos-1*scale,ypos+8.5*scale);
      v1[1].Set(xpos-1*scale,ypos+7.5*scale);
      v1[0].Set(xpos-4*scale,ypos+7.5*scale);
	  b2ChainShape chain3;
	  chain3.CreateChain(v1, 9);
      
      b2FixtureDef fd3;
      fd3.shape = &chain2;
      fd3.density = 10.0f;
      fd3.friction = 0.0f;
      fd3.restitution = 1.0f;
      
      
      b2FixtureDef fd2;
      fd2.shape = &chain3;
      fd2.density = 10.0f;
      fd2.friction = 0.0f;
      fd2.restitution = 1.0f;
      
      b2BodyDef engine;
      engine.position.Set(0.0f, 0.0f);
      engine.type = b2_staticBody;
             
      b2Body* box1 = m_world->CreateBody(&engine);
      box1->CreateFixture(&fd1);
      box1->CreateFixture(&fd2);
      box1->CreateFixture(&fd3);
}
///Below block corresponds to interior part of engine
    {
      //The steam engine scaled and relatively positioned
      //The (xpos,ypos) denotes the bottom center 
      b2Body* engine_int;
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
      b2FixtureDef enginefd;
      enginefd.shape = &chain;
      enginefd.density = 10.0f;
      enginefd.friction = 0.0f;
      enginefd.restitution = 1.0f;
      b2BodyDef enginebd;
      enginebd.type = b2_staticBody;
            enginebd.position.Set(0.0f, 0.0f);
      engine_int = m_world->CreateBody(&enginebd);
      engine_int->CreateFixture(&enginefd);
            
            
            b2Body* ext;
            b2BodyDef exhaustbd;
      b2FixtureDef exhaust;
	  b2EdgeShape shape; 
      shape.Set(b2Vec2(-2.25f*scale, 3.5f*scale), b2Vec2(2.25*scale, 3.5f*scale));
      exhaust.shape=&shape;
      exhaust.restitution = 0.f;
      exhaust.friction = 1;
      exhaust.density = 10.0f;
      exhaustbd.type = b2_staticBody;
      exhaustbd.position.Set(xpos, ypos);
      ext = m_world->CreateBody(&exhaustbd);
      ext->CreateFixture(&exhaust);
      ext->SetUserData( this );
}

///Piston rod of engine
{
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(xpos,ypos);
      bd->fixedRotation = true;
      //The open box
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 0.1;
      fd1->friction = 0.0;
      fd1->restitution = 1.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(0.15*scale,1.24*scale, b2Vec2(0*scale,1.75*scale), 0);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 0.1;
      fd2->friction = 0.0;
      fd2->restitution = 1.f; 
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(3*scale,0.2*scale, b2Vec2(-3.15*scale,1.75*scale), 0);
      fd2->shape = &bs2;
      fd2->filter.groupIndex = -1;
      b2Body* piston = m_world->CreateBody(bd);
      piston->CreateFixture(fd1);
      piston->CreateFixture(fd2);
}
///The upper cover of engine
	{
      b2PolygonShape shape;
      shape.SetAsBox(2.0f*scale, 0.08f*scale);
	
      b2BodyDef bd;
      bd.position.Set(xpos,ypos+ 8.5*scale);
      b2FixtureDef fd;
      fd.restitution=1.f;
      fd.shape=&shape;
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&fd);
    }
///The invisible block needed to block particles from going outside 
///the engine hull from valve rod ending   
    {
      b2PolygonShape shape1;
      shape1.SetAsBox(0.25f*scale, 0.1f*scale);
	
      b2BodyDef bd;
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->shape = new b2PolygonShape;
      bd.position.Set(xpos - 3.75*scale,ypos+ 5.5*scale);
      fd1->shape = &shape1;
      fd1->filter.groupIndex = -2;
      fd1->filter.maskBits  = 0x0001;
      fd1->filter.categoryBits  = 0x0002;
      fd1->restitution  = 0;
      fd1->friction  = 0;
      b2Body* block = m_world->CreateBody(&bd);
      block->CreateFixture(fd1);
      
    }
    {
      b2PolygonShape shape1;
      shape1.SetAsBox(0.5f*scale, 0.19f*scale);
	
      b2BodyDef bd;
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->shape = new b2PolygonShape;
      bd.position.Set(xpos - 4.5*scale,ypos+ 1.76*scale);
      fd1->shape = &shape1;
      fd1->filter.groupIndex = -2;
      fd1->filter.maskBits  = 0x0001;
      fd1->filter.categoryBits  = 0x0002;
      b2Body* block = m_world->CreateBody(&bd);
      block->CreateFixture(fd1);
      
    }
    {
      
      b2EdgeShape shape1; 
      shape1.Set(b2Vec2(0*scale,3.45*scale),b2Vec2(6*scale,3.45*scale));
	
      b2BodyDef bd;
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->shape = new b2EdgeShape;
      bd.position.Set(xpos ,ypos);
      fd1->shape = &shape1;
      fd1->filter.groupIndex = -1;
      fd1->filter.maskBits  = 0x0005;
      fd1->filter.categoryBits  = 0x0004;
      b2Body* block = m_world->CreateBody(&bd);
      block->CreateFixture(fd1);	
	}

    {
      
      b2EdgeShape shape1; 
      shape1.Set(b2Vec2(0*scale,3.85*scale),b2Vec2(6*scale,3.85*scale));
	
      b2BodyDef bd;
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->shape = new b2EdgeShape;
      bd.position.Set(xpos ,ypos);
      fd1->shape = &shape1;
      fd1->filter.groupIndex = -1;
      fd1->filter.maskBits  = 0x0005;
      fd1->filter.categoryBits  = 0x0004;
      b2Body* block = m_world->CreateBody(&bd);
      block->CreateFixture(fd1);	
	}

///The valve rod of engine    
        {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(xpos,ypos+5*scale);
      bd->fixedRotation = true;
      
      //The open box
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0;
      fd1->friction = 0;
      fd1->restitution = 1.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(1.9*scale,0.1*scale, b2Vec2(0.5f*scale,0.5f*scale), 0);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10.0;
      fd2->friction = 0;
      fd2->restitution = 1.f; 
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.2*scale,0.2*scale, b2Vec2(2.2*scale,0.2f*scale), 0);
      fd2->shape = &bs2;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 10.0;
      fd3->friction = 0;
      fd3->restitution = 1.f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.2*scale,.2*scale, b2Vec2(-1.2*scale,0.2f*scale), 0);
      fd3->shape = &bs3;
      b2FixtureDef *fd4 = new b2FixtureDef;
      fd4->density = 10.0;
      fd4->friction = 0;
      fd4->restitution = 1.f;
      fd4->filter.groupIndex = -1;
      fd4->shape = new b2PolygonShape;
      b2PolygonShape bs4;
      bs4.SetAsBox(3*scale,0.1*scale, b2Vec2(-4.f*scale,0.5f*scale), 0);
      fd4->shape = &bs4;
       
      b2Body* valveRod = m_world->CreateBody(bd);
      valveRod->CreateFixture(fd1);
      valveRod->CreateFixture(fd2);
      valveRod->CreateFixture(fd3);
      valveRod->CreateFixture(fd4);
}
{
	  int num_balls=10;
	  for (int i = 0; i < num_balls; i++) {
      float angle = (i / (float)num_balls) * 2 *3.1416;
      b2Vec2 rayDir( sinf(angle), cosf(angle) );
	  
	  b2Vec2 center = b2Vec2(xpos+0,ypos+8*scale);
	  int blastPower=100;
      b2BodyDef bd;
      bd.type = b2_dynamicBody;
      bd.fixedRotation = true; 
      bd.bullet = true; 
      bd.linearDamping = 0; 
      bd.gravityScale = 0;
      bd.position = center; 
      bd.linearVelocity = blastPower * rayDir;
      b2Body* particle = m_world->CreateBody( &bd );
	  particle->SetUserData( this );
      b2CircleShape circleShape;
      circleShape.m_radius = 0.1; // very small
  
      b2FixtureDef fd;
      fd.shape = &circleShape;
      fd.density = 60 ; 
      fd.friction = 0; 
      fd.restitution = 0.f;
      fd.filter.groupIndex=-1; 
      fd.filter.categoryBits = 0x0001; 
      particle->CreateFixture( &fd );
  }	
}
}
  sim_t *sim = new sim_t("Steam Engine Simulation", dominos_t::create);
}
