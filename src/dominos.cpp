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
          
    //Top horizontal shelf
    /// Top horizontal shelf: \n
    /// b2Body* ground: Pointer to the top horizontal shelf. \n
    /// Static box(density 0.0) with height 0.5 and width 12.0 and position of center (-31.0f, 30.0f) \n
    {
      b2PolygonShape shape;
      shape.SetAsBox(6.0f, 0.25f);
	
      b2BodyDef bd;
      bd.position.Set(-31.0f, 30.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }

    //Dominos
    /// 10 dominos are created inside a for loop: \n
    /// b2Body *body: Pointer to a domino. Dynamic box(density 20, friction 0.1) with height 2 and width 0.2. \n
    /// The ith domino is at position (-35.5 + 1.0*i. 31.25) \n
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 1.0f);
	
      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 20.0f;
      fd.friction = 0.1f;
		
      for (int i = 0; i < 10; ++i)
	{
	  b2BodyDef bd;
	  bd.type = b2_dynamicBody;
	  bd.position.Set(-35.5f + 1.0f * i, 31.25f);
	  b2Body* body = m_world->CreateBody(&bd);
	  body->CreateFixture(&fd);
	}
    }
      
    //Another horizontal shelf
    /// Second horizontal shelf: \n
    /// b2Body* ground: Pointer to the top horizontal shelf. \n
    /// Static box(density 0.0) with height 0.5 and width 14.0 and position of center (-19.0f, 26.0f) \n
    {
      b2PolygonShape shape;
      shape.SetAsBox(7.0f, 0.25f, b2Vec2(-20.f,20.f), 0.0f);
	
      b2BodyDef bd;
      bd.position.Set(1.0f, 6.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }


    //The pendulum that knocks the dominos off
    /// Pendulum: \n
    /// b2Body* b2: Pointer to pendulum's lower clamp: a static box of height 3 and width 0.5, with position (-36.5, 28.0) and density 10.0 \n
    /// b2Body* b4: Pointer to pendulum's bob: a dynamic box of height 0.5 and width 0.5 with initial position (-40.0, 33.0) and density 2.0 \n
    /// b2RevoluteJointDef jd: centers of the bob and clamp are anchored at (-37.0, 40.0) \n
    {
      b2Body* b2;
      {
	b2PolygonShape shape;
	shape.SetAsBox(0.25f, 1.5f);
	  
	b2BodyDef bd;
	bd.position.Set(-36.5f, 28.0f);
	b2 = m_world->CreateBody(&bd);
	b2->CreateFixture(&shape, 10.0f);
      }
	
      b2Body* b4;
      {
	b2PolygonShape shape;
	shape.SetAsBox(0.25f, 0.25f);
	  
	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	bd.position.Set(-40.0f, 33.0f);
	b4 = m_world->CreateBody(&bd);
	b4->CreateFixture(&shape, 2.0f);
      }
	
      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(-37.0f, 40.0f);
      jd.Initialize(b2, b4, anchor);
      m_world->CreateJoint(&jd);
    }
      
    //The train of small spheres
    /// The train of small spheres: \n
    /// b2Body* spherebody: Pointer to the train of small spheres: fixture ballfd(circle shape with radius 0.5) of density 1.0, friction 0.0 and restitution 0.0 \n
    /// 10 dynamic bodies with the above fixture are created inside a for loop \n
    /// The position of the ith sphere is (-22.2, 1.0*i. 26.6) \n
  
    {
      b2Body* spherebody;
	
      b2CircleShape circle;
      circle.m_radius = 0.5;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 1.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
	
      for (int i = 0; i < 10; ++i)
	{
	  b2BodyDef ballbd;
	  ballbd.type = b2_dynamicBody;
	  ballbd.position.Set(-22.2f + i*1.0, 26.6f);
	  spherebody = m_world->CreateBody(&ballbd);
	  spherebody->CreateFixture(&ballfd);
	}
    }

    //The pulley system
    /// The pulley system consisting of an open box, a bar and a pulley joint: \n
    /// b2BodyDef *bd: is a body object with position (-10, 15) and fixedRotation value set to true. \n
    /// Open box is a body named box1. The boundary of open box is made from 3 static box fixtures, viz. fd1, fd2 and fd3(density 10.0, friction 0.5, restitution 0.0). \n 
    /// bs1 shape: Bottom box is centered at (0, -1.9), has width 4, height 0.4. \n
    /// bs2 fixture: Right box is centered at (2.0, 0.0), has width 0.4, height 4. \n
    /// bs3: Left box is centered at (-2.0, 0.0), has width 0.4, height 4. \n
    /// b2Body* box1 is the pointer to this open box. It is created by passing b2BodyDef* bd to CreateBody(). Fixtures fd1, fd2 and fd3 are added to box1. \n
    
    /// The bar is a body named box2 constructed by passing b2BodyDef *bd to CreateBody(), with position changed to (10, 15) \n 
    /// The density for it's fixture fd1, is 34.0 \n
    /// The pulley joint is made from 4 anchor points: center of open box, center of bar and two hinged points. \n
    ///  b2Vec2 worldAnchorOnBody1(-10, 15) is Anchor point on body 1 in world axis \n
    ///  b2Vec2 worldAnchorOnBody2(10, 15) is Anchor point on body 2 in world axis \n
    ///  b2Vec2 worldAnchorGround1(-10, 20) is Anchor point for ground 1 in world axis \n
    ///  b2Vec2 worldAnchorGround2(10, 20) is Anchor point for ground 2 in world axis \n
    ///  float32 ratio = 1.0 is the dampening ratio \n
    ///  myjoint->Initialize() is called with arguments box1, box2, worldAnchorGround1, worldAnchorGround2,box1->GetWorldCenter(), box2->GetWorldCenter() and ratio \n
    ///  m_world->CreateJoint(myjoint) creates the pulley system in m_world. \n
    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(-10,15);
      bd->fixedRotation = true;
      
      //The open box
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0;
      fd1->friction = 0.5;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10.0;
      fd2->friction = 0.5;
      fd2->restitution = 0.f; 
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.2,2, b2Vec2(2.0f,0.f), 0);
      fd2->shape = &bs2;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 10.0;
      fd3->friction = 0.5;
      fd3->restitution = 0.f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.2,2, b2Vec2(-2.0f,0.f), 0);
      fd3->shape = &bs3;
       
      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      box1->CreateFixture(fd3);

      //The bar
      bd->position.Set(10,15);	
      fd1->density = 34.0;	  
      b2Body* box2 = m_world->CreateBody(bd);
      box2->CreateFixture(fd1);

      // The pulley joint
      b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorOnBody1(-10, 15); // Anchor point on body 1 in world axis
      b2Vec2 worldAnchorOnBody2(10, 15); // Anchor point on body 2 in world axis
      b2Vec2 worldAnchorGround1(-10, 20); // Anchor point for ground 1 in world axis
      b2Vec2 worldAnchorGround2(10, 20); // Anchor point for ground 2 in world axis
      float32 ratio = 1.0f; // Define ratio
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);
    }

    //The revolving horizontal platform
    /// The revolving horizontal platform: \n
    /// b2PolygonShape shape: a box shape of width 4.4, height 0.4 \n
	/// b2BodyDef bd: position is set to (14.0f, 14.0f) and type is dynamic \n
	/// b2Body* body: initialized by m_world->CreateBody(&bd) \n
	/// b2FixtureDef *fd: fixture with density 1 and shape as defined above. body->CreateFixture(fd) is called \n
	/// b2PolygonShape shape2: a box shape of width 0.4, height 4.0 \n
	/// b2BodyDef bd2: position is set to (14.0f, 16.0f) \n
	/// b2Body* body2: initialized by m_world->CreateBody(&bd2) \n
	/// b2RevoluteJointDef jointDef: bodyA set to body and bodyB set to body. LocalAnchorA and LocalAnchorB are set to (0, 0). collideConneceted is set to false. \n
   {
      b2PolygonShape shape;
      shape.SetAsBox(2.2f, 0.2f);
	
      b2BodyDef bd;
      bd.position.Set(14.0f, 14.0f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      b2PolygonShape shape2;
      shape2.SetAsBox(0.2f, 2.0f);
      b2BodyDef bd2;
      bd2.position.Set(14.0f, 16.0f);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
    }

    //The heavy sphere on the platform
    /// The heavy sphere on the left platform: \n
    /// b2CircleShape circle: radius 1.0 \n
    /// b2FixtureDef ballfd: shape from above circle, density 50.0, friction 0.0, restitution 0.0 \n
    /// b2BodyDef ballbd: dynamic body with position (14.0, 18.0)
    /// b2Body* sbody: body of the sphere, with value m_world->CreateBody(&ballbd) and fixture ballfd \n
    {
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 1.0;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 200.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(14.0f, 18.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }


    //The see-saw system at the bottom
    /// The see-saw system at the bottom consists of a triangle wedge, a plank on top of it and the light box:
    /// The triangle wedge: \n
    ///  b2Vec2 vertices[3]: array of 3 vertices (-1, 0), (1, 0), (0, 1.5) \n
    ///  b2PolygonShape poly: polygon of above 3 vertices \n
    /// b2FixtureDef wedgefd: fixture for wedge shape from above poly, density 10.0, friction 0.0, restitution 0.0 \n
    /// b2BodyDef wedgebd: wedge body definition with position (30.0, 0.0). \n
    /// b2Body* sbody: wedge body constructed by m_world->CreateBody(&wedgebd) and fixture wedgefd \n
    
    /// The plank on top of the wedge: \n
    /// b2PolygonShape shape: box of width 30.0, height 0.4 \n
    /// b2BodyDef bd2: definition of plank as dynamic body with position (30.0, 1.5) \n
    /// b2FixtureDef fd2: fixture for plank with shape as defined above, density 1.0 \n
    /// b2Body* body: plank body constructed by m_world->CreateBody(&bd2) and fixture fd2 \n
    /// b2RevoluteJointDef jd: joint between plank and wedge with anchor at (30.0, 1.5) \n
    
    /// The light box on right side of see-saw: \n
    /// b2PolygonShape shape2: box of width 4.0, height 4.0 \n
    /// b2BodyDef bd3: definition of box as dynamic body with position (40.0, 2.0) \n
    /// b2FixtureDef *fd3: fixture for box with shape as shape2, density 0.01 \n
    /// b2Body* body3: box body constructed by m_world->CreateBody(&bd3) and fixture fd3 \n
    {
      //The triangle wedge
      b2Body* sbody;
      b2PolygonShape poly;
      b2Vec2 vertices[3];
      vertices[0].Set(-1,0);
      vertices[1].Set(1,0);
      vertices[2].Set(0,1.5);
      poly.Set(vertices, 3);
      b2FixtureDef wedgefd;
      wedgefd.shape = &poly;
      wedgefd.density = 10.0f;
      wedgefd.friction = 0.0f;
      wedgefd.restitution = 0.0f;
      b2BodyDef wedgebd;
      wedgebd.position.Set(30.0f, 0.0f);
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);

      //The plank on top of the wedge
      b2PolygonShape shape;
      shape.SetAsBox(15.0f, 0.2f);
      b2BodyDef bd2;
      bd2.position.Set(30.0f, 1.5f);
      bd2.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd2);
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 1.f;
      fd2->shape = new b2PolygonShape;
      fd2->shape = &shape;
      body->CreateFixture(fd2);

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(30.0f, 1.5f);
      jd.Initialize(sbody, body, anchor);
      m_world->CreateJoint(&jd);

      //The light box on the right side of the see-saw
      b2PolygonShape shape2;
      shape2.SetAsBox(2.0f, 2.0f);
      b2BodyDef bd3;
      bd3.position.Set(43.0f, 2.0f);
      bd3.type = b2_dynamicBody;
      b2Body* body3 = m_world->CreateBody(&bd3);
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 0.005f;
      fd3->shape = new b2PolygonShape;
      fd3->shape = &shape2;
      body3->CreateFixture(fd3);
    }
    
    //My pulley system
    /// The second pulley system to the right, consisting of two bars and a pulley joint: \n
    /// b2BodyDef *bd: is a body object with position (25, 15) and fixedRotation value set to true. \n
  
      /// Left bar is a body named box1 constructed by passing b2BodyDef *bd to CreateBody() \n 
      /// b2PolygonShape bs1: a box of width 5.0 and height 0.4 \n
    /// b2FixtureDef *fd1: fixture with shape bs1, density 1.0, friction 0.5, restitution 0.0 \n
    /// fd2 is fixture for box2
    
      /// Right bar is a body named box2 constructed by passing b2BodyDef *bd to CreateBody(), with position changed to (36, 15-1.9) \n 
      /// b2PolygonShape bs2: a box of width 5.0 and height 0.4 \n
    /// b2FixtureDef *fd2: fixture with shape bs2, density 0.99, friction 0.5, restitution 0.0 \n
    /// fd2 is fixture for box2
    /// The pulley joint is made from 4 anchor points: centers of left and right bars and two hinged points. \n
    /// b2Vec2 worldAnchorOnBody1(25, 15) is Anchor point on body 1 in world axis \n
    /// b2Vec2 worldAnchorOnBody2(36, 15) is Anchor point on body 2 in world axis \n
    /// b2Vec2 worldAnchorGround1(25, 20) is Anchor point for ground 1 in world axis \n
    /// b2Vec2 worldAnchorGround2(36, 20) is Anchor point for ground 2 in world axis \n
    /// float32 ratio = 1.0 is the dampening ratio \n
    /// myjoint->Initialize() is called with arguments box1, box2, worldAnchorGround1, worldAnchorGround2,box1->GetWorldCenter(), box2->GetWorldCenter() and ratio \n
    /// m_world->CreateJoint(myjoint) creates the pulley system in m_world. \n
    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(25,15);
      bd->fixedRotation = true;
        
      //Left bar
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 1.0;
      fd1->friction = 0.5;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2.5,0.2, b2Vec2(0.f,-1.9f), 0);
      fd1->shape = &bs1;
         
      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
  
      //Right bar    
      bd->position.Set(36,15);         
      b2Body* box2 = m_world->CreateBody(bd);
       b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 0.99;
      fd2->friction = 0.5;
      fd2->restitution = 0.f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(2.5,0.2, b2Vec2(0.f,-1.9f), 0);
      fd2->shape = &bs1;
      box2->CreateFixture(fd2);
  
      // The pulley joint
      b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorOnBody1(25, 15); // Anchor point on body 1 in world axis
      b2Vec2 worldAnchorOnBody2(36, 15); // Anchor point on body 2 in world axis
      b2Vec2 worldAnchorGround1(25, 15); // Anchor point for ground 1 in world axis
      b2Vec2 worldAnchorGround2(36, 15); // Anchor point for ground 2 in world axis
      float32 ratio = 1.0f; // Define ratio
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);
    }
    
    
    
    /// The heavy sphere on the right platform: \n
    /// b2CircleShape circle: radius 1.0 \n
    /// b2FixtureDef ballfd: shape from above circle, density 200.0, friction 0.0, restitution 0.0 \n
    /// b2BodyDef ballbd: dynamic body with position (21.0, 16.0)
    /// b2Body* sbody: body of the sphere, with value m_world->CreateBody(&ballbd) and fixture ballfd \n
    {
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 1.0;
      
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 200.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(21.0f, 16.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }



	//Rotating bars at the bottom
    /// 5 rotating bars are created inside a for loop. \n
    /// For the ith bar: 
    /// b2PolygonShape shape: box of width 0.4, height 4.4 \n
    /// b2BodyDef bd: dynamic body with position (-2.5 - 3.5 * j, 2.5) \n
    /// b2FixtureDef *fd: fixture with density 1.0 and shape as defined above \n
    /// b2Body* body: constructed by m_world->CreateBody(&bd) using fixture fd \n
	
	/// b2PolygonShape shape2: box of width 5.0, height 1.0 \n
    /// b2BodyDef bd2: static body with position (-2.5 - 3.5 * j, 2.5) \n
    /// b2Body* body2: constructed by m_world->CreateBody(&bd2) using fixture fd \n
    /// b2RevoluteJointDef jointDef: bodyA set to body and bodyB set to body. LocalAnchorA and LocalAnchorB are set to (0, 0). collideConneceted is set to false. \n
    
	for(int j=0;j<5;j++)
    {
		b2PolygonShape shape;
		shape.SetAsBox(0.2f,2.2f);
		b2BodyDef bd;
		bd.position.Set(-2.5f - 3.5f * j,2.5f);
		bd.type=b2_dynamicBody;
		b2Body* body = m_world->CreateBody(&bd);
		b2FixtureDef *fd= new b2FixtureDef;
		fd->density = 1.f;
		fd->shape = new b2PolygonShape;
		fd->shape = &shape;
		body->CreateFixture(fd);
  
        b2PolygonShape shape2;
        shape2.SetAsBox(2.5f, 0.5f);
        b2BodyDef bd2;
        bd2.position.Set(-2.5f -3.5* j , 2.5f);
        b2Body* body2 = m_world->CreateBody(&bd2); 
        b2RevoluteJointDef jointDef;
  
        jointDef.bodyA = body;
        jointDef.bodyB = body2;
        jointDef.localAnchorA.Set(0,0);
        jointDef.localAnchorB.Set(0,0);
		jointDef.collideConnected = false;
		m_world->CreateJoint(&jointDef); 
    }
    /// Vertical wall on the bottom left: \n
    /// b2PolygonShape shape: a box of width 0.5, height 10.0 \n
    /// b2FixtureDef fd: shape as described above, friction 0.0, restitution 5.0 \n
    /// b2BodyDef bd: position set to (-30.0, 5.0)
    /// b2Body* bar: created from m_world->CreateBody(&bd), with fixture fd \n
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.25f, 5.0f);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->restitution = 5.0f;
      fd->friction = 0.0;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      b2BodyDef bd;
      bd.position.Set(-30.0f, 5.0f);
      b2Body* bar = m_world->CreateBody(&bd);
      bar->CreateFixture(fd);
    } 
	
    /// The other revolving horizontal platform: \n
    /// b2PolygonShape shape: a box shape of width 4.4, height 0.4 \n
	/// b2BodyDef bd: position is set to (21.0f, 12.0f) and type is dynamic \n
	/// b2Body* body: initialized by m_world->CreateBody(&bd) \n
	/// b2FixtureDef *fd: fixture with density 1.0 and shape as defined above. body->CreateFixture(fd) is called \n
	/// b2PolygonShape shape2: a box shape of width 0.4, height 4.0 \n
	/// b2BodyDef bd2: position is set to (21.0f, 14.0f) \n
	/// b2Body* body2: initialized by m_world->CreateBody(&bd2) \n
	/// b2RevoluteJointDef jointDef: bodyA set to body and bodyB set to body. LocalAnchorA and LocalAnchorB are set to (0, 0). collideConneceted is set to false. \n
    {
      b2PolygonShape shape;
      shape.SetAsBox(2.2f, 0.2f);
      
      b2BodyDef bd;
      bd.position.Set(21.0f, 12.0f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);
  
      b2PolygonShape shape2;
      shape2.SetAsBox(0.2f, 2.0f);
      b2BodyDef bd2;
      bd2.position.Set(21.0f, 14.0f);
      b2Body* body2 = m_world->CreateBody(&bd2);
  
      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
    }
}

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
