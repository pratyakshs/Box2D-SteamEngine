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

#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f

 namespace cs296
 {

	/**  The is the constructor 
	 * This is the documentation block for the constructor.
	 */ 

	 dominos_t::dominos_t()
	 {


	 	b2Body* b1;  
	 	{

	 		b2EdgeShape shape; 
	 		shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
	 		b2BodyDef bd; 
	 		b1 = m_world->CreateBody(&bd); 
	 		b1->CreateFixture(&shape, 0.0f);
	 	}

		//=========================================================================================================================
	 	{

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
	 		circle.m_p.Set(0.0f, 0.0f);

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

	 	{
	 		float rodLength = 7, outerRadius = 1.2, innerRadius = 0.7;

	 		b2BodyDef myBodyDef;
			myBodyDef.type = b2_kinematicBody; //this will be a dynamic body
			myBodyDef.position.Set(0, 20); //set the starting position
			myBodyDef.angle = 0; //set the starting angle
			b2Body* rod = m_world->CreateBody(&myBodyDef);

			b2PolygonShape boxShape;
			boxShape.SetAsBox(rodLength,0.5);

			b2FixtureDef boxFixtureDef;
			boxFixtureDef.shape = &boxShape;
			boxFixtureDef.density = 200.0;
			boxFixtureDef.friction = 100.0;
			rod->CreateFixture(&boxFixtureDef);

			b2CircleShape circle;
			circle.m_radius = outerRadius;
			circle.m_p.Set(-rodLength, 0);
			b2FixtureDef circle_fix;
			circle_fix.shape = &circle;
			circle_fix.density = 200.0;
			circle_fix.friction = 100.0;

			rod->CreateFixture(&circle_fix);
			circle.m_radius = innerRadius;
			rod->CreateFixture(&circle_fix);

			circle.m_p.Set(rodLength, 0);
			rod->CreateFixture(&circle_fix);
			circle.m_radius = outerRadius;
			rod->CreateFixture(&circle_fix);

			// rod->SetLinearVelocity(b2Vec2(5, 5));
			//myBodyDef.position

			b2RevoluteJointDef jointDef;
			jointDef.bodyA = rod;
			b2xxxJoint* joint = (b2xxxJoint*)world->CreateJoint( &jointDef );

		}




	}

	sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
