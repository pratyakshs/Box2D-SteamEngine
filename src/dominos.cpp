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
#define PI 3.14159
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
	 		// b1->CreateFixture(&shape, 0.0f);
	 		b2FixtureDef groundFixture;
	 		groundFixture.shape = &shape;
	 		groundFixture.friction = 0;
	 		b1->CreateFixture(&groundFixture);
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
	 		myBodyDef.gravityScale = 0;
			myBodyDef.type = b2_dynamicBody; //this will be a dynamic body
			myBodyDef.position.Set(0, 6); //set the starting position
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

			rod->SetLinearVelocity(b2Vec2(10, 0));
			rod->SetAngularVelocity(2);
			b2BodyDef myBodyDef2;
			// myBodyDef2.position.Set(-20, 10);
			// myBodyDef2.angle = 2*PI/3;
			myBodyDef2.type = b2_dynamicBody;
			b2Body* rod2 = m_world->CreateBody(&myBodyDef2);
			rod2->CreateFixture(&boxFixtureDef);

			rod2->CreateFixture(&circle_fix);
			circle.m_radius = innerRadius;
			rod2->CreateFixture(&circle_fix);
			circle.m_p.Set(-rodLength, 0);
			rod2->CreateFixture(&circle_fix);
			circle.m_radius = outerRadius;
			rod2->CreateFixture(&circle_fix);
			rod2->SetTransform( rod->GetPosition()-b2Vec2(2*rodLength,0), rod->GetAngle());

			b2RevoluteJointDef jointDef;
			jointDef.bodyA = rod;
			jointDef.bodyB = rod2;
			jointDef.collideConnected = false;
			jointDef.referenceAngle = PI/3;

			jointDef.localAnchorA.Set(-rodLength,0);
			jointDef.localAnchorB.Set(-rodLength,0);
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = 20;
			jointDef.motorSpeed = 30 * DEGTORAD;
			b2RevoluteJoint* joint = (b2RevoluteJoint*)m_world->CreateJoint( &jointDef );


			b2Body* wheel1;
			b2BodyDef wheelBodyDef;
			wheelBodyDef.type = b2_dynamicBody;
			b2FixtureDef wheelFixtureDef;
			wheelFixtureDef.density = 5;

			b2CircleShape wheelShape;
			wheelShape.m_p.Set(-0, 0);
			wheelShape.m_radius = 6;
			wheelFixtureDef.shape = &wheelShape;
			wheelFixtureDef.friction = 100;
			wheel1 = m_world->CreateBody(&wheelBodyDef);
			wheel1->CreateFixture(&wheelFixtureDef);
			wheel1->SetAngularVelocity(-20);
			wheel1->SetLinearVelocity(b2Vec2(100, 0));

			jointDef.bodyA = rod2;
			jointDef.bodyB = wheel1;
			jointDef.localAnchorA.Set(rodLength, 0);
			jointDef.localAnchorB = wheel1->GetPosition()+b2Vec2(0, 3);
			b2RevoluteJoint* joint2 = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef);

		}

		

		{
			b2Body* m_bodyA, *m_bodyB;
			b2RevoluteJoint *m_joint;
			//body and fixture defs - the common parts
			b2BodyDef bodyDef;
			bodyDef.type = b2_dynamicBody;
			b2FixtureDef fixtureDef;
			fixtureDef.density = 1;

  //two shapes
			b2PolygonShape boxShape;
			boxShape.SetAsBox(2,2);
			b2CircleShape circleShape;
			circleShape.m_radius = 2;     

  //make box a little to the left
			bodyDef.position.Set(-3, 10);
			fixtureDef.shape = &boxShape;
			m_bodyA = m_world->CreateBody( &bodyDef );
			m_bodyA->CreateFixture( &fixtureDef );

  //and circle a little to the right
			bodyDef.position.Set( 3, 10);
			fixtureDef.shape = &circleShape;
			m_bodyB = m_world->CreateBody( &bodyDef );
			m_bodyB->CreateFixture( &fixtureDef );

			b2RevoluteJointDef revoluteJointDef;
			revoluteJointDef.bodyA = m_bodyA;
			revoluteJointDef.bodyB = m_bodyB;
			revoluteJointDef.collideConnected = false;
			revoluteJointDef.localAnchorA.Set(2,2);
  //the top right corner of the box
			revoluteJointDef.localAnchorB.Set(0,0);
  //center of the circle
			m_joint = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef );

			// b2DebugDraw* m_debugDraw;
			// m_debugDraw.DrawString(5, m_textLine, "Current joint angle: %f deg", m_joint->GetJointAngle() * RADTODEG);
			// m_textLine += 15;
			// m_debugDraw.DrawString(5, m_textLine, "Current joint speed: %f deg/s", m_joint->GetJointSpeed() * RADTODEG);
			// m_textLine += 15;

			//place the bodyB anchor at the edge of the circle 
			revoluteJointDef.localAnchorB.Set(-2,0);

  //place the bodyA anchor outside the fixture
			revoluteJointDef.localAnchorA.Set(4,4);

			revoluteJointDef.enableLimit = true;
			revoluteJointDef.lowerAngle = -45 * DEGTORAD;
			revoluteJointDef.upperAngle =  45 * DEGTORAD;

			revoluteJointDef.enableMotor = true;
			revoluteJointDef.maxMotorTorque = 5;
			revoluteJointDef.motorSpeed = 90 * DEGTORAD;
		}




	}

	sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
