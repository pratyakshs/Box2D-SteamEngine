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
	 		groundFixture.friction = 0.0;
	 		b1->CreateFixture(&groundFixture);
	 	}

		//=========================================================================================================================
	 	{

	 	}	




	 // 	{
	 // 		float rodLength = 7, outerRadius = 1.2, innerRadius = 0.7;

	 // 		b2BodyDef myBodyDef;
	 // 		myBodyDef.gravityScale = 0;
		// 	myBodyDef.type = b2_dynamicBody; //this will be a dynamic body
		// 	myBodyDef.position.Set(0, 6); //set the starting position
		// 	myBodyDef.angle = 0; //set the starting angle
		// 	b2Body* rod = m_world->CreateBody(&myBodyDef);

		// 	b2PolygonShape boxShape;
		// 	boxShape.SetAsBox(rodLength,0.5);

		// 	b2FixtureDef boxFixtureDef;
		// 	boxFixtureDef.shape = &boxShape;
		// 	boxFixtureDef.density = 200.0;
		// 	boxFixtureDef.friction = 100.0;
		// 	rod->CreateFixture(&boxFixtureDef);


		// 	b2CircleShape circle;
		// 	circle.m_radius = outerRadius;
		// 	circle.m_p.Set(-rodLength, 0);
		// 	b2FixtureDef circle_fix;
		// 	circle_fix.shape = &circle;
		// 	circle_fix.density = 200.0;
		// 	circle_fix.friction = 100.0;

		// 	rod->CreateFixture(&circle_fix);
		// 	circle.m_radius = innerRadius;
		// 	rod->CreateFixture(&circle_fix);

		// 	circle.m_p.Set(rodLength, 0);
		// 	rod->CreateFixture(&circle_fix);
		// 	circle.m_radius = outerRadius;
		// 	rod->CreateFixture(&circle_fix);

		// 	rod->SetLinearVelocity(b2Vec2(10, 0));
		// 	rod->SetAngularVelocity(2);
		// 	b2BodyDef myBodyDef2;
		// 	// myBodyDef2.position.Set(-20, 10);
		// 	// myBodyDef2.angle = 2*PI/3;
		// 	myBodyDef2.type = b2_dynamicBody;
		// 	b2Body* rod2 = m_world->CreateBody(&myBodyDef2);
		// 	rod2->CreateFixture(&boxFixtureDef);

		// 	rod2->CreateFixture(&circle_fix);
		// 	circle.m_radius = innerRadius;
		// 	rod2->CreateFixture(&circle_fix);
		// 	circle.m_p.Set(-rodLength, 0);
		// 	rod2->CreateFixture(&circle_fix);
		// 	circle.m_radius = outerRadius;
		// 	rod2->CreateFixture(&circle_fix);
		// 	rod2->SetTransform( rod->GetPosition()-b2Vec2(2*rodLength,0), rod->GetAngle());

		// 	b2RevoluteJointDef jointDef;
		// 	jointDef.bodyA = rod;
		// 	jointDef.bodyB = rod2;
		// 	jointDef.collideConnected = false;
		// 	jointDef.referenceAngle = PI/3;

		// 	jointDef.localAnchorA.Set(-rodLength,0);
		// 	jointDef.localAnchorB.Set(-rodLength,0);
		// 	jointDef.enableMotor = true;
		// 	jointDef.maxMotorTorque = 20;
		// 	jointDef.motorSpeed = 30 * DEGTORAD;
		// 	b2RevoluteJoint* joint = (b2RevoluteJoint*)m_world->CreateJoint( &jointDef );


		// 	b2Body* wheel1;
		// 	b2BodyDef wheelBodyDef;
		// 	wheelBodyDef.type = b2_dynamicBody;
		// 	b2FixtureDef wheelFixtureDef;
		// 	wheelFixtureDef.density = 5;

		// 	b2CircleShape wheelShape;
		// 	wheelShape.m_p.Set(-0, 0);
		// 	wheelShape.m_radius = 6;
		// 	wheelFixtureDef.shape = &wheelShape;
		// 	wheelFixtureDef.friction = 100;
		// 	wheel1 = m_world->CreateBody(&wheelBodyDef);
		// 	wheel1->CreateFixture(&wheelFixtureDef);
		// 	wheel1->SetAngularVelocity(-20);
		// 	wheel1->SetLinearVelocity(b2Vec2(100, 0));

		// 	jointDef.bodyA = rod2;
		// 	jointDef.bodyB = wheel1;
		// 	jointDef.localAnchorA.Set(rodLength, 0);
		// 	jointDef.localAnchorB = wheel1->GetPosition()+b2Vec2(0, 3);
		// 	b2RevoluteJoint* joint2 = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef);

		// }
	 	float wheelRadius = 6, wheel2x = -20;
	 	float srLength = 1.5, srOuterRadius = 0.8, srInnerRadius = 0.5, srThickness = 0.5;
		b2Body* wheel2;
		{
			b2BodyDef wheelBodyDef;
			wheelBodyDef.type = b2_dynamicBody;
			wheelBodyDef.position.Set(wheel2x, wheelRadius);
			b2FixtureDef wheelFixtureDef;
			wheelFixtureDef.density = 5;

			b2CircleShape wheelShape;
			wheelShape.m_radius = wheelRadius;
			wheelFixtureDef.shape = &wheelShape;
			wheelFixtureDef.friction = 100;
			wheelFixtureDef.filter.groupIndex = -1;

			wheel2 = m_world->CreateBody(&wheelBodyDef);
			wheel2->CreateFixture(&wheelFixtureDef);
			// wheel2->SetAngularVelocity(-20);	
			// wheel2->SetLinearVelocity(b2Vec2(100, 0));
		}
		
		b2Body* smallRod2;
		{
			float rodLength = srLength, outerRadius = srOuterRadius, innerRadius = srInnerRadius;

	 		b2BodyDef myBodyDef;
			myBodyDef.type = b2_dynamicBody; //this will be a dynamic body
			smallRod2 = m_world->CreateBody(&myBodyDef);

			b2PolygonShape boxShape;
			boxShape.SetAsBox(rodLength, srThickness);

			b2FixtureDef boxFixtureDef;
			boxFixtureDef.shape = &boxShape;
			boxFixtureDef.density = 200.0;
			boxFixtureDef.friction = 100.0;
			boxFixtureDef.filter.groupIndex = -1;
			smallRod2->CreateFixture(&boxFixtureDef);


			b2CircleShape circle;
			circle.m_radius = outerRadius;
			circle.m_p.Set(-rodLength, 0);
			b2FixtureDef circle_fix;
			circle_fix.shape = &circle;
			circle_fix.density = 5.0;
			circle_fix.friction = 100.0;
			circle_fix.filter.groupIndex = -1;

			smallRod2->CreateFixture(&circle_fix);
			circle.m_radius = innerRadius;
			smallRod2->CreateFixture(&circle_fix);

			circle.m_p.Set(rodLength, 0);
			smallRod2->CreateFixture(&circle_fix);
			circle.m_radius = outerRadius;
			smallRod2->CreateFixture(&circle_fix);

			smallRod2->SetTransform(wheel2->GetPosition() + b2Vec2(rodLength, 0), 0);
		}

		
		{
			b2RevoluteJointDef jointDef2;
			jointDef2.bodyA = wheel2;
			jointDef2.bodyB = smallRod2;
			jointDef2.collideConnected = false;
			jointDef2.localAnchorA = b2Vec2(0,0);
			jointDef2.localAnchorB = b2Vec2(-srLength, 0);
			b2RevoluteJoint* joint2 = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef2);
		}



		float wheel3x = 0;
		b2Body* wheel3;
		{
			b2BodyDef wheelBodyDef;
			wheelBodyDef.type = b2_dynamicBody;
			wheelBodyDef.position.Set(wheel3x, wheelRadius);
			b2FixtureDef wheelFixtureDef;
			wheelFixtureDef.density = 5;

			b2CircleShape wheelShape;
			wheelShape.m_radius = wheelRadius;
			wheelFixtureDef.shape = &wheelShape;
			wheelFixtureDef.friction = 100;
			wheelFixtureDef.filter.groupIndex = -1;

			wheel3 = m_world->CreateBody(&wheelBodyDef);
			wheel3->CreateFixture(&wheelFixtureDef);
			// wheel3->SetAngularVelocity(-20);	
			// wheel3->SetLinearVelocity(b2Vec2(100, 0));
		}
		
		b2Body* smallRod3;
		{
			float rodLength = srLength, outerRadius = srOuterRadius, innerRadius = srInnerRadius;

	 		b2BodyDef myBodyDef;
			myBodyDef.type = b2_dynamicBody; //this will be a dynamic body
			smallRod3 = m_world->CreateBody(&myBodyDef);

			b2PolygonShape boxShape;
			boxShape.SetAsBox(rodLength, srThickness);

			b2FixtureDef boxFixtureDef;
			boxFixtureDef.shape = &boxShape;
			boxFixtureDef.density = 200.0;
			boxFixtureDef.friction = 100.0;
			boxFixtureDef.filter.groupIndex = -1;
			smallRod3->CreateFixture(&boxFixtureDef);


			b2CircleShape circle;
			circle.m_radius = outerRadius;
			circle.m_p.Set(-rodLength, 0);
			b2FixtureDef circle_fix;
			circle_fix.shape = &circle;
			circle_fix.density = 5.0;
			circle_fix.friction = 100.0;
			circle_fix.filter.groupIndex = -1;

			smallRod3->CreateFixture(&circle_fix);
			circle.m_radius = innerRadius;
			smallRod3->CreateFixture(&circle_fix);

			circle.m_p.Set(rodLength, 0);
			smallRod3->CreateFixture(&circle_fix);
			circle.m_radius = outerRadius;
			smallRod3->CreateFixture(&circle_fix);

			smallRod3->SetTransform(wheel3->GetPosition() + b2Vec2(rodLength, 0), 0);
		}

		
		{
			b2RevoluteJointDef jointDef3;
			jointDef3.bodyA = wheel3;
			jointDef3.bodyB = smallRod3;
			jointDef3.collideConnected = false;
			jointDef3.localAnchorA = b2Vec2(0,0);
			jointDef3.localAnchorB = b2Vec2(-srLength, 0);
			b2RevoluteJoint* joint2 = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef3);
		}




		float wheel4x = 20;
		b2Body* wheel4;
		{
			b2BodyDef wheelBodyDef;
			wheelBodyDef.type = b2_dynamicBody;
			wheelBodyDef.position.Set(wheel4x, wheelRadius);
			b2FixtureDef wheelFixtureDef;
			wheelFixtureDef.density = 5;

			b2CircleShape wheelShape;
			wheelShape.m_radius = wheelRadius;
			wheelFixtureDef.shape = &wheelShape;
			wheelFixtureDef.friction = 100;
			wheelFixtureDef.filter.groupIndex = -1;

			wheel4 = m_world->CreateBody(&wheelBodyDef);
			wheel4->CreateFixture(&wheelFixtureDef);
			// wheel4->SetAngularVelocity(-20);	
			// wheel4->SetLinearVelocity(b2Vec2(20, 0));
		}
		
		b2Body* smallRod4;
		{
			float rodLength = srLength, outerRadius = srOuterRadius, innerRadius = srInnerRadius;

	 		b2BodyDef myBodyDef;
			myBodyDef.type = b2_dynamicBody; //this will be a dynamic body
			smallRod4 = m_world->CreateBody(&myBodyDef);

			b2PolygonShape boxShape;
			boxShape.SetAsBox(rodLength, srThickness);

			b2FixtureDef boxFixtureDef;
			boxFixtureDef.shape = &boxShape;
			boxFixtureDef.density = 200.0;
			boxFixtureDef.friction = 100.0;
			boxFixtureDef.filter.groupIndex = -1;
			smallRod4->CreateFixture(&boxFixtureDef);


			b2CircleShape circle;
			circle.m_radius = outerRadius;
			circle.m_p.Set(-rodLength, 0);
			b2FixtureDef circle_fix;
			circle_fix.shape = &circle;
			circle_fix.density = 5.0;
			circle_fix.friction = 100.0;
			circle_fix.filter.groupIndex = -1;

			smallRod4->CreateFixture(&circle_fix);
			circle.m_radius = innerRadius;
			smallRod4->CreateFixture(&circle_fix);

			circle.m_p.Set(rodLength, 0);
			smallRod4->CreateFixture(&circle_fix);
			circle.m_radius = outerRadius;
			smallRod4->CreateFixture(&circle_fix);

			smallRod4->SetTransform(wheel4->GetPosition() + b2Vec2(rodLength, 0), 0);
		}

		
		{
			b2RevoluteJointDef jointDef4;
			jointDef4.bodyA = wheel4;
			jointDef4.bodyB = smallRod4;
			jointDef4.collideConnected = false;
			jointDef4.localAnchorA = b2Vec2(0,0);
			jointDef4.localAnchorB = b2Vec2(-srLength, 0);
			b2RevoluteJoint* joint2 = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef4);
		}





		float lrLength = (wheel4x - wheel2x)/2;
		b2Body* longRod1;
		{
			float rodLength = lrLength, outerRadius = srOuterRadius, innerRadius = srInnerRadius;

	 		b2BodyDef myBodyDef;
			myBodyDef.type = b2_dynamicBody; //this will be a dynamic body
			myBodyDef.position.Set(0, 30);
			longRod1 = m_world->CreateBody(&myBodyDef);

			b2PolygonShape boxShape;
			boxShape.SetAsBox(rodLength, srThickness);

			b2FixtureDef boxFixtureDef;
			boxFixtureDef.shape = &boxShape;
			boxFixtureDef.density = 200.0;
			boxFixtureDef.friction = 100.0;
			boxFixtureDef.filter.groupIndex = -1;
			longRod1->CreateFixture(&boxFixtureDef);


			b2CircleShape circle;
			circle.m_radius = outerRadius;
			circle.m_p.Set(-rodLength, 0);
			b2FixtureDef circle_fix;
			circle_fix.shape = &circle;
			circle_fix.density = 5.0;
			circle_fix.friction = 100.0;
			circle_fix.filter.groupIndex = -1;

			longRod1->CreateFixture(&circle_fix);
			circle.m_radius = innerRadius;
			longRod1->CreateFixture(&circle_fix);

			circle.m_p.Set(rodLength, 0);
			longRod1->CreateFixture(&circle_fix);
			circle.m_radius = outerRadius;
			longRod1->CreateFixture(&circle_fix);

			longRod1->SetTransform(smallRod3->GetPosition() + b2Vec2(srLength, 0), 0);
		}

		{
			b2RevoluteJointDef jointDef5;
			jointDef5.bodyA = smallRod2;
			jointDef5.bodyB = longRod1;
			jointDef5.collideConnected = false;
			jointDef5.localAnchorA = b2Vec2(srLength,0);
			jointDef5.localAnchorB = b2Vec2(-lrLength, 0);
			b2RevoluteJoint* joint2 = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef5);
		}

		{
			b2RevoluteJointDef jointDef5;
			jointDef5.bodyA = smallRod3;
			jointDef5.bodyB = longRod1;
			jointDef5.collideConnected = false;
			jointDef5.localAnchorA = b2Vec2(srLength,0);
			jointDef5.localAnchorB = b2Vec2(0, 0);
			b2RevoluteJoint* joint2 = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef5);
		}

		{
			b2RevoluteJointDef jointDef5;
			jointDef5.bodyA = smallRod4;
			jointDef5.bodyB = longRod1;
			jointDef5.collideConnected = false;
			jointDef5.localAnchorA = b2Vec2(srLength,0);
			jointDef5.localAnchorB = b2Vec2(lrLength, 0);
			b2RevoluteJoint* joint2 = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef5);
		}


		float grLength = (wheel4x - wheel2x)*0.8/2;
		b2Body* grayRod;
		{
			float rodLength = grLength, outerRadius = srOuterRadius, innerRadius = srInnerRadius;

	 		b2BodyDef myBodyDef;
			myBodyDef.type = b2_dynamicBody; //this will be a dynamic body
			myBodyDef.position.Set(0, 30);
			grayRod = m_world->CreateBody(&myBodyDef);

			b2PolygonShape boxShape;
			boxShape.SetAsBox(rodLength, srThickness);

			b2FixtureDef boxFixtureDef;
			boxFixtureDef.shape = &boxShape;
			boxFixtureDef.density = 200.0;
			boxFixtureDef.friction = 100.0;
			boxFixtureDef.filter.groupIndex = -1;
			grayRod->CreateFixture(&boxFixtureDef);


			b2CircleShape circle;
			circle.m_radius = outerRadius;
			circle.m_p.Set(-rodLength, 0);
			b2FixtureDef circle_fix;
			circle_fix.shape = &circle;
			circle_fix.density = 5.0;
			circle_fix.friction = 100.0;
			circle_fix.filter.groupIndex = -1;

			grayRod->CreateFixture(&circle_fix);
			circle.m_radius = innerRadius;
			grayRod->CreateFixture(&circle_fix);

			circle.m_p.Set(rodLength, 0);
			grayRod->CreateFixture(&circle_fix);
			circle.m_radius = outerRadius;
			grayRod->CreateFixture(&circle_fix);

			// grayRod->SetTransform(smallRod3->GetPosition() + b2Vec2(srLength, 0), 0);
		}

		{
			b2RevoluteJointDef jointDef5;
			jointDef5.bodyA = grayRod;
			jointDef5.bodyB = longRod1;
			jointDef5.collideConnected = false;
			jointDef5.localAnchorA = b2Vec2(-grLength,0);
			jointDef5.localAnchorB = b2Vec2(0, 0);
			b2RevoluteJoint* joint2 = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef5);
		}




		float pr1Length = wheelRadius;
		b2Body* pinkRod1;
		{
			float rodLength = pr1Length, outerRadius = srOuterRadius, innerRadius = srInnerRadius;

	 		b2BodyDef myBodyDef;
			myBodyDef.type = b2_dynamicBody; //this will be a dynamic body
			myBodyDef.position.Set(0, 30);
			pinkRod1 = m_world->CreateBody(&myBodyDef);

			b2PolygonShape boxShape;
			boxShape.SetAsBox(rodLength, srThickness);

			b2FixtureDef boxFixtureDef;
			boxFixtureDef.shape = &boxShape;
			boxFixtureDef.density = 200.0;
			boxFixtureDef.friction = 100.0;
			boxFixtureDef.filter.groupIndex = -1;
			pinkRod1->CreateFixture(&boxFixtureDef);


			b2CircleShape circle;
			circle.m_radius = outerRadius;
			circle.m_p.Set(-rodLength, 0);
			b2FixtureDef circle_fix;
			circle_fix.shape = &circle;
			circle_fix.density = 5.0;
			circle_fix.friction = 100.0;
			circle_fix.filter.groupIndex = -1;

			pinkRod1->CreateFixture(&circle_fix);
			circle.m_radius = innerRadius;
			pinkRod1->CreateFixture(&circle_fix);

			circle.m_p.Set(rodLength, 0);
			pinkRod1->CreateFixture(&circle_fix);
			circle.m_radius = outerRadius;
			pinkRod1->CreateFixture(&circle_fix);

			// pinkRod1->SetTransform(smallRod3->GetPosition() + b2Vec2(srLength, 0), 0);
		}

		{
			b2RevoluteJointDef jointDef5;
			jointDef5.bodyA = pinkRod1;
			jointDef5.bodyB = grayRod;
			jointDef5.collideConnected = false;
			jointDef5.localAnchorA = b2Vec2(-pr1Length,0);
			jointDef5.localAnchorB = b2Vec2(-grLength, 0);
			b2RevoluteJoint* joint2 = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef5);
		}




		float pr2Length = grLength*0.9;
		b2Body* pinkRod2;
		{
			float rodLength = pr2Length, outerRadius = srOuterRadius, innerRadius = srInnerRadius;

	 		b2BodyDef myBodyDef;
			myBodyDef.type = b2_dynamicBody; //this will be a dynamic body
			myBodyDef.position.Set(0, 30);
			pinkRod2 = m_world->CreateBody(&myBodyDef);

			b2PolygonShape boxShape;
			boxShape.SetAsBox(rodLength, srThickness);

			b2FixtureDef boxFixtureDef;
			boxFixtureDef.shape = &boxShape;
			boxFixtureDef.density = 200.0;
			boxFixtureDef.friction = 100.0;
			boxFixtureDef.filter.groupIndex = -1;
			pinkRod2->CreateFixture(&boxFixtureDef);


			b2CircleShape circle;
			circle.m_radius = outerRadius;
			circle.m_p.Set(-rodLength, 0);
			b2FixtureDef circle_fix;
			circle_fix.shape = &circle;
			circle_fix.density = 5.0;
			circle_fix.friction = 100.0;
			circle_fix.filter.groupIndex = -1;

			pinkRod2->CreateFixture(&circle_fix);
			circle.m_radius = innerRadius;
			pinkRod2->CreateFixture(&circle_fix);

			circle.m_p.Set(rodLength, 0);
			pinkRod2->CreateFixture(&circle_fix);
			circle.m_radius = outerRadius;
			pinkRod2->CreateFixture(&circle_fix);

			// pinkRod2->SetTransform(smallRod3->GetPosition() + b2Vec2(srLength, 0), 0);
		}

		{
			b2RevoluteJointDef jointDef5;
			jointDef5.bodyA = pinkRod2;
			jointDef5.bodyB = pinkRod1;
			jointDef5.collideConnected = false;
			jointDef5.localAnchorA = b2Vec2(-pr2Length,0);
			jointDef5.localAnchorB = b2Vec2(pr1Length, 0);
			b2RevoluteJoint* joint2 = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef5);
		}

	}

	sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
