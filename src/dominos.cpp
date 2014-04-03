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
	 		groundFixture.friction = 100.0;
	 		b1->CreateFixture(&groundFixture);
	 	}

		//=========================================================================================================================
	 	{

	 	}	

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
			boxFixtureDef.friction = 0;
			boxFixtureDef.filter.groupIndex = -1;
			smallRod2->CreateFixture(&boxFixtureDef);


			b2CircleShape circle;
			circle.m_radius = outerRadius;
			circle.m_p.Set(-rodLength, 0);
			b2FixtureDef circle_fix;
			circle_fix.shape = &circle;
			circle_fix.density = 5.0;
			circle_fix.friction = 0;
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
			boxFixtureDef.friction = 0;
			boxFixtureDef.filter.groupIndex = -1;
			smallRod3->CreateFixture(&boxFixtureDef);


			b2CircleShape circle;
			circle.m_radius = outerRadius;
			circle.m_p.Set(-rodLength, 0);
			b2FixtureDef circle_fix;
			circle_fix.shape = &circle;
			circle_fix.density = 5.0;
			circle_fix.friction = 0;
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
			boxFixtureDef.friction = 0;
			boxFixtureDef.filter.groupIndex = -1;
			smallRod4->CreateFixture(&boxFixtureDef);


			b2CircleShape circle;
			circle.m_radius = outerRadius;
			circle.m_p.Set(-rodLength, 0);
			b2FixtureDef circle_fix;
			circle_fix.shape = &circle;
			circle_fix.density = 5.0;
			circle_fix.friction = 0;
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
			boxFixtureDef.friction = 0;
			boxFixtureDef.filter.groupIndex = -1;
			longRod1->CreateFixture(&boxFixtureDef);


			b2CircleShape circle;
			circle.m_radius = outerRadius;
			circle.m_p.Set(-rodLength, 0);
			b2FixtureDef circle_fix;
			circle_fix.shape = &circle;
			circle_fix.density = 5.0;
			circle_fix.friction = 0;
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
			myBodyDef.position.Set(grLength + srLength, wheelRadius);
			grayRod = m_world->CreateBody(&myBodyDef);

			b2PolygonShape boxShape;
			boxShape.SetAsBox(rodLength, srThickness);

			b2FixtureDef boxFixtureDef;
			boxFixtureDef.shape = &boxShape;
			boxFixtureDef.density = 200.0;
			boxFixtureDef.friction = 0;
			boxFixtureDef.filter.groupIndex = -1;
			grayRod->CreateFixture(&boxFixtureDef);


			b2CircleShape circle;
			circle.m_radius = outerRadius;
			circle.m_p.Set(-rodLength, 0);
			b2FixtureDef circle_fix;
			circle_fix.shape = &circle;
			circle_fix.density = 5.0;
			circle_fix.friction = 0;
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
			myBodyDef.position.Set(0, wheelRadius);
			pinkRod1 = m_world->CreateBody(&myBodyDef);

			b2PolygonShape boxShape;
			boxShape.SetAsBox(rodLength, srThickness);

			b2FixtureDef boxFixtureDef;
			boxFixtureDef.shape = &boxShape;
			boxFixtureDef.density = 200.0;
			boxFixtureDef.friction = 0;
			boxFixtureDef.filter.groupIndex = -1;
			pinkRod1->CreateFixture(&boxFixtureDef);


			b2CircleShape circle;
			circle.m_radius = outerRadius;
			circle.m_p.Set(-rodLength, 0);
			b2FixtureDef circle_fix;
			circle_fix.shape = &circle;
			circle_fix.density = 5.0;
			circle_fix.friction = 0;
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
			myBodyDef.position.Set(0, wheelRadius);
			pinkRod2 = m_world->CreateBody(&myBodyDef);

			b2PolygonShape boxShape;
			boxShape.SetAsBox(rodLength, srThickness);

			b2FixtureDef boxFixtureDef;
			boxFixtureDef.shape = &boxShape;
			boxFixtureDef.density = 200.0;
			boxFixtureDef.friction = 0;
			boxFixtureDef.filter.groupIndex = -1;
			pinkRod2->CreateFixture(&boxFixtureDef);


			b2CircleShape circle;
			circle.m_radius = outerRadius;
			circle.m_p.Set(-rodLength, 0);
			b2FixtureDef circle_fix;
			circle_fix.shape = &circle;
			circle_fix.density = 5.0;
			circle_fix.friction = 0;
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



		b2Body* trainBox;
		float trainHeight = 15, trainWidth = 30;
		{
			
			b2BodyDef myBodyDef;
			myBodyDef.type = b2_dynamicBody; //this will be a dynamic body
			myBodyDef.position.Set(0, trainHeight + wheelRadius*0.6);
			trainBox = m_world->CreateBody(&myBodyDef);
			trainBox->SetGravityScale(0);
			trainBox->SetLinearVelocity(b2Vec2(5,0));

			b2PolygonShape boxShape;
			boxShape.SetAsBox(trainWidth, trainHeight);

			b2FixtureDef boxFixtureDef;
			boxFixtureDef.shape = &boxShape;
			boxFixtureDef.density = 200.0;
			boxFixtureDef.friction = 0;
			boxFixtureDef.filter.groupIndex = -1;
			trainBox->CreateFixture(&boxFixtureDef);
		}

		{
			b2RevoluteJointDef jointDef5;
			jointDef5.bodyA = wheel2;
			jointDef5.bodyB = trainBox;
			jointDef5.collideConnected = false;
			jointDef5.localAnchorA = b2Vec2(0,0);
			jointDef5.localAnchorB = b2Vec2(wheel2x-wheel3x, -trainHeight+0.4*wheelRadius);
			b2RevoluteJoint* joint2 = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef5);
		}
		{
			b2RevoluteJointDef jointDef5;
			jointDef5.bodyA = wheel3;
			jointDef5.bodyB = trainBox;
			jointDef5.collideConnected = false;
			jointDef5.localAnchorA = b2Vec2(0,0);
			jointDef5.localAnchorB = b2Vec2(0, -trainHeight+0.4*wheelRadius);
			b2RevoluteJoint* joint2 = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef5);
		}

		{
			b2RevoluteJointDef jointDef5;
			jointDef5.bodyA = wheel4;
			jointDef5.bodyB = trainBox;
			jointDef5.collideConnected = false;
			jointDef5.localAnchorA = b2Vec2(0,0);
			jointDef5.localAnchorB = b2Vec2(wheel4x-wheel3x, -trainHeight+0.4*wheelRadius);
			b2RevoluteJoint* joint2 = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef5);
		}
		
	}

	sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
