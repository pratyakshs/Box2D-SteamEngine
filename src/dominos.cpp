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

#include<stdio.h>
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

#include<math.h>

#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#define PI 3.14159

 float xpos = 80;///the x-ordinate of engine center. Center refers to bottom center of engine
	float scale = 1.2;///the scale for engine
	float ypos = 0;///the y-ordinate of engine center. Center refers to bottom center of engine
	bool accl = false;///This  varible controls the acceleration of steam enngine
	bool stop = false;///This controls the breaks on engine
	bool checker =false;
	float wheelRadius, wheel2x, wheel3x, wheel4x;


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
	 	
		//the big train box
		b2Body* trainBox;
		float trainHeight = 15, trainWidth = 30;
		{
			
			b2BodyDef myBodyDef;
			myBodyDef.type = b2_dynamicBody; //this will be a dynamic body
			myBodyDef.position.Set(0, trainHeight + wheelRadius*0.6);
			trainBox = m_world->CreateBody(&myBodyDef);
			trainBox->SetGravityScale(0);
			// trainBox->SetLinearVelocity(b2Vec2(5,0));

			b2PolygonShape boxShape;
			boxShape.SetAsBox(trainWidth, trainHeight);

			b2FixtureDef boxFixtureDef;
			boxFixtureDef.shape = &boxShape;
			boxFixtureDef.density = 200.0;
			boxFixtureDef.friction = 0;
			boxFixtureDef.filter.groupIndex = -1;
			trainBox->CreateFixture(&boxFixtureDef);
		}


	 	//leftmost wheel
	 	wheelRadius = 6;
	 	wheel2x = -20;
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
			wheel2->SetTransform(trainBox->GetPosition() + b2Vec2(wheel2x-wheel3x, -trainHeight+0.4*wheelRadius), wheel2->GetAngle());
	 	}

	 	//small rod connected to leftmost wheel
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

			smallRod2->SetTransform(wheel2->GetPosition() + b2Vec2(rodLength*cos(smallRod2->GetAngle()), rodLength*sin(smallRod2->GetAngle())), 0);
		}

		//joint between smallRod2 and wheel2
		{
			b2RevoluteJointDef jointDef2;
			jointDef2.bodyA = wheel2;
			jointDef2.bodyB = smallRod2;
			jointDef2.collideConnected = false;
			jointDef2.localAnchorA = b2Vec2(0,0);
			jointDef2.localAnchorB = b2Vec2(-srLength, 0);
			b2RevoluteJoint* joint2 = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef2);
		}


		//middle wheel
		wheel3x = 0;
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
			wheel3->SetTransform(trainBox->GetPosition() + b2Vec2(wheel3x, -trainHeight+0.4*wheelRadius), wheel3->GetAngle());
		}

		//small rod connected to middle wheel center
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

			smallRod3->SetTransform(wheel3->GetPosition() + b2Vec2(rodLength*cos(smallRod3->GetAngle()), rodLength*sin(smallRod3->GetAngle())), 0);
		}

		//joint between smallrod3 and wheel3
		{
			b2RevoluteJointDef jointDef3;
			jointDef3.bodyA = wheel3;
			jointDef3.bodyB = smallRod3;
			jointDef3.collideConnected = false;
			jointDef3.localAnchorA = b2Vec2(0,0);
			jointDef3.localAnchorB = b2Vec2(-srLength, 0);
			b2RevoluteJoint* joint2 = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef3);
		}



		//rightmost wheel
		wheel4x = 20;
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
			wheel4->SetTransform(trainBox->GetPosition() + b2Vec2(wheel4x-wheel3x, -trainHeight+0.4*wheelRadius), wheel4->GetAngle());
		}
		
		//small rod connected to rightmost wheel center
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

			smallRod4->SetTransform(wheel4->GetPosition() + b2Vec2(rodLength*cos(smallRod4->GetAngle()), rodLength*sin(smallRod4->GetAngle())), 0);
		}

		//joint between wheel4 and smallrod4
		{
			b2RevoluteJointDef jointDef4;
			jointDef4.bodyA = wheel4;
			jointDef4.bodyB = smallRod4;
			jointDef4.collideConnected = false;
			jointDef4.localAnchorA = b2Vec2(0,0);
			jointDef4.localAnchorB = b2Vec2(-srLength, 0);
			b2RevoluteJoint* joint2 = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef4);
		}




		//long black rod, connected to all three small rods
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

			longRod1->SetTransform(smallRod3->GetPosition() + b2Vec2(srLength*cos(smallRod3->GetAngle()), srLength*sin(smallRod3->GetAngle())), longRod1->GetAngle());
		}

		//joint between longrod and leftmost smallrod
		{
			b2RevoluteJointDef jointDef5;
			jointDef5.bodyA = smallRod2;
			jointDef5.bodyB = longRod1;
			jointDef5.collideConnected = false;
			jointDef5.localAnchorA = b2Vec2(srLength,0);
			jointDef5.localAnchorB = b2Vec2(-lrLength, 0);
			b2RevoluteJoint* joint2 = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef5);
		}

		//joint between longrod and middle smallrod
		{
			b2RevoluteJointDef jointDef5;
			jointDef5.bodyA = smallRod3;
			jointDef5.bodyB = longRod1;
			jointDef5.collideConnected = false;
			jointDef5.localAnchorA = b2Vec2(srLength,0);
			jointDef5.localAnchorB = b2Vec2(0, 0);
			b2RevoluteJoint* joint2 = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef5);
		}

		//joint between longrod and rightmost smallrod
		{
			b2RevoluteJointDef jointDef5;
			jointDef5.bodyA = smallRod4;
			jointDef5.bodyB = longRod1;
			jointDef5.collideConnected = false;
			jointDef5.localAnchorA = b2Vec2(srLength,0);
			jointDef5.localAnchorB = b2Vec2(lrLength, 0);
			b2RevoluteJoint* joint2 = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef5);
		}

		//gray rod, smaller than the longrod
		float grLength = (wheel4x - wheel2x)*0.8/2;
		b2Body* grayRod;
		{
			float rodLength = grLength, outerRadius = srOuterRadius, innerRadius = srInnerRadius;

			b2BodyDef myBodyDef;
			myBodyDef.type = b2_dynamicBody; //this will be a dynamic body
			// myBodyDef.position.Set(grLength + srLength, wheelRadius);
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

			grayRod->SetTransform(longRod1->GetPosition() + b2Vec2(rodLength*cos(grayRod->GetAngle()), rodLength*sin(grayRod->GetAngle())), 0);
		}

		//joint between longrod1 center and grayrod end
		{
			b2RevoluteJointDef jointDef5;
			jointDef5.bodyA = grayRod;
			jointDef5.bodyB = longRod1;
			jointDef5.collideConnected = false;
			jointDef5.localAnchorA = b2Vec2(-grLength,0);
			jointDef5.localAnchorB = b2Vec2(0, 0);
			b2RevoluteJoint* joint2 = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef5);
		}



		//smaller pink rod
		float pr1Length = wheelRadius;
		b2Body* pinkRod1;
		{
			float rodLength = pr1Length, outerRadius = srOuterRadius, innerRadius = srInnerRadius;

			b2BodyDef myBodyDef;
			myBodyDef.type = b2_dynamicBody; //this will be a dynamic body
			myBodyDef.angle = 0;
			// myBodyDef.position.Set(0, wheelRadius);
			// myBodyDef.position = smallRod3->GetPosition()
			//  + b2Vec2(srLength*cos(smallRod3->GetAngle()), srLength*sin(smallRod3->GetAngle()))
			//  + b2Vec2(pr1Length*cos(pinkRod1->GetAngle()), pr1Length*sin(pinkRod1->GetAngle()))	;
			
			myBodyDef.angle = PI/6;
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

			// cos(pinkrod1->GetAngle());
			// printf("Hello");
			// printf("%f %f \n", pr1Length*cos(pinkRod1->GetAngle()), pr1Length*sin(pinkRod1->GetAngle()));

			pinkRod1->SetTransform(smallRod3->GetPosition()
			 + b2Vec2(srLength*cos(smallRod3->GetAngle()), srLength*sin(smallRod3->GetAngle()))
			 + b2Vec2(pr1Length*cos(pinkRod1->GetAngle()), pr1Length*sin(pinkRod1->GetAngle()))
				, pinkRod1->GetAngle());
		}

		//joint between pinkrod1 and grayrod
		{
			b2RevoluteJointDef jointDef5;
			jointDef5.bodyA = pinkRod1;
			jointDef5.bodyB = grayRod;
			jointDef5.collideConnected = false;
			jointDef5.localAnchorA = b2Vec2(-pr1Length,0);
			jointDef5.localAnchorB = b2Vec2(-grLength, 0);
			b2RevoluteJoint* joint2 = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef5);
		}



		//longer pink rod
		float pr2Length = grLength*0.9;
		b2Body* pinkRod2;
		{
			float rodLength = pr2Length, outerRadius = srOuterRadius, innerRadius = srInnerRadius;

			b2BodyDef myBodyDef;
			myBodyDef.type = b2_dynamicBody; //this will be a dynamic body
			// myBodyDef.position.Set(0, wheelRadius);
			myBodyDef.angle = PI/3;
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

			pinkRod2->SetTransform(pinkRod1->GetPosition()
			 + b2Vec2(pr1Length*cos(pinkRod1->GetAngle()), srLength*sin(pinkRod1->GetAngle()))
			 + b2Vec2(pr2Length*cos(pinkRod2->GetAngle()), pr2Length*sin(pinkRod2->GetAngle()))
				, pinkRod2->GetAngle());
		}

		//joint between pinkrod2 and pinkrod1
		{	
			b2RevoluteJointDef jointDef5;
			jointDef5.bodyA = pinkRod2;
			jointDef5.bodyB = pinkRod1;
			jointDef5.collideConnected = false;
			jointDef5.localAnchorA = b2Vec2(-pr2Length,0);
			jointDef5.localAnchorB = b2Vec2(pr1Length, 0);
			b2RevoluteJoint* joint2 = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef5);
		}


		//fixing wheel2 to the box
		{
			b2RevoluteJointDef jointDef5;
			jointDef5.bodyA = wheel2;
			jointDef5.bodyB = trainBox;
			jointDef5.collideConnected = false;
			jointDef5.localAnchorA = b2Vec2(0,0);
			jointDef5.localAnchorB = b2Vec2(wheel2x-wheel3x, -trainHeight+0.4*wheelRadius);
			b2RevoluteJoint* joint2 = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef5);
		}

		//fixing wheel3 to the box
		{
			b2RevoluteJointDef jointDef5;
			jointDef5.bodyA = wheel3;
			jointDef5.bodyB = trainBox;
			jointDef5.collideConnected = false;
			jointDef5.localAnchorA = b2Vec2(0,0);
			jointDef5.localAnchorB = b2Vec2(0, -trainHeight+0.4*wheelRadius);
			b2RevoluteJoint* joint2 = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef5);
		}

		//fixing wheel4 to the box
		{
			b2RevoluteJointDef jointDef5;
			jointDef5.bodyA = wheel4;
			jointDef5.bodyB = trainBox;
			jointDef5.collideConnected = false;
			jointDef5.localAnchorA = b2Vec2(0,0);
			jointDef5.localAnchorB = b2Vec2(wheel4x-wheel3x, -trainHeight+0.4*wheelRadius);
			b2RevoluteJoint* joint2 = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef5);
		}
		





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
			b2FixtureDef exhaust;
			b2EdgeShape shape; 
			shape.Set(b2Vec2(-2.25f*scale+xpos, 3.5f*scale+ypos), b2Vec2(2.25*scale+xpos, 3.5f*scale+ypos));
			exhaust.shape=&shape;
			exhaust.restitution = 0.f;
			exhaust.friction = 1;
			exhaust.density = 10.0f;
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
			engine_int->CreateFixture(&exhaust);

		}

///Piston rod of engine
		b2Body* piston;
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
			piston = m_world->CreateBody(bd);
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

sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
