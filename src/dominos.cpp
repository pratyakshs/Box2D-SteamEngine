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

float wheelRadius = 6, wheel2x, wheel3x, wheel4x;
float xpos = 8;///the x-ordinate of engine center. Center refers to bottom center of engine
float scale = 3;//1.6;///the scale for engine
float ypos = (wheelRadius - scale*1.75);///the y-ordinate of engine center. Center refers to bottom center of engine
bool accl = false;///This  varible controls the acceleration of steam enngine
bool stop = false;///This controls the breaks on engine
bool checker = false;

b2Body* engineExt;

namespace cs296
{

	/**  The is the constructor 
	 * This is the documentation block for the constructor.
	 */ 

	 dominos_t::dominos_t()
	 {


	 	b2Body* b1;  
	 	{

	 		b2PolygonShape shape; 
	 		shape.SetAsBox(90, 0.01);
	 		b2BodyDef bd; 
	 		b1 = m_world->CreateBody(&bd); 
	 		bd.position.Set(0, -0.0000000001);
	 		b2FixtureDef groundFixture;
	 		groundFixture.shape = &shape;
	 		groundFixture.friction = 0.0;
	 		groundFixture.density = 10000.0;
	 		b1->CreateFixture(&groundFixture);
	 	}

		//=========================================================================================================================

		//the big train box
	 	b2Body* trainBox;
	 	float trainHeight = 15, trainWidth = 30, trainx = -50, trainy = trainHeight + wheelRadius*0.6;
	 	{

	 		b2BodyDef myBodyDef;
					myBodyDef.type = b2_dynamicBody; //this will be a dynamic body
					myBodyDef.position.Set(trainx, trainy);
					trainBox = m_world->CreateBody(&myBodyDef);
					trainBox->SetGravityScale(0);
					// trainBox->SetLinearVelocity(b2Vec2(5,0));

					b2PolygonShape boxShape;
					boxShape.SetAsBox(trainWidth, trainHeight);

					b2FixtureDef boxFixtureDef;
					boxFixtureDef.shape = &boxShape;
					boxFixtureDef.density = 10.0;
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
					wheelFixtureDef.friction = 0;
					wheelFixtureDef.filter.groupIndex = -1;

					wheel2 = m_world->CreateBody(&wheelBodyDef);
					wheel2->CreateFixture(&wheelFixtureDef);
					wheel2->SetAngularVelocity(-20);	
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
					b2WeldJointDef jointDef2;
					jointDef2.bodyA = wheel2;
					jointDef2.bodyB = smallRod2;
					jointDef2.collideConnected = false;
					jointDef2.localAnchorA = b2Vec2(0,0);
					jointDef2.localAnchorB = b2Vec2(-srLength, 0);
					b2WeldJoint* joint2 = (b2WeldJoint*)m_world->CreateJoint(&jointDef2);
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
					wheelFixtureDef.friction = 0;
					wheelFixtureDef.filter.groupIndex = -1;

					wheel3 = m_world->CreateBody(&wheelBodyDef);
					wheel3->CreateFixture(&wheelFixtureDef);
					wheel3->SetAngularVelocity(-20);	
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
					b2WeldJointDef jointDef3;
					jointDef3.bodyA = wheel3;
					jointDef3.bodyB = smallRod3;
					jointDef3.collideConnected = false;
					jointDef3.localAnchorA = b2Vec2(0,0);
					jointDef3.localAnchorB = b2Vec2(-srLength, 0);
					b2WeldJoint* joint2 = (b2WeldJoint*)m_world->CreateJoint(&jointDef3);
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
					wheelFixtureDef.friction = 0;
					wheelFixtureDef.filter.groupIndex = -1;

					wheel4 = m_world->CreateBody(&wheelBodyDef);
					wheel4->CreateFixture(&wheelFixtureDef);
					wheel4->SetAngularVelocity(-20);	
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
					b2WeldJointDef jointDef4;
					jointDef4.bodyA = wheel4;
					jointDef4.bodyB = smallRod4;
					jointDef4.collideConnected = false;
					jointDef4.localAnchorA = b2Vec2(0,0);
					jointDef4.localAnchorB = b2Vec2(-srLength, 0);
					b2WeldJoint* joint2 = (b2WeldJoint*)m_world->CreateJoint(&jointDef4);
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
					
					// myBodyDef.position.Set(0, wheelRadius);
					// myBodyDef.position = smallRod3->GetPosition()
					//  + b2Vec2(srLength*cos(smallRod3->GetAngle()), srLength*sin(smallRod3->GetAngle()))
					//  + b2Vec2(pr1Length*cos(pinkRod1->GetAngle()), pr1Length*sin(pinkRod1->GetAngle()))	;

					// printf("%f", 3.75*scale/pr1Length);
					myBodyDef.angle = asin(3.73*scale/(2*pr1Length));
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
				float pr2Length = (xpos - trainx - 2*srLength*cos(wheel3->GetAngle()) - 2*pr1Length*cos(pinkRod1->GetAngle()) - 7*scale)/2;
				b2Body* pinkRod2;
				{
					float rodLength = pr2Length, outerRadius = srOuterRadius, innerRadius = srInnerRadius;

					b2BodyDef myBodyDef;
					myBodyDef.type = b2_dynamicBody; //this will be a dynamic body
					// myBodyDef.position.Set(0, wheelRadius);
					myBodyDef.angle = 0;
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
						+ b2Vec2(pr1Length*cos(pinkRod1->GetAngle()), pr1Length*sin(pinkRod1->GetAngle()))
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


//======================================================================================================


				/*{
      //The steam engine scaled and relatively positioned
      //The (,) denotes the bottom center of engine hull
					b2Vec2 vertices[18];
					vertices[17].Set(-4*scale,+0.5*scale);
					vertices[16].Set(-4*scale,+1.5*scale);
					vertices[15].Set(-5*scale,+1.5*scale);
					vertices[14].Set(-5*scale,+0*scale);
					vertices[13].Set(+5*scale,+0*scale);
					vertices[12].Set(+5*scale,+5*scale);
					vertices[11].Set(+4*scale,+5*scale);
					vertices[10].Set(+4*scale,+7.5*scale);
					vertices[9].Set(+1*scale,+7.5*scale);
					vertices[8].Set(+1*scale,+8.5*scale);
					vertices[7].Set(+0.5*scale,+8.5*scale);
					vertices[6].Set(+0.5*scale,+7*scale);
					vertices[5].Set(+3.5*scale,+7*scale);
					vertices[4].Set(+3.5*scale,+5*scale);
					vertices[3].Set(+2*scale,+5*scale);
					vertices[2].Set(+4*scale,+3.5*scale);
					vertices[1].Set(+4*scale,+0.5*scale);
					vertices[0].Set(-4*scale,+0.5*scale);

					b2ChainShape chain;
					chain.CreateChain(vertices, 18);
	  //chain.CreateLoop(vertices, 8);
					b2FixtureDef fd1;
					fd1.shape = &chain;
					fd1.density = 1000.0f;
					fd1.friction = 0.0f;
					fd1.restitution = 1.0f;


					b2Vec2 v2[10];
					b2Vec2 v1[9];
					v2[9].Set(-5*scale,5*scale);
					v2[8].Set(-5*scale,2*scale);
					v2[7].Set(-4*scale,2*scale);
					v2[6].Set(-4*scale,3.5*scale);
					v2[5].Set(-2*scale,5*scale);
					v2[4].Set(-3.5*scale,5*scale);
					v2[3].Set(-3.5*scale,5.4*scale);
					v2[2].Set(-4*scale,5.4*scale);
					v2[1].Set(-4*scale,5*scale);
					v2[0].Set(-5*scale,5*scale);
					b2ChainShape chain2;
					chain2.CreateChain(v2, 10);
					v1[8].Set(-4*scale,7.5*scale);
					v1[7].Set(-4*scale,5.6*scale);
					v1[6].Set(-3.5*scale,5.6*scale);
					v1[5].Set(-3.5*scale,7*scale);
					v1[4].Set(-0.5*scale,7*scale);
					v1[3].Set(-0.5*scale,8.5*scale);
					v1[2].Set(-1*scale,8.5*scale);
					v1[1].Set(-1*scale,7.5*scale);
					v1[0].Set(-4*scale,7.5*scale);
					b2ChainShape chain3;
					chain3.CreateChain(v1, 9);

					b2Vec2 v4[11];
					v4[10].Set(-3.25*scale,+3*scale);
					v4[9].Set(+3.25*scale,+3*scale);
					v4[8].Set(+1.25*scale,+5*scale);
					v4[7].Set(+1*scale,+5*scale);
					v4[6].Set(+1*scale,+4.5*scale);
					v4[5].Set(+2.25*scale,+3.25*scale);
					v4[4].Set(-2.25*scale,+3.25*scale);
					v4[3].Set(-1*scale,+4.5*scale);
					v4[2].Set(-1*scale,+5*scale);
					v4[1].Set(-1.25*scale,+5*scale);
					v4[0].Set(-3.25*scale,+3*scale);
					b2ChainShape chain4;
					chain4.CreateChain(v4, 11);
	  //chain.CreateLoop(vertices, 8);
			//b2FixtureDef exhaust;
			//b2EdgeShape shape; 
			//shape.Set(b2Vec2(-2.25f*scale, 3.5f*scale), b2Vec2(2.25*scale, 3.5f*scale));
			//exhaust.shape=&shape;
			//exhaust.restitution = 0.f;
			//exhaust.friction = 1;
			//exhaust.density = 10.0f;
					b2FixtureDef enginefd;
					enginefd.shape = &chain4;
					enginefd.density = 10.0f;
					enginefd.friction = 0.0f;
					enginefd.restitution = 1.0f;

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
					engine.position.Set(xpos, ypos);
					engine.type = b2_staticBody;
					engine.gravityScale = 10;

					b2PolygonShape shape4;
					shape4.SetAsBox(2.0f*scale, 0.08f*scale,b2Vec2(0*scale,8.5*scale),0);
					b2FixtureDef fd4;
					fd4.restitution=1.f;
					fd4.density=000.f;
					fd4.shape=&shape4;


					b2PolygonShape shape5;
					shape5.SetAsBox(0.25f*scale, 0.1f*scale,b2Vec2(-3.75*scale,5.5*scale),0);

					b2FixtureDef *fd5 = new b2FixtureDef;
					fd5->shape = new b2PolygonShape;
					fd5->shape = &shape5;
					fd5->filter.groupIndex = -2;
					fd5->filter.maskBits  = 0x0001;
					fd5->filter.categoryBits  = 0x0002;


					b2PolygonShape shape6;
					shape6.SetAsBox(0.5f*scale, 0.19f*scale,b2Vec2(-4.5*scale,1.76*scale),0);

					b2FixtureDef *fd6 = new b2FixtureDef;
					fd6->shape = new b2PolygonShape;

					fd6->shape = &shape6;
					fd6->filter.groupIndex = -2;
					fd6->filter.maskBits  = 0x0001;
					fd6->filter.categoryBits  = 0x0002;


					b2PolygonShape shape7;
					shape7.SetAsBox(4*scale, 0.2f*scale,b2Vec2(0*scale,0.25*scale),0);

					b2FixtureDef *fd7 = new b2FixtureDef;
					fd7->shape = new b2PolygonShape;

					fd7->shape = &shape7;
					fd7->density = 10.f;

					b2EdgeShape ext1; 
					ext1.Set(b2Vec2(0*scale,3.85*scale),b2Vec2(6*scale,3.85*scale));
					b2EdgeShape ext2; 
					ext2.Set(b2Vec2(0*scale,3.45*scale),b2Vec2(6*scale,3.45*scale));

					b2FixtureDef *up = new b2FixtureDef;
					up->shape = new b2EdgeShape;
					up->shape = &ext1;
					up->filter.groupIndex = -1;
					up->filter.maskBits  = 0x0005;
					up->filter.categoryBits  = 0x0004;

					b2FixtureDef *down = new b2FixtureDef;
					down->shape = new b2EdgeShape;
					down->shape = &ext2;
					down->filter.groupIndex = -1;
					down->filter.maskBits  = 0x0005;
					down->filter.categoryBits  = 0x0004;

					engineExt = m_world->CreateBody(&engine);
					engineExt->CreateFixture(&fd1);
					engineExt->CreateFixture(&fd2);
					engineExt->CreateFixture(&fd3);
					engineExt->CreateFixture(&enginefd);
			//engineExt->CreateFixture(&exhaust);
					engineExt->CreateFixture(&fd4);
					engineExt->CreateFixture(fd5);
					engineExt->CreateFixture(fd6);
					engineExt->CreateFixture(fd7);
					engineExt->CreateFixture(up);
					engineExt->CreateFixture(down);
			//engineExt->CreateFixture(ext_entry);
				}*/


				/*b2Body* ext;
				{
      //The steam engine scaled and relatively positioned
      //The (xpos,ypos) denotes the bottom center       
					b2BodyDef exhaustbd;
					exhaustbd.position.Set(xpos+0*scale, ypos+3.7*scale);
					b2CircleShape ext_entry;
					ext_entry.m_radius = 0.2*scale;
					b2FixtureDef *fd7 = new b2FixtureDef;
					fd7->shape = new b2CircleShape;
					fd7->shape=&ext_entry;
					fd7->restitution = 0.f;
					fd7->friction = 1;
					fd7->density = 60.0f;
					exhaustbd.type = b2_dynamicBody;
					ext = m_world->CreateBody(&exhaustbd);
					ext->CreateFixture(fd7);
					ext->SetUserData( this );
				}


		//joint between enigine exterior and exhaust
				{
					b2WeldJointDef weldJointDef;
					weldJointDef.bodyA = engineExt;
					weldJointDef.bodyB = ext;
					weldJointDef.collideConnected = false;
					weldJointDef.localAnchorA.Set(0,3.7*scale);
					weldJointDef.localAnchorB.Set(0,0);
					b2Joint *m_joint = (b2WeldJoint*)m_world->CreateJoint( &weldJointDef );
				}*/

				float pistonRodLength = (xpos - trainx- 0.15*scale - 2*srLength*cos(smallRod3->GetAngle()) - grLength*2*cos(grayRod->GetAngle()))/2;
				b2Body* piston;
				{
					b2BodyDef *bd = new b2BodyDef;
					bd->type = b2_dynamicBody;
					bd->position.Set(xpos,ypos);
					bd->fixedRotation = true;
					b2FixtureDef *fd1 = new b2FixtureDef;
					fd1->density = 10;
					fd1->friction = 0.0;
					fd1->restitution = 1.f;
					fd1->shape = new b2PolygonShape;
					b2PolygonShape bs1;
					bs1.SetAsBox(0.15*scale,1.24*scale, b2Vec2(3*scale,1.75*scale), 0);
					fd1->shape = &bs1;
					b2FixtureDef *fd2 = new b2FixtureDef;
					fd2->density = 10;
					fd2->friction = 0.0;
					fd2->restitution = 1.f; 
					fd2->shape = new b2PolygonShape;
					b2PolygonShape bs2;
					bs2.SetAsBox(pistonRodLength+1.5*scale, 0.25*scale, b2Vec2(-0.15*scale-pistonRodLength+1.5*scale,1.75*scale), 0);
					fd2->shape = &bs2;
					fd2->filter.groupIndex = -1;
					piston = m_world->CreateBody(bd);
					piston->CreateFixture(fd1);
					piston->CreateFixture(fd2);
					// piston->SetLinearVelocity(b2Vec2(2,0));
				}

				//prismatic joint between piston and engine exterior
				{
					b2PrismaticJointDef prismaticJointDef;
					prismaticJointDef.bodyA = b1;
					prismaticJointDef.bodyB = piston;
					prismaticJointDef.collideConnected = true;
					prismaticJointDef.localAxisA.Set(1,0);

					prismaticJointDef.localAnchorA.Set(0,ypos);
					prismaticJointDef.localAnchorB.Set(0,0);

					b2Joint *m_joint = (b2PrismaticJoint*)m_world->CreateJoint( &prismaticJointDef );
				}    


///The valve rod of engine  
				b2Body* valveRod;  
				{
					b2BodyDef *bd = new b2BodyDef;
					bd->type = b2_dynamicBody;
					bd->position.Set(xpos,ypos+5*scale);
					bd->fixedRotation = true;

      //The open box
					b2FixtureDef *fd1 = new b2FixtureDef;
					fd1->density = 100.0;
					fd1->friction = 0;
					fd1->restitution = 1.f;
					fd1->shape = new b2PolygonShape;
					b2PolygonShape bs1;
					bs1.SetAsBox(1.9*scale,0.1*scale, b2Vec2(0.5f*scale,0.5f*scale)-b2Vec2(2*scale,0), 0);
					fd1->shape = &bs1;
					b2FixtureDef *fd2 = new b2FixtureDef;
					fd2->density = 100.0;
					fd2->friction = 0;
					fd2->restitution = 1.f; 
					fd2->shape = new b2PolygonShape;
					b2PolygonShape bs2;
					bs2.SetAsBox(0.2*scale,0.19*scale, b2Vec2(2.2*scale,0.19f*scale)-b2Vec2(2*scale,0), 0);
					fd2->shape = &bs2;
					b2FixtureDef *fd3 = new b2FixtureDef;
					fd3->density = 100.0;
					fd3->friction = 0;
					fd3->restitution = 1.f;
					fd3->shape = new b2PolygonShape;
					b2PolygonShape bs3;
					bs3.SetAsBox(0.2*scale,0.19*scale, b2Vec2(-1.2*scale,0.19f*scale)-b2Vec2(2*scale,0), 0);
					fd3->shape = &bs3;
					b2FixtureDef *fd4 = new b2FixtureDef;
					fd4->density = 100.0;
					fd4->friction = 0;
					fd4->restitution = 1.f;
					fd4->filter.groupIndex = -1;
					fd4->shape = new b2PolygonShape;
					b2PolygonShape bs4;
					bs4.SetAsBox(3*scale,0.1*scale, b2Vec2(-4.f*scale,0.5f*scale)-b2Vec2(2*scale,0), 0);
					fd4->shape = &bs4;

					valveRod = m_world->CreateBody(bd);
					valveRod->CreateFixture(fd1);
					valveRod->CreateFixture(fd2);
					valveRod->CreateFixture(fd3);
					valveRod->CreateFixture(fd4);
				}



				//joint between pinkrod2 and valverod
				{	
					b2RevoluteJointDef jointDef5;
					jointDef5.bodyA = pinkRod2;
					jointDef5.bodyB = valveRod;
					jointDef5.collideConnected = false;
					jointDef5.localAnchorA = b2Vec2(pr2Length*cos(pinkRod2->GetAngle()),pr2Length*sin(pinkRod2->GetAngle()));
					jointDef5.localAnchorB = b2Vec2(-7*scale, 0.48*scale);
					b2RevoluteJoint* joint2 = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef5);
				}

				//prismatic joint between valve rod and engine exterior
				{


					b2PrismaticJointDef prismaticJointDef;
					prismaticJointDef.bodyA = b1;
					prismaticJointDef.bodyB = valveRod;
					prismaticJointDef.collideConnected = true;
					prismaticJointDef.localAxisA.Set(1,0);

					prismaticJointDef.localAnchorA.Set(0,ypos+5*scale);
					prismaticJointDef.localAnchorB.Set(0,0);

					b2Joint *m_joint = (b2PrismaticJoint*)m_world->CreateJoint( &prismaticJointDef );
				}


	//prismatic joint between valve rod and engine exterior
				/*{


					b2PrismaticJointDef prismaticJointDef;
					prismaticJointDef.bodyA = engineExt;
					prismaticJointDef.bodyB = valveRod;
					prismaticJointDef.collideConnected = true;
					prismaticJointDef.localAxisA.Set(1,0);

					prismaticJointDef.localAnchorA.Set(-4*scale,5.4*scale);
					prismaticJointDef.localAnchorB.Set(-4*scale,0.4*scale);

					b2Joint *m_joint = (b2PrismaticJoint*)m_world->CreateJoint( &prismaticJointDef );
				}*/


				//prismatic joint between engine exterior and ground
				/*{

					b2PrismaticJointDef prismaticJointDef;
					prismaticJointDef.bodyA = engineExt;
					prismaticJointDef.bodyB = b1;
					prismaticJointDef.collideConnected = true;
					prismaticJointDef.localAxisA.Set(1,0);

					prismaticJointDef.localAnchorA.Set(0,0);
					prismaticJointDef.localAnchorB.Set(xpos,ypos);

					b2Joint *m_joint = (b2PrismaticJoint*)m_world->CreateJoint( &prismaticJointDef );
				}

				{
					int num_balls = 10;
					for (int i = 0; i < num_balls; i++) {
						float angle = (rand() % 361)/360.0 * 2 * 3.1416;
						b2Vec2 rayDir( sinf(angle), cosf(angle) );

						b2Vec2 center = engineExt->GetPosition() + b2Vec2(((rand()%100)-50)/200.0+0,((rand()%100)-50)/200.0+8*scale);
						int blastPower=10;
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
		      circleShape.m_radius = 0.05; // very small

		      b2FixtureDef fd;
		      fd.shape = &circleShape;
		      fd.density = 0.01 ; 
		      fd.friction = 0; 
		      fd.restitution = 0.f;
		      fd.filter.groupIndex=-1; 
		      fd.filter.categoryBits = 0x0001;
		      particle->SetUserData( this ); 
		      particle->CreateFixture( &fd );
		  }	
		}

		//box on which engine rests
		b2Body *lowerBox;
		{
			b2BodyDef bd;
			float height = ypos/2;
			bd.position.Set(xpos, height);
			bd.type = b2_dynamicBody;

			b2FixtureDef fd;
			b2PolygonShape shape;
			shape.SetAsBox(5, ypos/2);
			fd.density = 20;
			fd.friction = 10.0;
			fd.shape = &shape;
			lowerBox = m_world->CreateBody(&bd);
			lowerBox->CreateFixture(&fd);
		}

		{
			b2DistanceJointDef jointDef2;
			jointDef2.bodyA = engineExt;
			jointDef2.bodyB = lowerBox;
			jointDef2.collideConnected = false;
			jointDef2.localAnchorA = b2Vec2(0,0);
			jointDef2.localAnchorB = b2Vec2(0, ypos/2);
			b2WeldJoint* joint2 = (b2WeldJoint*)m_world->CreateJoint(&jointDef2);
		}
		//joint between smallRod2 and wheel2
		{
			b2DistanceJointDef jointDef2;
			jointDef2.bodyA = engineExt;
			jointDef2.bodyB = lowerBox;
			jointDef2.collideConnected = false;
			jointDef2.localAnchorA = b2Vec2(0,0);
			jointDef2.localAnchorB = b2Vec2(0, ypos/2);
			b2WeldJoint* joint2 = (b2WeldJoint*)m_world->CreateJoint(&jointDef2);
		}
		//joint between smallRod2 and wheel2
		{
			b2DistanceJointDef jointDef2;
			jointDef2.bodyA = engineExt;
			jointDef2.bodyB = lowerBox;
			jointDef2.collideConnected = false;
			jointDef2.localAnchorA = b2Vec2(0,0);
			jointDef2.localAnchorB = b2Vec2(0, ypos/2);
			b2WeldJoint* joint2 = (b2WeldJoint*)m_world->CreateJoint(&jointDef2);
		}*/
		
		
		
		
		/*{
			b2DistanceJointDef jointDef2;
			jointDef2.bodyA = engineExt;
			jointDef2.bodyB = trainBox;
			jointDef2.collideConnected = false;
			jointDef2.localAnchorA = b2Vec2(0,0);
			jointDef2.localAnchorB = b2Vec2(-trainx+xpos, ypos - trainy);
			b2DistanceJoint* joint2 = (b2DistanceJoint*)m_world->CreateJoint(&jointDef2);
		}*/
		
		//joint between piston and grayrod
				{
					b2RevoluteJointDef jointDef5;
					jointDef5.bodyA = grayRod;
					jointDef5.bodyB = piston;
					jointDef5.collideConnected = false;
					jointDef5.localAnchorA = b2Vec2(grLength*cos(grayRod->GetAngle()),grLength*sin(grayRod->GetAngle()));
					jointDef5.localAnchorB = b2Vec2(-0.15*scale -pistonRodLength*2, 1.75*scale);
					b2RevoluteJoint* joint2 = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef5);
				}

	}

	sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
