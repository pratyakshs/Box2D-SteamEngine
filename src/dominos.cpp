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

#define addCircles(obj) circle.m_radius = outerRadius;circle.m_p.Set(-rodLength, 0);\
 (obj)->CreateFixture(&circle_fix);\
 circle.m_radius = innerRadius;\
 (obj)->CreateFixture(&circle_fix);\
 circle.m_p.Set(rodLength, 0);\
 (obj)->CreateFixture(&circle_fix);\
 circle.m_radius = outerRadius;\
 (obj)->CreateFixture(&circle_fix)

 b2Vec2 origin(0,0);

 namespace cs296
 {

	/**  The is the constructor 
	 * This is the documentation block for the constructor.
	 */ 

	 dominos_t::dominos_t()
	 {


	 	b2Body* b1;
	 	float xpos,ypos,scale,xpos_e,ypos_e,scale_e;
	 	scale_e=2;
	 	float xpos_p=70 + 18*scale_e;
	 	float ypos_p=15-6.7*scale_e;
	 	xpos=70+18*scale_e;
	 	ypos=15-4*scale_e;
	 	scale=1.49;
	 	xpos_e=70;
	 	ypos_e=15;
	 	{

	 		b2PolygonShape shape; 
	 		shape.SetAsBox(90, 0.01);
	 		b2BodyDef bd; 
	 		b1 = m_world->CreateBody(&bd); 
	 		bd.position.Set(0, -0.0000000001);
	 		b2FixtureDef groundFixture;
	 		groundFixture.shape = &shape;
	 		groundFixture.friction = 10.0;
	 		groundFixture.density = 10000.0;
	 		b1->CreateFixture(&groundFixture);
	 	}

	 	b2Body *wheel1, *wheel2, *wheel3; 
	 	float wheelRadius = 6;
	 	{
	 		b2BodyDef bd;
	 		bd.type = b2_dynamicBody;
	 		bd.position = origin + b2Vec2(0, wheelRadius);
	 		wheel1 = m_world->CreateBody(&bd);
	 		b2CircleShape circle;
	 		circle.m_radius = wheelRadius;
	 		b2FixtureDef fd;
	 		fd.shape = &circle;
	 		fd.density = 10.0;
	 		fd.friction = 10.0;
	 		fd.filter.groupIndex = -1;
	 		wheel1->CreateFixture(&fd);

	 		bd.position += b2Vec2(2.1*wheelRadius, 0);
	 		wheel2 = m_world->CreateBody(&bd);
	 		wheel2->CreateFixture(&fd);

	 		bd.position += b2Vec2(2.1*wheelRadius, 0);
	 		wheel3 = m_world->CreateBody(&bd);
	 		wheel3->CreateFixture(&fd);
	 		wheel3->SetAngularVelocity(10);
	 		wheel2->SetAngularVelocity(10);
	 		wheel1->SetAngularVelocity(10);
	 	}

	 	b2Body *piston,*smallRod1, *smallRod2, *smallRod3, *longRod, *dRod, *msRod, *rod1, *rod2, *rod3, *rod4, *rod5,*valveRod,*engineBox;
	 	float lrLength = (wheel3->GetPosition() - wheel1->GetPosition()).x/2, msrLength, drLength, length1, length2, length3;
	 	{
	 		float rodLength = 1.25, outerRadius = 0.6, innerRadius = 0.3, srThickness = 0.3;

	 		b2BodyDef myBodyDef;
	 		myBodyDef.type = b2_dynamicBody;
	 		myBodyDef.angle = -PI/2;
	 		smallRod3 = m_world->CreateBody(&myBodyDef);
	 		smallRod2 = m_world->CreateBody(&myBodyDef);
	 		smallRod1 = m_world->CreateBody(&myBodyDef);

	 		b2PolygonShape boxShape;
	 		boxShape.SetAsBox(rodLength, srThickness);

	 		b2FixtureDef boxFixtureDef;
	 		boxFixtureDef.shape = &boxShape;
	 		boxFixtureDef.density = 10.0;
	 		boxFixtureDef.friction = 10;
	 		boxFixtureDef.filter.groupIndex = -1;
	 		smallRod1->CreateFixture(&boxFixtureDef);
	 		smallRod2->CreateFixture(&boxFixtureDef);
	 		smallRod3->CreateFixture(&boxFixtureDef);

	 		b2CircleShape circle;
	 		circle.m_radius = outerRadius;
	 		circle.m_p.Set(-rodLength, 0);
	 		b2FixtureDef circle_fix;
	 		circle_fix.shape = &circle;
	 		circle_fix.density = 5.0;
	 		circle_fix.friction = 10;
	 		circle_fix.filter.groupIndex = -1;

	 		addCircles(smallRod1);
	 		addCircles(smallRod2);
	 		addCircles(smallRod3);

	 		smallRod1->SetTransform(wheel1->GetPosition() - b2Vec2(0, rodLength), -PI/2);
	 		smallRod2->SetTransform(wheel2->GetPosition() - b2Vec2(0, rodLength), -PI/2);
	 		smallRod3->SetTransform(wheel3->GetPosition() - b2Vec2(0, rodLength), -PI/2);

	 		b2WeldJointDef jointDef;
	 		jointDef.Initialize(wheel1, smallRod1, wheel1->GetWorldCenter());
	 		b2WeldJoint* joint = (b2WeldJoint*)m_world->CreateJoint(&jointDef);
	 		jointDef.Initialize(wheel2, smallRod2, wheel2->GetWorldCenter());
	 		joint = (b2WeldJoint*)m_world->CreateJoint(&jointDef);
	 		jointDef.Initialize(wheel3, smallRod3, wheel3->GetWorldCenter());
	 		joint = (b2WeldJoint*)m_world->CreateJoint(&jointDef);

	 		{
	 			myBodyDef.angle = 0; rodLength = lrLength;
	 			longRod = m_world->CreateBody(&myBodyDef);
	 			boxShape.SetAsBox(lrLength,srThickness);
	 			longRod->CreateFixture(&boxFixtureDef);
	 			addCircles(longRod);
	 			circle.m_p.Set(0,0);
	 			longRod->CreateFixture(&circle_fix);
	 			circle.m_radius = innerRadius;
	 			longRod->CreateFixture(&circle_fix);
	 			longRod->SetTransform(wheel2->GetPosition() - b2Vec2(0, 2.5), 0);

	 			b2RevoluteJointDef jointDef;
	 			jointDef.Initialize(longRod, smallRod1, longRod->GetWorldCenter() - b2Vec2(lrLength, 0));
	 			b2RevoluteJoint* joint = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
	 			jointDef.Initialize(longRod, smallRod2, longRod->GetWorldCenter());
	 			joint = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
	 			jointDef.Initialize(longRod, smallRod3, longRod->GetWorldCenter() + b2Vec2(lrLength, 0));
	 			joint = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
	 		}


	 		{
	 			float x = 18.2, y = 2.5, drLength = sqrt(x*x+y*y)/2, theta = atan(y/x);
	 			rodLength = drLength;
	 			myBodyDef.angle = theta;
	 			dRod = m_world->CreateBody(&myBodyDef);
	 			boxShape.SetAsBox(drLength,srThickness);
	 			dRod->CreateFixture(&boxFixtureDef);
	 			addCircles(dRod);
	 			dRod->SetTransform(longRod->GetPosition()+b2Vec2(rodLength*cos(theta), rodLength*sin(theta)), theta);
	 			b2RevoluteJointDef jointDef;
	 			jointDef.Initialize(longRod, dRod, longRod->GetWorldCenter());
	 			b2RevoluteJoint* joint = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
	 		}

	 		{
	 			float x = 2, y = 2.7, theta = atan(y/x);
	 			rodLength = msrLength = sqrt(x*x+y*y)/2;
	 			myBodyDef.angle = theta;
	 			msRod = m_world->CreateBody(&myBodyDef);
	 			boxShape.SetAsBox(msrLength,srThickness);
	 			msRod->CreateFixture(&boxFixtureDef);
	 			addCircles(msRod);
	 			msRod->SetTransform(longRod->GetPosition()+b2Vec2(rodLength*cos(theta), rodLength*sin(theta)), theta);
	 			b2WeldJointDef jointDef;
	 			jointDef.Initialize(wheel2, msRod, longRod->GetWorldCenter());
	 			b2WeldJoint* joint = (b2WeldJoint*)m_world->CreateJoint(&jointDef);
	 		}

	 		{
	 			float x = 10.3, y = 0.4, theta = atan(y/x);
	 			rodLength = length1 = sqrt(x*x+y*y)/2;
	 			myBodyDef.angle = theta;
	 			rod1 = m_world->CreateBody(&myBodyDef);
	 			boxShape.SetAsBox(rodLength,srThickness);
	 			rod1->CreateFixture(&boxFixtureDef);
	 			addCircles(rod1);
	 			rod1->SetTransform(msRod->GetPosition()+b2Vec2(msrLength*cos(msRod->GetAngle()), msrLength*sin(msRod->GetAngle()))+b2Vec2(rodLength*cos(theta), rodLength*sin(theta)), theta);
	 			b2RevoluteJointDef jointDef;
	 			jointDef.Initialize(msRod, rod1, msRod->GetWorldCenter()+b2Vec2(msrLength*cos(msRod->GetAngle()), msrLength*sin(msRod->GetAngle())));
	 			b2RevoluteJoint* joint = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
	 		}
	 	}

	 	{
	 		float scale_v=1.7;
	 		b2BodyDef *bd = new b2BodyDef;
	 		bd->type = b2_dynamicBody;
	 		bd->position.Set(xpos,ypos);
	 		bd->fixedRotation = true;

	 		b2FixtureDef *fd1 = new b2FixtureDef;
	 		fd1->density = 100.0;
	 		fd1->friction = 0;
	 		fd1->restitution = 1.f;
	 		fd1->shape = new b2PolygonShape;
	 		b2PolygonShape bs1;
	 		bs1.SetAsBox(2*scale_v,0.9*scale_v, b2Vec2(0*scale_v,0.f*scale_v), 0);
	 		fd1->shape = &bs1;
	 		b2FixtureDef *fd2 = new b2FixtureDef;
	 		fd2->density = 100.0;
	 		fd2->friction = 0;
	 		fd2->restitution = 1.f;
	 		//fd2->filter.groupIndex = -1; 
	 		fd2->shape = new b2PolygonShape;
	 		b2PolygonShape bs2;
	 		bs2.SetAsBox(0.2*scale_v,1*scale_v, b2Vec2(-2*scale_v,0.f*scale_v), 0);
	 		fd2->shape = &bs2;
	 		b2FixtureDef *fd3 = new b2FixtureDef;
	 		fd3->density = 100.0;
	 		fd3->friction = 0;
	 		fd3->restitution = 1.f;
	 		fd3->shape = new b2PolygonShape;
	 		b2PolygonShape bs3;
	 		bs3.SetAsBox(0.2*scale_v,1*scale_v, b2Vec2(2*scale_v,0.f*scale_v), 0);
	 		fd3->shape = &bs3;
	 		b2FixtureDef *fd4 = new b2FixtureDef;
	 		fd4->density = 100.0;
	 		fd4->friction = 0;
	 		fd4->restitution = 1.f;
	 		fd4->filter.groupIndex = -1;
	 		fd4->shape = new b2PolygonShape;
	 		b2PolygonShape bs4;
	 		bs4.SetAsBox(2*scale_v,0.2*scale_v, b2Vec2(-4.f*scale_v,0.f*scale_v), 0);
	 		fd4->shape = &bs4;
	 		bd->gravityScale=0;
	 		valveRod = m_world->CreateBody(bd);
	 		valveRod->CreateFixture(fd1);
	 		valveRod->CreateFixture(fd2);
	 		valveRod->CreateFixture(fd3);
	 		valveRod->CreateFixture(fd4);
	 		valveRod->SetLinearVelocity(b2Vec2(2,0));
	 	}

	 	{
	 		b2PrismaticJointDef prismaticJointDef;
	 		prismaticJointDef.bodyA = b1;
	 		prismaticJointDef.bodyB = valveRod;
	 		prismaticJointDef.collideConnected = true;
	 		prismaticJointDef.localAxisA.Set(1,0);

	 		prismaticJointDef.localAnchorA.Set(0,ypos);
	 		prismaticJointDef.localAnchorB.Set(0,0);

	 		b2Joint *m_joint = (b2PrismaticJoint*)m_world->CreateJoint( &prismaticJointDef );
	 	}
	 	{
	 		b2BodyDef *bd = new b2BodyDef;
	 		bd->type = b2_dynamicBody;
	 		bd->position.Set(xpos_p,ypos_p);
	 		bd->fixedRotation = true;
	 		b2FixtureDef *fd1 = new b2FixtureDef;
	 		fd1->density = 10;
	 		fd1->friction = 0.0;
	 		fd1->restitution = 1.f;
	 		fd1->shape = new b2PolygonShape;
	 		b2PolygonShape bs1;
	 		bs1.SetAsBox(0.15*scale,2*scale, b2Vec2(0,0*scale), 0);
	 		fd1->shape = &bs1;
	 		b2FixtureDef *fd2 = new b2FixtureDef;
	 		fd2->density = 10;
	 		fd2->friction = 0.0;
	 		fd2->restitution = 1.f; 
	 		fd2->shape = new b2PolygonShape;
	 		b2PolygonShape bs2;
	 		bs2.SetAsBox(1.5*scale,0.25*scale, b2Vec2(-1.65*scale,0*scale), 0);
	 		fd2->shape = &bs2;
	 		fd2->filter.groupIndex = -1;
	 		bd->gravityScale=0.f;
	 		piston = m_world->CreateBody(bd);
	 		piston->CreateFixture(fd1);
	 		piston->CreateFixture(fd2);
			 piston->SetLinearVelocity(b2Vec2(2,0));
	 	}
	 					//prismatic joint between piston and engine exterior
	 	{
	 		b2PrismaticJointDef prismaticJointDef;
	 		prismaticJointDef.bodyA = b1;
	 		prismaticJointDef.bodyB = piston;
	 		prismaticJointDef.collideConnected = true;
	 		prismaticJointDef.localAxisA.Set(1,0);

	 		prismaticJointDef.localAnchorA.Set(0,ypos_p);
	 		prismaticJointDef.localAnchorB.Set(0,0);

	 		b2Joint *m_joint = (b2PrismaticJoint*)m_world->CreateJoint( &prismaticJointDef );
	 	}
	 	{
	 		b2BodyDef *bd = new b2BodyDef;
	 		bd->type = b2_dynamicBody;
	 		bd->position.Set(xpos_e,ypos_e);
	 		bd->fixedRotation = true;
	 		b2FixtureDef *fd1 = new b2FixtureDef;
	 		fd1->density = 10;
	 		fd1->friction = 0.0;
	 		fd1->restitution = 1.f;
	 		fd1->shape = new b2PolygonShape;
	 		b2PolygonShape bs1;
	 		bs1.SetAsBox(15*scale_e,5*scale_e, b2Vec2(0,0), 0);
	 		fd1->shape = &bs1;
	 		fd1->filter.groupIndex = -1;
	 		b2FixtureDef *fd2 = new b2FixtureDef;
	 		fd2->density = 10;
	 		fd2->friction = 0.0;
	 		fd2->restitution = 1.f; 
	 		fd2->shape = new b2PolygonShape;
	 		b2PolygonShape bs2;
	 		bs2.SetAsBox(3*scale_e,0.2*scale_e, b2Vec2(18.f*scale_e,-8.4f*scale_e), 0);
	 		fd2->shape = &bs2;
	 		b2FixtureDef *fd3 = new b2FixtureDef;
	 		fd3->density = 10;
	 		fd3->friction = 0.0;
	 		fd3->restitution = 1.f; 
	 		fd3->shape = new b2PolygonShape;
	 		b2PolygonShape bs3;
	 		bs3.SetAsBox(0.4*scale_e,1.5*scale_e, b2Vec2(20.6f*scale_e,-6.5f*scale_e), 0);
	 		fd3->shape = &bs3;
	 		b2FixtureDef *fd4 = new b2FixtureDef;
	 		fd4->density = 10;
	 		fd4->friction = 0.0;
	 		fd4->restitution = 1.f; 
	 		fd4->shape = new b2PolygonShape;
	 		b2PolygonShape bs4;
	 		bs4.SetAsBox(0.4*scale_e,1.5*scale_e, b2Vec2(15.6f*scale_e,-6.5f*scale_e), 0);
	 		fd4->shape = &bs4;
	 		fd4->filter.groupIndex = -1;
	 		b2FixtureDef *fd5 = new b2FixtureDef;
	 		fd5->density = 10;
	 		fd5->friction = 0.0;
	 		fd5->restitution = 1.f;
	 		fd5->filter.groupIndex = -1; 
	 		fd5->shape = new b2PolygonShape;
	 		b2PolygonShape bs5;
	 		bs5.SetAsBox(0.1*scale_e,1*scale_e, b2Vec2(15.1f*scale_e,-4.f*scale_e), 0);
	 		fd5->shape = &bs5;
	 		b2FixtureDef *fd6 = new b2FixtureDef;
	 		fd6->density = 10;
	 		fd6->friction = 0.0;
	 		fd6->restitution = 1.f; 
	 		fd6->shape = new b2PolygonShape;
	 		b2PolygonShape bs6;
	 		bs6.SetAsBox(0.1*scale_e,1*scale_e, b2Vec2(21.1f*scale_e,-4.f*scale_e), 0);
	 		fd6->shape = &bs6;
	 		b2FixtureDef *fd7 = new b2FixtureDef;
	 		fd7->density = 10;
	 		fd7->friction = 0.0;
	 		fd7->restitution = 1.f; 
	 		fd7->shape = new b2PolygonShape;
	 		b2PolygonShape bs7;
	 		bs7.SetAsBox(1.25*scale_e,0.2*scale_e, b2Vec2(16.25f*scale_e,-2.8f*scale_e), 0);
	 		fd7->shape = &bs7;
	 		b2FixtureDef *fd8 = new b2FixtureDef;
	 		fd8->density = 10;
	 		fd8->friction = 0.0;
	 		fd8->restitution = 1.f; 
	 		fd8->shape = new b2PolygonShape;
	 		b2PolygonShape bs8;
	 		bs8.SetAsBox(1.25*scale_e,0.2*scale_e, b2Vec2(19.75f*scale_e,-2.8f*scale_e), 0);
	 		fd8->shape = &bs8;
	 		b2FixtureDef *fd9 = new b2FixtureDef;
	 		fd9->density = 10;
	 		fd9->friction = 0.0;
	 		fd9->restitution = 1.f; 
	 		fd9->shape = new b2PolygonShape;
	 		b2PolygonShape bs9;
	 		b2Vec2 vertices[4];
	 		vertices[0].Set(16.5*scale_e,-5.2*scale_e);
	 		vertices[1].Set(19.7*scale_e,-5.2*scale_e);
	 		vertices[2].Set(19.2*scale_e,-5*scale_e);
	 		vertices[3].Set(17.0*scale_e,-5*scale_e);
	 		bs9.Set(vertices,4);
	 		fd9->shape = &bs9;
	 		// fd2->filter.groupIndex = -1;
	 		 bd->gravityScale=0.f;
	 		engineBox = m_world->CreateBody(bd);
	 		engineBox->CreateFixture(fd1);
	 		engineBox->CreateFixture(fd2);
	 		engineBox->CreateFixture(fd3);
	 		engineBox->CreateFixture(fd4);
	 		engineBox->CreateFixture(fd5);
	 		engineBox->CreateFixture(fd6);
	 		engineBox->CreateFixture(fd7);
	 		engineBox->CreateFixture(fd8);
	 		engineBox->CreateFixture(fd9);
					// piston->SetLinearVelocity(b2Vec2(2,0));
	 	}

	 }

	 sim_t *sim = new sim_t("Dominos", dominos_t::create);
	}
