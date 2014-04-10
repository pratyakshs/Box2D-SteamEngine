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
#include <stdint.h>

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include "GL/freeglut.h"
#endif

#include <cstring>
 using namespace std;

#include "dominos.hpp"

#include <math.h>

/// Constant PI
#define PI 3.14159

/// Utility funtion for absolute value.
#define abs(x) ((x) > 0)? (x) : -(x)

/// Macro which adds two circles at both the ends a b2Body* variable <b>'obj'</b>. Mainly used to add circles at the ends of various rods.
#define addCircles(obj) circle.m_radius = outerRadius;circle.m_p.Set(-rodLength, 0);\
 (obj)->CreateFixture(&circle_fix);\
 circle.m_radius = innerRadius;\
 (obj)->CreateFixture(&circle_fix);\
 circle.m_p.Set(rodLength, 0);\
 (obj)->CreateFixture(&circle_fix);\
 circle.m_radius = outerRadius;\
 (obj)->CreateFixture(&circle_fix)

/// Passing a b2Body* variable <b>'obj'</b> makes it a rod. lx is length along x-axis,ly along y-axis. The total length of rod is stored in the variable passed as <b>'len'</b>
#define createRod(obj, len, lx, ly) float x = (lx), y = (ly), theta = atan(y/x);\
 if(x<0 && y>0) theta = PI-abs(theta);\
 else if(x<0 && y<0) theta = PI+theta;\
 rodLength = (len) = sqrt(x*x+y*y)/2;\
 myBodyDef.angle = theta;\
 (obj) = m_world->CreateBody(&myBodyDef);\
 boxShape.SetAsBox(rodLength,srThickness);\
 (obj)->CreateFixture(&boxFixtureDef);\
 addCircles((obj))

///The conversion from polar to cartesian
#define polar(r, theta) b2Vec2((r)*cos(theta), (r)*sin(theta))

/// Defines the origin of coordinates
 b2Vec2 origin(0,0);

/// The accleration variable 
 bool accl=false;

/// Boolean for stop
 bool stop=false;

 /// The scale of engine
 float scale_e=2/1.1;

 /// The main engineBox,
b2Body* engineBox;	
 namespace cs296
 {

	/** dominos_t() This is the constructor for dominos <hr>
	 */ 

 	dominos_t::dominos_t()
 	{

	 	/// The ground body
 		b2Body* b1;
	 	/*! b2Body* b1 : This is <b>GROUND</b> body <hr> 
	 	*/
	 	/*! <b>GLOBAL VARIABLES: </b>
	 		xpos_e = x-position of engine \n
	 		ypos_e = y-position of engine \n
	 		scale_e = scale of engine \n
	 		xpos,ypos,scale correspond to valveRod \n
	 		xpos_p,ypos_p , scale_p correspond to pistonRod \n
	 		scale_p is the scale for all scales
	 	*/
	 	float xpos,ypos,scale,xpos_e,ypos_e,scale_p = 1.1;
	 	scale_e = 2/scale_p;
	 	//2.1*wheelRadius
	 	xpos_e = 12.6;
	 	ypos_e = 20 - 1.8;
	 	xpos=xpos_e+18*scale_e;
	 	ypos=ypos_e-4*scale_e;	
	 	scale = 1.49/scale_p;
	 	float xpos_p=xpos_e+ 18*scale_e;
	 	float ypos_p=ypos_e-6.7*scale_e;
	 	///b2Body* b1: Variable for ground. Its shape is a box of length 1800*2 and height 0.01*2 <hr>
	 	{

	 		b2PolygonShape shape; 
	 		shape.SetAsBox(1800, 0.01);
	 		b2BodyDef bd; 
	 		b1 = m_world->CreateBody(&bd); 
	 		bd.position.Set(0, -0.01);
	 		b2FixtureDef groundFixture;
	 		groundFixture.shape = &shape;
	 		groundFixture.friction = 10.0;
	 		groundFixture.density = 10000.0;
	 		b1->CreateFixture(&groundFixture);
	 	}
	 	/// b2body* wheel1,wheel2,wheel3:  
	 	///	The three wheels for the engine. \n
	 	/// The wheels have radius 6 units each <hr>
	 	b2Body *wheel1, *wheel2, *wheel3;
	 	float wheelRadius = 6;
	 	// The definitions for three wheels of the train
	 	{
	 		b2BodyDef bd;
	 		bd.type = b2_dynamicBody;
	 		bd.position = origin + b2Vec2(0, wheelRadius);
	 		wheel1 = m_world->CreateBody(&bd);
	 		b2CircleShape circle;
	 		circle.m_radius = wheelRadius;
	 		b2FixtureDef fd;
	 		fd.shape = &circle;
	 		fd.density = 1.0;
	 		fd.friction = 10.0;
	 		fd.filter.groupIndex = -1;
	 		wheel1->CreateFixture(&fd);

	 		bd.position += b2Vec2(2.1*wheelRadius, 0);
	 		wheel2 = m_world->CreateBody(&bd);
	 		wheel2->CreateFixture(&fd);

	 		bd.position += b2Vec2(2.1*wheelRadius, 0);
	 		wheel3 = m_world->CreateBody(&bd);
	 		wheel3->CreateFixture(&fd);
	 		// wheel3->SetAngularVelocity(-15);
	 		// wheel2->SetAngularVelocity(-15);
	 		// wheel1->SetAngularVelocity(-15);
	 	}

	 	/** All the bodies used in simulation are declared here
	 	 small rods the rods connecting the center of wheels to longrod which connects all the three wheels
	 	 rod11 is the rod which the 3 wheel system to piston
	 	 rod12 is rod which is welded to center wheel
	 	 ext1 and ext2 are two exhaust bodies 
	 	 piston is the pistonrod of the engine
	 	 valveRod is the valveRod of the engine
	 	 write here about rod1-rod10
	 	*/

	 	 /// *smallRod1, *smallRod2, *smallRod3, *longRod, *rod11, *rod12, *rod1, *rod2, *rod3, *rod4, *rod5,*rod7, *rod8, *rod9, *rod10, *rod11, *rod12: b2Body* variables for various rods <hr>
	 	 b2Body *smallRod1, *smallRod2, *smallRod3, *longRod, *rod11, *rod12, *rod1, *rod2, *rod3, *rod4, *rod5,*rod7, *rod8, *rod9, *rod10, *valveRod,	*ext1,*ext2,*piston;
	 	/// float lrLength: The lenghth of long rod
	 	 float lrLength = (wheel3->GetPosition() - wheel1->GetPosition()).x/2, length12, length11, length1, length2, length3, length4, length5,length7, length8, length9, length10;
	 	/// The definition for all the bodies connecting the wheels and joints
	 	 {
	 	 	float rodLength = 1.25, outerRadius = 0.6, innerRadius = 0.3, srThickness = 0.3;
	 		///The definitions for three smallrods
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
	 	 	boxFixtureDef.density = 1.0;
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
	 	 	circle_fix.density = 1.0;
	 	 	circle_fix.friction = 10;
	 	 	circle_fix.filter.groupIndex = -1;

	 	 	
	 	 	addCircles(smallRod1);
	 	 	addCircles(smallRod2);
	 	 	addCircles(smallRod3);

	 	 	
	 	 	smallRod1->SetTransform(wheel1->GetPosition() - b2Vec2(0, rodLength), -PI/2);
	 	 	smallRod2->SetTransform(wheel2->GetPosition() - b2Vec2(0, rodLength), -PI/2);
	 	 	smallRod3->SetTransform(wheel3->GetPosition() - b2Vec2(0, rodLength), -PI/2);

	 		///The joint definitions between smallRods and corresponding wheels
	 	 	b2WeldJointDef jointDef;
	 	 	jointDef.Initialize(wheel1, smallRod1, wheel1->GetWorldCenter());
	 	 	(b2WeldJoint*)m_world->CreateJoint(&jointDef);
	 	 	jointDef.Initialize(wheel2, smallRod2, wheel2->GetWorldCenter());
	 	 	(b2WeldJoint*)m_world->CreateJoint(&jointDef);
	 	 	jointDef.Initialize(wheel3, smallRod3, wheel3->GetWorldCenter());
	 	 	(b2WeldJoint*)m_world->CreateJoint(&jointDef);
	 		///The definitions of longrod and the joints of longrod is declared here
	 	 	{
	 			///The definition of longRod
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
	 			///The definition of joints for longrod and smallRods <hr>
	 	 		b2RevoluteJointDef jointDef;
	 	 		jointDef.Initialize(longRod, smallRod1, longRod->GetWorldCenter() - b2Vec2(lrLength, 0));
	 	 		(b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
	 	 		jointDef.Initialize(longRod, smallRod2, longRod->GetWorldCenter());
	 	 		(b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
	 	 		jointDef.Initialize(longRod, smallRod3, longRod->GetWorldCenter() + b2Vec2(lrLength, 0));
	 	 		(b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
	 	 	}

	 	 	/// Various rods are created with the required dimensions by calling the createRod() macro. \n
	 	 	/// After creating the rod, it's position is shifted to the appropriate position by using the SetTransform() function \n
	 	 	/// Then various joints are created for the rod (if any). <hr>
	 	 	{
	 	 		createRod(rod11, length11, 18.2, 2.5);
	 	 		b2Vec2 pos = longRod->GetPosition()+polar(rodLength, atan(2.5/18.2));
	 	 		rod11->SetTransform(pos, theta);
	 	 		b2RevoluteJointDef jointDef;
	 	 		jointDef.Initialize(longRod, rod11, longRod->GetWorldCenter()); 
	 	 		(b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
	 	 	}

	 	 	{
	 	 		createRod(rod12, length12, 2, 2.7);
	 	 		b2Vec2 pos = longRod->GetPosition()+polar(rodLength, atan(2.7/2));
	 	 		rod12->SetTransform(pos, theta);
	 	 		b2WeldJointDef jointDef;
	 	 		jointDef.Initialize(wheel2, rod12, longRod->GetWorldCenter());
	 	 		(b2WeldJoint*)m_world->CreateJoint(&jointDef);
	 	 	}

	 	 	{
	 	 		createRod(rod1, length1, 10.3, 0.4);
	 	 		b2Vec2 pos = rod12->GetPosition()+polar(length12, rod12->GetAngle())+polar(rodLength, atan(0.4/10.3));
	 	 		rod1->SetTransform(pos, theta);
	 	 		b2RevoluteJointDef jointDef;
	 	 		jointDef.Initialize(rod12, rod1, rod12->GetWorldCenter()+b2Vec2(length12*cos(rod12->GetAngle()), length12*sin(rod12->GetAngle())));
	 	 		(b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
	 	 	}

	 	 	{
	 	 		createRod(rod2, length2, 0.000000001, 3.5);
	 	 		b2Vec2 pos = rod1->GetPosition()+polar(length1, rod1->GetAngle())+polar(rodLength, theta);
	 	 		rod2->SetTransform(pos, theta);
	 	 		b2RevoluteJointDef jointDef;
	 	 		jointDef.Initialize(rod2, rod1, rod1->GetWorldCenter()+polar(length1, rod1->GetAngle()));
	 	 		(b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
	 	 	}

	 	 	{
	 	 		createRod(rod4, length4, -1, 2);
	 	 		b2Vec2 pos = rod2->GetPosition()+polar(length2, rod2->GetAngle())-polar(rodLength, theta);
	 	 		rod4->SetTransform(pos, theta);
	 	 		b2WeldJointDef jointDef;
	 	 		jointDef.Initialize(rod2, rod4, rod2->GetWorldCenter()+polar(length2, rod2->GetAngle()));
	 	 		(b2WeldJoint*)m_world->CreateJoint(&jointDef);
	 	 	}

	 	 	{
	 	 		createRod(rod3, length3, 12.5, 2.5);
	 			// createRod(rod3, length3, 12.5/2, 2.5/2);
	 	 		b2Vec2 pos = rod2->GetPosition() + polar(length2, rod2->GetAngle()) + b2Vec2(10.5, 2) - polar(length3, theta);
	 	 		rod3->SetTransform(pos, theta);
	 	 		b2RevoluteJointDef jointDef;
	 	 		jointDef.Initialize(rod2, rod3, rod2->GetWorldCenter()+polar(length2, rod2->GetAngle()));
	 	 		(b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
	 	 	}

	 	 	{
	 	 		createRod(rod5, length5, -0.8, 3.4);
	 	 		b2Vec2 pos = rod3->GetPosition() - polar(length3, rod3->GetAngle()) - polar(rodLength, theta);
	 	 		rod5->SetTransform(pos, theta);
	 	 		b2RevoluteJointDef jointDef;
	 	 		jointDef.Initialize(rod3, rod5, rod3->GetWorldCenter()-polar(length3, rod3->GetAngle()));
	 	 		(b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
	 	 	}

	 	 	{
	 	 		createRod(rod7, length7, 0.8, 9.0);
	 	 		b2Vec2 pos = rod3->GetPosition()+polar(length3, rod3->GetAngle())-polar(rodLength, theta);

	 	 		circle.m_p.Set(rodLength - (1.1)/sin(theta), 0); 
	 			//this is the place where valveRod connects to rod7
	 	 		rod7->CreateFixture(&circle_fix);

	 	 		rod7->SetTransform(pos, theta);

	 	 		b2RevoluteJointDef jointDef;
	 	 		jointDef.Initialize(rod7, rod3, rod3->GetWorldCenter()+polar(length3, rod3->GetAngle()));
	 	 		(b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
	 	 	}

	 	 	{
	 	 		createRod(rod8, length8, 4, 0.6);
	 	 		b2Vec2 pos = rod7->GetPosition()-polar(length7, rod7->GetAngle())-polar(rodLength, theta);

	 	 		rod8->SetTransform(pos, theta);
	 	 		b2RevoluteJointDef jointDef;
	 	 		jointDef.Initialize(rod7, rod8, rod7->GetWorldCenter()-polar(length7, rod7->GetAngle()));
	 	 		(b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
	 	 	}

	 	 	{
	 	 		createRod(rod9, length9, 0.000000001, 3.55);
	 	 		b2Vec2 pos = rod8->GetPosition()-polar(length8, rod8->GetAngle())+polar(rodLength, theta);
	 	 		rod9->SetTransform(pos, theta);
	 	 		b2RevoluteJointDef jointDef;
	 	 		jointDef.Initialize(rod9, rod8, rod8->GetWorldCenter()-polar(length8, rod8->GetAngle()));
	 	 		(b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
	 	 	}

	 	 	{
	 	 		createRod(rod10, length10, 0.000000001, 3.6);
	 	 		b2Vec2 pos = rod9->GetPosition()+polar(length9, rod9->GetAngle());
	 	 		rod10->SetTransform(pos, theta);

	 	 		b2RevoluteJointDef jointDef;
	 	 		jointDef.Initialize(rod10, rod11, rod10->GetWorldCenter());
	 	 		(b2RevoluteJoint*)m_world->CreateJoint(&jointDef);

	 	 		b2PrismaticJointDef jointDef2;
	 	 		jointDef2.Initialize(rod10, b1, rod10->GetWorldCenter(), b2Vec2(1,0));
	 	 		(b2PrismaticJoint*)m_world->CreateJoint(&jointDef2);

	 	 		b2WeldJointDef jointDef3;
	 	 		jointDef3.Initialize(rod10, rod9, rod9->GetWorldCenter()+polar(length9, rod9->GetAngle()));
	 	 		(b2WeldJoint*)m_world->CreateJoint(&jointDef3);
	 	 	}
	 	 }
////////////////////////////////////////////////////////////////////////////
	 	///The definition of <b>vavleRod</b> <hr>
	 	 {
	 	 	float scale_v = 1.70/scale_p;
	 	 	b2BodyDef *bd = new b2BodyDef;
	 	 	bd->type = b2_dynamicBody;
	 	 	bd->position.Set(xpos,ypos);
	 	 	bd->fixedRotation = true;
	 		///Fixutre :<b>fd1</b> in valveRod\n
	 		///fd1 is the fixture for middle part of the valveRod \n
	 		///This has groupIndex=-1in order to avoid collision with particles 

	 	 	b2FixtureDef *fd1 = new b2FixtureDef;
	 	 	fd1->density = 1.0;
	 	 	fd1->friction = 0;
	 	 	fd1->restitution = 1.f;
	 	 	fd1->shape = new b2PolygonShape;
	 	 	b2PolygonShape bs1;
	 	 	bs1.SetAsBox(2*scale_v,0.9*scale_v, b2Vec2(0.62*scale_v,0.f*scale_v), 0);
	 	 	fd1->shape = &bs1;

	 		///Fixutre :<b>fd2</b> in valveRod\n
	 		///fd2 is the fixture definition for left vertical box part of the valveRod
	 	 	b2FixtureDef *fd2 = new b2FixtureDef;
	 	 	fd2->density = 1.0;
	 	 	fd2->friction = 0;
	 	 	fd2->restitution = 1.f;
	 	 	fd1->filter.groupIndex = -1;
	 	 	fd1->filter.maskBits = 0x1000; 
	 	 	fd2->shape = new b2PolygonShape;
	 	 	b2PolygonShape bs2;
	 	 	bs2.SetAsBox(0.2*scale_v,1*scale_v, b2Vec2(-1.28*scale_v,0.f*scale_v), 0);
	 	 	fd2->shape = &bs2;

	 	 	///Fixutre :<b>fd3</b> in valveRod\n
	 		///fd3 is the fixture definition for right vertical part of the valveRod
	 	 	b2FixtureDef *fd3 = new b2FixtureDef;
	 	 	fd3->density = 1.0;
	 	 	fd3->friction = 0;
	 	 	fd3->restitution = 1.f;
	 	 	fd3->shape = new b2PolygonShape;
	 	 	b2PolygonShape bs3;
	 	 	bs3.SetAsBox(0.2*scale_v,1*scale_v, b2Vec2(2.52*scale_v,0.f*scale_v), 0);
	 	 	fd3->shape = &bs3;

	 	 	///Fixutre :<b>fd4</b> in valveRod\n
	 		///fd4 is the fixture definition for left horizontal rod of valveRod which connects to 3-wheel system
	 		///This has groupIndex=-1 & maskBits set to some arbitrary number in 
	 		///order to avoid collision with particles and engine parts with which it overlaps
	 	 	b2FixtureDef *fd4 = new b2FixtureDef;
	 	 	fd4->density = 1.0;
	 	 	fd4->friction = 0;
	 	 	fd4->restitution = 1.f;
	 	 	fd4->filter.groupIndex = -1;
	 	 	fd4->filter.maskBits = 0x1000;
	 	 	fd4->shape = new b2PolygonShape;
	 	 	b2PolygonShape bs4;
	 	 	bs4.SetAsBox(2.5*scale_v,0.2*scale_v, b2Vec2(-3.88f*scale_v,0.f*scale_v), 0);
	 	 	fd4->shape = &bs4;


	 	 	valveRod = m_world->CreateBody(bd);
	 	 	valveRod->CreateFixture(fd1);
	 	 	valveRod->CreateFixture(fd2);
	 	 	valveRod->CreateFixture(fd3);
	 	 	valveRod->CreateFixture(fd4);
	 	 }
	 	/// 		The <b>primatic joint</b> between ground and valveRod
	 	 {
	 	 	b2PrismaticJointDef prismaticJointDef;
	 	 	prismaticJointDef.bodyA = b1;
	 	 	prismaticJointDef.bodyB = valveRod;
	 	 	prismaticJointDef.collideConnected = true;
	 	 	prismaticJointDef.localAxisA.Set(1,0);

	 	 	prismaticJointDef.localAnchorA.Set(0,ypos);
	 	 	prismaticJointDef.localAnchorB.Set(0,0);

	 	 	(b2PrismaticJoint*)m_world->CreateJoint( &prismaticJointDef );
	 	 }
	 	///The definition for <b>piston</b>
	 	 {
	 	 	b2BodyDef *bd = new b2BodyDef;
	 	 	bd->type = b2_dynamicBody;
	 	 	bd->position.Set(xpos_p,ypos_p);
	 	 	bd->fixedRotation = true;
	 	 	///Fixutre :<b>fd1</b> in piston\n
	 		///fd1 is the fixture definition for vertical Box part of the piston
	 	 	b2FixtureDef *fd1 = new b2FixtureDef;
	 	 	fd1->density = 1;
	 	 	fd1->friction = 0.0;
	 	 	fd1->restitution = 1.f;
	 	 	fd1->shape = new b2PolygonShape;
	 	 	b2PolygonShape bs1;
	 	 	bs1.SetAsBox(0.15*scale,2*scale, b2Vec2(0,0*scale), 0);
	 	 	fd1->shape = &bs1;

	 	 	///Fixutre :<b>fd2</b> in piston\n
	 		///fd2 is the fixture definition for horizontal rod of pistonRod which connects to 3-wheel system
	 		///This has groupIndex=-1 & maskBits set to some arbitrary number in 
	 		///order to avoid collision with particles and engine parts with which it overlaps

	 	 	b2FixtureDef *fd2 = new b2FixtureDef;
	 	 	fd2->density = 1;
	 	 	fd2->friction = 0.0;
	 	 	fd2->restitution = 1.f; 
	 	 	fd2->shape = new b2PolygonShape;
	 	 	b2PolygonShape bs2;
	 	 	bs2.SetAsBox(5.455*scale,0.25*scale, b2Vec2(-5.45*scale,0*scale), 0);
	 	 	fd2->shape = &bs2;
	 	 	fd2->filter.groupIndex = -1;
	 	 	fd2->filter.maskBits = 0x1000;


	 	 	piston = m_world->CreateBody(bd);
	 	 	piston->CreateFixture(fd1);
	 	 	piston->CreateFixture(fd2);
	 	 }

	 	///		<b>prismatic joint</b> between piston and ground
	 	 {
	 	 	b2PrismaticJointDef prismaticJointDef;
	 	 	prismaticJointDef.bodyA = b1;
	 	 	prismaticJointDef.bodyB = piston;
	 	 	prismaticJointDef.collideConnected = true;
	 	 	prismaticJointDef.localAxisA.Set(1,0);

	 	 	prismaticJointDef.localAnchorA.Set(0,ypos_p);
	 	 	prismaticJointDef.localAnchorB.Set(0,0);

	 	 	(b2PrismaticJoint*)m_world->CreateJoint( &prismaticJointDef );
	 	 }
	 	 ///The definition for engineBody
	 	 {
	 	 	b2BodyDef *bd = new b2BodyDef;
	 	 	bd->type = b2_dynamicBody;
	 	 	bd->position.Set(xpos_e,ypos_e);
	 	 	bd->fixedRotation = true;

	 	 	///Fixutre :<b>fd1</b> in engineBox\n
	 	 	///This fixture corresponds to main trainBox
	 	 	b2FixtureDef *fd1 = new b2FixtureDef;
	 	 	fd1->density = 1;
	 	 	fd1->friction = 0.0;
	 	 	fd1->restitution = 1.f;
	 	 	fd1->shape = new b2PolygonShape;
	 	 	b2PolygonShape bs1;
	 	 	bs1.SetAsBox(15*scale_e,5*scale_e, b2Vec2(0,0), 0);
	 	 	fd1->shape = &bs1;
	 	 	fd1->filter.groupIndex = -1;
	 	 	fd1->filter.categoryBits = 0x0010;
	 	 	fd1->filter.maskBits = 0x0001;
	 	 	
	 	 	///Fixutre :<b>fd2</b> in engineBox\n
	 	 	///This fixture corresponds to bottom part of engineBody
	 	 	b2FixtureDef *fd2 = new b2FixtureDef;
	 	 	fd2->density = 1;
	 	 	fd2->friction = 0.0;
	 	 	fd2->restitution = 1.f; 
	 	 	fd2->shape = new b2PolygonShape;
	 	 	b2PolygonShape bs2;
	 	 	bs2.SetAsBox(3*scale_e,0.2*scale_e, b2Vec2(18.f*scale_e,-8.4f*scale_e), 0);
	 	 	fd2->shape = &bs2;
	 	 	b2FixtureDef *fd3 = new b2FixtureDef;
	 	 	
	 	 	///Fixutre :<b>fd3</b> in engineBox\n
	 	 	///This fixture corresponds to vertical body which is located at bottom right of engineBody
	 	 	fd3->density = 1;
	 	 	fd3->friction = 0.0;
	 	 	fd3->restitution = 1.f; 
	 	 	fd3->shape = new b2PolygonShape;
	 	 	b2PolygonShape bs3;
	 	 	bs3.SetAsBox(0.4*scale_e,1.6*scale_e, b2Vec2(20.6f*scale_e,-6.6f*scale_e), 0);
	 	 	fd3->shape = &bs3;
	 	 	b2FixtureDef *fd4 = new b2FixtureDef;
	 	 	
	 	 	///Fixutre :<b>fd4</b> in engineBox\n
	 	 	///This fixture corresponds to vertical body which is located at bottom left of engineBody
	 	 	fd4->density = 1;
	 	 	fd4->friction = 0.0;
	 	 	fd4->restitution = 1.f; 
	 	 	fd4->shape = new b2PolygonShape;
	 	 	b2PolygonShape bs4;
	 	 	bs4.SetAsBox(0.4*scale_e,1.6*scale_e, b2Vec2(15.6f*scale_e,-6.6f*scale_e), 0);
	 	 	fd4->shape = &bs4;
	 	 	fd4->filter.groupIndex = -1;
	 	 	fd4->filter.maskBits = 0x0001;
	 	 	b2FixtureDef *fd5 = new b2FixtureDef;
	 	 	
	 	 	///Fixutre :<b>fd5</b> in engineBox\n
	 	 	///This fixture corresponds to vertical body which is located at  top left of engineBody
	 	 	fd5->density = 1;
	 	 	fd5->friction = 0.0;
	 	 	fd5->restitution = 1.f;
	 	 	fd5->filter.groupIndex = -1;
	 	 	fd5->filter.maskBits = 0x0001; 
	 	 	fd5->shape = new b2PolygonShape;
	 	 	b2PolygonShape bs5;
	 	 	bs5.SetAsBox(0.1*scale_e,1*scale_e, b2Vec2(15.1f*scale_e,-4.f*scale_e), 0);
	 	 	fd5->shape = &bs5;
	 	 	b2FixtureDef *fd6 = new b2FixtureDef;
	 	 	
	 	 	///Fixutre :<b>fd6</b> in engineBox\n
	 	 	///This fixture corresponds to vertical body which is located at  top right of engineBody
	 	 	fd6->density = 1;
	 	 	fd6->friction = 0.0;
	 	 	fd6->restitution = 1.f; 
	 	 	fd6->shape = new b2PolygonShape;
	 	 	b2PolygonShape bs6;
	 	 	bs6.SetAsBox(0.1*scale_e,1*scale_e, b2Vec2(21.1f*scale_e,-4.f*scale_e), 0);
	 	 	fd6->shape = &bs6;
	 	 	
	 	 	///Fixutre :<b>fd7</b> in engineBox\n
	 	 	///This fixture corresponds to horizontal body which is located at  top left of engineBody
	 	 	b2FixtureDef *fd7 = new b2FixtureDef;
	 	 	fd7->density = 1;
	 	 	fd7->friction = 0.0;
	 	 	fd7->restitution = 1.f; 
	 	 	fd7->shape = new b2PolygonShape;
	 	 	b2PolygonShape bs7;
	 	 	bs7.SetAsBox(1.25*scale_e,0.2*scale_e, b2Vec2(16.25f*scale_e,-2.8f*scale_e), 0);
	 	 	fd7->shape = &bs7;
	 	 	
	 	 	///Fixutre :<b>fd8</b> in engineBox\n
	 	 	///This fixture corresponds to horizontal body which is located at  top right of engineBody
	 	 	b2FixtureDef *fd8 = new b2FixtureDef;
	 	 	fd8->density = 1;
	 	 	fd8->friction = 0.0;
	 	 	fd8->restitution = 1.f; 
	 	 	fd8->shape = new b2PolygonShape;
	 	 	b2PolygonShape bs8;
	 	 	bs8.SetAsBox(1.25*scale_e,0.2*scale_e, b2Vec2(19.75f*scale_e,-2.8f*scale_e), 0);
	 	 	fd8->shape = &bs8;

	 	 	///Fixutre :<b>fd9</b> in engineBox\n
	 	 	///This fixture corresponds to horizontal body which is located at center of engineBody
	 	 	b2FixtureDef *fd9 = new b2FixtureDef;
	 	 	fd9->density = 1;
	 	 	fd9->friction = 0.0;
	 	 	fd9->restitution = 1.f; 
	 	 	fd9->shape = new b2PolygonShape;
	 	 	b2PolygonShape bs9;	
	 	 	b2Vec2 vertices[4];
	 	 	vertices[0].Set(17*scale_e,-5.2*scale_e);
	 	 	vertices[1].Set(19.2*scale_e,-5.2*scale_e);
	 	 	vertices[2].Set(18.7*scale_e,-5*scale_e);
	 	 	vertices[3].Set(17.5*scale_e,-5*scale_e);
	 	 	bs9.Set(vertices,4);
	 	 	fd9->shape = &bs9;

	 	 	///Fixutre :<b>fd10</b> in engineBox\n
	 	 	///This fixture corresponds to chain top left of engineBody
	 	 	b2Vec2 v4[4];
	 	 	v4[3].Set(17.8*scale_e,+5*scale_e);
	 	 	v4[2].Set(17.8*scale_e,+2.5*scale_e);
	 	 	v4[1].Set(15.2*scale_e,0*scale_e);
	 	 	v4[0].Set(15.2*scale_e,-3*scale_e);
	 	 	b2ChainShape chain4;
	 	 	chain4.CreateChain(v4, 4);
	 	 	b2FixtureDef *fd10 = new b2FixtureDef;
	 	 	fd10->density = 1;
	 	 	fd10->friction = 0.0;
	 	 	fd10->restitution = 1.f;
	 	 	fd10->shape=new b2ChainShape;
	 	 	fd10->shape=&chain4;

	 	 	///Fixutre :<b>fd11</b> in engineBox\n
	 	 	///This fixture corresponds to chain top right of engineBody
	 	 	b2Vec2 v3[4];
	 	 	v3[3].Set(18.2*scale_e,+5*scale_e);
	 	 	v3[2].Set(18.2*scale_e,+2.5*scale_e);
	 	 	v3[1].Set(21*scale_e,0*scale_e);
	 	 	v3[0].Set(21*scale_e,-3*scale_e);
	 	 	b2ChainShape chain3;
	 	 	chain3.CreateChain(v3, 4);
	 	 	b2FixtureDef *fd11 = new b2FixtureDef;
	 	 	fd11->density = 1;
	 	 	fd11->friction = 0.0;
	 	 	fd11->restitution = 1.f;
	 	 	fd11->shape=&chain3;

	 	 	///Fixutre :<b>fd12</b> in engineBox\n
	 	 	///This fixture corresponds to chain on center that is part exhuast system of engineBody

			b2Vec2 v2[5];
	 	 	v2[4].Set(20.6*scale_e,-3*scale_e);
	 	 	v2[3].Set(20.6*scale_e,-0.5*scale_e);
	 	 	v2[2].Set(18	*scale_e,+2*scale_e);
	 	 	v2[1].Set(15.6*scale_e,-0.5*scale_e);
	 	 	v2[0].Set(15.6*scale_e,-3*scale_e);
	 	 	b2ChainShape chain2;
	 	 	chain2.CreateChain(v2, 5);
	 	 	b2FixtureDef *fd12 = new b2FixtureDef;
	 	 	fd12->density = 1;
	 	 	fd12->friction = 0.0;
	 	 	fd12->restitution = 1.f;
	 	 	fd12->shape=&chain2;
	 	 	
	 	 	///Fixutre :<b>fd13</b> in engineBox\n
	 	 	///This fixture corresponds to chain on center of engineBody
	 	 	b2Vec2 v5[4];
	 	 	v5[3].Set(18.5*scale_e,-3*scale_e);
	 	 	v5[2].Set(18.5	*scale_e,-2*scale_e);
	 	 	v5[1].Set(17.5*scale_e,-2*scale_e);
	 	 	v5[0].Set(17.5*scale_e,-3*scale_e);
	 	 	b2ChainShape chain5;
	 	 	chain5.CreateChain(v5, 4);
	 	 	b2FixtureDef *fd13 = new b2FixtureDef;
	 	 	fd13->density = 1;
	 	 	fd13->friction = 0.0;
	 	 	fd13->restitution = 1.f;
	 	 	fd13->shape=&chain5;

	 	 	///Fixutre :<b>fd14</b> in engineBox\n
	 	 	///This fixture corresponds to horizontal body which looks like rails located on\n
	 	 	///part of engine.fd14 denotes top rail
	 	 	b2FixtureDef *fd14 = new b2FixtureDef;
	 	 	fd14->density = 1;
	 	 	fd14->friction = 0.0;
	 	 	fd14->restitution = 1.f;
	 	 	fd14->filter.groupIndex =-1; 
	 	 	fd14->shape = new b2PolygonShape;
	 	 	b2PolygonShape bs14;
	 	 	bs14.SetAsBox(3.2*scale_e,0.1*scale_e, b2Vec2(12.f*scale_e,-6.7f*scale_e+1.8), 0);
	 	 	fd14->shape = &bs14;

	 	
	 		///Fixutre :<b>fd15</b> in engineBox\n
	 	 	///This fixture corresponds to horizontal body which looks like rails located on\n
	 	 	///part of engine.fd15 denotes bottom rail
	 	 	b2FixtureDef *fd15 = new b2FixtureDef;
	 	 	fd15->density = 1;
	 	 	fd15->friction = 0.0;
	 	 	fd15->restitution = 1.f; 
	 	 	fd15->shape = new b2PolygonShape;
	 	 	b2PolygonShape bs15;
	 	 	bs15.SetAsBox(3.2*scale_e,0.1*scale_e, b2Vec2(12.f*scale_e,-6.7f*scale_e-1.8), 0);
	 	 	fd15->shape = &bs15;
 			fd15->filter.groupIndex =-1;
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
	 	 	engineBox->CreateFixture(fd10);
	 	 	engineBox->CreateFixture(fd11);
	 	 	engineBox->CreateFixture(fd12);
	 	 	engineBox->CreateFixture(fd13);
	 	 	engineBox->CreateFixture(fd14);
	 	 	engineBox->CreateFixture(fd15);
	 	 }
	 	 ///<hr>
	 	 /// <b>Revoulte</b> joints between EngineBox and wheels
	 	 ///1)The revolutejoint between engineBox and wheel1
	 	 {
	 	 	b2RevoluteJointDef jointDef5;
	 	 	jointDef5.bodyA = wheel1;
	 	 	jointDef5.bodyB = engineBox;
	 	 	jointDef5.collideConnected = false;
	 	 	jointDef5.localAnchorA = b2Vec2(0,0);
	 	 	jointDef5.localAnchorB = b2Vec2(-xpos_e,-ypos_e+wheelRadius);
	 	 	(b2RevoluteJoint*)m_world->CreateJoint(&jointDef5);
	 	 }

	 	 ///2)The revolutejoint between engineBox and wheel2
	 	 {
	 	 	b2RevoluteJointDef jointDef5;
	 	 	jointDef5.bodyA = wheel2;
	 	 	jointDef5.bodyB = engineBox;
	 	 	jointDef5.collideConnected = false;
	 	 	jointDef5.localAnchorA = b2Vec2(0,0);
	 	 	jointDef5.localAnchorB = b2Vec2(-xpos_e+2.1*wheelRadius,-ypos_e+wheelRadius);
	 	 	(b2RevoluteJoint*)m_world->CreateJoint(&jointDef5);
	 	 }

	 	 ///3)The revolutejoint between engineBox and wheel3
	 	 {
	 	 	b2RevoluteJointDef jointDef5;
	 	 	jointDef5.bodyA = wheel3;
	 	 	jointDef5.bodyB = engineBox;
	 	 	jointDef5.collideConnected = false;
	 	 	jointDef5.localAnchorA = b2Vec2(0,0);
	 	 	jointDef5.localAnchorB = b2Vec2(-xpos_e+4.2*wheelRadius,-ypos_e+wheelRadius);
	 	 	(b2RevoluteJoint*)m_world->CreateJoint(&jointDef5);
	 	 }
	 	 ///<hr>
	 	 ///The definition of exhaust body <b>ext1</b> \n
	 	 ///This has setUserdata as value 2
	 	 {       
	 	 	b2BodyDef exhaustbd;
	 	 	exhaustbd.position.Set(xpos_e+15.4*scale_e, ypos_e-3.3*scale_e);
	 	 	b2CircleShape ext_entry;
	 	 	ext_entry.m_radius = 0.2*scale_e;
	 	 	b2FixtureDef *fd7 = new b2FixtureDef;
	 	 	fd7->shape = new b2CircleShape;	
	 	 	fd7->shape=&ext_entry;
	 	 	fd7->restitution = 1.f;
	 	 	fd7->friction = 1;
	 	 	fd7->density = 1.0f;
	 	 	exhaustbd.type = b2_dynamicBody;
	 	 	ext1 = m_world->CreateBody(&exhaustbd);
	 	 	ext1->CreateFixture(fd7);
	 	 	int left=2;	
	 	 	void* ls = (void*) (intptr_t) left;
	 	 	ext1->SetUserData(ls);
	 	 }

	 	 ///<b>Weld<\b> joint between engineBox and exhuast1\n
	 	 {
	 	 	b2WeldJointDef weldJointDef;
	 	 	weldJointDef.bodyA = engineBox;
	 	 	weldJointDef.bodyB = ext1;
	 	 	weldJointDef.collideConnected = false;
	 	 	weldJointDef.localAnchorA.Set(15.4*scale_e,-3.3*scale_e);
	 	 	weldJointDef.localAnchorB.Set(0,0);
	 	 	(b2WeldJoint*)m_world->CreateJoint( &weldJointDef );
	 	 }
	 	 
	 	 ///The definition of exhaust body <b>ext2</b>\n
	 	 ///This has setUser data as value 3\n
	 	 {       
	 	 	b2BodyDef exhaustbd;
	 	 	exhaustbd.position.Set(xpos_e+20.8*scale_e, ypos_e-3.3*scale_e);
	 	 	b2CircleShape ext_entry;
	 	 	ext_entry.m_radius = 0.2*scale_e;
	 	 	b2FixtureDef *fd7 = new b2FixtureDef;
	 	 	fd7->shape = new b2CircleShape;
	 	 	fd7->shape=&ext_entry;
	 	 	fd7->restitution = 1.f;
	 	 	fd7->friction = 1;
	 	 	fd7->density = 1.0f;
	 	 	exhaustbd.type = b2_dynamicBody;
	 	 	ext2 = m_world->CreateBody(&exhaustbd);
	 	 	ext2->CreateFixture(fd7);
	 	 	int right=3;
	 	 	void* ls = (void*) (intptr_t) right;	
	 	 	ext2->SetUserData(ls);
	 	 }

	 	 ///<b>Weld</b> joint between engineBox and exhuast2\n<hr>
	 	 {
	 	 	b2WeldJointDef weldJointDef;
	 	 	weldJointDef.bodyA = engineBox;
	 	 	weldJointDef.bodyB = ext2;
	 	 	weldJointDef.collideConnected = false;
	 	 	weldJointDef.localAnchorA.Set(20.8*scale_e,-3.3*scale_e);
	 	 	weldJointDef.localAnchorB.Set(0,0);
	 	    (b2WeldJoint*)m_world->CreateJoint( &weldJointDef );
	 	 }
	 	 ///<b> Revolute Joints </b> which connect 3- wheel system to <b>trainBox</b> and <b>engineBody</b>\n
	 	 {	
			///1)revolutejoint between rod4 and enginebox\n
	 	 	b2RevoluteJointDef jointDef;
	 	 	jointDef.Initialize(rod4, engineBox, rod4->GetWorldCenter() - polar(length4, rod4->GetAngle()));
	 	    (b2RevoluteJoint*)m_world->CreateJoint(&jointDef);

			///2)revolutejoint between rod5 and enginebox\n
	 	 	jointDef.Initialize(rod5, engineBox, rod5->GetWorldCenter() - polar(length5, rod5->GetAngle()));
	 	 	(b2RevoluteJoint*)m_world->CreateJoint(&jointDef);

			///3)revolutejoint between rod10 and piston\n
	 	 	jointDef.Initialize(rod10, piston, rod10->GetWorldCenter());
	 	 	(b2RevoluteJoint*)m_world->CreateJoint(&jointDef);		

			///4)revolutejoint between rod7 and valveRod\n
	 	 	jointDef.Initialize(rod7, valveRod, rod7->GetWorldCenter() + polar(length7, rod7->GetAngle()) - polar(1.1/sin(rod7->GetAngle()), rod7->GetAngle()));
	 	 	(b2RevoluteJoint*)m_world->CreateJoint(&jointDef);		
	 	 }
	 	}
	 	sim_t *sim = new sim_t("Dominos", dominos_t::create);
	 }
