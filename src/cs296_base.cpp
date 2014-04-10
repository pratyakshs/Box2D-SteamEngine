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

#include "cs296_base.hpp"
#include "dominos.hpp"
#include <cstdio>
#include <stdio.h>    
#include <stdlib.h>     
#include <time.h>
#include <vector>
#include <queue>
#include <stdint.h>     
//! Standard namespace for using std::queue and other functions 
using namespace std;
using namespace cs296; 
//!The scaleof engine
extern float scale_e;
//!The variable which defines accleration
extern bool accl;
//!The variable which defines deaccleration
extern bool stop;
//! The main engineBox
extern b2Body* engineBox;
//!The count of number of particles which collide with ext1
int count1=0;
//!The count of number of particles which collide with ext2
int count2=0;
//!The count of number of times step() is called 
int time_step_count=0;
//!The custom defined class for storing time of creation of objects along with pointer to object 
struct smoke{
  b2Body* mybody;
  int time_stamp;
  smoke(){
  }
};
/// This vector stores all objects which contact with exhaust and are to be deleted
vector <b2Body*> del_list;
/// This stores all exhaust bodies which will be deleted after 200 time_step_counts
queue <smoke*> smoke_list;
///This class is defined checking some type of collisions
class MyContactListener : public b2ContactListener
{
  //! This function is called when any bodies contact
  void BeginContact(b2Contact* contact) {
      ///get the data from all fixtures from here
      ///1 corresponds to ball
      ///2 corresponds to ext1(exhaust1)
      ///3 corresponds to ext2(exhaust2)
    void* bodyAUserData = contact->GetFixtureA()->GetBody()->GetUserData();
    void* bodyBUserData = contact->GetFixtureB()->GetBody()->GetUserData();
    if (bodyAUserData && bodyBUserData)
    {
      int a =*((int*)(&bodyAUserData));
      int b =*((int*)(&bodyBUserData));
      if(a == 1 || b == 1){count1++;
        if(a==1) del_list.push_back(contact->GetFixtureA()->GetBody());
        else del_list.push_back(contact->GetFixtureB()->GetBody());}
        if(a == 1 || b == 1){count2++;
          if(a==1) del_list.push_back(contact->GetFixtureA()->GetBody());
          else del_list.push_back(contact->GetFixtureB()->GetBody());}
        }
      }

      void EndContact(b2Contact* contact) {
      }
    };
    ///An object of mycontactListner is declared
    MyContactListener myContactListenerInstance;
    base_sim_t::base_sim_t()
    {
     b2Vec2 gravity;
     gravity.Set(0.0f, -10.0f);
     m_world = new b2World(gravity);
     m_world->SetContactListener(&myContactListenerInstance);
     m_text_line = 30;

     m_point_count = 0;

     m_world->SetDebugDraw(&m_debug_draw);

     m_step_count = 0;

     b2BodyDef body_def;
     m_ground_body = m_world->CreateBody(&body_def);

     memset(&m_max_profile, 0, sizeof(b2Profile));
     memset(&m_total_profile, 0, sizeof(b2Profile));
   }

   base_sim_t::~base_sim_t()
   {
	// By deleting the world, we delete the bomb, mouse joint, etc.
    time_step_count =0;
    count1=0;
    count2=0;
    del_list.clear();
    delete m_world;
    m_world = NULL;
  }

  void base_sim_t::pre_solve(b2Contact* contact, const b2Manifold* oldManifold)
  {
    const b2Manifold* manifold = contact->GetManifold();

    b2Fixture* fixtureA = contact->GetFixtureA();
    b2Fixture* fixtureB = contact->GetFixtureB();  
    if (manifold->pointCount == 0)
    {
      return;
    }



    b2PointState state1[b2_maxManifoldPoints], state2[b2_maxManifoldPoints];
    b2GetPointStates(state1, state2, oldManifold, manifold);

    b2WorldManifold world_manifold;
    contact->GetWorldManifold(&world_manifold);

    for (int32 i = 0; i < manifold->pointCount && m_point_count < k_max_contact_points; ++i)
    { 
      contact_point_t* cp = m_points + m_point_count;
      cp->fixtureA = fixtureA;
      cp->fixtureB = fixtureB;
      cp->position = world_manifold.points[i];
      cp->normal = world_manifold.normal;
      cp->state = state2[i];
      ++m_point_count;
    }
  }

  void base_sim_t::draw_title(int x, int y, const char *string)
  {
    m_debug_draw.DrawString(x, y, string);
  }

  void base_sim_t::step(settings_t* settings)
  {
    float32 time_step = settings->hz > 0.0f ? 1.0f / settings->hz : float32(0.0f);

    if (settings->pause)
    {
      if (settings->single_step)
      {
       settings->single_step = 0;
     }
     else
     {
       time_step = 0.0f;
     }

     m_debug_draw.DrawString(5, m_text_line, "****PAUSED****");
     m_text_line += 15;
   }

   uint32 flags = 0;
   flags += settings->draw_shapes			* b2Draw::e_shapeBit;
   flags += settings->draw_joints			* b2Draw::e_jointBit;
   flags += settings->draw_AABBs			* b2Draw::e_aabbBit;
   flags += settings->draw_pairs			* b2Draw::e_pairBit;
   flags += settings->draw_COMs				* b2Draw::e_centerOfMassBit;
   m_debug_draw.SetFlags(flags);

   m_world->SetWarmStarting(settings->enable_warm_starting > 0);
   m_world->SetContinuousPhysics(settings->enable_continuous > 0);
   m_world->SetSubStepping(settings->enable_sub_stepping > 0);

   m_point_count = 0;



   m_world->Step(time_step, settings->velocity_iterations, settings->position_iterations);
  	  //if(counter!=0)printf("%d\n", counter);
   for(int j=0;j<2;j++)
   {
     int num_balls;
     float pos;
     if(j==0){num_balls=count1;pos=15.4;}
     else {num_balls=count2;pos=20.8;}
     for (int i = 0; i < num_balls; i++) {
      float angle =0;//(rand() % 361)/360.0 * 2 * 3.1416;
      b2Vec2 rayDir( sinf(angle), cosf(angle) );
      b2Vec2 center = engineBox->GetPosition()+b2Vec2(pos*scale_e,-2.5*scale_e);
      int blastPower=1;
      b2BodyDef bd;
      bd.type = b2_dynamicBody;
      bd.fixedRotation = true; // rotation not necessary
      bd.bullet = true; // prevent tunneling at high speed
      bd.linearDamping = 0; // drag due to moving through air
      bd.gravityScale = -2; // ignore gravity
      bd.position = center; // start at blast center
      bd.linearVelocity = blastPower * rayDir;
      b2Body* body = m_world->CreateBody( &bd );
      b2CircleShape circleShape;
      circleShape.m_radius = 0.05; // very small

      b2FixtureDef fd;
      fd.shape = &circleShape;  
      fd.density = 1; // very high - shared across all particles
      fd.friction = 0; // friction not necessary
      fd.restitution = 1.f; // high restitution to reflect off obstacles
      fd.filter.groupIndex = -2; // particles should not collide with each other
      fd.filter.categoryBits = 0x0005; 
      // fd.filter.maskBits = 0x0004; 
      body->CreateFixture( &fd );
      smoke* part = new smoke();
      part->time_stamp=time_step_count;
      part->mybody=body;
      smoke_list.push(part);
    }
  }
  count1=0;
  count2=0;
  time_step_count++;
  while(!smoke_list.empty()) 
  {
    if(smoke_list.front()->time_stamp +200 <= time_step_count){
      m_world->DestroyBody(smoke_list.front()->mybody);
      smoke_list.pop();}
      else break;
  }
    {
     int numballs;
     if(accl){numballs=5;accl=false;}
     //else if(stop)numballs=0;
     //else if(time_step_count%15 == 0)numballs=1;
     else numballs=0;
     for (int i = 0; i < numballs; i++) {
      float angle = (rand() % 361)/360.0 * 2 * 3.1416;
      b2Vec2 rayDir( sinf(angle), cosf(angle) );
      b2Vec2 center =engineBox->GetPosition()+b2Vec2(18*scale_e,-3 *scale_e);
      int blastPower=100;
      b2BodyDef bd;
      bd.type = b2_dynamicBody;
      bd.fixedRotation = true; // rotation not necessary
      bd.bullet = true; // prevent tunneling at high speed
      bd.linearDamping = 0; // drag due to moving through air
      bd.gravityScale = 0; // ignore gravity
      bd.position = center; // start at this center
      bd.linearVelocity = blastPower * rayDir;
      b2Body* body = m_world->CreateBody( &bd );
      int ballIndex=1;
      void* ls = (void*) (intptr_t) ballIndex;
      body->SetUserData(ls);
      b2CircleShape circleShape;
      circleShape.m_radius = 0.05; // small radius is set
      b2FixtureDef fd;
      fd.shape = &circleShape;
      fd.density = 500;  
      fd.friction = 0; 
      fd.restitution = 1.f; 
      fd.filter.groupIndex = -5; // particles should not collide with each other
      fd.filter.categoryBits = 0x0001; 
      body->CreateFixture( &fd );
    }	
  }

  for (unsigned int i=0;i<del_list.size();i++)
  {
    if(del_list[i]->IsBullet()){m_world->DestroyBody(del_list[i]);}	
  }
  del_list.clear();  
  
  m_world->DrawDebugData();
  
  if (time_step > 0.0f)
  {
    ++m_step_count;
  }
  
  if (settings->draw_stats)
  {
    int32 body_count = m_world->GetBodyCount();
    int32 contact_count = m_world->GetContactCount();
    int32 joint_count = m_world->GetJointCount();
    m_debug_draw.DrawString(5, m_text_line, "bodies/contacts/joints = %d/%d/%d", body_count, contact_count, joint_count);
    m_text_line += 15;

    int32 proxy_count = m_world->GetProxyCount();
    int32 height = m_world->GetTreeHeight();
    int32 balance = m_world->GetTreeBalance();
    float32 quality = m_world->GetTreeQuality();
    m_debug_draw.DrawString(5, m_text_line, "proxies/height/balance/quality = %d/%d/%d/%g", proxy_count, height, balance, quality);
    m_text_line += 15;
  }
  
  // Track maximum profile times
  {
    const b2Profile& p = m_world->GetProfile();
    m_max_profile.step = b2Max(m_max_profile.step, p.step);
    m_max_profile.collide = b2Max(m_max_profile.collide, p.collide);
    m_max_profile.solve = b2Max(m_max_profile.solve, p.solve);
    m_max_profile.solveInit = b2Max(m_max_profile.solveInit, p.solveInit);
    m_max_profile.solveVelocity = b2Max(m_max_profile.solveVelocity, p.solveVelocity);
    m_max_profile.solvePosition = b2Max(m_max_profile.solvePosition, p.solvePosition);
    m_max_profile.solveTOI = b2Max(m_max_profile.solveTOI, p.solveTOI);
    m_max_profile.broadphase = b2Max(m_max_profile.broadphase, p.broadphase);
    
    m_total_profile.step += p.step;
    m_total_profile.collide += p.collide;
    m_total_profile.solve += p.solve;
    m_total_profile.solveInit += p.solveInit;
    m_total_profile.solveVelocity += p.solveVelocity;
    m_total_profile.solvePosition += p.solvePosition;
    m_total_profile.solveTOI += p.solveTOI;
    m_total_profile.broadphase += p.broadphase;
  }
  
  if (settings->draw_profile)
  {
    const b2Profile& p = m_world->GetProfile();

    b2Profile ave_profile;
    memset(&ave_profile, 0, sizeof(b2Profile));
    if (m_step_count > 0)
    {
     float32 scale = 1.0f / m_step_count;
     ave_profile.step = scale * m_total_profile.step;
     ave_profile.collide = scale * m_total_profile.collide;
     ave_profile.solve = scale * m_total_profile.solve;
     ave_profile.solveInit = scale * m_total_profile.solveInit;
     ave_profile.solveVelocity = scale * m_total_profile.solveVelocity;
     ave_profile.solvePosition = scale * m_total_profile.solvePosition;
     ave_profile.solveTOI = scale * m_total_profile.solveTOI;
     ave_profile.broadphase = scale * m_total_profile.broadphase;
   }

   m_debug_draw.DrawString(5, m_text_line, "step [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.step, ave_profile.step, m_max_profile.step);
   m_text_line += 15;
   m_debug_draw.DrawString(5, m_text_line, "collide [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.collide, ave_profile.collide, m_max_profile.collide);
   m_text_line += 15;
   m_debug_draw.DrawString(5, m_text_line, "solve [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solve, ave_profile.solve, m_max_profile.solve);
   m_text_line += 15;
   m_debug_draw.DrawString(5, m_text_line, "solve init [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveInit, ave_profile.solveInit, m_max_profile.solveInit);
   m_text_line += 15;
   m_debug_draw.DrawString(5, m_text_line, "solve velocity [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveVelocity, ave_profile.solveVelocity, m_max_profile.solveVelocity);
   m_text_line += 15;
   m_debug_draw.DrawString(5, m_text_line, "solve position [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solvePosition, ave_profile.solvePosition, m_max_profile.solvePosition);
   m_text_line += 15;
   m_debug_draw.DrawString(5, m_text_line, "solveTOI [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveTOI, ave_profile.solveTOI, m_max_profile.solveTOI);
   m_text_line += 15;
   m_debug_draw.DrawString(5, m_text_line, "broad-phase [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.broadphase, ave_profile.broadphase, m_max_profile.broadphase);
   m_text_line += 15;
 }

 if (settings->draw_contact_points)
 {
      //const float32 k_impulseScale = 0.1f;
  const float32 k_axis_scale = 0.3f;

  for (int32 i = 0; i < m_point_count; ++i)
  {
   contact_point_t* point = m_points + i;

   if (point->state == b2_addState)
   {
	      // Add
     m_debug_draw.DrawPoint(point->position, 10.0f, b2Color(0.3f, 0.95f, 0.3f));
   }
   else if (point->state == b2_persistState)
   {
	      // Persist
     m_debug_draw.DrawPoint(point->position, 5.0f, b2Color(0.3f, 0.3f, 0.95f));
   }

   if (settings->draw_contact_normals == 1)
   {
     b2Vec2 p1 = point->position;
     b2Vec2 p2 = p1 + k_axis_scale * point->normal;
     m_debug_draw.DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.9f));
   }
   else if (settings->draw_contact_forces == 1)
   {
	      //b2Vec2 p1 = point->position;
	      //b2Vec2 p2 = p1 + k_forceScale * point->normalForce * point->normal;
	      //DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.3f));
   }

   if (settings->draw_friction_forces == 1)
   {
	      //b2Vec2 tangent = b2Cross(point->normal, 1.0f);
	      //b2Vec2 p1 = point->position;
	      //b2Vec2 p2 = p1 + k_forceScale * point->tangentForce * tangent;
	      //DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.3f));
   }
 }
}
}
