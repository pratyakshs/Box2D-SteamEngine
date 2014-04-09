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


#ifndef _CS296BASE_HPP_
#define _CS296BASE_HPP_

#include "render.hpp"
#include <Box2D/Box2D.h>
#include <cstdlib>

#define RAND_LIMIT 32767

namespace cs296
{
  class base_sim_t;
  struct settings_t;
  
  typedef base_sim_t* sim_create_fcn(); 

  struct settings_t
  {
    settings_t() :
      view_center(0.0f, 20.0f), //!<The origin coordinates are set to (0.0, 20.0)
      hz(60.0f), //!<Frequency of simulation initialized to 60.0 Hz
      velocity_iterations(8),//!<Velocity iterations initialized to 8.
      position_iterations(3),//!<Position iterations intialized to 3.
      draw_shapes(1),//!<Initialized this to 1. Shapes of all box2d objects will be shown.
      draw_joints(1),//!<Initialized this to 1. All box2d joints will be shown.
      draw_AABBs(0),//!<Initialized this to 0. Axis-aligned bounding boxes(AABB) will not be shown.
      draw_pairs(0),
      draw_contact_points(0),//!<Initialized this to 0. Contact points between colliding objects will not be shown.
      draw_contact_normals(0),//!<Initialized this to 0. Contact normals between colliding objects will not be shown.
      draw_contact_forces(0),//!<Initialized this to 0. Contact forces will not be shown.
      draw_friction_forces(0),//!<Initialized this to 0. Frictional forces will not be drawn.
      draw_COMs(0),//!<Initialized this to 0. Center of mass of objects will not be drawn.
      draw_stats(0),//!<Initialized this to 0. Information of bodies,contacts and joints of simulation will not be shown.
      draw_profile(0),//!<Initializing this to 0 will not show the calculation done by box2D.
      enable_warm_starting(1),//!<Initializing this to 1 will simulate object collisions more accurately.
      enable_continuous(1),//!<Initializing this to 1 enables continuous collision detection(ccd)
      enable_sub_stepping(0),//!<Initializing this to 0 means not to resolve the collision after moving the bodies to time of impact
      pause(0),//!<Initializing this to 0 means the simulation is not pause.
      single_step(0)//!<Initializing this to 0 mean not to increment in single steps in simulation.
    {}
    
    
    
    
    b2Vec2 view_center; //!<This option shifts the origin of screen to the value of this vector. All positions are relative to this vector.
    float32 hz; //!<Represents the frequency of simulation. On decresing this simulation beccomes faster but inaccurate.
    int32 velocity_iterations;//!<In the velocity phase, the solver computes the impulses necessary for the bodies to move correctly.This phase has a iteration count intialized to 8.Using fewer iterations increases performance but accuracy suffers.
    int32 position_iterations;//!< In the position phase, the solver adjusts the positions of the bodies to reduce overlap and joint detachment.This phase has a iteration count intialized to 3.Using fewer iterations increases performance but accuracy suffers.
    int32 draw_shapes;//!< This variable shows the shape of all box2d objects
    int32 draw_joints;//!< This variable shows all joints of all box2d joints
    int32 draw_AABBs;//!<  This variable shows axis-aligned bounding boxes(AABB). It is intialized to 0  
    int32 draw_pairs;//!< 
    int32 draw_contact_points;//!< This option when checked shows contact points between colliding objects
    int32 draw_contact_normals;//!< This option when checked shows contact normals between colliding objects
    int32 draw_contact_forces;//!< This option shows contact forces .It is intialized to 0
    int32 draw_friction_forces;//!< This option draws frictional forces.It is intialized to 0
    int32 draw_COMs;//!< This option draws center of masses of objects.It is intialized to 0 
    int32 draw_stats;//!< This option shows information bodies,contacts and joints of simulation on screen.It is intialized to 0
    int32 draw_profile;//!<This option shows all calculation done by box2d on screen. This option is intialzed to 0.
    int32 enable_warm_starting;//!< This feature is used for simualting object collitions more accurately
    int32 enable_continuous;//!< Box2d uses Continuous collision detection(CCD).Bodies are moved to their first (Time of imapct)TOI and then halted for the remainder of the time step.This is intialized to 1.
    int32 enable_sub_stepping;//!< This option resolves the collision after moving the bodies to time of impact
    int32 pause;//!< This option pauses the simulation. It is intialized to 0.
    int32 single_step;//!< This option moves only single time step in simulation. It is intialized to 0
  };
//!<struct settings_t: contains all the variables to control the simulation settings 


  struct sim_t
  {
    const char *name;//!< A pointer to character called name
    sim_create_fcn *create_fcn;//!< This is an instance  of bas_sim_t class

    /// A constructor of class sim_t  which creates a object with the 2 given parameters as given values
    sim_t(const char *_name, sim_create_fcn *_create_fcn): 
      name(_name), create_fcn(_create_fcn) {;}
  };
  //!< struct sim_t: This starts the simulation 
  
  
  
  extern sim_t *sim;//!< extern keyword tells that *sim is defined in someother file.
  
  
  const int32 k_max_contact_points = 2048;  //!<The maximum number of contact points 
  
  
  struct contact_point_t
  {
    b2Fixture* fixtureA;//!< The pointer to fixture of Body A which stores  all attributes of body(height,width,frixtional values, restitution etc) 
    b2Fixture* fixtureB;//!< The pointer to fixture of Body B which stores  all attributes of body(height,width,frixtional values, restitution etc) 
    b2Vec2 normal;//!< The normal vector which results as a collision of bodies at the point of contact
    b2Vec2 position;//!< The position vector of the point of contact
    b2PointState state;//!< This variable stores the state of point
  };
//!< This struct has all the information about contact point  between 2 colliding objects

  
  class base_sim_t : public b2ContactListener
  {
  public:
    
    base_sim_t(); //!<Constructs the world and defines its gravity vector
    
    //! Virtual destructors - amazing objects. Why are these necessary?
    virtual ~base_sim_t();
    
    void set_text_line(int32 line) { m_text_line = line; }  //!<Sets the line at which debugging messages are to be printed, used in m_debug_draw.DrawString()
    void draw_title(int x, int y, const char *string);    //!<Prints the string on the screen
    
    virtual void step(settings_t* settings); //!< This function assigns the settings to variables in m_world and m_debug_draw

    virtual void keyboard(unsigned char key) { B2_NOT_USED(key); }//!< This function checks if keyboard char key is pressed and accordingly responds
    virtual void keyboard_up(unsigned char key) { B2_NOT_USED(key); }//!< This function checks if keyboard arrows keys are pressed

    void shift_mouse_down(const b2Vec2& p) { B2_NOT_USED(p); }//!< This function checks if shift_mouse_down is used
    virtual void mouse_down(const b2Vec2& p) { B2_NOT_USED(p); }//!< This function checks ifmouse_down is used
    virtual void mouse_up(const b2Vec2& p) { B2_NOT_USED(p); }//!< This function checks if mouse_up is used
    void mouse_move(const b2Vec2& p) { B2_NOT_USED(p); }//!< This function checks if mouse is moved

    
   
    virtual void joint_destroyed(b2Joint* joint) { B2_NOT_USED(joint); }
     //!< Let derived tests know that a joint was destroyed.
     
    virtual void begin_contact(b2Contact* contact) { B2_NOT_USED(contact); }
    //!< Callbacks for derived classes for begin contact
    
    virtual void end_contact(b2Contact* contact) { B2_NOT_USED(contact); }
    //!< Callbacks for derived classes end contact
    
    virtual void pre_solve(b2Contact* contact, const b2Manifold* oldManifold);
    //!< Callbacks for derived classes pre solving the collision
    
    virtual void post_solve(const b2Contact* contact, const b2ContactImpulse* impulse)
    {
      B2_NOT_USED(contact);
      B2_NOT_USED(impulse);
    }//!< Callbacks for derived classes for post collision
    b2World* get_world(){return m_world;}
  //!How are protected members different from private memebers of a class in C++ ?
  protected:	

    //! What are Friend classes?
    friend class contact_listener_t;//!< Declaration of friend class
    
    b2Body* m_ground_body;//!<  A variable for ground of b2body 
    b2AABB m_world_AABB;//!< b2AABB(axis aligned bounding box) declaration of world for 
    contact_point_t m_points[k_max_contact_points];//!< an array for storing all contact points
    int32 m_point_count;//!< counter for m_point

    debug_draw_t m_debug_draw;//!<debig_draw_t For  drawing objects
    int32 m_text_line;//!<Number of strings is stored here
    b2World* m_world;//!<The world in which simluation is being created

    int32 m_step_count;//!< max_step_count is stored here
    
    b2Profile m_max_profile;//!< Stores maximum number of profiles
    b2Profile m_total_profile;//!< Stores total number of profiles used
  };
    //!< This class contains information about various interaction of user with simulation. This is also responsible for input from keyboard and mouse .

}

#endif
