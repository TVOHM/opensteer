// ----------------------------------------------------------------------------
//
//
// OpenSteer -- Steering Behaviors for Autonomous Characters
//
// Copyright (c) 2002-2003, Sony Computer Entertainment America
// Original author: Craig Reynolds <craig_reynolds@playstation.sony.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//
//
// ----------------------------------------------------------------------------
//
//
// OpenSteer Preys
// 
// 09-26-02 cwr: created 
//
//
// ----------------------------------------------------------------------------


#include <sstream>
#include <time.h>
#include <vector>
#include "OpenSteer/SimpleVehicle.h"
#include "OpenSteer/OpenSteerDemo.h"
#include "OpenSteer/Proximity.h"




// Include names declared in the OpenSteer namespace into the namespaces to search to find names.
using namespace OpenSteer;

// ----------------------------------------------------------------------------


typedef OpenSteer::AbstractProximityDatabase<AbstractVehicle*> ProximityDatabase;
typedef OpenSteer::AbstractTokenForProximityDatabase<AbstractVehicle*> ProximityToken;


// ----------------------------------------------------------------------------


class Prey : public OpenSteer::SimpleVehicle
{
public:

    // type for a flock: an STL vector of Prey pointers
    typedef std::vector<Prey*> groupType;


    // constructor
    Prey (ProximityDatabase& pd)
    {
        // allocate a token for this Prey in the proximity database
        proximityToken = NULL;
        newPD (pd);

        // reset all Prey state
        reset ();
    }


    // destructor
    ~Prey ()
    {
        // delete this Prey's token in the proximity database
        delete proximityToken;
    }


    // reset state
    void reset (void)
    {
        // reset the vehicle
        SimpleVehicle::reset ();

        // steering force is clipped to this magnitude
        setMaxForce (27);

        // velocity is clipped to this magnitude
        setMaxSpeed (9);

        // initial slow speed
        setSpeed (maxSpeed() * 0.3f);

        // randomize initial orientation
        regenerateOrthonormalBasisUF (RandomUnitVector ());

        // randomize initial position
        setPosition (RandomVectorInUnitRadiusSphere () * 20);

        // notify proximity database that our position has changed
        proximityToken->updateForNewPosition (position());
    }


    // draw this Prey into the scene
    virtual void draw (void)
    {
        drawBasic3dSphericalVehicle (*this, gGray70);
        // drawTrail ();
    }


    // per frame simulation update
    void update (const float currentTime, const float elapsedTime)
    {
        // steer to flock and perhaps to stay within the spherical boundary
        applySteeringForce (steerToFlock () + handleBoundary(), elapsedTime);

        // notify proximity database that our position has changed
        proximityToken->updateForNewPosition (position());
    }


    // basic flocking
    virtual Vec3 steerToFlock (void)
    {
        const float separationRadius =  5.0f;
        const float separationAngle  = -0.707f;
        const float separationWeight =  12.0f;

        const float alignmentRadius = 7.5f;
        const float alignmentAngle  = 0.7f;
        const float alignmentWeight = 8.0f;

        const float cohesionRadius = 9.0f;
        const float cohesionAngle  = -0.15f;
        const float cohesionWeight = 8.0f;

        const float maxRadius = maxXXX (separationRadius,
                                        maxXXX (alignmentRadius,
                                                cohesionRadius));

        // find all flockmates within maxRadius using proximity database
        neighbors.clear();
        proximityToken->findNeighbors (position(), maxRadius, neighbors);

        // determine each of the three component behaviors of flocking
        const Vec3 separation = steerForSeparation (separationRadius,
                                                    separationAngle,
                                                    neighbors);
        const Vec3 alignment  = steerForAlignment  (alignmentRadius,
                                                    alignmentAngle,
                                                    neighbors);
        const Vec3 cohesion   = steerForCohesion   (cohesionRadius,
                                                    cohesionAngle,
                                                    neighbors);

        // apply weights to components (save in variables for annotation)
        const Vec3 separationW = separation * separationWeight;
        const Vec3 alignmentW = alignment * alignmentWeight;
        const Vec3 cohesionW = cohesion * cohesionWeight;

        // annotation
        // const float s = 0.1;
        // annotationLine (position, position + (separationW * s), gRed);
        // annotationLine (position, position + (alignmentW  * s), gOrange);
        // annotationLine (position, position + (cohesionW   * s), gYellow);

        return separationW + alignmentW + cohesionW;
    }


    // Take action to stay within sphereical boundary.  Returns steering
    // value (which is normally zero) and may take other side-effecting
    // actions such as kinematically changing the Prey's position.
    Vec3 handleBoundary (void)
    {
        // while inside the sphere do noting
        if (position().length() < worldRadius) return Vec3::zero;

        // once outside, select strategy
        switch (boundaryCondition)
        {
        case 0:
            {
                // steer back when outside
                const Vec3 seek = xxxsteerForSeek (Vec3::zero);
                const Vec3 lateral = seek.perpendicularComponent (forward ());
                return lateral;
            }
        case 1:
            {
                // wrap around (teleport)
                setPosition (position().sphericalWrapAround (Vec3::zero,
                                                             worldRadius));
                return Vec3::zero;
            }
        }
        return Vec3::zero; // should not reach here
    }


    // make Preys "bank" as they fly
    void regenerateLocalSpace (const Vec3& newVelocity,
                               const float elapsedTime)
    {
        regenerateLocalSpaceForBanking (newVelocity, elapsedTime);
    }

    // switch to new proximity database -- just for demo purposes
    void newPD (ProximityDatabase& pd)
    {
        // delete this Prey's token in the old proximity database
        delete proximityToken;

        // allocate a token for this Prey in the proximity database
        proximityToken = pd.allocateToken (this);
    }


    // cycle through various boundary conditions
    static void nextBoundaryCondition (void)
    {
        const int max = 2;
        boundaryCondition = (boundaryCondition + 1) % max;
    }
    static int boundaryCondition;

    // a pointer to this Prey's interface object for the proximity database
    ProximityToken* proximityToken;

    // allocate one and share amoung instances just to save memory usage
    // (change to per-instance allocation to be more MP-safe)
    static AVGroup neighbors;

    static float worldRadius;
};


AVGroup Prey::neighbors;
float Prey::worldRadius = 50.0f;
int Prey::boundaryCondition = 0;

// Derivative class from Prey. Similar mechanics but slightly different behaviours.
class Predator : public Prey
{
public:
	Predator(ProximityDatabase& pd) : Prey (pd)
	{
		quarry = nullptr;
	}
	// Overloads Prey draw function to draw predators as red
    void draw (void)
    {
        drawBasic3dSphericalVehicle (*this, gRed);
    }
	// A bit hacky - Overloads Prey steerToFlock function to look for prey and then chase prey. If none found it wanders.
	Vec3 steerToFlock (void)
	{
		if(quarry == nullptr)
			selectQuarry();

		if(quarry == nullptr)
			return steerForWander(5.0f);
		else
			return steerForPursuit(*quarry);
	}
	// Searches neighbouring agents for prey. If any are found it selects a random one and sets it as the quarry.
	void selectQuarry()
	{
        neighbors.clear();
        proximityToken->findNeighbors (position(), 10.0f, neighbors);

		std::vector<AbstractVehicle*> local_prey;

		for(auto itr = neighbors.begin(); itr != neighbors.end(); itr++)
		{
			// When an AbstractVehicle that is a prey is dynamic_cast as a predator, nullptr is returned, wheras predators return valid pointers.
			// This can be used to determine which agents are predators and which are prey.
			Predator* predator = dynamic_cast<Predator*>((*itr));
			if(predator == nullptr)
				local_prey.push_back((*itr));
		}

		if(!local_prey.empty())
			quarry = local_prey.at(rand() % local_prey.size());
	}
private:
	// The target for this predator. If this is nullptr, the predator will wander until it encounters a prey.
	AbstractVehicle* quarry;
};


// ----------------------------------------------------------------------------
// PlugIn for OpenSteerDemo


class PredatorPreyPlugIn : public PlugIn
{
public:
    PredatorPreyPlugIn() : PlugIn()
	{
		srand((unsigned int)time(NULL));
	}

    const char* name (void) {return "Predator Prey Simulation";}

    float selectionOrderSortKey (void) {return 0.0001f;}

    virtual ~PredatorPreyPlugIn() {} // be more "nice" to avoid a compiler warning

    void open (void)
    {
        // make the database used to accelerate proximity queries
        cyclePD = -1;
        nextPD ();

        // make default-sized flock
        population = 0;
        for (int i = 0; i < 200; i++) addPreyToFlock ();
		for (int i = 0; i < 10; i++) addPredatorToFlock ();

        // initialize camera
        OpenSteerDemo::init3dCamera (*OpenSteerDemo::selectedVehicle);
        OpenSteerDemo::camera.mode = Camera::cmFixed;
        OpenSteerDemo::camera.fixedDistDistance = OpenSteerDemo::cameraTargetDistance;
        OpenSteerDemo::camera.fixedDistVOffset = 0;
        OpenSteerDemo::camera.lookdownDistance = 20;
        OpenSteerDemo::camera.aimLeadTime = 0.5;
        OpenSteerDemo::camera.povOffset.set (0, 0.5, -2);
    }

    void update (const float currentTime, const float elapsedTime)
    {
        // update flock simulation for each Prey
        for (iterator i = flock.begin(); i != flock.end(); i++)
        {
            (**i).update (currentTime, elapsedTime);
        }
    }

    void redraw (const float currentTime, const float elapsedTime)
    {
        // selected vehicle (user can mouse click to select another)
        AbstractVehicle& selected = *OpenSteerDemo::selectedVehicle;

        // vehicle nearest mouse (to be highlighted)
        AbstractVehicle& nearMouse = *OpenSteerDemo::vehicleNearestToMouse ();

        // update camera
        OpenSteerDemo::updateCamera (currentTime, elapsedTime, selected);

        // draw each Prey in flock
        for (iterator i = flock.begin(); i != flock.end(); i++) (**i).draw ();

        // highlight vehicle nearest mouse
        OpenSteerDemo::drawCircleHighlightOnVehicle (nearMouse, 1, gGray70);

        // highlight selected vehicle
        OpenSteerDemo::drawCircleHighlightOnVehicle (selected, 1, gGray50);

        // display status in the upper left corner of the window
        std::ostringstream status;
        status << "[F1/F2] " << population << " Preys";
        status << "\n[F3]    PD type: ";
        switch (cyclePD)
        {
        case 0: status << "LQ bin lattice"; break;
        case 1: status << "brute force";    break;
        }
        status << "\n[F4]    Boundary: ";
        switch (Prey::boundaryCondition)
        {
        case 0: status << "steer back when outside"; break;
        case 1: status << "wrap around (teleport)";  break;
        }
        status << std::endl;
        const float h = drawGetWindowHeight ();
        const Vec3 screenLocation (10, h-50, 0);
        draw2dTextAt2dLocation (status, screenLocation, gGray80);
    }

    void close (void)
    {
        // delete each member of the flock
        while (population > 0) removePreyFromFlock ();

        // delete the proximity database
        delete pd;
        pd = NULL;
    }

    void reset (void)
    {
        // reset each Prey in flock
        for (iterator i = flock.begin(); i != flock.end(); i++) (**i).reset();

        // reset camera position
        OpenSteerDemo::position3dCamera (*OpenSteerDemo::selectedVehicle);

        // make camera jump immediately to new position
        OpenSteerDemo::camera.doNotSmoothNextMove ();
    }

    // for purposes of demonstration, allow cycling through various
    // types of proximity databases.  this routine is called when the
    // OpenSteerDemo user pushes a function key.
    void nextPD (void)
    {
        // save pointer to old PD
        ProximityDatabase* oldPD = pd;

        // allocate new PD
        const int totalPD = 2;
        switch (cyclePD = (cyclePD + 1) % totalPD)
        {
        case 0:
            {
                const Vec3 center;
                const float div = 10.0f;
                const Vec3 divisions (div, div, div);
                const float diameter = Prey::worldRadius * 1.1f * 2;
                const Vec3 dimensions (diameter, diameter, diameter);
                typedef LQProximityDatabase<AbstractVehicle*> LQPDAV;
                pd = new LQPDAV (center, dimensions, divisions);
                break;
            }
        case 1:
            {
                pd = new BruteForceProximityDatabase<AbstractVehicle*> ();
                break;
            }
        }

        // switch each Prey to new PD
        for (iterator i=flock.begin(); i!=flock.end(); i++) (**i).newPD(*pd);

        // delete old PD (if any)
        delete oldPD;
    }

    void handleFunctionKeys (int keyNumber)
    {
        switch (keyNumber)
        {
        case 1:  addPreyToFlock ();               break;
        case 2:  removePreyFromFlock ();          break;
        case 3:  nextPD ();                       break;
        case 4:  Prey::nextBoundaryCondition ();  break;
        }
    }

    void printMiniHelpForFunctionKeys (void)
    {
        std::ostringstream message;
        message << "Function keys handled by ";
        message << '"' << name() << '"' << ':' << std::ends;
        OpenSteerDemo::printMessage (message);
        OpenSteerDemo::printMessage ("  F1     add a Prey to the flock.");
        OpenSteerDemo::printMessage ("  F2     remove a Prey from the flock.");
        OpenSteerDemo::printMessage ("  F3     use next proximity database.");
        OpenSteerDemo::printMessage ("  F4     next flock boundary condition.");
        OpenSteerDemo::printMessage ("");
    }

    void addPreyToFlock (void)
    {
        population++;
        Prey* prey = new Prey (*pd);
        flock.push_back (prey);
        if (population == 1) OpenSteerDemo::selectedVehicle = prey;
    }

    void addPredatorToFlock (void)
    {
        population++;
		Predator* predator = new Predator (*pd);
        flock.push_back (predator);
        if (population == 1) OpenSteerDemo::selectedVehicle = predator;
    }

    void removePreyFromFlock (void)
    {
        if (population > 0)
        {
            // save a pointer to the last Prey, then remove it from the flock
            const Prey* prey = flock.back();
            flock.pop_back();
            population--;

            // if it is OpenSteerDemo's selected vehicle, unselect it
            if (prey == OpenSteerDemo::selectedVehicle)
                OpenSteerDemo::selectedVehicle = NULL;

            // delete the Prey
            delete prey;
        }
    }

    // return an AVGroup containing each Prey of the flock
    const AVGroup& allVehicles (void) {return (const AVGroup&)flock;}

    // flock: a group (STL vector) of pointers to all Preys
    Prey::groupType flock;
    typedef Prey::groupType::const_iterator iterator;

    // pointer to database used to accelerate proximity queries
    ProximityDatabase* pd;

    // keep track of current flock size
    int population;

    // which of the various proximity databases is currently in use
    int cyclePD;
};


PredatorPreyPlugIn gPreyPlugIn;



// ----------------------------------------------------------------------------
