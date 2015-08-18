#define dDOUBLE

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include <map>

//#define NUM 4         // Number of links

//dWorldID    world;       // A dynamic world
/*
dBodyID     link[NUM];    // Links　link[0] is a base
dJointID    joint[NUM];  // Joints    joint[0] is a fixed joint between a base and a ground

static double THETA[NUM] = { 0.0, 0.0, 0.0, 0.0};  // Target joint angels[rad]
static double l[NUM]  = { 0.10, 0.90, 1.00, 1.00};  // Length of links[m]
static double r[NUM]  = { 0.20, 0.04, 0.04, 0.04};  // Radius of links[m]
*/

class DrawBody{ 
public:

    virtual void doDraw() = 0;

    // --------------------------------------------------------------------------
    static const std::map<size_t,DrawBody *> & getDrawMap(){return worldDrawMap;};

    explicit DrawBody(){
        counter++;

        unique_n_counter++;
        unique_n = unique_n_counter;

        worldDrawMap[unique_n] = this;
    }

    ~DrawBody(){
        worldDrawMap[unique_n] = 0;

        counter--;
        if (counter == 0){
            worldDrawMap.clear();
        }
    }

private:
    size_t unique_n;
    static size_t unique_n_counter;

    static size_t counter;
    static std::map<size_t,DrawBody *> worldDrawMap;
};

size_t DrawBody::counter = 0;
size_t DrawBody::unique_n_counter = 0;
std::map<size_t,DrawBody *> DrawBody::worldDrawMap;


typedef struct {
    dBodyID     body;   // a body
    dGeomID     geom; //  a geometry
    dReal       radius, length, mass; // radius[m], length[m], mass[kg]

} dPart;


class Leg{

    friend class LegDraw;

public:

    Leg( const dWorldID & _world){
        createLeg( _world );
    }

protected:

    void createLeg( const dWorldID & _world ){
        // read leg parameters there 
        lower_part.mass = 0.5;
        lower_part.radius = 0.1;
        lower_part.length = 0.5;

        dReal lower_part_pos[3] =  {0.0, 0.0, lower_part.length};

        upper_part.mass = 0.5;
        upper_part.radius = 0.1;
        upper_part.length = 0.5;

        dReal upper_part_pos[3] =  {0.0, 0.0, upper_part.length + lower_part.length};

        createPart(_world,lower_part, lower_part_pos);
        createPart(_world,upper_part, upper_part_pos);

        jointHinge = dJointCreateHinge(_world, 0); // Create a hinge joint
        dJointAttach(jointHinge, lower_part.body, upper_part.body ); // Attach the joint
        dJointSetHingeAnchor(jointHinge, 1.0,0.0,0.0);//anchor_x[j], anchor_y[j],anchor_z[j]);
    }

    void createPart(const dWorldID & _world , dPart & part, dReal * pos3){
        dMass mass;
        part.body = dBodyCreate(_world);
        dBodySetPosition(part.body, pos3[0] , pos3[1], pos3[2]); // Set a position
        dMassSetZero(&mass);                        // Set mass parameter to zero
        dMassSetCapsuleTotal(&mass,part.mass,3,part.radius, part.length);  // Calculate mass parameter
        dBodySetMass(part.body, &mass);  // Set mass
    }

    dPart     upper_part, lower_part;
    dJointID  jointHinge;
};

    // DRAW CLASS METHOD
class LegDraw : public DrawBody{
    public:
        LegDraw( const Leg * _leg ):leg(_leg){
        }
        void doDraw(){
            if (!leg) return;
            dsSetColor(0.0,0.0,1.0);
            dsDrawCapsuleD(dBodyGetPosition(leg->lower_part.body), dBodyGetRotation(leg->lower_part.body), leg->lower_part.length, leg->lower_part.radius);
            dsSetColor(1.0,0.0,0.0);
            dsDrawCapsuleD(dBodyGetPosition(leg->upper_part.body), dBodyGetRotation(leg->upper_part.body), leg->upper_part.length, leg->upper_part.radius);
            //TEST 
            double k1 =  1.0,  fMax  = 10.0; // k1: proportional gain,  fMax：Max torque[Nm]
            //dJointSetHingeParam(jointHinge, dParamVel, k1); // Set angular velocity[m/s]
            //dJointSetHingeParam(jointHinge, dParamFMax, fMax); // Set max torque[N/m]
            printf("\r %6d:",dJointGetHingeAngle(leg->jointHinge)); 
        }
    private:
        const Leg * leg;
    };

void control() {  /***  P control  ****/
    /* static int step = 0;     // Steps of simulation    
    double k1 =  10.0,  fMax  = 100.0; // k1: proportional gain,  fMax：Max torque[Nm]

    printf("\r%6d:",step++);
    for (int j = 1; j < NUM; j++) {
    double tmpAngle = dJointGetHingeAngle(joint[j]);  // Present angle[rad]
    double z = THETA[j] - tmpAngle;  // z: residual=target angle - present angle
    dJointSetHingeParam(joint[j], dParamVel, k1*z); // Set angular velocity[m/s]
    dJointSetHingeParam(joint[j], dParamFMax, fMax); // Set max torque[N/m]
    }
    */
}

void start() { /*** Initialize drawing API ***/
    float xyz[3] = {  3.04f, 1.28f, 0.76f};   // View point x, y, z　[m]
    float hpr[3] = { -160.0f, 4.50f, 0.00f};  // View direction(heading, pitch, roll)　[°]
    dsSetViewpoint(xyz,hpr);   // Set view-point and gaze direction
}

void command(int cmd) { /*** Keyboard function ***/
    /* switch (cmd) {
    case 'i':  THETA[1] += 0.05; break;  // When j key is pressed, THETA[1] is increases at 0.05[rad]
    case 'f':  THETA[1] -= 0.05; break;
    case 'j':  THETA[2] += 0.05; break;
    case 'd':  THETA[2] -= 0.05; break;
    case 'l':  THETA[3] += 0.05; break;
    case 's':  THETA[3] -= 0.05; break;
    }
    if (THETA[1] <   - M_PI)    THETA[1] =  - M_PI;
    if (THETA[1] >     M_PI)    THETA[1] =    M_PI;
    if (THETA[2] <  -2*M_PI/3)  THETA[2] =  - 2*M_PI/3;
    if (THETA[2] >   2*M_PI/3)  THETA[2] =    2*M_PI/3;
    if (THETA[3] <  -2*M_PI/3)  THETA[3] =  - 2*M_PI/3;
    if (THETA[3] >   2*M_PI/3)  THETA[3] =    2*M_PI/3;*/
}

// MAIN -------------------------------------------------------------
// Simulation loop
dWorldID world;

void init() { /*** Initialize drawing API ***/
    float xyz[3] = {  3.04f, 1.28f, 0.76f};   // View point x, y, z　[m]
    float hpr[3] = { -160.0f, 4.50f, 0.00f};  // View direction(heading, pitch, roll)　[°]
    dsSetViewpoint(xyz,hpr);   // Set view-point and gaze direction
}

void simLoop(int pause) {

    //control();

    dWorldStep(world, 0.0001);

    for ( std::map<size_t, DrawBody *>::const_iterator it = DrawBody::getDrawMap().begin(); it != DrawBody::getDrawMap().end(); ++it){
        DrawBody * drawBody = it->second;
        if (drawBody) {
            drawBody->doDraw();
        }
    }
}

int main(int argc, char *argv[]) {
    dsFunctions fn;
    double x[4] = {0.00}, y[4] = {0.00};  // Center of gravity
    double z[4] = { 0.05, 0.50, 1.50, 2.55};

    double m[4] = {10.00, 2.00, 2.00, 2.00};       // mass

    double anchor_x[4]  = {0.00}, anchor_y[4] = {0.00};// anchors of joints
    double anchor_z[4] = { 0.00, 0.10, 1.00, 2.00};

    double axis_x[4]  = { 0.00, 0.00, 0.00, 0.00};  // axises of joints
    double axis_y[4]  = { 0.00, 0.00, 1.00, 1.00};
    double axis_z[4]  = { 1.00, 1.00, 0.00, 0.00};

    fn.version = DS_VERSION;
    fn.start   = &init;
    fn.step    = &simLoop;
    fn.command = &command;
    fn.path_to_textures = "drawstuff_texture";

    dInitODE();  // Initialize ODE
    world = dWorldCreate();  // Create a world
    dWorldSetGravity(world, 0, 0, -9.8);

    bool isDrawingEnabled = true;

    Leg leg(world);
    LegDraw legDraw(&leg);
    
    /*
    for (int i = 0; i < NUM; i++) {
    dMass mass;
    link[i] = dBodyCreate(world);
    dBodySetPosition(link[i], x[i], y[i], z[i]); // Set a position
    dMassSetZero(&mass);      // Set mass parameter to zero
    dMassSetCapsuleTotal(&mass,m[i],3,r[i],l[i]);  // Calculate mass parameter
    dBodySetMass(link[i], &mass);  // Set mass
    }

    joint[0] = dJointCreateFixed(world, 0); // A fixed joint
    dJointAttach(joint[0], link[0], 0);     // Attach the joint between the ground and the base
    dJointSetFixed(joint[0]);               // Set the fixed joint

    for (int j = 1; j < NUM; j++) {
    joint[j] = dJointCreateHinge(world, 0); // Create a hinge joint
    dJointAttach(joint[j], link[j-1], link[j]); // Attach the joint
    dJointSetHingeAnchor(joint[j], anchor_x[j], anchor_y[j],anchor_z[j]);
    dJointSetHingeAxis(joint[j], axis_x[j], axis_y[j], axis_z[j]);
    }*/

    dsSimulationLoop(argc, argv, 640, 480, &fn); //　Simulation loop

    dCloseODE();
    return 0;
}