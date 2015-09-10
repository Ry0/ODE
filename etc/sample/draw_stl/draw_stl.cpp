

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include "StlReader.h"

static dWorldID world;
static dSpaceID space;
static dJointGroupID contactgroup;

static dBodyID body;
static dGeomID geom;


#ifdef dDOUBLE
    #define dsDrawBox dsDrawBoxD
    #define dsDrawTriangle dsDrawTriangleD
#endif

ode_utils::StlReader<float,dTriIndex> *mesh;


void createMeshObj( dWorldID world_, dSpaceID space_, dBodyID &body_, dGeomID &geom_, dReal mass_)
{

    body_ = dBodyCreate( world );

    dTriMeshDataID data;
    data = dGeomTriMeshDataCreate();
    dGeomTriMeshDataBuildSingle(data, mesh->getVertices(), 3*sizeof(float), mesh->getVertexCount(),
        mesh->getIndices(), mesh->getIndexCount(), 3*sizeof(dTriIndex));

    geom_ = dCreateTriMesh(space_, data, 0, 0, 0);
    dGeomSetData( geom_, data );

    dMass m;
    dMassSetTrimesh( &m, mass_, geom_ );
    dGeomSetPosition( geom_, -m.c[0], -m.c[1], -m.c[2] );
    dMassTranslate( &m, -m.c[0], -m.c[1], -m.c[2] );
    dBodySetMass( body_, &m );

    dGeomSetBody( geom_, body_ );
}

// è’ìÀåüèoópä÷êî
static void nearCallback( void *data, dGeomID o1, dGeomID o2 )
{
	const int MAX_CONTACTS = 10;

	dBodyID b1 = dGeomGetBody( o1 ); // ï®ëÃ1
	dBodyID b2 = dGeomGetBody( o2 ); // ï®ëÃ2

	if ( b1 && b2 && dAreConnectedExcluding( b1, b2, dJointTypeContact ) )
	return; // è’ìÀëŒè€Ç≈Ç»Ç¢ï®ëÃÇÃè’ìÀÇÕÇÕÇ∏Ç∑

	dContact contact[MAX_CONTACTS];
	for ( int i=0; i<MAX_CONTACTS; i++ )
	{
        // ï®ëÃìØémÇÃê⁄êGéûÇÃÉpÉâÉÅÅ[É^ê›íË
        contact[i].surface.mode = dContactBounce | dContactSoftCFM | dContactSoftERP;
        contact[i].surface.mu = dInfinity;   // ñÄéCåWêî
        contact[i].surface.mu2 = 0;
        contact[i].surface.bounce = 0.5;     // îΩî≠åWêî
        contact[i].surface.bounce_vel = 0.1;
        contact[i].surface.soft_cfm = 0.0001;  // CFMê›íË
        contact[i].surface.soft_erp = 0.1;
	}

	// è’ìÀåüèo
	int numc = dCollide( o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof( dContact ) );
	if ( numc > 0 )
	{
		for ( int i=0; i<numc; i++ )
		{
			// è’ìÀÇÃî≠ê∂
			dJointID c = dJointCreateContact( world, contactgroup, contact+i );
			dJointAttach( c, b1, b2 );
		}
	}
}



// start simulation - set viewpoint
static void start()
{
  static float xyz[3] = { 0.f, -5.f, 1.f };
  static float hpr[3] = { 90.f, 15.f, 0.f };

  dsSetViewpoint( xyz, hpr );
}

// simulation loop
static void simLoop( int pause )
{
    if (!pause)
    {
        dSpaceCollide( space, 0, &nearCallback );

        dWorldStep( world, 0.01 );

        dJointGroupEmpty( contactgroup );
    }


    dsSetColor( 1, 1, 0 );
    {
        const dReal* pos = dGeomGetPosition(geom);
        const dReal* rot = dGeomGetRotation(geom);

        for (int ii = 0; ii < mesh->getIndexCount()/3; ii++) {
            const dReal v[9] = {
                mesh->getVertices()[mesh->getIndices()[ii*3+0]*3 + 0],
                mesh->getVertices()[mesh->getIndices()[ii*3+0]*3 + 1],
                mesh->getVertices()[mesh->getIndices()[ii*3+0]*3 + 2],
                mesh->getVertices()[mesh->getIndices()[ii*3+1]*3 + 0],
                mesh->getVertices()[mesh->getIndices()[ii*3+1]*3 + 1],
                mesh->getVertices()[mesh->getIndices()[ii*3+1]*3 + 2],
                mesh->getVertices()[mesh->getIndices()[ii*3+2]*3 + 0],
                mesh->getVertices()[mesh->getIndices()[ii*3+2]*3 + 1],
                mesh->getVertices()[mesh->getIndices()[ii*3+2]*3 + 2]
            };
            dsDrawTriangle( pos, rot, &v[0], &v[3], &v[6], 1 );
        }
    }
}


int main( int argc, char* argv[] )
{
    // setup pointers to drawstuff callback functions
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start   = &start;
    fn.step    = &simLoop;
    fn.command = 0;
    fn.stop    = 0;
    fn.path_to_textures = "textures";

    dInitODE();

    // create world
    world = dWorldCreate();
    space = dHashSpaceCreate( 0 );
    contactgroup = dJointGroupCreate( 0 );
    dCreatePlane( space, 0, 0, 1, 0 );
    dWorldSetCFM (world,1e-5);
    dWorldSetERP (world,0.1);
    dWorldSetGravity( world, 0.0, 0.0, -9.8 );


    mesh = new ode_utils::StlReader<float,dTriIndex>("monkey_binary.stl");
    if (!mesh->isCompleted()) {
        printf("NG\n");
        printf("msg:\n%s", mesh->message() );
        exit(1);
    }

    // create MeshObj
    createMeshObj( world, space, body, geom, 1.0 );
    dBodySetPosition( body, 0.0, 0.0, 3.0 );
    dMatrix3 R;
    dRFromAxisAndAngle( R, 1, 0, 0, 90.0);
    dBodySetRotation( body, R);

    // starting simulation
    dsSimulationLoop( argc, argv, 320, 240, &fn );

    dJointGroupDestroy( contactgroup );
    dSpaceDestroy( space );
    dWorldDestroy( world );
    return 0;
}

