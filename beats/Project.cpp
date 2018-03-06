#ifndef _Ogre_H_
#include <ogre.h>
#endif
#include "OIS.h"
#include <SdkCameraMan.h>
//#include <CEGUI.h>
//#include <CEGUIOgreRenderer.h>
#include <Terrain\include\OgreTerrainGroup.h>
#include <Terrain\include\OgreTerrain.h>
#include "btBulletDynamicsCommon.h"
#include "btHeightfieldTerrainShape.h"
//#include "CEGUIManager.h"
#include <vector>
#include <stdlib.h>

using namespace Ogre;

    //function defs
    ManualObject* createCubeMesh(Ogre::String name, Ogre::String matName);


    // this pattern updates the scenenode position when it changes within the bullet simulation
    // taken from BulletMotionState docs page24
    class MyMotionState : public btMotionState {
    public:
        MyMotionState(const btTransform &initialpos, Ogre::SceneNode *node) {
            mVisibleobj = node;
            mPos1 = initialpos;
        }
        virtual ~MyMotionState() {    }
        void setNode(Ogre::SceneNode *node) {
            mVisibleobj = node;
        }
        virtual void getWorldTransform(btTransform &worldTrans) const {
            worldTrans = mPos1;
        }
        virtual void setWorldTransform(const btTransform &worldTrans) {
            if(NULL == mVisibleobj) return; // silently return before we set a node
            btQuaternion rot = worldTrans.getRotation();
            mVisibleobj->setOrientation(rot.w(), rot.x(), rot.y(), rot.z());
            btVector3 pos = worldTrans.getOrigin();
            // TODO **** XXX need to fix this up such that it renders properly since this doesnt know the scale of the node
            // also the getCube function returns a cube that isnt centered on Z
            mVisibleobj->setPosition(pos.x(), pos.y()+5, pos.z()-5);
        }
    protected:
        Ogre::SceneNode *mVisibleobj;
        btTransform mPos1;

    };

	#define BIT(x) (1<<(x))
		enum collisiontypes {
			COL_NOTHING = 0, //<Collide with nothing
			COL_TANK = BIT(0), //<Collide with ships
			COL_WALL = BIT(1), //<Collide with walls
			COL_BULLET = BIT(2), //<Collide with powerups
			COL_ENEMY = BIT(3)
		};
	struct CollisionObject{
		int id;
		bool hit;
		SceneNode* node;
		btRigidBody* body;
		CollisionObject(btRigidBody* b,SceneNode* n,int i) : body(b),node(n),id(i),hit(false) {}
	};


    class Application : public FrameListener, public OIS::KeyListener, public OIS::MouseListener
    {
    public:
		Application(void):mCameraMan(0),mSceneMgr(0)
		{
			tankCollidesWith = COL_WALL | COL_BULLET | COL_ENEMY;
			wallCollidesWith = COL_BULLET | COL_ENEMY;
			bulletCollidesWith = COL_WALL | COL_BULLET | COL_ENEMY; //COL_TANK |
			enemyCollidesWith = COL_WALL | COL_BULLET | COL_ENEMY;
			delay=0;
		}
        void go()
        {
           mContinue = true;
            createRoot();
            defineResources();
            setupRenderSystem();
            createRenderWindow();
            initializeResourceGroups();
            createBulletSim();
			setupScene();
            setupInputSystem();
            //setupCEGUI();
            createFrameListener();
            startRenderLoop();

        }

        ~Application()
        {
            mInputManager->destroyInputObject(mKeyboard);
			mInputManager->destroyInputObject(mMouse);
            OIS::InputManager::destroyInputSystem(mInputManager);

           
            delete mRoot;

            // cleanup bulletdyanmics

           //cleanup in the reverse order of creation/initialization
           //remove the rigidbodies from the dynamics world and delete them
           for (i=dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
           {
              btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
              btRigidBody* body = btRigidBody::upcast(obj);
              if (body && body->getMotionState())
              {
                 delete body->getMotionState();
              }
              dynamicsWorld->removeCollisionObject( obj );
              delete obj;
           }

           //delete collision shapes
           for (int j=0;j<collisionShapes.size();j++)
           {
              btCollisionShape* shape = collisionShapes[j];
			  if(collisionShapes[j]){
				  collisionShapes[j] = 0;
				  delete shape;
			  }
           }

           delete dynamicsWorld;
           delete solver;
           delete overlappingPairCache;
           delete dispatcher;
           delete collisionConfiguration;
		   if (mCameraMan) delete mCameraMan;
        }
	protected:
		int tankCollidesWith;
		int wallCollidesWith;
		int bulletCollidesWith;
		int enemyCollidesWith;


		void CheckCollisions(){
			bool lock=false;
			bool lock2=false;
			int TotalManifolds=dynamicsWorld->getDispatcher()->getNumManifolds();

			for(int i=0;i<TotalManifolds;i++){
				btPersistentManifold* Manifold = dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
				//We're not only going to record if one of the objects was a Box, but actually get the pointer to the box in case want to manipulate it further
				btCollisionObject *BoxObject0(0), *BoxObject1(0);

				//getBody0 and getBody1 access the actual collision objects. The collision group is actually contained in what is called a Broadphase Proxy which is accessed using the
				//getBroadphaseHanlde() method on the body. We will check to see if this collision group is our box group
				//if(Manifold->getBody0()->getBroadphaseHandle()->m_collisionFilterGroup == COL_BULLET)
					BoxObject0 = const_cast<btCollisionObject*>(Manifold->getBody0());

				//Now check the other object
				//if(Manifold->getBody1()->getBroadphaseHandle()->m_collisionFilterGroup == COL_BULLET)
					BoxObject1 = const_cast<btCollisionObject*>(Manifold->getBody1());

				//Here is where we actually execute some special action, even on the box itself, if it is indeed a box (if the pointer to the box object is non-zero or non-NULL)
				//if(BoxObject0){
				//	((CollisionObject*)BoxObject0->getUserPointer())->hit=true;
				//}
				//if(BoxObject1){
				//	((CollisionObject*)BoxObject1->getUserPointer())->hit=true;
				//}

				//Bullet hits wall
				if((((CollisionObject*)BoxObject0->getUserPointer())->id==0)&&(((CollisionObject*)BoxObject1->getUserPointer())->id==3)){
					//((Entity*)((CollisionObject*)BoxObject1->getUserPointer())->node->getAttachedObject(0))->setMaterialName("Custom/Red");
				}
				else if((((CollisionObject*)BoxObject0->getUserPointer())->id==3)&&(((CollisionObject*)BoxObject1->getUserPointer())->id==0)){
					//((Entity*)((CollisionObject*)BoxObject0->getUserPointer())->node->getAttachedObject(0))->setMaterialName("Custom/Red");
				}
				//Bullet hits bullet
				if((((CollisionObject*)BoxObject0->getUserPointer())->id==3)&&(((CollisionObject*)BoxObject1->getUserPointer())->id==3)){
					((Entity*)((CollisionObject*)BoxObject1->getUserPointer())->node->getAttachedObject(0))->setMaterialName("Custom/Blue");
					((Entity*)((CollisionObject*)BoxObject0->getUserPointer())->node->getAttachedObject(0))->setMaterialName("Custom/Blue");
					destroyObject((CollisionObject*)BoxObject1->getUserPointer());
					destroyObject((CollisionObject*)BoxObject0->getUserPointer());
				}
				//Bullet hits gEnemy
				if((((CollisionObject*)BoxObject0->getUserPointer())->id==4)&&(((CollisionObject*)BoxObject1->getUserPointer())->id==3)){
					Vector3 pos=((CollisionObject*)BoxObject0->getUserPointer())->node->getPosition();
					btVector3 btpos=btVector3(pos.x,pos.y,pos.z);
					destroyObject((CollisionObject*)BoxObject1->getUserPointer());
					destroyObject((CollisionObject*)BoxObject0->getUserPointer());
					CreateEnemyRed(btpos);
					CreateEnemyRed(btpos);
				}
				else if((((CollisionObject*)BoxObject0->getUserPointer())->id==3)&&(((CollisionObject*)BoxObject1->getUserPointer())->id==4)){
					Vector3 pos=((CollisionObject*)BoxObject1->getUserPointer())->node->getPosition();
					btVector3 btpos=btVector3(pos.x,pos.y,pos.z);
					destroyObject((CollisionObject*)BoxObject1->getUserPointer());
					destroyObject((CollisionObject*)BoxObject0->getUserPointer());
					CreateEnemyRed(btpos);
					CreateEnemyRed(btpos);
				}
				//Bullet hits rEnemy
				if((((CollisionObject*)BoxObject0->getUserPointer())->id==5)&&(((CollisionObject*)BoxObject1->getUserPointer())->id==3)){
					Vector3 pos=((CollisionObject*)BoxObject0->getUserPointer())->node->getPosition();
					btVector3 btpos=btVector3(pos.x,pos.y,pos.z);
					destroyObject((CollisionObject*)BoxObject1->getUserPointer());
					destroyObject((CollisionObject*)BoxObject0->getUserPointer());

				}
				else if((((CollisionObject*)BoxObject0->getUserPointer())->id==3)&&(((CollisionObject*)BoxObject1->getUserPointer())->id==5)){
					Vector3 pos=((CollisionObject*)BoxObject1->getUserPointer())->node->getPosition();
					btVector3 btpos=btVector3(pos.x,pos.y,pos.z);
					destroyObject((CollisionObject*)BoxObject1->getUserPointer());
					destroyObject((CollisionObject*)BoxObject0->getUserPointer());

				}
				//Enemy hits player
				if((((CollisionObject*)BoxObject0->getUserPointer())->id==-1)&&(((CollisionObject*)BoxObject1->getUserPointer())->id==4))
				{
					if(pLife.size()>0){
					if(!lock){
							removeLife();
							lock=true;
					}
					}
					destroyObject((CollisionObject*)BoxObject1->getUserPointer());
				}
				else if((((CollisionObject*)BoxObject0->getUserPointer())->id==4)&&(((CollisionObject*)BoxObject1->getUserPointer())->id==-1)){
					if(pLife.size()>0){
					if(!lock){
							removeLife();
							lock=true;
					}
					}
					destroyObject((CollisionObject*)BoxObject0->getUserPointer());
				}
				if((((CollisionObject*)BoxObject0->getUserPointer())->id==-1)&&(((CollisionObject*)BoxObject1->getUserPointer())->id==5))
				{
					if(pLife.size()>0){
						if(!lock2){
							removeLife();
							lock2=true;
						}
					}
					destroyObject((CollisionObject*)BoxObject1->getUserPointer());
				}
				else if((((CollisionObject*)BoxObject0->getUserPointer())->id==5)&&(((CollisionObject*)BoxObject1->getUserPointer())->id==-1)){

					if(pLife.size()>0){
						if(!lock2){
							removeLife();
							lock2=true;
						}
					}
					destroyObject((CollisionObject*)BoxObject0->getUserPointer());
				}
			}
		}
    private:
        Root *mRoot;
		
        SceneManager *mSceneMgr;
        OIS::Keyboard *mKeyboard;
		OIS::Mouse *mMouse;
        OIS::InputManager *mInputManager;
//		CEGUIManager* ceguiManager;
		Camera *cam;
        bool mContinue;
	    Ogre::TerrainGlobalOptions* mTerrainGlobals;
        Ogre::TerrainGroup* mTerrainGroup;
        bool mTerrainsImported;
          // scene objects
		ManualObject *cmo;
		OgreBites::SdkCameraMan* mCameraMan;
		int maxBullets;
		int numBullets;
		float delay;

        // bullet dynamics
         int i;
         btDefaultCollisionConfiguration* collisionConfiguration;
         btCollisionDispatcher* dispatcher;
         btBroadphaseInterface* overlappingPairCache;
         btSequentialImpulseConstraintSolver* solver;
         btDiscreteDynamicsWorld* dynamicsWorld;
         btCollisionShape* groundShape;
         btAlignedObjectArray<btCollisionShape*> collisionShapes;
		 std::vector<CollisionObject*> bodies;
		 std::vector<SceneNode*> pLife;

         
	CollisionObject* getTank(){
		for(int i=0;i<bodies.size();i++){
			if(bodies[i]->id==-1)
				return bodies[i];
		}
	} 
	// frame listener
    bool frameStarted(const FrameEvent &evt)
        {
		delay = delay+evt.timeSinceLastFrame;
		static bool mMouseDown = false;     // If a mouse button is depressed
		static Ogre::Real mToggle = 0.0;    // The time left until next toggle
		static Ogre::Real mRotate = 0.13;   // The rotate constant
		static Ogre::Real mMove = 250;      // The movement constant
		bool currMouse = mMouse->getMouseState().buttonDown(OIS::MB_Left);
	
		mMouseDown = currMouse;

		mToggle -= evt.timeSinceLastFrame;
		btTransform Trans;
		Trans.setIdentity();
		Trans=getTank()->body->getWorldTransform();

		//for moving
		Ogre::Vector3 transVector = Ogre::Vector3::ZERO;
		Ogre::Vector3 curPos=Ogre::Vector3::ZERO;
		if (mKeyboard->isKeyDown(OIS::KC_W)) // Forward
		{
		transVector.z -= mMove;
		}
		if (mKeyboard->isKeyDown(OIS::KC_S)) // Backward
		{
		transVector.z += mMove;
		}
		if (mKeyboard->isKeyDown(OIS::KC_A)) // Left - yaw or strafe
		{
			if(mKeyboard->isKeyDown( OIS::KC_LSHIFT ))
			{
			// Yaw left
			mSceneMgr->getSceneNode("Tank")->yaw(Ogre::Degree(mRotate * 10));
			} else {
			transVector.x -= mMove; // Strafe left
			}
		}
		if (mKeyboard->isKeyDown(OIS::KC_D)) // Right - yaw or strafe
		{
			if(mKeyboard->isKeyDown( OIS::KC_LSHIFT ))
			{
				// Yaw right
				mSceneMgr->getSceneNode("Tank")->yaw(Ogre::Degree(-mRotate * 10));
			} else {
				transVector.x += mMove; // Strafe right
			}
		}
		if (mKeyboard->isKeyDown(OIS::KC_U)) // Up
		{
			transVector.y += mMove;
		}
		if (mKeyboard->isKeyDown(OIS::KC_O)) // Down
		{
			transVector.y -= mMove;
		}

		
		mSceneMgr->getSceneNode("Tank")->translate(transVector * evt.timeSinceLastFrame);//, Ogre::Node::TS_LOCAL);
		curPos=getTank()->node->getPosition();
		btVector3 btTransVector=btVector3(curPos.x,curPos.y,curPos.z);
		Trans.setOrigin(btTransVector);
		getTank()->body->setWorldTransform(Trans);
		//getTank()->body->translate(btTransVector * evt.timeSinceLastFrame);

		
          mKeyboard->capture();
		  mMouse->capture();

		  //mCameraMan->frameRenderingQueued(evt);

            // update physics simulation

		if(delay>=7){
			EnemySpawner();
			delay=0;
		}
		CheckCollisions();
		  dynamicsWorld->stepSimulation(evt.timeSinceLastFrame);
		  
            return mContinue;
        }

       // KeyListener
       bool keyPressed(const OIS::KeyEvent &e) {
          switch (e.key) {
                case OIS::KC_ESCAPE:
                    mContinue = false;
                     break;
				case OIS::KC_1:
					EnemySpawner();
					break;
				case OIS::KC_SPACE:
					Vector3 direction = mSceneMgr->getSceneNode("Tank")->_getDerivedOrientation() * Vector3::UNIT_Z;
					Vector3 position = mSceneMgr->getSceneNode("Tank")->getPosition();
					btVector3 pos= btVector3(position.x,position.y,position.z);
					shootBullet(pos,direction);
					numBullets++;
          }
		//mCameraMan->injectKeyDown(e);
          return true;
       }

       bool keyReleased(const OIS::KeyEvent &e) {
		  // CEGUI::System::getSingleton().injectKeyUp(e.key);  
		   mCameraMan->injectKeyUp(e);
		   return true; 
	   }

        void createRoot()
        {
            mRoot = new Root("plugins_d.cfg","ogre.cfg","Ogre.log");
        }

        void defineResources()
        {
            String secName, typeName, archName;
            ConfigFile cf;
            cf.load("resources_d.cfg");

            ConfigFile::SectionIterator seci = cf.getSectionIterator();
            while (seci.hasMoreElements())
            {
                secName = seci.peekNextKey();
                ConfigFile::SettingsMultiMap *settings = seci.getNext();
                ConfigFile::SettingsMultiMap::iterator i;
                for (i = settings->begin(); i != settings->end(); ++i)
                {
                    typeName = i->first;
                    archName = i->second;
                    ResourceGroupManager::getSingleton().addResourceLocation(archName, typeName, secName);
                }
            }
        }

        void setupRenderSystem()
        {
            if (!mRoot->restoreConfig() && !mRoot->showConfigDialog())
                throw Exception(52, "User canceled the config dialog!", "Application::setupRenderSystem()");
        }

        void createRenderWindow()
        {
            mRoot->initialise(true, "Tutorial Render Window");
        }

        void initializeResourceGroups()
        {
            TextureManager::getSingleton().setDefaultNumMipmaps(5);
            ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
        }


	void buildStage(){

		//ground
		Ogre::Plane plane(Ogre::Vector3::UNIT_Y, 0);
		Ogre::MeshManager::getSingleton().createPlane("ground", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		plane, 1000,1000, 20, 20, true, 1, 5, 5, Ogre::Vector3::UNIT_Z);
		Ogre::Entity* entGround = mSceneMgr->createEntity("GroundEntity", "ground");
		SceneNode* gNode=mSceneMgr->getRootSceneNode()->createChildSceneNode();
		gNode->attachObject(entGround);
		entGround->setMaterialName("Examples/BumpyMetal");
		entGround->setCastShadows(false);
		{
		btTransform Transform;
		Transform.setIdentity();
		Transform.setOrigin(btVector3(0,0,0));
		btDefaultMotionState *MotionState = new  btDefaultMotionState(Transform);
		btCollisionShape *basePlaneShape = new btStaticPlaneShape(btVector3(0,1,0), 0); 
		btVector3 LocalInertia;
		basePlaneShape->calculateLocalInertia(0,LocalInertia);
		btRigidBody *basePlaneBody = new btRigidBody(0,MotionState, basePlaneShape,LocalInertia);
		basePlaneBody->setFriction(0);
		basePlaneBody->setCollisionShape(basePlaneShape);
		
		dynamicsWorld->addRigidBody(basePlaneBody);

		bodies.push_back(new CollisionObject(basePlaneBody,gNode,1));
		basePlaneBody->setUserPointer(bodies[bodies.size()-1]);

		collisionShapes.push_back(basePlaneShape);

		}
		//Pillars
		//btVector3 size=btVector3(50,250,50);
		//btVector3 pos1=btVector3(0,50,0);
		//CreateCube(pos1,0,size,"pillar1");

		//walls
		Ogre::Plane Nwall(Vector3::UNIT_Y, 0);
		Ogre::MeshManager::getSingleton().createPlane("topWall", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		Nwall, 1000,100, 20, 20, true, 1, 5, 5, Ogre::Vector3::UNIT_Z);
		Ogre::Entity* entWall1 = mSceneMgr->createEntity("NorthWall", "topWall");
		Ogre::SceneNode* nWall = mSceneMgr->getRootSceneNode()->createChildSceneNode();
		nWall->attachObject(entWall1);
		entWall1->setMaterialName("Custom/BrushedMetal");
		nWall->pitch(Ogre::Degree(90));
		nWall->setPosition(0,50,-500);
		{
		Ogre::AxisAlignedBox boundingB = entWall1->getBoundingBox();
		//boundingB.scale(Vector3(scale.getX(),scale.getY(),scale.getZ()));
		Vector3 size=Vector3(1000,50,2);
		//size = boundingB.getSize()*0.95f;
		btTransform Transform;
		Transform.setIdentity();
		Transform.setOrigin(btVector3(0,50,-500));
		MyMotionState *MotionState = new MyMotionState(Transform,nWall);
	   //Give the rigid body half the size
       // of our cube and tell it to create a BoxShape (cube)
       btVector3 HalfExtents(size.x,size.y,size.z);
       btCollisionShape *wall1 = new btBoxShape(HalfExtents);
	   btVector3 LocalInertia;
       wall1->calculateLocalInertia(0, LocalInertia);
	   btRigidBody *wall1body = new btRigidBody(0, MotionState, wall1, LocalInertia);
	   wall1body->setRestitution(0.75f);
	   wall1body->setFriction(0);
       // Store a pointer to the Ogre Node so we can update it later
       //wall1body->setUserPointer((void *) (nWall));

		dynamicsWorld->addRigidBody(wall1body);
		bodies.push_back(new CollisionObject(wall1body,nWall,0));
		wall1body->setUserPointer(bodies[bodies.size()-1]);

		collisionShapes.push_back(wall1);
		}

		Ogre::Plane Swall(Vector3::UNIT_Y, 0);
		Ogre::MeshManager::getSingleton().createPlane("BottomWall", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		Swall, 1000,100, 20, 20, true, 1, 5, 5, Ogre::Vector3::UNIT_Z);
		Ogre::Entity* entWall2 = mSceneMgr->createEntity("SouthWall", "BottomWall");
		Ogre::SceneNode* sWall = mSceneMgr->getRootSceneNode()->createChildSceneNode();
		sWall->attachObject(entWall2);
		entWall2->setMaterialName("Custom/MetalNet");
		sWall->pitch(Ogre::Degree(90));
		sWall->setPosition(0,50,500);
		Vector3 Ssize=Vector3(1000,50,2);
		createRigidbody(0,Ssize,sWall);

		Ogre::Plane Ewall(Vector3::UNIT_Y, 0);
		Ogre::MeshManager::getSingleton().createPlane("RightWall", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		Ewall, 1000,100, 20, 20, true, 1, 5, 5, Ogre::Vector3::UNIT_Z);
		Ogre::Entity* entWall3 = mSceneMgr->createEntity("EastWall", "RightWall");
		Ogre::SceneNode* eWall = mSceneMgr->getRootSceneNode()->createChildSceneNode();
		eWall->attachObject(entWall3);
		entWall3->setMaterialName("Custom/MetalNet");
		eWall->pitch(Ogre::Degree(90));
		eWall->roll(Ogre::Degree(90));
		eWall->setPosition(500,50,0);
		Vector3 Esize=Vector3(2,50,1000);
		createRigidbody(0,Esize,eWall);

		Ogre::Plane Wwall(Vector3::UNIT_Y, 0);
		Ogre::MeshManager::getSingleton().createPlane("LeftWall", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		Wwall, 1000,100, 20, 20, true, 1, 5, 5, Ogre::Vector3::UNIT_Z);
		Ogre::Entity* entWall4 = mSceneMgr->createEntity("WestWall", "LeftWall");
		Ogre::SceneNode* wWall = mSceneMgr->getRootSceneNode()->createChildSceneNode();
		wWall->attachObject(entWall4);
		entWall4->setMaterialName("Custom/MetalNet");
		wWall->pitch(Ogre::Degree(90));
		wWall->roll(Ogre::Degree(-90));
		wWall->setPosition(-500,50,0);
		Vector3 Wsize=Vector3(2,50,1000);
		createRigidbody(0,Wsize,wWall);

		}
	void createRigidbody(btScalar mass, Vector3 size, Ogre::SceneNode *node){
		btTransform Transform;
		Transform.setIdentity();
		Vector3 origin= Vector3(node->getPosition());
		btVector3 btOrigin=btVector3(origin.x,origin.y,origin.z);
		Transform.setOrigin(btOrigin);
		MyMotionState *MotionState = new MyMotionState(Transform,node);

       btVector3 HalfExtents(size.x,size.y,size.z);
       btCollisionShape *shape = new btBoxShape(HalfExtents);
	   btVector3 LocalInertia;
       shape->calculateLocalInertia(0, LocalInertia);
	   btRigidBody *Rigidbody = new btRigidBody(mass, MotionState, shape, LocalInertia);
	   Rigidbody->setRestitution(0.75f);
	   Rigidbody->setFriction(0);
       // Store a pointer to the Ogre Node so we can update it later
      //Rigidbody->setUserPointer((void *) (node));
	   dynamicsWorld->addRigidBody(Rigidbody,COL_WALL,wallCollidesWith);
	   bodies.push_back(new CollisionObject(Rigidbody,node,0));
		Rigidbody->setUserPointer(bodies[bodies.size()-1]);

		collisionShapes.push_back(shape);
	}
    void CreateCube(const btVector3 &Position, btScalar Mass,const btVector3 &scale,char * name){
		// empty ogre vectors for the cubes size and position
       Ogre::Vector3 size = Ogre::Vector3::ZERO;
       Ogre::Vector3 pos = Ogre::Vector3::ZERO;
       SceneNode *boxNode;
	   Entity *boxentity;
       // Convert the bullet physics vector to the ogre vector
       pos.x = Position.getX();
       pos.y = Position.getY();
       pos.z = Position.getZ();
       boxentity = mSceneMgr->createEntity(name, "cube.mesh");
	   //boxentity->setScale(Vector3(scale.x,scale.y,scale.z));
       boxentity->setCastShadows(true);
       boxNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
       boxNode->attachObject(boxentity);
	   boxNode->scale(Vector3(scale.getX(),scale.getY(),scale.getZ()));
       //boxNode->setScale(Vector3(0.1,0.1,0.1));
       Ogre::AxisAlignedBox boundingB = boxentity->getBoundingBox();
	   //Ogre::AxisAlignedBox boundingB = boxNode->_getWorldAABB();
	   boundingB.scale(Vector3(scale.getX(),scale.getY(),scale.getZ()));
	   size = boundingB.getSize()*0.95f;
	   btTransform Transform;
       Transform.setIdentity();
       Transform.setOrigin(Position);
	   MyMotionState *MotionState = new MyMotionState(Transform,boxNode);
	   //Give the rigid body half the size
       // of our cube and tell it to create a BoxShape (cube)
       btVector3 HalfExtents(size.x*0.5f,size.y*0.5f,size.z*0.5f);
       btCollisionShape *Shape = new btBoxShape(HalfExtents);
	   btVector3 LocalInertia;
       Shape->calculateLocalInertia(Mass, LocalInertia);
	   btRigidBody *RigidBody = new btRigidBody(Mass, MotionState, Shape, LocalInertia);

       // Store a pointer to the Ogre Node so we can update it later
       //RigidBody->setUserPointer((void *) (boxNode));

    // Add it to the physics world
        dynamicsWorld->addRigidBody(RigidBody);

		bodies.push_back(new CollisionObject(RigidBody,boxNode,2));
		RigidBody->setUserPointer(bodies[bodies.size()-1]);
		collisionShapes.push_back(Shape);
	}

	btRigidBody* CreateBullet(const btVector3 &Position, btScalar Mass,const btVector3 &scale){
		// empty ogre vectors for the cubes size and position
       Ogre::Vector3 size = Ogre::Vector3::ZERO;
       Ogre::Vector3 pos = Ogre::Vector3::ZERO;
       SceneNode *boxNode;
	   Entity *boxentity;
       // Convert the bullet physics vector to the ogre vector
       pos.x = Position.getX();
       pos.y = Position.getY();
       pos.z = Position.getZ();
       //boxentity = mSceneMgr->createEntity("cube.mesh");
	   boxentity = mSceneMgr->createEntity(Ogre::SceneManager::PT_SPHERE);
	   //boxentity->setScale(Vector3(scale.x,scale.y,scale.z));
       boxentity->setCastShadows(true);
       boxNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
       boxNode->attachObject(boxentity);
	   boxNode->scale(Vector3(scale.getX(),scale.getY(),scale.getZ()));
       //boxNode->setScale(Vector3(0.1,0.1,0.1));
       Ogre::AxisAlignedBox boundingB = boxentity->getBoundingBox();
	   //Ogre::AxisAlignedBox boundingB = boxNode->_getWorldAABB();
	   boundingB.scale(Vector3(scale.getX(),scale.getY(),scale.getZ()));
	   size = boundingB.getSize()*0.95f;
	   btTransform Transform;
       Transform.setIdentity();
       Transform.setOrigin(Position);
	   MyMotionState *MotionState = new MyMotionState(Transform,boxNode);
	   //Give the rigid body half the size
       // of our cube and tell it to create a BoxShape (cube)
       btVector3 HalfExtents(size.x*0.5f,size.y*0.5f,size.z*0.5f);
       btCollisionShape *Shape = new btBoxShape(HalfExtents);
	   btVector3 LocalInertia;
       Shape->calculateLocalInertia(Mass, LocalInertia);
	   btRigidBody *RigidBody = new btRigidBody(Mass, MotionState, Shape, LocalInertia);
	   RigidBody->setRestitution(1.0f);
	   RigidBody->setFriction(0);
       // Store a pointer to the Ogre Node so we can update it later
      // RigidBody->setUserPointer((void *) (boxNode));

    // Add it to the physics world
        dynamicsWorld->addRigidBody(RigidBody,COL_BULLET,bulletCollidesWith);
		bodies.push_back(new CollisionObject(RigidBody,boxNode,3));
		RigidBody->setUserPointer(bodies[bodies.size()-1]);
		collisionShapes.push_back(Shape);
		return RigidBody;
	}
	Vector3 getRandomDirection(){
		std::vector<Vector3> dir(8);
		dir[0]=Vector3(1,0,0);
		dir[1]=Vector3(0,0,1);
		dir[2]=Vector3(1,0,1);
		dir[3]=Vector3(-1,0,0);
		dir[4]=Vector3(1,0,-1);
		dir[5]=Vector3(-1,0,-1);
		dir[6]=Vector3(0,0,-1);
		dir[7]=Vector3(-1,0,1);
		int r= rand()%8;
		return dir[r];

	}
	  void destroyObject(CollisionObject * ptrToOgreObject)
	        {
				// delete the ogre aspect of the object
				// detach the entity from the parent sceneNode, destroy the entity, destroy the sceneNode, and set the sceneNode to NULL
				Entity* ent= ((Entity*) ptrToOgreObject->node->getAttachedObject(0));
				ptrToOgreObject->node->getAttachedObject(0)->detachFromParent();
				mSceneMgr->destroyEntity(ent);
	            mSceneMgr->destroySceneNode(ptrToOgreObject->node);
	            ptrToOgreObject->node = NULL;
	                                    
	            // delete the bullet aspect of the object, ours should always have motion state
	            if(ptrToOgreObject->body && ptrToOgreObject->body->getMotionState())
					delete ptrToOgreObject->body->getMotionState();
	            
				delete ptrToOgreObject->body->getCollisionShape();

				dynamicsWorld->removeCollisionObject(ptrToOgreObject->body);
	                                        
				ptrToOgreObject->body = NULL;
	   
	                                        
				delete ptrToOgreObject;                     
				ptrToOgreObject = NULL;

				//bodies.erase(*ptrToOgreObject);
	  }

	void shootBullet(const btVector3& Position,Vector3 Direction){
		//Define our bullet speed
		static float Speed = 100.f;
		//Create the projectile box at the camera's parent node position.
		btRigidBody* BoxBody = CreateBullet(Position,1.0f,btVector3(0.25,0.25,0.25));

		//Vector conversion
		btVector3 FireVelocity = btVector3(Direction.x, Direction.y,Direction.z);

		//Now as we discussed above, we want our vector to have a certain speed. We first normalize it, and then multiply it by Speed
		FireVelocity.normalize();
		FireVelocity *= (Speed*10);

		//Now we finally propel our box
		BoxBody->setLinearVelocity(FireVelocity); //Remember that accelerations were game units? So are velocities
	}
	void AddVelocity(btRigidBody* body,float speed,Vector3 Direction){
		//Vector conversion
		btVector3 FireVelocity = btVector3(Direction.x, Direction.y,Direction.z);

		//Now as we discussed above, we want our vector to have a certain speed. We first normalize it, and then multiply it by Speed
		FireVelocity.normalize();
		FireVelocity *= (speed*10);

		//Now we finally propel our box
		body->setLinearVelocity(FireVelocity); //Remember that accelerations were game units? So are velocities
	}
	void EnemySpawner(){
		//cannot use btVector3 for the vector because of alignment issue
			std::vector<Vector3> spawns(4);
			spawns[0]=Vector3(300,10,300);
			spawns[1]=Vector3(300,10,-300);
			spawns[2]=Vector3(-300,10,300);
			spawns[3]=Vector3(-300,10,-300);
			int pos=rand()%4;
			btVector3 loc=btVector3(spawns[pos].x,spawns[pos].y,spawns[pos].z);
			CreateEnemyGreen(loc);
	}
	void CreateEnemyGreen(const btVector3 &Position){
		// empty ogre vectors for the cubes size and position
       Ogre::Vector3 size = Ogre::Vector3::ZERO;
       Ogre::Vector3 pos = Ogre::Vector3::ZERO;
       SceneNode *boxNode;
	   Entity *boxentity;
       // Convert the bullet physics vector to the ogre vector
       pos.x = Position.getX();
       pos.y = Position.getY();
       pos.z = Position.getZ();
       boxentity = mSceneMgr->createEntity("cube.mesh");
       boxentity->setCastShadows(true);
	   boxentity->setMaterialName("Custom/Green");
       boxNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
       boxNode->attachObject(boxentity);
	   boxNode->scale(Vector3(1,1,1));
       Ogre::AxisAlignedBox boundingB = boxentity->getBoundingBox();
	   boundingB.scale(Vector3(1,1,1));
	   size = boundingB.getSize()*0.95f;
	   btTransform Transform;
       Transform.setIdentity();
       Transform.setOrigin(Position);
	   MyMotionState *MotionState = new MyMotionState(Transform,boxNode);
	   //Give the rigid body half the size
       // of our cube and tell it to create a BoxShape (cube)
       btVector3 HalfExtents(size.x*0.5f,size.y*0.5f,size.z*0.5f);
       btCollisionShape *Shape = new btBoxShape(HalfExtents);
	   btVector3 LocalInertia;
       Shape->calculateLocalInertia(1, LocalInertia);
	   btRigidBody *RigidBody = new btRigidBody(1, MotionState, Shape, LocalInertia);
	   RigidBody->setRestitution(1.0f);
	   RigidBody->setFriction(0);
    // Add it to the physics world
        dynamicsWorld->addRigidBody(RigidBody,COL_ENEMY,enemyCollidesWith);
		bodies.push_back(new CollisionObject(RigidBody,boxNode,4));
		RigidBody->setUserPointer(bodies[bodies.size()-1]);
		collisionShapes.push_back(Shape);

		AddVelocity(RigidBody,100,getRandomDirection());
		}
	void CreateEnemyRed(const btVector3 &Position){
	// empty ogre vectors for the cubes size and position
       Ogre::Vector3 size = Ogre::Vector3::ZERO;
       Ogre::Vector3 pos = Ogre::Vector3::ZERO;
       SceneNode *boxNode;
	   Entity *boxentity;
       // Convert the bullet physics vector to the ogre vector
       pos.x = Position.getX();
       pos.y = Position.getY();
       pos.z = Position.getZ();
       boxentity = mSceneMgr->createEntity("cube.mesh");
       boxentity->setCastShadows(true);
	   boxentity->setMaterialName("Custom/Red");
       boxNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
       boxNode->attachObject(boxentity);
	   boxNode->scale(Vector3(0.5,0.5,0.5));
       Ogre::AxisAlignedBox boundingB = boxentity->getBoundingBox();
	   boundingB.scale(Vector3(0.5,0.5,0.5));
	   size = boundingB.getSize()*0.95f;
	   btTransform Transform;
       Transform.setIdentity();
       Transform.setOrigin(Position);
	   MyMotionState *MotionState = new MyMotionState(Transform,boxNode);
	   //Give the rigid body half the size
       // of our cube and tell it to create a BoxShape (cube)
       btVector3 HalfExtents(size.x*0.5f,size.y*0.5f,size.z*0.5f);
       btCollisionShape *Shape = new btBoxShape(HalfExtents);
	   btVector3 LocalInertia;
       Shape->calculateLocalInertia(1, LocalInertia);
	   btRigidBody *RigidBody = new btRigidBody(1, MotionState, Shape, LocalInertia);
	   RigidBody->setRestitution(1.0f);
	   RigidBody->setFriction(0);
	 
    // Add it to the physics world
        dynamicsWorld->addRigidBody(RigidBody,COL_ENEMY,enemyCollidesWith);
		bodies.push_back(new CollisionObject(RigidBody,boxNode,5));
		RigidBody->setUserPointer(bodies[bodies.size()-1]);
		collisionShapes.push_back(Shape);

		AddVelocity(RigidBody,100,getRandomDirection());
	}
//----------------------------------------------------------------------------------------------------------------
	void initLife(){
		//pLife.resize(5);
		
		//Life
		Ogre::Entity* life1=mSceneMgr->createEntity("cube.mesh");
		Ogre::SceneNode* LifeNode1=mSceneMgr->getRootSceneNode()->createChildSceneNode(Vector3(-500,200,-600));
		life1->setMaterialName("Custom/Yellow");
		LifeNode1->attachObject(life1);
		pLife.push_back(LifeNode1);

		Ogre::Entity* life2=mSceneMgr->createEntity("cube.mesh");
		Ogre::SceneNode* LifeNode2=mSceneMgr->getRootSceneNode()->createChildSceneNode(Vector3(-250,200,-600));
		life2->setMaterialName("Custom/Yellow");
		LifeNode2->attachObject(life2);
		pLife.push_back(LifeNode2);

		Ogre::Entity* life3=mSceneMgr->createEntity("cube.mesh");
		Ogre::SceneNode* LifeNode3=mSceneMgr->getRootSceneNode()->createChildSceneNode(Vector3(0,200,-600));
		life3->setMaterialName("Custom/Yellow");
		LifeNode3->attachObject(life3);
		pLife.push_back(LifeNode3);

		Ogre::Entity* life4=mSceneMgr->createEntity("cube.mesh");
		Ogre::SceneNode* LifeNode4=mSceneMgr->getRootSceneNode()->createChildSceneNode(Vector3(250,200,-600));
		life4->setMaterialName("Custom/Yellow");
		LifeNode4->attachObject(life4);
		pLife.push_back(LifeNode4);

		Ogre::Entity* life5=mSceneMgr->createEntity("cube.mesh");
		Ogre::SceneNode* LifeNode5=mSceneMgr->getRootSceneNode()->createChildSceneNode(Vector3(500,200,-600));
		life5->setMaterialName("Custom/Yellow");
		LifeNode5->attachObject(life5);
		pLife.push_back(LifeNode5);
	}
	void removeLife(){
		SceneNode* temp=pLife.back();
		Entity* ent= ((Entity*) temp->getAttachedObject(0));
		temp->getAttachedObject(0)->detachFromParent();
		mSceneMgr->destroyEntity(ent);
	    mSceneMgr->destroySceneNode(temp);
		pLife.pop_back();
	}
    void setupScene()
        {
            mSceneMgr = mRoot->createSceneManager(ST_GENERIC, "Default SceneManager");
			// Create the camera
			cam = mSceneMgr->createCamera("PlayerCam");

			cam->setPosition(Ogre::Vector3(0, 750, 1000));
			cam->lookAt(Ogre::Vector3(0,0,0));
			cam->setNearClipDistance(0.1);
			cam->setFarClipDistance(50000);

			mCameraMan = new OgreBites::SdkCameraMan(cam);   // create a default camera controller

			Viewport *vp = mRoot->getAutoCreatedWindow()->addViewport(cam);

			initLife();

			//"Tank"
			Ogre::Entity* ogreHead = mSceneMgr->createEntity("Head", "ogrehead.mesh");
			Ogre::SceneNode* tNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("Tank",Ogre::Vector3(0,20,0));
			//tNode->yaw(Ogre::Degree(180));
			tNode->attachObject(ogreHead);

			Ogre::AxisAlignedBox boundingB = ogreHead->getBoundingBox();
		   Vector3 size = boundingB.getSize()*0.95f;
		   btTransform Transform;
		   Transform.setIdentity();
		   Transform.setOrigin(btVector3(0,20,0));
		   MyMotionState *MotionState = new MyMotionState(Transform,tNode);
		   
		   btVector3 HalfExtents(size.x*0.5f,size.y*0.5f,size.z*0.5f);
		   btCollisionShape *Shape = new btBoxShape(HalfExtents);
		   btVector3 LocalInertia;
		   Shape->calculateLocalInertia(0, LocalInertia);
		   btRigidBody *RigidBody = new btRigidBody(0, MotionState, Shape, LocalInertia);
		   RigidBody->setRestitution(1.0f);
		   RigidBody->setFriction(0);
		   
		// Add it to the physics world
			dynamicsWorld->addRigidBody(RigidBody);//,COL_TANK,tankCollidesWith);

			bodies.push_back(new CollisionObject(RigidBody,tNode,-1));
			RigidBody->setUserPointer(bodies[bodies.size()-1]);
			collisionShapes.push_back(Shape);

			//Light
			Ogre::Light* directionalLight = mSceneMgr->createLight("directionalLight");
			directionalLight->setType(Ogre::Light::LT_DIRECTIONAL);
			directionalLight->setDiffuseColour(Ogre::ColourValue(.25, .25, 0));
			directionalLight->setSpecularColour(Ogre::ColourValue(.25, .25, 0));
			directionalLight->setDirection(Ogre::Vector3( 0, -1, 1 ));


            if (mRoot->getRenderSystem()->getCapabilities()->hasCapability(Ogre::RSC_INFINITE_FAR_PLANE))
            {
                  cam->setFarClipDistance(0);   // enable infinite far clip distance if we can
             }
  

            mSceneMgr->setAmbientLight(ColourValue(.5, .5, .5));
            mSceneMgr->setShadowTechnique( SHADOWTYPE_STENCIL_ADDITIVE );
			//cmo = createCubeMesh("mcube", "");
			//cmo->convertToMesh("cube");

          		  buildStage();

        }
	/*CEGUI::MouseButton convertButton(OIS::MouseButtonID buttonID)
        {
			switch (buttonID)
			{
			case OIS::MB_Left:
				return CEGUI::LeftButton;
 
			case OIS::MB_Right:
				return CEGUI::RightButton;
 
			case OIS::MB_Middle:
				return CEGUI::MiddleButton;
 
			default:
				return CEGUI::LeftButton;
		EnemySpawner()
			pos=randomNum()1-4

		CreateEnemyGreen(pos){

		}
			}
        }*/
		bool mouseMoved( const OIS::MouseEvent &arg )
		{
			//CEGUI::System &sys = CEGUI::System::getSingleton();
			//sys.injectMouseMove(arg.state.X.rel, arg.state.Y.rel);
			// Scroll wheel.
			//if (arg.state.Z.rel)
			//sys.injectMouseWheelChange(arg.state.Z.rel / 120.0f);
			//mCameraMan->injectMouseMove(arg);
			return true;
		}
		//-------------------------------------------------------------------------------------
		bool mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id )
		{
			//CEGUI::System::getSingleton().injectMouseButtonDown(convertButton(id));
			mCameraMan->injectMouseDown(arg, id);
			return true;
		}
		//-------------------------------------------------------------------------------------
		bool mouseReleased( const OIS::MouseEvent &arg, OIS::MouseButtonID id )
		{
			//CEGUI::System::getSingleton().injectMouseButtonUp(convertButton(id));
			mCameraMan->injectMouseUp(arg, id);
			return true;
		}
//-------------------------------------------------------------------------------------
        void setupInputSystem()
        {
            size_t windowHnd = 0;
            std::ostringstream windowHndStr;
            OIS::ParamList pl;
            RenderWindow *win = mRoot->getAutoCreatedWindow();

            win->getCustomAttribute("WINDOW", &windowHnd);
            windowHndStr << windowHnd;
            pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));
            mInputManager = OIS::InputManager::createInputSystem(pl);

            try
            {
                 mKeyboard = static_cast<OIS::Keyboard*>(mInputManager->createInputObject(OIS::OISKeyboard, true));
				 mMouse = static_cast<OIS::Mouse*>(mInputManager->createInputObject( OIS::OISMouse, true ));
                 mKeyboard->setEventCallback(this);
				 mMouse->setEventCallback(this);
                // mMouse = static_cast<OIS::Mouse*>(mInputManager->createInputObject(OIS::OISMouse, false));
                //mJoy = static_cast<OIS::JoyStick*>(mInputManager->createInputObject(OIS::OISJoyStick, false));
            }
            catch (const OIS::Exception &e)
            {
                throw new Exception(42, e.eText, "Application::setupInputSystem");
            }
        }
		
   //     void setupCEGUI()
   //     {
			//
   //         // Other CEGUI setup here.
			//Ogre::TexturePtr tex = mRoot->getTextureManager()->createManual( "RTT",Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
   //                              Ogre::TEX_TYPE_2D,512,512, 0,Ogre::PF_R8G8B8,Ogre::TU_RENDERTARGET);
			// CEGUI::SubscriberSlot  evnt= CEGUI::Event::Subscriber(&Application::quit, this);
   //          ceguiManager = ceguiManager->getSingleton();
   //          ceguiManager->initialize( evnt,tex);
   //     }
		
        void createFrameListener()
        {
            mRoot->addFrameListener(this);
        }

        void startRenderLoop()
        {
            mRoot->startRendering();
        }

        void createBulletSim(void) {
             ///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
             collisionConfiguration = new btDefaultCollisionConfiguration();

             ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
             dispatcher = new   btCollisionDispatcher(collisionConfiguration);

             ///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
             overlappingPairCache = new btDbvtBroadphase();

             ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
             solver = new btSequentialImpulseConstraintSolver;

             dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,overlappingPairCache,solver,collisionConfiguration);
             dynamicsWorld->setGravity(btVector3(0,-1500,0));


        }
    };



    #if OGRE_PLATFORM == PLATFORM_WIN32 || OGRE_PLATFORM == OGRE_PLATFORM_WIN32
    #define WIN32_LEAN_AND_MEAN
    #include "windows.h"

    INT WINAPI WinMain(HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT)
    #else
    int main(int argc, char **argv)
    #endif
    {
        try
        {
            Application app;
            app.go();
        }
        catch(Exception& e)
        {
    #if OGRE_PLATFORM == PLATFORM_WIN32 || OGRE_PLATFORM == OGRE_PLATFORM_WIN32
            MessageBoxA(NULL, e.getFullDescription().c_str(), "An exception has occurred!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
    #else
            fprintf(stderr, "An exception has occurred: %s\n",
                e.getFullDescription().c_str());
    #endif
        }

        return 0;
    }



    // make a cube, no cube primiatives in ogre
    // yanked from betajaen
    //http://www.ogre3d.org/forums/viewtopic.php?p=301318&sid=ce193664e1d3d7c4af509e6f4e2718c6
    ManualObject* createCubeMesh(Ogre::String name, Ogre::String matName) {

       ManualObject* cube = new ManualObject(name);

       cube->begin(matName);

       cube->position(0.5,-0.5,1.0);cube->normal(0.408248,-0.816497,0.408248);cube->textureCoord(1,0);
       cube->position(-0.5,-0.5,0.0);cube->normal(-0.408248,-0.816497,-0.408248);cube->textureCoord(0,1);
       cube->position(0.5,-0.5,0.0);cube->normal(0.666667,-0.333333,-0.666667);cube->textureCoord(1,1);
       cube->position(-0.5,-0.5,1.0);cube->normal(-0.666667,-0.333333,0.666667);cube->textureCoord(0,0);
       cube->position(0.5,0.5,1.0);cube->normal(0.666667,0.333333,0.666667);cube->textureCoord(1,0);
       cube->position(-0.5,-0.5,1.0);cube->normal(-0.666667,-0.333333,0.666667);cube->textureCoord(0,1);
       cube->position(0.5,-0.5,1.0);cube->normal(0.408248,-0.816497,0.408248);cube->textureCoord(1,1);
       cube->position(-0.5,0.5,1.0);cube->normal(-0.408248,0.816497,0.408248);cube->textureCoord(0,0);
       cube->position(-0.5,0.5,0.0);cube->normal(-0.666667,0.333333,-0.666667);cube->textureCoord(0,1);
       cube->position(-0.5,-0.5,0.0);cube->normal(-0.408248,-0.816497,-0.408248);cube->textureCoord(1,1);
       cube->position(-0.5,-0.5,1.0);cube->normal(-0.666667,-0.333333,0.666667);cube->textureCoord(1,0);
       cube->position(0.5,-0.5,0.0);cube->normal(0.666667,-0.333333,-0.666667);cube->textureCoord(0,1);
       cube->position(0.5,0.5,0.0);cube->normal(0.408248,0.816497,-0.408248);cube->textureCoord(1,1);
       cube->position(0.5,-0.5,1.0);cube->normal(0.408248,-0.816497,0.408248);cube->textureCoord(0,0);
       cube->position(0.5,-0.5,0.0);cube->normal(0.666667,-0.333333,-0.666667);cube->textureCoord(1,0);
       cube->position(-0.5,-0.5,0.0);cube->normal(-0.408248,-0.816497,-0.408248);cube->textureCoord(0,0);
	   cube->position(-0.5,0.5,1.0);cube->normal(-0.408248,0.816497,0.408248);cube->textureCoord(1,0);
       cube->position(0.5,0.5,0.0);cube->normal(0.408248,0.816497,-0.408248);cube->textureCoord(0,1);
       cube->position(-0.5,0.5,0.0);cube->normal(-0.666667,0.333333,-0.666667);cube->textureCoord(1,1);
       cube->position(0.5,0.5,1.0);cube->normal(0.666667,0.333333,0.666667);cube->textureCoord(0,0);

       cube->triangle(0,1,2);      cube->triangle(3,1,0);
       cube->triangle(4,5,6);      cube->triangle(4,7,5);
       cube->triangle(8,9,10);      cube->triangle(10,7,8);
       cube->triangle(4,11,12);   cube->triangle(4,13,11);
       cube->triangle(14,8,12);   cube->triangle(14,15,8);
       cube->triangle(16,17,18);   cube->triangle(16,19,17);
       cube->end();

       return cube;
    }

