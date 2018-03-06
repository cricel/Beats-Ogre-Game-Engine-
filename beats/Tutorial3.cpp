#include "Tutorial3.h"

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
		if (NULL == mVisibleobj) return; // silently return before we set a node
		btQuaternion rot = worldTrans.getRotation();
		mVisibleobj->setOrientation(rot.w(), rot.x(), rot.y(), rot.z());
		btVector3 pos = worldTrans.getOrigin();
		// TODO **** XXX need to fix this up such that it renders properly since this doesnt know the scale of the node
		// also the getCube function returns a cube that isnt centered on Z
		mVisibleobj->setPosition(pos.x(), pos.y() + 5, pos.z() - 5);
	}
protected:
	Ogre::SceneNode *mVisibleobj;
	btTransform mPos1;
};

Ogre::Vector3 getRandomDirection(){
	std::vector<Ogre::Vector3> dir(1);
	dir[0] = Ogre::Vector3(0, 0, 1);
	return dir[0];

}

 
TutorialApplication::TutorialApplication()
  : mTerrainGroup(0),
    mTerrainGlobals(0),
    mInfoLabel(0)
{
	tankCollidesWith = COL_WALL | COL_BULLET | COL_ENEMY;
	wallCollidesWith = COL_BULLET | COL_ENEMY;
	bulletCollidesWith = COL_WALL | COL_BULLET | COL_ENEMY; //COL_TANK |
	enemyCollidesWith = COL_WALL | COL_BULLET | COL_ENEMY;
	delay = 0;
}
 
TutorialApplication::~TutorialApplication()
{
	CEGUI::OgreRenderer::destroySystem();
	
}

CollisionObject* TutorialApplication::getTank(){
	for (int i = 0; i<bodies.size(); i++){
		if (bodies[i]->id == -1)
			return bodies[i];
	}
}

void TutorialApplication::destroyObject(CollisionObject * ptrToOgreObject)
{
	// delete the ogre aspect of the object
	// detach the entity from the parent sceneNode, destroy the entity, destroy the sceneNode, and set the sceneNode to NULL
	Ogre::Entity* ent = ((Ogre::Entity*)ptrToOgreObject->node->getAttachedObject(0));
	ptrToOgreObject->node->getAttachedObject(0)->detachFromParent();
	mSceneMgr->destroyEntity(ent);
	mSceneMgr->destroySceneNode(ptrToOgreObject->node);
	ptrToOgreObject->node = NULL;

	// delete the bullet aspect of the object, ours should always have motion state
	if (ptrToOgreObject->body && ptrToOgreObject->body->getMotionState())
		delete ptrToOgreObject->body->getMotionState();

	delete ptrToOgreObject->body->getCollisionShape();

	dynamicsWorld->removeCollisionObject(ptrToOgreObject->body);

	ptrToOgreObject->body = NULL;


	delete ptrToOgreObject;
	ptrToOgreObject = NULL;

	//bodies.erase(*ptrToOgreObject);
}


void TutorialApplication::CheckCollisions(){
	bool lock = false;
	bool lock2 = false;
	int TotalManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();

	for (int i = 0; i<TotalManifolds; i++){
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
		//gain life
		if ((((CollisionObject*)BoxObject0->getUserPointer())->id == 3) && (((CollisionObject*)BoxObject1->getUserPointer())->id == -2)){
			if (mKeyboard->isKeyDown(OIS::KC_Z))
			{
				hitsound();
				gainLife();
				destroyObject((CollisionObject*)BoxObject1->getUserPointer());
			}
			else{
				destroyObject((CollisionObject*)BoxObject1->getUserPointer());
				misssound();
			}
		}
		else if ((((CollisionObject*)BoxObject0->getUserPointer())->id == -2) && (((CollisionObject*)BoxObject1->getUserPointer())->id == 3)){
			if (mKeyboard->isKeyDown(OIS::KC_Z))
			{
				hitsound();
				gainLife();
				destroyObject((CollisionObject*)BoxObject0->getUserPointer());
			}
			else{
				destroyObject((CollisionObject*)BoxObject0->getUserPointer());
				misssound();
			}

		}
		//Enemy hits Key--X
		if ((((CollisionObject*)BoxObject0->getUserPointer())->id == 4) && (((CollisionObject*)BoxObject1->getUserPointer())->id == -2)){
			Ogre::Vector3 pos = ((CollisionObject*)BoxObject0->getUserPointer())->node->getPosition();
			btVector3 btpos = btVector3(pos.x, pos.y, pos.z);
			if (mKeyboard->isKeyDown(OIS::KC_X))
			{
				hitsound();
				gainLife();
				destroyObject((CollisionObject*)BoxObject1->getUserPointer());
			}
			else{
				destroyObject((CollisionObject*)BoxObject1->getUserPointer());
				misssound();
			}
		}
		else if ((((CollisionObject*)BoxObject0->getUserPointer())->id == -2) && (((CollisionObject*)BoxObject1->getUserPointer())->id == 4)){
			Ogre::Vector3 pos = ((CollisionObject*)BoxObject0->getUserPointer())->node->getPosition();
			btVector3 btpos = btVector3(pos.x, pos.y, pos.z);
			if (mKeyboard->isKeyDown(OIS::KC_X))
			{
				hitsound();
				gainLife();
				destroyObject((CollisionObject*)BoxObject0->getUserPointer());
			}
			else{
				destroyObject((CollisionObject*)BoxObject0->getUserPointer());
				misssound();
			}

		}
		//Enemy hits Key--SPACE
		if ((((CollisionObject*)BoxObject0->getUserPointer())->id == 5) && (((CollisionObject*)BoxObject1->getUserPointer())->id == -2)){
			Ogre::Vector3 pos = ((CollisionObject*)BoxObject0->getUserPointer())->node->getPosition();
			btVector3 btpos = btVector3(pos.x, pos.y, pos.z);
			if (mKeyboard->isKeyDown(OIS::KC_SPACE))
			{
				hitsound();
				gainLife();
				destroyObject((CollisionObject*)BoxObject1->getUserPointer());
			}
			else{
				destroyObject((CollisionObject*)BoxObject1->getUserPointer());
				misssound();
			}
		}
		else if ((((CollisionObject*)BoxObject0->getUserPointer())->id == -2) && (((CollisionObject*)BoxObject1->getUserPointer())->id == 5)){
			Ogre::Vector3 pos = ((CollisionObject*)BoxObject0->getUserPointer())->node->getPosition();
			btVector3 btpos = btVector3(pos.x, pos.y, pos.z);
			if (mKeyboard->isKeyDown(OIS::KC_SPACE))
			{
				hitsound();
				gainLife();
				destroyObject((CollisionObject*)BoxObject0->getUserPointer());
			}
			else{
				destroyObject((CollisionObject*)BoxObject0->getUserPointer());
				misssound();
			}

		}
		//Enemy hits Key--C
		if ((((CollisionObject*)BoxObject0->getUserPointer())->id == 6) && (((CollisionObject*)BoxObject1->getUserPointer())->id == -2)){
			Ogre::Vector3 pos = ((CollisionObject*)BoxObject0->getUserPointer())->node->getPosition();
			btVector3 btpos = btVector3(pos.x, pos.y, pos.z);
			if (mKeyboard->isKeyDown(OIS::KC_C))
			{
				hitsound();
				gainLife();
				destroyObject((CollisionObject*)BoxObject1->getUserPointer());
			}
			else{
				destroyObject((CollisionObject*)BoxObject1->getUserPointer());
				misssound();
			}
		}
		else if ((((CollisionObject*)BoxObject0->getUserPointer())->id == -2) && (((CollisionObject*)BoxObject1->getUserPointer())->id == 6)){
			Ogre::Vector3 pos = ((CollisionObject*)BoxObject0->getUserPointer())->node->getPosition();
			btVector3 btpos = btVector3(pos.x, pos.y, pos.z);
			if (mKeyboard->isKeyDown(OIS::KC_C))
			{
				hitsound();
				gainLife();
				destroyObject((CollisionObject*)BoxObject0->getUserPointer());
			}
			else{
				destroyObject((CollisionObject*)BoxObject0->getUserPointer());
				misssound();
			}

		}
		//Enemy hits Key--V
		if ((((CollisionObject*)BoxObject0->getUserPointer())->id == 7) && (((CollisionObject*)BoxObject1->getUserPointer())->id == -2)){
			Ogre::Vector3 pos = ((CollisionObject*)BoxObject0->getUserPointer())->node->getPosition();
			btVector3 btpos = btVector3(pos.x, pos.y, pos.z);
			if (mKeyboard->isKeyDown(OIS::KC_V))
			{
				hitsound();
				gainLife();
				destroyObject((CollisionObject*)BoxObject1->getUserPointer());
			}
			else{
				destroyObject((CollisionObject*)BoxObject1->getUserPointer());
				misssound();
			}
		}
		else if ((((CollisionObject*)BoxObject0->getUserPointer())->id == -2) && (((CollisionObject*)BoxObject1->getUserPointer())->id == 7)){
			Ogre::Vector3 pos = ((CollisionObject*)BoxObject0->getUserPointer())->node->getPosition();
			btVector3 btpos = btVector3(pos.x, pos.y, pos.z);
			if (mKeyboard->isKeyDown(OIS::KC_V))
			{
				hitsound();
				gainLife();
				destroyObject((CollisionObject*)BoxObject0->getUserPointer());
			}
			else{
				destroyObject((CollisionObject*)BoxObject0->getUserPointer());
				misssound();
			}

		}
		/////////////////////////////
		////                     ////
		/////////////////////////////
		//Enemy hits Key--Z
		if ((((CollisionObject*)BoxObject0->getUserPointer())->id == 3) && (((CollisionObject*)BoxObject1->getUserPointer())->id == 2)){
			if (mKeyboard->isKeyDown(OIS::KC_Z)) 
			{
				hitsound();
				gameScore = gameScore + 10*diff;
				destroyObject((CollisionObject*)BoxObject1->getUserPointer());
			}
			else{
				if (pLife.size()>0){
					if (!lock){
						removeLife();
						lock = true;
					}
				}
				destroyObject((CollisionObject*)BoxObject1->getUserPointer());
				misssound();
			}
		}
		else if ((((CollisionObject*)BoxObject0->getUserPointer())->id == 2) && (((CollisionObject*)BoxObject1->getUserPointer())->id == 3)){
			if (mKeyboard->isKeyDown(OIS::KC_Z)) 
			{
				gameScore = gameScore + 10*diff;
				destroyObject((CollisionObject*)BoxObject0->getUserPointer());
				hitsound();
			}
			else{
				if (pLife.size()>0){
					if (!lock){
						removeLife();
						lock = true;
					}
				}
				destroyObject((CollisionObject*)BoxObject0->getUserPointer());
				misssound();
			}
			
		}
		//Enemy hits Key--X
		if ((((CollisionObject*)BoxObject0->getUserPointer())->id == 4) && (((CollisionObject*)BoxObject1->getUserPointer())->id == 2)){
			Ogre::Vector3 pos = ((CollisionObject*)BoxObject0->getUserPointer())->node->getPosition();
			btVector3 btpos = btVector3(pos.x, pos.y, pos.z);
			if (mKeyboard->isKeyDown(OIS::KC_X))
			{
				gameScore = gameScore + 10*diff;
				destroyObject((CollisionObject*)BoxObject1->getUserPointer());
				hitsound();
			}
			else{
				if (pLife.size()>0){
					if (!lock){
						removeLife();
						lock = true;
					}
				}
				destroyObject((CollisionObject*)BoxObject1->getUserPointer());
				misssound();
			}
		}
		else if ((((CollisionObject*)BoxObject0->getUserPointer())->id == 2) && (((CollisionObject*)BoxObject1->getUserPointer())->id == 4)){
			Ogre::Vector3 pos = ((CollisionObject*)BoxObject0->getUserPointer())->node->getPosition();
			btVector3 btpos = btVector3(pos.x, pos.y, pos.z);
			if (mKeyboard->isKeyDown(OIS::KC_X))
			{
				gameScore = gameScore + 10*diff;
				destroyObject((CollisionObject*)BoxObject0->getUserPointer());
				hitsound();
			}
			else{
				if (pLife.size()>0){
					if (!lock){
						removeLife();
						lock = true;
					}
				}
				destroyObject((CollisionObject*)BoxObject0->getUserPointer());
				misssound();
			}

		}
		//Enemy hits Key--SPACE
		if ((((CollisionObject*)BoxObject0->getUserPointer())->id == 5) && (((CollisionObject*)BoxObject1->getUserPointer())->id == 2)){
			Ogre::Vector3 pos = ((CollisionObject*)BoxObject0->getUserPointer())->node->getPosition();
			btVector3 btpos = btVector3(pos.x, pos.y, pos.z);
			if (mKeyboard->isKeyDown(OIS::KC_SPACE))
			{
				gameScore = gameScore + 10*diff;
				destroyObject((CollisionObject*)BoxObject1->getUserPointer());
				hitsound();
			}
			else{
				if (pLife.size()>0){
					if (!lock){
						removeLife();
						lock = true;
					}
				}
				destroyObject((CollisionObject*)BoxObject1->getUserPointer());
				misssound();
			}
		}
		else if ((((CollisionObject*)BoxObject0->getUserPointer())->id == 2) && (((CollisionObject*)BoxObject1->getUserPointer())->id == 5)){
			Ogre::Vector3 pos = ((CollisionObject*)BoxObject0->getUserPointer())->node->getPosition();
			btVector3 btpos = btVector3(pos.x, pos.y, pos.z);
			if (mKeyboard->isKeyDown(OIS::KC_SPACE))
			{
				gameScore = gameScore + 10*diff;
				destroyObject((CollisionObject*)BoxObject0->getUserPointer());
				hitsound();
			}
			else{
				if (pLife.size()>0){
					if (!lock){
						removeLife();
						lock = true;
					}
				}
				destroyObject((CollisionObject*)BoxObject0->getUserPointer());
				misssound();
			}

		}
		//Enemy hits Key--C
		if ((((CollisionObject*)BoxObject0->getUserPointer())->id == 6) && (((CollisionObject*)BoxObject1->getUserPointer())->id == 2)){
			Ogre::Vector3 pos = ((CollisionObject*)BoxObject0->getUserPointer())->node->getPosition();
			btVector3 btpos = btVector3(pos.x, pos.y, pos.z);
			if (mKeyboard->isKeyDown(OIS::KC_C))
			{
				gameScore = gameScore + 10*diff;
				destroyObject((CollisionObject*)BoxObject1->getUserPointer());
				hitsound();
			}
			else{
				if (pLife.size()>0){
					if (!lock){
						removeLife();
						lock = true;
					}
				}
				destroyObject((CollisionObject*)BoxObject1->getUserPointer());
				misssound();
			}
		}
		else if ((((CollisionObject*)BoxObject0->getUserPointer())->id == 2) && (((CollisionObject*)BoxObject1->getUserPointer())->id == 6)){
			Ogre::Vector3 pos = ((CollisionObject*)BoxObject0->getUserPointer())->node->getPosition();
			btVector3 btpos = btVector3(pos.x, pos.y, pos.z);
			if (mKeyboard->isKeyDown(OIS::KC_C))
			{
				gameScore = gameScore + 10*diff;
				destroyObject((CollisionObject*)BoxObject0->getUserPointer());
				hitsound();
			}
			else{
				if (pLife.size()>0){
					if (!lock){
						removeLife();
						lock = true;
					}
				}
				destroyObject((CollisionObject*)BoxObject0->getUserPointer());
				misssound();
			}

		}
		//Enemy hits Key--V
		if ((((CollisionObject*)BoxObject0->getUserPointer())->id == 7) && (((CollisionObject*)BoxObject1->getUserPointer())->id == 2)){
			Ogre::Vector3 pos = ((CollisionObject*)BoxObject0->getUserPointer())->node->getPosition();
			btVector3 btpos = btVector3(pos.x, pos.y, pos.z);
			if (mKeyboard->isKeyDown(OIS::KC_V))
			{
				gameScore = gameScore + 10*diff;
				destroyObject((CollisionObject*)BoxObject1->getUserPointer());
				hitsound();
			}
			else{
				if (pLife.size()>0){
					if (!lock){
						removeLife();
						lock = true;
					}
				}
				destroyObject((CollisionObject*)BoxObject1->getUserPointer());
				misssound();
			}
		}
		else if ((((CollisionObject*)BoxObject0->getUserPointer())->id == 2) && (((CollisionObject*)BoxObject1->getUserPointer())->id == 7)){
			Ogre::Vector3 pos = ((CollisionObject*)BoxObject0->getUserPointer())->node->getPosition();
			btVector3 btpos = btVector3(pos.x, pos.y, pos.z);
			if (mKeyboard->isKeyDown(OIS::KC_V))
			{
				gameScore = gameScore + 10*diff;
				destroyObject((CollisionObject*)BoxObject0->getUserPointer());
				hitsound();
			}
			else{
				if (pLife.size()>0){
					if (!lock){
						removeLife();
						lock = true;
					}
				}
				destroyObject((CollisionObject*)BoxObject0->getUserPointer());
				misssound();
			}

		}
	}
}
void TutorialApplication::CreateCube(const btVector3 &Position, btScalar Mass, const btVector3 &scale, char * name){
	// empty ogre vectors for the cubes size and position
	Ogre::Vector3 size = Ogre::Vector3::ZERO;
	Ogre::Vector3 pos = Ogre::Vector3::ZERO;
	Ogre::SceneNode *boxNode;
	Ogre::Entity *boxentity;
	// Convert the bullet physics vector to the ogre vector
	pos.x = Position.getX();
	pos.y = Position.getY();
	pos.z = Position.getZ();
	boxentity = mSceneMgr->createEntity(name, "cube.mesh");
	//boxentity->setScale(Vector3(scale.x,scale.y,scale.z));
	boxentity->setCastShadows(true);
	boxNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	boxNode->attachObject(boxentity);
	boxNode->scale(Ogre::Vector3(scale.getX(), scale.getY(), scale.getZ()));
	//boxNode->setScale(Vector3(0.1,0.1,0.1));
	Ogre::AxisAlignedBox boundingB = boxentity->getBoundingBox();
	//Ogre::AxisAlignedBox boundingB = boxNode->_getWorldAABB();
	boundingB.scale(Ogre::Vector3(scale.getX(), scale.getY(), scale.getZ()));
	size = boundingB.getSize()*0.95f;
	btTransform Transform;
	Transform.setIdentity();
	Transform.setOrigin(Position);
	MyMotionState *MotionState = new MyMotionState(Transform, boxNode);
	//Give the rigid body half the size
	// of our cube and tell it to create a BoxShape (cube)
	btVector3 HalfExtents(size.x*0.5f, size.y*0.5f, size.z*0.5f);
	btCollisionShape *Shape = new btBoxShape(HalfExtents);
	btVector3 LocalInertia;
	Shape->calculateLocalInertia(Mass, LocalInertia);
	btRigidBody *RigidBody = new btRigidBody(Mass, MotionState, Shape, LocalInertia);

	// Store a pointer to the Ogre Node so we can update it later
	RigidBody->setUserPointer((void *)(boxNode));

	// Add it to the physics world
	dynamicsWorld->addRigidBody(RigidBody);
	collisionShapes.push_back(Shape);
}
void TutorialApplication::createBulletSim(void) {
	///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
	collisionConfiguration = new btDefaultCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	dispatcher = new   btCollisionDispatcher(collisionConfiguration);

	///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
	overlappingPairCache = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	solver = new btSequentialImpulseConstraintSolver;

	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
	dynamicsWorld->setGravity(btVector3(0, -100, 0));
	
	std::vector<Ogre::Vector3> test(5);
	test[0] = Ogre::Vector3(-400, 10, 470);
	test[1] = Ogre::Vector3(-200, 10, 470);
	test[2] = Ogre::Vector3(0, 10, 470);
	test[3] = Ogre::Vector3(200, 10, 470);
	test[4] = Ogre::Vector3(400, 10, 470);
	btVector3 loc1 = btVector3(test[0].x, test[0].y, test[0].z);
	btVector3 loc2 = btVector3(test[1].x, test[1].y, test[1].z);
	btVector3 loc3 = btVector3(test[2].x, test[2].y, test[2].z);
	btVector3 loc4 = btVector3(test[3].x, test[3].y, test[3].z);
	btVector3 loc5 = btVector3(test[4].x, test[4].y, test[4].z);

	CreateEnemyRed(loc1, 3);
	CreateEnemyRed(loc2, 4);
	CreateEnemyRed(loc3, 5);
	CreateEnemyRed(loc4, 6);
	CreateEnemyRed(loc5, 7);

  }




void TutorialApplication::createScene()
{
	//CEGUI
	mRenderer = &CEGUI::OgreRenderer::bootstrapSystem();

	CEGUI::ImageManager::setImagesetDefaultResourceGroup("Imagesets");
	CEGUI::Font::setDefaultResourceGroup("Fonts");
	CEGUI::Scheme::setDefaultResourceGroup("Schemes");
	CEGUI::WidgetLookManager::setDefaultResourceGroup("LookNFeel");
	CEGUI::WindowManager::setDefaultResourceGroup("Layouts");
	CEGUI::WindowManager& wmgr = CEGUI::WindowManager::getSingleton();
	CEGUI::SchemeManager::getSingleton().createFromFile("TaharezLook.scheme");
	CEGUI::System::getSingleton().getDefaultGUIContext().getMouseCursor().setDefaultImage("TaharezLook/MouseArrow");
	CEGUI::Window* myRoot = CEGUI::WindowManager::getSingleton().createWindow("DefaultWindow", "_MasterRoot");
	CEGUI::System::getSingleton().getDefaultGUIContext().setRootWindow(myRoot);

	//button sign windows
	//button topleft
	CEGUI::Window*temp0 = CEGUI::WindowManager::getSingleton().createWindow("TaharezLook/StaticText", "temp0");
	temp0->setProperty("HorzFormatting", "WordWrapLeftAligned");
	myRoot->addChild(temp0);
	temp0->setPosition(CEGUI::UVector2(CEGUI::UDim(0.02f, 0.0f), CEGUI::UDim(0.25f, 0.0f)));
	temp0->setSize(CEGUI::USize(CEGUI::UDim(0.21f, 0.0f), CEGUI::UDim(0.1f, 0.0f)));
	temp0->setText("Press 'P' for on/off sound effect");
	//button topright
	CEGUI::Window*temp00 = CEGUI::WindowManager::getSingleton().createWindow("TaharezLook/StaticText", "temp00");
	temp00->setProperty("HorzFormatting", "WordWrapLeftAligned");
	myRoot->addChild(temp00);
	temp00->setPosition(CEGUI::UVector2(CEGUI::UDim(0.77f, 0.0f), CEGUI::UDim(0.25f, 0.0f)));
	temp00->setSize(CEGUI::USize(CEGUI::UDim(0.21f, 0.0f), CEGUI::UDim(0.1f, 0.0f)));
	temp00->setText("Press 'UP'/'DOWN' for speed up/down");
	//button z
	CEGUI::Window*temp = CEGUI::WindowManager::getSingleton().createWindow("TaharezLook/StaticText", "temp");
	temp->setProperty("HorzFormatting", "WordWrapLeftAligned");
	myRoot->addChild(temp);
	temp->setPosition(CEGUI::UVector2(CEGUI::UDim(0.15f, 0.0f), CEGUI::UDim(0.85f, 0.0f)));
	temp->setSize(CEGUI::USize(CEGUI::UDim(0.05f, 0.0f), CEGUI::UDim(0.05f, 0.0f)));
	temp->setText("Z");
	//button x
	CEGUI::Window*temp1 = CEGUI::WindowManager::getSingleton().createWindow("TaharezLook/StaticText", "temp1");
	temp1->setProperty("HorzFormatting", "WordWrapLeftAligned");
	myRoot->addChild(temp1);
	temp1->setPosition(CEGUI::UVector2(CEGUI::UDim(0.31f, 0.0f), CEGUI::UDim(0.85f, 0.0f)));
	temp1->setSize(CEGUI::USize(CEGUI::UDim(0.05f, 0.0f), CEGUI::UDim(0.05f, 0.0f)));
	temp1->setText("X");
	//button space
	CEGUI::Window*temp2 = CEGUI::WindowManager::getSingleton().createWindow("TaharezLook/StaticText", "temp2");
	temp2->setProperty("HorzFormatting", "WordWrapLeftAligned");
	myRoot->addChild(temp2);
	temp2->setPosition(CEGUI::UVector2(CEGUI::UDim(0.48f, 0.0f), CEGUI::UDim(0.85f, 0.0f)));
	temp2->setSize(CEGUI::USize(CEGUI::UDim(0.05f, 0.0f), CEGUI::UDim(0.05f, 0.0f)));
	temp2->setText("SPACE");
	//button c
	CEGUI::Window*temp3 = CEGUI::WindowManager::getSingleton().createWindow("TaharezLook/StaticText", "temp3");
	temp3->setProperty("HorzFormatting", "WordWrapLeftAligned");
	myRoot->addChild(temp3);
	temp3->setPosition(CEGUI::UVector2(CEGUI::UDim(0.64f, 0.0f), CEGUI::UDim(0.85f, 0.0f)));
	temp3->setSize(CEGUI::USize(CEGUI::UDim(0.05f, 0.0f), CEGUI::UDim(0.05f, 0.0f)));
	temp3->setText("C");
	//button v
	CEGUI::Window*temp4 = CEGUI::WindowManager::getSingleton().createWindow("TaharezLook/StaticText", "temp4");
	temp4->setProperty("HorzFormatting", "WordWrapLeftAligned");
	myRoot->addChild(temp4);
	temp4->setPosition(CEGUI::UVector2(CEGUI::UDim(0.80f, 0.0f), CEGUI::UDim(0.85f, 0.0f)));
	temp4->setSize(CEGUI::USize(CEGUI::UDim(0.05f, 0.0f), CEGUI::UDim(0.05f, 0.0f)));
	temp4->setText("V");

	//make score window
	scr = CEGUI::WindowManager::getSingleton().createWindow("TaharezLook/StaticText", "scrVal");
	scr->setProperty("HorzFormatting", "WordWrapLeftAligned");
	myRoot->addChild(scr);
	scr->setPosition(CEGUI::UVector2(CEGUI::UDim(0.02f, 0.0f), CEGUI::UDim(0.05f, 0.0f)));
	scr->setSize(CEGUI::USize(CEGUI::UDim(0.1f, 0.0f), CEGUI::UDim(0.1f, 0.0f)));
	gameScore = 0;
	scr->setText(" Score: " + std::to_string(gameScore));

	//make timer window
	tim = CEGUI::WindowManager::getSingleton().createWindow("TaharezLook/StaticText", "timVal");
	tim->setProperty("HorzFormatting", "WordWrapLeftAligned");
	myRoot->addChild(tim);
	tim->setPosition(CEGUI::UVector2(CEGUI::UDim(0.88f, 0.0f), CEGUI::UDim(0.05f, 0.0f)));
	tim->setSize(CEGUI::USize(CEGUI::UDim(0.1f, 0.0f), CEGUI::UDim(0.1f, 0.0f)));
	gameTimer = 240;
	tim->setText(" Timer: " + std::to_string(gameTimer));

	//setup gameover screen
	over = CEGUI::WindowManager::getSingleton().createWindow("TaharezLook/StaticText", "overVal");
	over->setProperty("HorzFormatting", "WordWrapCenterAligned");
	myRoot->addChild(over);
	over->setPosition(CEGUI::UVector2(CEGUI::UDim(0.25f, 0.0f), CEGUI::UDim(0.25f, 0.0f)));
	over->setSize(CEGUI::USize(CEGUI::UDim(0.5f, 0.0f), CEGUI::UDim(0.5f, 0.0f)));
	over->hide();

	//setup difficulty choosing screen
	pick = CEGUI::WindowManager::getSingleton().createWindow("TaharezLook/StaticText", "pickVal");
	pick->setProperty("HorzFormatting", "WordWrapCenterAligned");
	myRoot->addChild(pick);
	pick->setPosition(CEGUI::UVector2(CEGUI::UDim(0.25f, 0.0f), CEGUI::UDim(0.25f, 0.0f)));
	pick->setSize(CEGUI::USize(CEGUI::UDim(0.5f, 0.0f), CEGUI::UDim(0.5f, 0.0f)));
	pick->hide();

	mCamera->setPosition(Ogre::Vector3(0, 750, 1000));
	mCamera->lookAt(Ogre::Vector3(0, 0, 0));
	mCamera->setNearClipDistance(0.1);
	mCamera->setFarClipDistance(50000);

	//"Tank"
	Ogre::Entity* ogreHead = mSceneMgr->createEntity("Head", "athene.mesh");
	Ogre::SceneNode* tNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("Tank", Ogre::Vector3(1000, 1000, 1000));
	//tNode->yaw(Ogre::Degree(180));
	tNode->attachObject(ogreHead);

	Ogre::AxisAlignedBox boundingB = ogreHead->getBoundingBox();
	Ogre::Vector3 size = boundingB.getSize()*0.95f;
	btTransform Transform;
	Transform.setIdentity();
	Transform.setOrigin(btVector3(0, 20, 0));
	MyMotionState *MotionState = new MyMotionState(Transform, tNode);

	btVector3 HalfExtents(size.x*0.5f, size.y*0.5f, size.z*0.5f);
	btCollisionShape *Shape = new btBoxShape(HalfExtents);
	btVector3 LocalInertia;
	Shape->calculateLocalInertia(0, LocalInertia);
	btRigidBody *RigidBody = new btRigidBody(0, MotionState, Shape, LocalInertia);
	RigidBody->setRestitution(1.0f);
	RigidBody->setFriction(0);
	createBulletSim();
	// Add it to the physics world
	dynamicsWorld->addRigidBody(RigidBody);//,COL_TANK,tankCollidesWith);

	bodies.push_back(new CollisionObject(RigidBody, tNode, -1));
	RigidBody->setUserPointer(bodies[bodies.size() - 1]);
	collisionShapes.push_back(Shape);

	//Light
	Ogre::Light* directionalLight = mSceneMgr->createLight("directionalLight");
	directionalLight->setType(Ogre::Light::LT_DIRECTIONAL);
	directionalLight->setDiffuseColour(Ogre::ColourValue(.25, .25, 0));
	directionalLight->setSpecularColour(Ogre::ColourValue(.25, .25, 0));
	directionalLight->setDirection(Ogre::Vector3(0, -1, 1));

	mSceneMgr->setAmbientLight(Ogre::ColourValue(.5, .5, .5));
	mSceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_ADDITIVE);
  bool infiniteClip =
    mRoot->getRenderSystem()->getCapabilities()->hasCapability(
      Ogre::RSC_INFINITE_FAR_PLANE);
 
  if (infiniteClip)
    mCamera->setFarClipDistance(0);
  else
    mCamera->setFarClipDistance(50000);
  
 
  //ground
  Ogre::Plane plane(Ogre::Vector3::UNIT_Y, 0);
  Ogre::MeshManager::getSingleton().createPlane("ground", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
	  plane, 1000, 1000, 20, 20, true, 1, 5, 5, Ogre::Vector3::UNIT_Z);
  Ogre::Entity* entGround = mSceneMgr->createEntity("GroundEntity", "ground");
  Ogre::SceneNode* gNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
  gNode->attachObject(entGround);
  entGround->setMaterialName("Examples/BumpyMetal");
  entGround->setCastShadows(false);
  {
	  btTransform Transform;
	  Transform.setIdentity();
	  Transform.setOrigin(btVector3(0, 0, 0));
	  btDefaultMotionState *MotionState = new  btDefaultMotionState(Transform);
	  btCollisionShape *basePlaneShape = new btStaticPlaneShape(btVector3(0, 1, 0), 0);
	  btVector3 LocalInertia;
	  basePlaneShape->calculateLocalInertia(0, LocalInertia);
	  btRigidBody *basePlaneBody = new btRigidBody(0, MotionState, basePlaneShape, LocalInertia);
	  basePlaneBody->setFriction(0);
	  basePlaneBody->setCollisionShape(basePlaneShape);

	  dynamicsWorld->addRigidBody(basePlaneBody);

	  bodies.push_back(new CollisionObject(basePlaneBody, gNode, 1));
	  basePlaneBody->setUserPointer(bodies[bodies.size() - 1]);

	  collisionShapes.push_back(basePlaneShape);

  }

  //walls
  Ogre::Plane Nwall(Ogre::Vector3::UNIT_Y, 0);
  Ogre::MeshManager::getSingleton().createPlane("topWall", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
	  Nwall, 1000, 100, 20, 20, true, 1, 5, 5, Ogre::Vector3::UNIT_Z);
  Ogre::Entity* entWall1 = mSceneMgr->createEntity("NorthWall", "topWall");
  Ogre::SceneNode* nWall = mSceneMgr->getRootSceneNode()->createChildSceneNode();
  nWall->attachObject(entWall1);
  entWall1->setMaterialName("Custom/BrushedMetal");
  nWall->pitch(Ogre::Degree(90));
  nWall->setPosition(0, 50, -500);
  {
	  Ogre::AxisAlignedBox boundingB = entWall1->getBoundingBox();
	  //boundingB.scale(Vector3(scale.getX(),scale.getY(),scale.getZ()));
	  Ogre::Vector3 size = Ogre::Vector3(1000, 50, 2);
	  //size = boundingB.getSize()*0.95f;
	  btTransform Transform;
	  Transform.setIdentity();
	  Transform.setOrigin(btVector3(0, 50, -500));
	  MyMotionState *MotionState = new MyMotionState(Transform, nWall);
	  //Give the rigid body half the size
	  // of our cube and tell it to create a BoxShape (cube)
	  btVector3 HalfExtents(size.x, size.y, size.z);
	  btCollisionShape *wall1 = new btBoxShape(HalfExtents);
	  btVector3 LocalInertia;
	  wall1->calculateLocalInertia(0, LocalInertia);
	  btRigidBody *wall1body = new btRigidBody(0, MotionState, wall1, LocalInertia);
	  wall1body->setRestitution(0.75f);
	  wall1body->setFriction(0);
	  // Store a pointer to the Ogre Node so we can update it later
	  //wall1body->setUserPointer((void *) (nWall));

	  dynamicsWorld->addRigidBody(wall1body);
	  bodies.push_back(new CollisionObject(wall1body, nWall, 0));
	  wall1body->setUserPointer(bodies[bodies.size() - 1]);

	  collisionShapes.push_back(wall1);
  }

  Ogre::Plane Swall(0,1,0,0);
  Ogre::MeshManager::getSingleton().createPlane("BottomWall", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
	  Swall, 1000, 100, 20, 20, true, 1, 5, 5, Ogre::Vector3::UNIT_Z);
  Ogre::Entity* entWall2 = mSceneMgr->createEntity("SouthWall", "BottomWall");
  Ogre::SceneNode* sWall = mSceneMgr->getRootSceneNode()->createChildSceneNode();
  sWall->attachObject(entWall2);
  entWall2->setMaterialName("Custom/MetalNet");
  sWall->pitch(Ogre::Degree(90));
  sWall->setPosition(0, 50, 500);
  Ogre::Vector3 Ssize = Ogre::Vector3(1000, 50, 2);
  createRigidbody(0, Ssize, sWall);
  

  Ogre::Plane Ewall(Ogre::Vector3::UNIT_Y, 0);
  Ogre::MeshManager::getSingleton().createPlane("RightWall", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
	  Ewall, 1000, 100, 20, 20, true, 1, 5, 5, Ogre::Vector3::UNIT_Z);
  Ogre::Entity* entWall3 = mSceneMgr->createEntity("EastWall", "RightWall");
  Ogre::SceneNode* eWall = mSceneMgr->getRootSceneNode()->createChildSceneNode();
  eWall->attachObject(entWall3);
  entWall3->setMaterialName("Custom/MetalNet");
  eWall->pitch(Ogre::Degree(90));
  eWall->roll(Ogre::Degree(90));
  eWall->setPosition(500, 50, 0);
  Ogre::Vector3 Esize = Ogre::Vector3(2, 50, 1000);
  createRigidbody(0, Esize, eWall);

  Ogre::Plane Wwall(Ogre::Vector3::UNIT_Y, 0);
  Ogre::MeshManager::getSingleton().createPlane("LeftWall", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
	  Wwall, 1000, 100, 20, 20, true, 1, 5, 5, Ogre::Vector3::UNIT_Z);
  Ogre::Entity* entWall4 = mSceneMgr->createEntity("WestWall", "LeftWall");
  Ogre::SceneNode* wWall = mSceneMgr->getRootSceneNode()->createChildSceneNode();
  wWall->attachObject(entWall4);
  entWall4->setMaterialName("Custom/MetalNet");
  wWall->pitch(Ogre::Degree(90));
  wWall->roll(Ogre::Degree(-90));
  wWall->setPosition(-500, 50, 0);
  Ogre::Vector3 Wsize = Ogre::Vector3(2, 50, 1000);
  createRigidbody(0, Wsize, wWall);
  music(diff);

  //initialize values
  //gameSetup();
  playing = false;
  choosesongs();
  highScore = 0;
}

void TutorialApplication::music(int num){
	if (num == 0){
		engine = irrklang::createIrrKlangDevice();
		engine->play2D("getout.ogg", true);
	}
	if (num == 1){
		engine = irrklang::createIrrKlangDevice();
		engine->play2D("1.wav", true);
	}
	if (num == 2){
		engine = irrklang::createIrrKlangDevice();
		engine->play2D("2.wav", true);
	}
	if (num == 3){
		engine = irrklang::createIrrKlangDevice();
		engine->play2D("3.wav", true);
	}
	
}

void TutorialApplication::createRigidbody(btScalar mass, Ogre::Vector3 size, Ogre::SceneNode *node){
	btTransform Transform;
	Transform.setIdentity();
	Ogre::Vector3 origin = Ogre::Vector3(node->getPosition());
	btVector3 btOrigin = btVector3(origin.x, origin.y, origin.z);
	Transform.setOrigin(btOrigin);
	MyMotionState *MotionState = new MyMotionState(Transform, node);

	btVector3 HalfExtents(size.x, size.y, size.z);
	btCollisionShape *shape = new btBoxShape(HalfExtents);
	btVector3 LocalInertia;
	shape->calculateLocalInertia(0, LocalInertia);
	btRigidBody *Rigidbody = new btRigidBody(mass, MotionState, shape, LocalInertia);
	Rigidbody->setRestitution(0.75f);
	Rigidbody->setFriction(0);
	// Store a pointer to the Ogre Node so we can update it later
	//Rigidbody->setUserPointer((void *) (node));
	dynamicsWorld->addRigidBody(Rigidbody, COL_WALL, wallCollidesWith);
	bodies.push_back(new CollisionObject(Rigidbody, node, 0));
	Rigidbody->setUserPointer(bodies[bodies.size() - 1]);

	collisionShapes.push_back(shape);
}
 

void  TutorialApplication::AddVelocity(btRigidBody* body, float speed, Ogre::Vector3 Direction){
	//Vector conversion
	btVector3 FireVelocity = btVector3(Direction.x, Direction.y, Direction.z);

	//Now as we discussed above, we want our vector to have a certain speed. We first normalize it, and then multiply it by Speed
	FireVelocity.normalize();
	FireVelocity *= (speed * 10);

	//Now we finally propel our box
	body->setLinearVelocity(FireVelocity); //Remember that accelerations were game units? So are velocities
}
void  TutorialApplication::EnemySpawner(){
	//cannot use btVector3 for the vector because of alignment issue
	std::vector<Ogre::Vector3> spawns(5);
	spawns[0] = Ogre::Vector3(-200, 10, -500);
	spawns[1] = Ogre::Vector3(200, 10, -500);
	spawns[2] = Ogre::Vector3(400, 10, -500);
	spawns[3] = Ogre::Vector3(-400, 10, -500);
	spawns[4] = Ogre::Vector3(0, 10, -500);
	int pos = rand() % 5;
	btVector3 loc = btVector3(spawns[pos].x, spawns[pos].y, spawns[pos].z);
	CreateEnemyGreen(loc);
}

void  TutorialApplication::LifeSpawner(){
	//cannot use btVector3 for the vector because of alignment issue
	std::vector<Ogre::Vector3> spawns(5);
	spawns[0] = Ogre::Vector3(-200, 10, -500);
	spawns[1] = Ogre::Vector3(200, 10, -500);
	spawns[2] = Ogre::Vector3(400, 10, -500);
	spawns[3] = Ogre::Vector3(-400, 10, -500);
	spawns[4] = Ogre::Vector3(0, 10, -500);
	int pos = rand() % 5;
	btVector3 loc = btVector3(spawns[pos].x, spawns[pos].y, spawns[pos].z);
	CreateEnemyBlue(loc);
}

void TutorialApplication::CreateEnemyGreen(const btVector3 &Position){
	// empty ogre vectors for the cubes size and position
	Ogre::Vector3 size = Ogre::Vector3::ZERO;
	Ogre::Vector3 pos = Ogre::Vector3::ZERO;
	Ogre::SceneNode *boxNode;
	Ogre::Entity *boxentity;
	// Convert the bullet physics vector to the ogre vector
	pos.x = Position.getX();
	pos.y = Position.getY();
	pos.z = Position.getZ();
	boxentity = mSceneMgr->createEntity("cube.mesh");
	boxentity->setCastShadows(true);
	boxentity->setMaterialName("Custom/White");
	boxNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	boxNode->attachObject(boxentity);
	boxNode->scale(Ogre::Vector3(0.5, 0.5, 0.5));
	Ogre::AxisAlignedBox boundingB = boxentity->getBoundingBox();
	boundingB.scale(Ogre::Vector3(1, 1, 1));
	size = boundingB.getSize()*0.95f;
	btTransform Transform;
	Transform.setIdentity();
	Transform.setOrigin(Position);
	MyMotionState *MotionState = new MyMotionState(Transform, boxNode);
	//Give the rigid body half the size
	// of our cube and tell it to create a BoxShape (cube)
	btVector3 HalfExtents(size.x*0.5f, size.y*0.5f, size.z*0.5f);
	btCollisionShape *Shape = new btBoxShape(HalfExtents);
	btVector3 LocalInertia;
	Shape->calculateLocalInertia(1, LocalInertia);
	btRigidBody *RigidBody = new btRigidBody(1, MotionState, Shape, LocalInertia);
	RigidBody->setRestitution(1.0f);
	RigidBody->setFriction(0);
	// Add it to the physics world
	dynamicsWorld->addRigidBody(RigidBody, COL_ENEMY, enemyCollidesWith);
	bodies.push_back(new CollisionObject(RigidBody, boxNode, 2));
	RigidBody->setUserPointer(bodies[bodies.size() - 1]);
	collisionShapes.push_back(Shape);

	AddVelocity(RigidBody, 100, getRandomDirection());
}

void TutorialApplication::CreateEnemyBlue(const btVector3 &Position){
	// empty ogre vectors for the cubes size and position
	Ogre::Vector3 size = Ogre::Vector3::ZERO;
	Ogre::Vector3 pos = Ogre::Vector3::ZERO;
	Ogre::SceneNode *boxNode;
	Ogre::Entity *boxentity;
	// Convert the bullet physics vector to the ogre vector
	pos.x = Position.getX();
	pos.y = Position.getY();
	pos.z = Position.getZ();
	boxentity = mSceneMgr->createEntity("cube.mesh");
	boxentity->setCastShadows(true);
	boxentity->setMaterialName("Custom/Green");
	boxNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	boxNode->attachObject(boxentity);
	boxNode->scale(Ogre::Vector3(0.5, 0.5, 0.5));
	Ogre::AxisAlignedBox boundingB = boxentity->getBoundingBox();
	boundingB.scale(Ogre::Vector3(1, 1, 1));
	size = boundingB.getSize()*0.95f;
	btTransform Transform;
	Transform.setIdentity();
	Transform.setOrigin(Position);
	MyMotionState *MotionState = new MyMotionState(Transform, boxNode);
	//Give the rigid body half the size
	// of our cube and tell it to create a BoxShape (cube)
	btVector3 HalfExtents(size.x*0.5f, size.y*0.5f, size.z*0.5f);
	btCollisionShape *Shape = new btBoxShape(HalfExtents);
	btVector3 LocalInertia;
	Shape->calculateLocalInertia(1, LocalInertia);
	btRigidBody *RigidBody = new btRigidBody(1, MotionState, Shape, LocalInertia);
	RigidBody->setRestitution(1.0f);
	RigidBody->setFriction(0);
	// Add it to the physics world
	dynamicsWorld->addRigidBody(RigidBody, COL_ENEMY, enemyCollidesWith);
	bodies.push_back(new CollisionObject(RigidBody, boxNode, -2));
	RigidBody->setUserPointer(bodies[bodies.size() - 1]);
	collisionShapes.push_back(Shape);

	AddVelocity(RigidBody, 100, getRandomDirection());
}

void TutorialApplication::CreateEnemyRed(const btVector3 &Position, int num){
	// empty ogre vectors for the cubes size and position
	Ogre::Vector3 size = Ogre::Vector3::ZERO;
	Ogre::Vector3 pos = Ogre::Vector3::ZERO;
	Ogre::SceneNode *boxNode;
	Ogre::Entity *boxentity;
	// Convert the bullet physics vector to the ogre vector
	pos.x = Position.getX();
	pos.y = Position.getY();
	pos.z = Position.getZ();
	boxentity = mSceneMgr->createEntity("cube.mesh");
	boxentity->setCastShadows(true);
	boxentity->setMaterialName("Custom/Red");
	boxNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	boxNode->attachObject(boxentity);
	boxNode->scale(Ogre::Vector3(1, 0.7, 0.5));
	Ogre::AxisAlignedBox boundingB = boxentity->getBoundingBox();
	boundingB.scale(Ogre::Vector3(0.5, 0.5, 0.5));
	size = boundingB.getSize()*0.95f;
	btTransform Transform;
	Transform.setIdentity();
	Transform.setOrigin(Position);
	MyMotionState *MotionState = new MyMotionState(Transform, boxNode);
	//Give the rigid body half the size
	// of our cube and tell it to create a BoxShape (cube)
	btVector3 HalfExtents(size.x*0.5f, size.y*0.5f, size.z*0.5f);
	btCollisionShape *Shape = new btBoxShape(HalfExtents);
	btVector3 LocalInertia;
	Shape->calculateLocalInertia(1, LocalInertia);
	btRigidBody *RigidBody = new btRigidBody(1, MotionState, Shape, LocalInertia);
	RigidBody->setRestitution(1.0f);
	RigidBody->setFriction(0);

	// Add it to the physics world
	dynamicsWorld->addRigidBody(RigidBody, COL_ENEMY, enemyCollidesWith);
	bodies.push_back(new CollisionObject(RigidBody, boxNode, num));
	RigidBody->setUserPointer(bodies[bodies.size() - 1]);
	collisionShapes.push_back(Shape);

}
//----------------------------------------------------------------------------------------------------------------
void TutorialApplication::gainLife(){
	if (pLife.size() == 4){
		Ogre::Entity* life1 = mSceneMgr->createEntity("ogrehead.mesh");
		Ogre::SceneNode* LifeNode1 = mSceneMgr->getRootSceneNode()->createChildSceneNode(Ogre::Vector3(400, 200, -600));
		LifeNode1->attachObject(life1);
		pLife.push_back(LifeNode1);
	}
	if (pLife.size() == 3){
		Ogre::Entity* life1 = mSceneMgr->createEntity("ogrehead.mesh");
		Ogre::SceneNode* LifeNode1 = mSceneMgr->getRootSceneNode()->createChildSceneNode(Ogre::Vector3(200, 200, -600));
		LifeNode1->attachObject(life1);
		pLife.push_back(LifeNode1);
	}
	if (pLife.size() == 2){
		Ogre::Entity* life1 = mSceneMgr->createEntity("ogrehead.mesh");
		Ogre::SceneNode* LifeNode1 = mSceneMgr->getRootSceneNode()->createChildSceneNode(Ogre::Vector3(0, 200, -600));
		LifeNode1->attachObject(life1);
		pLife.push_back(LifeNode1);
	}
	if (pLife.size() == 1){
		Ogre::Entity* life1 = mSceneMgr->createEntity("ogrehead.mesh");
		Ogre::SceneNode* LifeNode1 = mSceneMgr->getRootSceneNode()->createChildSceneNode(Ogre::Vector3(-200, 200, -600));
		LifeNode1->attachObject(life1);
		pLife.push_back(LifeNode1);
	}
}

void TutorialApplication::initLife(){
	Ogre::Entity* life1 = mSceneMgr->createEntity("ogrehead.mesh");
	Ogre::SceneNode* LifeNode1 = mSceneMgr->getRootSceneNode()->createChildSceneNode(Ogre::Vector3(-400, 200, -600));
	LifeNode1->attachObject(life1);
	pLife.push_back(LifeNode1);

	Ogre::Entity* life2 = mSceneMgr->createEntity("ogrehead.mesh");
	Ogre::SceneNode* LifeNode2 = mSceneMgr->getRootSceneNode()->createChildSceneNode(Ogre::Vector3(-200, 200, -600));
	LifeNode2->attachObject(life2);
	pLife.push_back(LifeNode2);

	Ogre::Entity* life3 = mSceneMgr->createEntity("ogrehead.mesh");
	Ogre::SceneNode* LifeNode3 = mSceneMgr->getRootSceneNode()->createChildSceneNode(Ogre::Vector3(0, 200, -600));
	LifeNode3->attachObject(life3);
	pLife.push_back(LifeNode3);

	Ogre::Entity* life4 = mSceneMgr->createEntity("ogrehead.mesh");
	Ogre::SceneNode* LifeNode4 = mSceneMgr->getRootSceneNode()->createChildSceneNode(Ogre::Vector3(200, 200, -600));
	LifeNode4->attachObject(life4);
	pLife.push_back(LifeNode4);

	Ogre::Entity* life5 = mSceneMgr->createEntity("ogrehead.mesh");
	Ogre::SceneNode* LifeNode5 = mSceneMgr->getRootSceneNode()->createChildSceneNode(Ogre::Vector3(400, 200, -600));
	LifeNode5->attachObject(life5);
	pLife.push_back(LifeNode5);
}

void TutorialApplication::removeLife(){
	Ogre::SceneNode* temp = pLife.back();
	Ogre::Entity* ent = ((Ogre::Entity*)temp->getAttachedObject(0));
	temp->getAttachedObject(0)->detachFromParent();
	mSceneMgr->destroyEntity(ent);
	mSceneMgr->destroySceneNode(temp);
	pLife.pop_back();
}

void TutorialApplication::cleanLife(){
	while (pLife.size()){
		Ogre::SceneNode* temp = pLife.back();
		Ogre::Entity* ent = ((Ogre::Entity*)temp->getAttachedObject(0));
		temp->getAttachedObject(0)->detachFromParent();
		mSceneMgr->destroyEntity(ent);
		mSceneMgr->destroySceneNode(temp);
		pLife.pop_back();
	}
}

void TutorialApplication::createFrameListener()
{
  BaseApplication::createFrameListener();
 
  mTrayMgr->hideTrays();
}
 
void TutorialApplication::destroyScene()
{
  OGRE_DELETE mTerrainGroup;
  OGRE_DELETE mTerrainGlobals;
  engine->drop();
}

	
	   
 
bool TutorialApplication::frameRenderingQueued(const Ogre::FrameEvent& fe)
{
  BaseApplication::frameRenderingQueued(fe);
  if (mShutDown) return false;
  delay = delay + fe.timeSinceLastFrame;
  lifedelay = lifedelay + fe.timeSinceLastFrame;
  static bool mMouseDown = false;     // If a mouse button is depressed
  static Ogre::Real mToggle = 0.0;    // The time left until next toggle
  static Ogre::Real mRotate = 0.13;   // The rotate constant
  static Ogre::Real mMove = 250;      // The movement constant
  bool currMouse = mMouse->getMouseState().buttonDown(OIS::MB_Left);

  mMouseDown = currMouse;

  mToggle -= fe.timeSinceLastFrame;
  btTransform Trans;
  Trans.setIdentity();
  Trans = getTank()->body->getWorldTransform();

  //for moving
  Ogre::Vector3 transVector = Ogre::Vector3::ZERO;
  Ogre::Vector3 curPos = Ogre::Vector3::ZERO;
 
  /*
  if (mKeyboard->isKeyDown(OIS::KC_SPACE)){
	  Ogre::Vector3 direction = mSceneMgr->getSceneNode("Tank")->_getDerivedOrientation() * Ogre::Vector3::UNIT_Z;
	  Ogre::Vector3 position = mSceneMgr->getSceneNode("Tank")->getPosition();
	  btVector3 pos = btVector3(position.x, position.y, position.z);
	  shootBullet(pos, direction);
	  numBullets++;
  }
  */
  mSceneMgr->getSceneNode("Tank")->translate(transVector * fe.timeSinceLastFrame);//, Ogre::Node::TS_LOCAL);
  curPos = getTank()->node->getPosition();
  btVector3 btTransVector = btVector3(curPos.x, curPos.y, curPos.z);
  Trans.setOrigin(btTransVector);
  getTank()->body->setWorldTransform(Trans);
  //getTank()->body->translate(btTransVector * evt.timeSinceLastFrame);

  //LifeSpawner
  // update physics simulation
  static Ogre::Real second = 0;
  if (playing){
	  if (second < 1 && playing)
		  second += fe.timeSinceLastFrame;
	  else if (playing) {
		  second = 0;
		  if (gameTimer > 0)
			  --gameTimer;
	  }
	  if (diff == 1){
		  if (lifedelay >= 30 * muti){
			  LifeSpawner();
			  lifedelay = 0;
		  }
		  else if (delay >= 1.5 * muti){
			  EnemySpawner();
			  delay = 0;
		  }
		  if (gameScore >= 800){
			  diff = 2;
		  }
	  }
	  if (diff == 2){
		  if (lifedelay >= 20 * muti){
			  LifeSpawner();
			  lifedelay = 0;
		  }
		  else if (delay >= 1 * muti){
			  EnemySpawner();
			  delay = 0;
		  }
		  if (gameScore >= 2400){
			  diff = 3;
		  }
	  }
	  if (diff == 3){
		  if (lifedelay >= 10 * muti){
			  LifeSpawner();
			  lifedelay = 0;
		  }
		  else if (delay >= 0.5 * muti){
			  EnemySpawner();
			  delay = 0;
		  }
		  if (gameScore >= 7200){
			  diff = 4;
		  }
	  }
	  if (diff == 4){
		  if (delay >= 0.25 * muti){
			  EnemySpawner();
			  delay = 0;
		  }
	  }
  }

  tim->setText(" Timer: " + std::to_string(gameTimer));
  scr->setText(" Score: " + std::to_string(gameScore));
  CheckCollisions();

  dynamicsWorld->stepSimulation(fe.timeSinceLastFrame);

  if (gameTimer == 0) //check if time has run out
	  gameOver();
  if (pLife.size() == 0){
	  gameOver();
  }
  return true;
}

void TutorialApplication::misssound(){
	if (soundsbool){
		engine->play2D("miss.wav");
	}
}

void TutorialApplication::hitsound(){
	if (soundsbool){
		engine->play2D("hit.wav");
	}
}
void TutorialApplication::choosesongs(){
	choosing = true;
	over->hide();
	pick->setText("                                                      BEATS\n\nRules:            This is a song beats game, you can lisen to the beats and play\n the game. Press the key 'Z' 'X' 'SPACE' 'C' 'V' to play, when the white cube\n hit the red cube, press the corresponding key to gain points, or the life if\n the cube is green, each game you got 5 chance to miss. game over after 5\n life is gone or the song is over.\n game will speed up at different points at different level:\n                                  Easy: 800, middle: 2400, hard: 7200\n\n                                                  Choose Songs:\n\n                             (Y) for ''Water Under The Bridge''  (Easy)\n                             (U) for ''Too Far''                            (middle)\n                             (I)  for ''Shpe Of You''                     (hard)\n"); 
	pick->show();

}
 
void TutorialApplication::gameOver(){
	playing = false;	//disable user input

	if (highScore < gameScore)		//update running high score
		highScore = gameScore;

	//hide windows to be replaced by the game over screen
	tim->hide();
	scr->hide();

	//activate the gam over window
	over->setText("                                                     Game Over \n                                              Your Final " + scr->getText() + "\n                                                   High Score: " + std::to_string(highScore) +
		"\n\n                                               Play Again: (Enter)\n                                           Quit Game: (Esc)");
	over->show();

}
void TutorialApplication::gamesetup(){
	cleanLife();
	engine->drop();
	playing = true;
	muti = 1;
	initLife();
	pick->hide();
	over->hide();
	tim->show();
	scr->show();
	gameTimer = 240;
	gameScore = 0;
}

bool TutorialApplication::keyPressed(const OIS::KeyEvent &arg)
{
 if (arg.key == OIS::KC_ESCAPE)
	{
		mShutDown = true;
	}

 if (arg.key == OIS::KC_UP)
 {
	 muti = muti * 2.0;
 }

 if (arg.key == OIS::KC_DOWN)
 {
	 muti = muti / 2.0;
 }

 if (arg.key == OIS::KC_P)
 {
	 if (soundsbool){
		 soundsbool = false;
	 }
	 else{
		 soundsbool = true;
	 }
 }
 if (arg.key == OIS::KC_Y && !playing)
 {
	 diff = 1;
	 gamesetup();
	 music(diff);
 }

 if (arg.key == OIS::KC_U && !playing)
 {
	 diff = 2;
	 gamesetup();
	 music(diff);
 }

 if (arg.key == OIS::KC_I && !playing)
 {
	 diff = 3;
	 gamesetup();
	 music(diff);
 }

 if (arg.key == OIS::KC_RETURN && !playing)
 {
	 choosesongs();
 }

	//mCameraMan->injectKeyDown(arg);
	return true;
}
//---------------------------------------------------------------------------
bool TutorialApplication::keyReleased(const OIS::KeyEvent &arg)
{
	//mCameraMan->injectKeyUp(arg);
	return true;
}
//---------------------------------------------------------------------------
bool TutorialApplication::mouseMoved(const OIS::MouseEvent &arg)
{
	//if (mTrayMgr->injectMouseMove(arg)) return true;
	//mCameraMan->injectMouseMove(arg);
	//CEGUI::GUIContext& context = CEGUI::System::getSingleton().getDefaultGUIContext();
	//context.injectMouseMove(arg.state.X.rel, arg.state.Y.rel);
	//mCameraMan->injectMouseMove(arg);
	return true;
}
//---------------------------------------------------------------------------
bool TutorialApplication::mousePressed(const OIS::MouseEvent &arg, OIS::MouseButtonID id)
{
	//if (mTrayMgr->injectMouseDown(arg, id)) return true;
	//mCameraMan->injectMouseDown(arg, id);
	return true;
}
//---------------------------------------------------------------------------
bool TutorialApplication::mouseReleased(const OIS::MouseEvent &arg, OIS::MouseButtonID id)
{
	//if (mTrayMgr->injectMouseUp(arg, id)) return true;
	//mCameraMan->injectMouseUp(arg, id);
	return true;
}

#if Ogre_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
#endif
 
#ifdef __cplusplus
extern "C" {
#endif
 
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
  INT WINAPI WinMain( HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT )
#else
  int main(int argc, char *argv[])
#endif
  {
    // Create application object
    TutorialApplication app;
 
    try {
      app.go();
    } catch( Ogre::Exception& e ) {
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
      MessageBox( NULL, e.getFullDescription().c_str(), "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else
      std::cerr << "An exception has occured: " <<
	e.getFullDescription().c_str() << std::endl;
#endif
    }
 
    return 0;
  }
 
#ifdef __cplusplus
}
#endif