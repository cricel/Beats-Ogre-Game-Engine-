#include <Ogre.h>
#include <Terrain/OgreTerrain.h>
#include <Terrain/OgreTerrainGroup.h>
#include "OgreManualObject.h"
#include "btBulletDynamicsCommon.h"
#include "btHeightfieldTerrainShape.h"
#include <CEGUI/CEGUI.h>
#include <CEGUI/RendererModules/Ogre/Renderer.h>
#include "BaseApplication.h"
#include <vector>
#include <irrKlang.h>
#pragma comment(lib, "irrKlang.lib") // link with irrKlang.dll

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
	Ogre::SceneNode* node;
	btRigidBody* body;
	bool setdelete;
	CollisionObject(btRigidBody* b, Ogre::SceneNode* n, int i) : body(b), node(n), id(i), hit(false) {}
};
 
class TutorialApplication : public BaseApplication
{
public:
  TutorialApplication();
  virtual ~TutorialApplication();
 
protected:
  CEGUI::OgreRenderer *mRenderer;
  virtual void createScene();
  virtual void createFrameListener();
  virtual void destroyScene();
  virtual bool frameRenderingQueued(const Ogre::FrameEvent& fe);
  bool keyPressed(const OIS::KeyEvent &arg);
  bool keyReleased(const OIS::KeyEvent &arg);
  bool mouseMoved(const OIS::MouseEvent &arg);
  bool mousePressed(const OIS::MouseEvent &arg, OIS::MouseButtonID id);
  bool mouseReleased(const OIS::MouseEvent &arg, OIS::MouseButtonID id);
  void CheckCollisions(); 
  void  shootBullet(const btVector3& Position, Ogre::Vector3 Direction);
  void  AddVelocity(btRigidBody* body, float speed, Ogre::Vector3 Direction);
  void  EnemySpawner();
  void  LifeSpawner();
  void CreateEnemyGreen(const btVector3 &Position);
  void CreateEnemyBlue(const btVector3 &Position);
  void CreateEnemyRed(const btVector3 &Position, int num);
  void CreateEnemyKey(Ogre::Vector3 &Position, int num);
  void createRigidbody(btScalar mass, Ogre::Vector3 size, Ogre::SceneNode *node);
  void destroyObject(CollisionObject * ptrToOgreObject);
  void removeLife();
  void cleanLife();
  void initLife();
  void gainLife();
  Ogre::ManualObject *createCubeMesh(Ogre::String name, Ogre::String matName);
  int tankCollidesWith;
  int wallCollidesWith;
  int bulletCollidesWith;
  int enemyCollidesWith;
 
private:
  CEGUI::Window* scr; CEGUI::Window* tim;
  CEGUI::Window* over; CEGUI::Window* pick;
  int gameScore; int gameTimer;
  int highScore; int diff = 0;
  float muti = 1;
  bool playing;
  void choosesongs();
  void misssound();
  void hitsound();
  void gameOver();
  void gamesetup();
  bool choosing;
  bool soundsbool = true;
  void music(int num);

  irrklang::ISoundEngine* engine;
  irrklang::ISoundEngine* engineclick;
  void defineTerrain(long x, long y);
  void initBlendMaps(Ogre::Terrain* terrain);
  void configureTerrainDefaults(Ogre::Light* light);
  void createBulletSim(void);
  bool mTerrainsImported;
  bool mConinue = true;
  CollisionObject* TutorialApplication::getTank();
  Ogre::TerrainGroup* mTerrainGroup;
  Ogre::TerrainGlobalOptions* mTerrainGlobals;
 
  int maxBullets;
  int numBullets;
  float delay;
  float lifedelay;
  OgreBites::Label* mInfoLabel;
  btRigidBody* CreateBullet(const btVector3 &Position, btScalar Mass, const btVector3 &scale);
  void CreateCube(const btVector3 &Position, btScalar Mass, const btVector3 &scale, char * name);
  btDefaultCollisionConfiguration* collisionConfiguration;
  btCollisionDispatcher* dispatcher;
  btBroadphaseInterface* overlappingPairCache;
  btSequentialImpulseConstraintSolver* solver;
  btDiscreteDynamicsWorld* dynamicsWorld;
  btCollisionShape* groundShape;
  btAlignedObjectArray<btCollisionShape*> collisionShapes;
  std::vector<CollisionObject*> bodies;
  std::vector<Ogre::SceneNode*> pLife;
 
};
 

