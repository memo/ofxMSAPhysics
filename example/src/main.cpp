
#include "ofMain.h"
#include "MSAPhysics3D.h"

using namespace msa::physics;

#define	SPRING_MIN_STRENGTH		0.005
#define SPRING_MAX_STRENGTH		0.1

#define	SPRING_MIN_WIDTH		1
#define SPRING_MAX_WIDTH		3

#define NODE_MIN_RADIUS			5
#define NODE_MAX_RADIUS			15

#define MIN_MASS				1
#define MAX_MASS				3

#define MIN_BOUNCE				0.2
#define MAX_BOUNCE				0.9

#define	FIX_PROBABILITY			10		// % probability of a particle being fixed on creation

#define FORCE_AMOUNT			10

#define EDGE_DRAG				0.98

#define	GRAVITY					1

#define MAX_ATTRACTION			10
#define MIN_ATTRACTION			3

#define SECTOR_COUNT			1		// currently there is a bug at sector borders

class ofApp : public ofBaseApp {
public:

    bool				mouseAttract	= false;
    bool				doMouseXY		= false;		// pressing left mmouse button moves mouse in XY plane
    bool				doMouseYZ		= false;		// pressing right mouse button moves mouse in YZ plane
    bool				doRender		= true;
    int					forceTimer		= false;

    float				rotSpeed		= 0;
    float				mouseMass		= 1;

    int             	width;
    int         		height;

    ofImage				ballImage;
    World3D_ptr         world;
    Particle3D_ptr      mouseNode;

    void initScene() {
        // clear all particles and springs etc
        world->clear();

        world->draw();

        mouseNode->getMass();

        // you can add your own particles to the physics system

        mouseNode->makeFixed();
        mouseNode->setMass(MIN_MASS);
        mouseNode->moveTo(ofVec3f(0, 0, 0));
        mouseNode->setRadius(NODE_MAX_RADIUS);

        // or tell the system to create and add particles
        world->makeParticle(ofVec3f(-width/4, 0, -width/4), MIN_MASS)->makeFixed();		// create a node in top left back and fix
        world->makeParticle(ofVec3f( width/4, 0, -width/4), MIN_MASS)->makeFixed();		// create a node in top right back and fix
        world->makeParticle(ofVec3f(-width/4, 0,  width/4), MIN_MASS)->makeFixed();		// create a node in top left front and fix
        world->makeParticle(ofVec3f( width/4, 0,  width/4), MIN_MASS)->makeFixed();		// create a node in top right front and fix
    }


    //--------------------------------------------------------------
    void setup(){
        ofBackground(255, 255, 255);
        ofSetVerticalSync(true);
        ofSetFrameRate(60);

        ballImage.loadImage("ball.png");

        width = ofGetWidth();
        height = ofGetHeight();

        // initialize our physics world
        world = World3D::create();
        mouseNode = world->makeParticle();

        //	world->verbose = true;			// dump activity to log
        world->setGravity(ofVec3f(0, GRAVITY, 0));

        // set world dimensions, not essential, but speeds up collision
        world->setWorldSize(ofVec3f(-width/2, -height, -width/2), ofVec3f(width/2, height, width/2));
        world->setSectorCount(SECTOR_COUNT);
        world->setDrag(0.97f);
        world->setDrag(1);		// FIXTHIS
        world->enableCollision();

        initScene();

        // setup lighting
        GLfloat mat_shininess[] = { 50.0 };
        GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
        GLfloat light_position[] = { 0, height/2, 0.0, 0.0 };
        glShadeModel(GL_SMOOTH);

        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
        glLightfv(GL_LIGHT0, GL_POSITION, light_position);

        glEnable(GL_COLOR_MATERIAL);
        glEnable(GL_LIGHT0);

        // enable back-face culling (so we can see through the walls)
        glCullFace(GL_BACK);
        glEnable(GL_CULL_FACE);
    }


    void addRandomParticle() {
        float posX		= ofRandom(-width/2, width/2);
        float posY		= ofRandom(0, height);
        float posZ		= ofRandom(-width/2, width/2);
        float mass		= ofRandom(MIN_MASS, MAX_MASS);
        float bounce	= ofRandom(MIN_BOUNCE, MAX_BOUNCE);
        float radius	= ofMap(mass, MIN_MASS, MAX_MASS, NODE_MIN_RADIUS, NODE_MAX_RADIUS);

        // world->makeParticle returns a particle pointer so you can customize it

        shared_ptr< msa::physics::ParticleT<ofVec3f> > p1;
        Particle3D_ptr p2;
        shared_ptr<Particle3D> p3;

        p1 = p2;
        p1 = p3;
        p2 = p3;

        shared_ptr<msa::physics::Particle3D> p = world->makeParticle(ofVec3f(posX, posY, posZ));

        // and set a bunch of properties (you don't have to set all of them, there are defaults)
        p->setMass(mass)->setBounce(bounce)->setRadius(radius)->enableCollision()->makeFree();

        // add an attraction to the mouseNode
        if(mouseAttract) world->makeAttraction(mouseNode, p, ofRandom(MIN_ATTRACTION, MAX_ATTRACTION));
    }

    void addRandomSpring() {
        Particle3D_ptr a = world->getParticle((int)ofRandom(0, world->numberOfParticles()));
        Particle3D_ptr b = world->getParticle((int)ofRandom(0, world->numberOfParticles()));
        world->makeSpring(a, b, ofRandom(SPRING_MIN_STRENGTH, SPRING_MAX_STRENGTH), ofRandom(10, width/2));
    }


    void killRandomParticle() {
        Particle3D_ptr p = world->getParticle(floor(ofRandom(0, world->numberOfParticles())));
        if(p && p != mouseNode) p->kill();
    }

    void killRandomSpring() {
        Spring3D_ptr s = world->getSpring( floor(ofRandom(0, world->numberOfSprings())));
        if(s) s->kill();
    }

//    void killRandomConstraint() {
//        Constraint3D_ptrc = world->getConstraint(floor(ofRandom(0, world->numberOfConstraints())));
//        if(c) c->kill();
//    }


    void toggleMouseAttract() {
        mouseAttract = !mouseAttract;
        if(mouseAttract) {
            // loop through all particles and add attraction to mouse
            // (doesn't matter if we attach attraction from mouse-mouse cos it won't be added internally
            for(int i=0; i<world->numberOfParticles(); i++) world->makeAttraction(mouseNode, world->getParticle(i), ofRandom(MIN_ATTRACTION, MAX_ATTRACTION));
        } else {
            // loop through all existing attractsions and delete them
            for(int i=0; i<world->numberOfAttractions(); i++) world->getAttraction(i)->kill();
        }
    }

    void addRandomForce(float f) {
        forceTimer = f;
        for(int i=0; i<world->numberOfParticles(); i++) {
            Particle3D_ptr p = world->getParticle(i);
            if(p->isFree()) p->addVelocity(ofVec3f(ofRandom(-f, f), ofRandom(-f, f), ofRandom(-f, f)));
        }
    }

    void lockRandomParticles() {
        for(int i=0; i<world->numberOfParticles(); i++) {
            Particle3D_ptr p = world->getParticle(i);
            if(ofRandom(0, 100) < FIX_PROBABILITY) p->makeFixed();
            else p->makeFree();
        }
        mouseNode->makeFixed();
    }

    void unlockRandomParticles() {
        for(int i=0; i<world->numberOfParticles(); i++) {
            Particle3D_ptr p = world->getParticle(i);
            p->makeFree();
        }
        mouseNode->makeFixed();
    }



    //--------------------------------------------------------------
    void update() {
        width = ofGetWidth();
        height = ofGetHeight();

        world->update();
    }


    //--------------------------------------------------------------
    void draw() {
        if(doRender) {

            ofEnableAlphaBlending();
            glEnable(GL_DEPTH_TEST);

            glPushMatrix();

            glTranslatef(width/2, 0, -width / 3);		// center scene
            static float rot = 0;
            glRotatef(rot, 0, 1, 0);			// rotate scene
            rot += rotSpeed;						// slowly increase rotation (to get a good 3D view)

            if(forceTimer) {
                float translateMax = forceTimer;
                glTranslatef(ofRandom(-translateMax, translateMax), ofRandom(-translateMax, translateMax), ofRandom(-translateMax, translateMax));
                forceTimer--;
            }


            glDisable(GL_LIGHTING);
            glBegin(GL_QUADS);
            // draw right wall
            glColor3f(.9, 0.9, 0.9);		glVertex3f(width/2, height+1, width/2);
            glColor3f(1, 1, 1);				glVertex3f(width/2, -height, width/2);
            glColor3f(0.95, 0.95, 0.95);	glVertex3f(width/2, -height, -width/2);
            glColor3f(.85, 0.85, 0.85);		glVertex3f(width/2, height+1, -width/2);

            // back wall
            glColor3f(.9, 0.9, 0.9);		glVertex3f(width/2, height+1, -width/2);
            glColor3f(1, 1, 1);				glVertex3f(width/2, -height, -width/2);
            glColor3f(0.95, 0.95, 0.95);	glVertex3f(-width/2, -height, -width/2);
            glColor3f(.85, 0.85, 0.85);		glVertex3f(-width/2, height+1, -width/2);

            // left wall
            glColor3f(.9, 0.9, 0.9);		glVertex3f(-width/2, height+1, -width/2);
            glColor3f(1, 1, 1);				glVertex3f(-width/2, -height, -width/2);
            glColor3f(0.95, 0.95, 0.95);	glVertex3f(-width/2, -height, width/2);
            glColor3f(.85, 0.85, 0.85);		glVertex3f(-width/2, height+1, width/2);

            // front wall
            glColor3f(0.95, 0.95, 0.95);	glVertex3f(width/2, -height, width/2);
            glColor3f(.85, 0.85, 0.85);		glVertex3f(width/2, height+1, width/2);
            glColor3f(.9, 0.9, 0.9);		glVertex3f(-width/2, height+1, width/2);
            glColor3f(1, 1, 1);				glVertex3f(-width/2, -height, width/2);

            // floor
            glColor3f(.8, 0.8, 0.8);
            glVertex3f(width/2, height+1, width/2);
            glVertex3f(width/2, height+1, -width/2);
            glVertex3f(-width/2, height+1, -width/2);
            glVertex3f(-width/2, height+1, width/2);
            glEnd();

            //		glEnable(GL_LIGHTING);

            // draw springs
            glColor4f(0.5, 0.5, 0.5, 0.5);
            for(int i=0; i<world->numberOfSprings(); i++) {
                Spring3D_ptr spring = world->getSpring(i);
                Particle3D_ptr a = spring->getOneEnd();
                Particle3D_ptr b = spring->getTheOtherEnd();
                ofVec3f vec = b->getPosition() - a->getPosition();
                float dist = vec.length();
                float angle = acos( vec.z / dist ) * RAD_TO_DEG;
                if(vec.z <= 0 ) angle = -angle;
                float rx = -vec.y * vec.z;
                float ry =  vec.x * vec.z;

                glPushMatrix();
                glTranslatef(a->getPosition().x, a->getPosition().y, a->getPosition().z);
                glRotatef(angle, rx, ry, 0.0);
                float size  = ofMap(spring->getStrength(), SPRING_MIN_STRENGTH, SPRING_MAX_STRENGTH, SPRING_MIN_WIDTH, SPRING_MAX_WIDTH);

                glScalef(size, size, dist);
                glTranslatef(0, 0, 0.5);
                ofBox(1);
                glPopMatrix();
            }


            // draw particles
            glAlphaFunc(GL_GREATER, 0.5);

            ofEnableNormalizedTexCoords();
            ballImage.getTextureReference().bind();
            for(int i=0; i<world->numberOfParticles(); i++) {
                Particle3D_ptr p = world->getParticle(i);
                if(p->isFixed()) glColor4f(1, 0, 0, 1);
                else glColor4f(1, 1, 1, 1);

                glEnable(GL_ALPHA_TEST);

                // draw ball
                glPushMatrix();
                glTranslatef(p->getPosition().x, p->getPosition().y, p->getPosition().z);
                glRotatef(180-rot, 0, 1, 0);

                glBegin(GL_QUADS);
                glTexCoord2f(0, 0); glVertex2f(-p->getRadius(), -p->getRadius());
                glTexCoord2f(1, 0); glVertex2f(p->getRadius(), -p->getRadius());
                glTexCoord2f(1, 1); glVertex2f(p->getRadius(), p->getRadius());
                glTexCoord2f(0, 1); glVertex2f(-p->getRadius(), p->getRadius());
                glEnd();
                glPopMatrix();

                glDisable(GL_ALPHA_TEST);

                float alpha = ofMap(p->getPosition().y, -height * 1.5, height, 0, 1);
                if(alpha>0) {
                    glPushMatrix();
                    glTranslatef(p->getPosition().x, height, p->getPosition().z);
                    glRotatef(-90, 1, 0, 0);
                    glColor4f(0, 0, 0, alpha * alpha * alpha * alpha);
                    //				ofCircle(0, 0, p->getRadius());
                    float r = p->getRadius() * alpha;
                    glBegin(GL_QUADS);
                    glTexCoord2f(0, 0); glVertex2f(-r, -r);
                    glTexCoord2f(1, 0); glVertex2f(r, -r);
                    glTexCoord2f(1, 1); glVertex2f(r, r);
                    glTexCoord2f(0, 1); glVertex2f(-r, r);
                    glEnd();
                    glPopMatrix();
                }

            }
            ballImage.getTextureReference().unbind();
            ofDisableNormalizedTexCoords();


            glPopMatrix();
        }

        glDisable(GL_BLEND);
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_LIGHTING);
        glColor4f(0, 0, 0, 1);
        ofDrawBitmapString(" FPS: " + ofToString(ofGetFrameRate(), 2)
                           + " | Number of particles: " + ofToString(world->numberOfParticles(), 2)
                           + " | Number of springs: " + ofToString(world->numberOfSprings(), 2)
                           + " | Mouse Mass: " + ofToString(mouseNode->getMass(), 2)
                           + "\nLook at source code keyPressed to see keyboard shortcuts"
                           , 20, 15);




    }


    //--------------------------------------------------------------
    void keyPressed  (int key){
        switch(key) {
        case 'a': toggleMouseAttract(); break;
        case 'p': for(int i=0; i<100; i++) addRandomParticle(); break;
        case 'P': for(int i=0; i<100; i++) killRandomParticle(); break;
        case 's': addRandomSpring(); break;
        case 'S': killRandomSpring(); break;
        case 'c': world->isCollisionEnabled() ? world->disableCollision() : world->enableCollision(); break;
       // case 'C': killRandomConstraint(); break;
        case 'r': doRender ^= true; break;
        case 'f': addRandomForce(FORCE_AMOUNT); break;
        case 'F': addRandomForce(FORCE_AMOUNT * 3); break;
        case 'l': lockRandomParticles(); break;
        case 'u': unlockRandomParticles(); break;
        case ' ': initScene(); break;
        case 'x': doMouseXY = true; break;
        case 'z': doMouseYZ = true; break;
        case ']': rotSpeed += 0.01f; break;
        case '[': rotSpeed -= 0.01f; break;
        case '+': mouseNode->setMass(mouseNode->getMass() +0.1); break;
        case '-': mouseNode->setMass(mouseNode->getMass() -0.1); break;
        case 'm': mouseNode->hasCollision() ? mouseNode->disableCollision() : mouseNode->enableCollision();
        }
    }

    //--------------------------------------------------------------
    void keyReleased  (int key){
        switch(key) {
        case 'x': doMouseXY = false; break;
        case 'z': doMouseYZ = false; break;
        }

    }

    //--------------------------------------------------------------
    void mouseMoved(int x, int y ) {
        static int oldMouseX = -10000;
        static int oldMouseY = -10000;
        int velX = x - oldMouseX;
        int velY = y - oldMouseY;
        if(doMouseXY) mouseNode->moveBy(ofVec3f(velX, velY, 0));
        if(doMouseYZ) mouseNode->moveBy(ofVec3f(velX, 0, velY));
        oldMouseX = x;
        oldMouseY = y;
    }

    //--------------------------------------------------------------
    void mouseDragged(int x, int y, int button){
        switch(button) {
        case 0:	doMouseXY = true; mouseMoved(x, y); break;
        case 2: doMouseYZ = true; mouseMoved(x, y); break;
        }
    }

    //--------------------------------------------------------------
    void mousePressed(int x, int y, int button){
    }

    //--------------------------------------------------------------
    void mouseReleased(){
        doMouseXY = doMouseYZ = false;
    }

};

//========================================================================
int main( ){
    ofSetupOpenGL(1280, 720, OF_WINDOW);			// <-------- setup the GL context
    ofRunApp(new ofApp());
}
