#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"
#include <vector>
#include <string>
#include <iostream>

struct Node {
    olc::vd2d pos;
    olc::vd2d lastPos;
    olc::vd2d vel;
    olc::vd2d force;
    bool locked = false;
    double mass = 1.0;
    Node() {

    }
    Node(olc::vd2d p) {
        pos = p;
        lastPos = p;
        vel = olc::vd2d(0, 0);
        force = olc::vd2d(0, 0);
    }
    void ApplyForce(olc::vd2d f) {
        force += f;
    }

    void Update(float &fElapsedTime) {
        olc::vd2d lastPosition = pos;
        if (!locked) {
            pos += (pos - lastPos) + (force) * (fElapsedTime * fElapsedTime / mass);
            vel = (pos - lastPos) / fElapsedTime;
        }
        lastPos = lastPosition;
    }

    void Lock() { 
        locked = true; 
        ResetForce();
        vel = olc::vd2d(0, 0);
        lastPos = pos;
    }
    void Unlock() { locked = false; }
    void ToggleLock() { 
        locked?Unlock():Lock(); 
    }
    void ResetForce() { force = olc::vd2d(0, 0); }
};
struct Spring {
    double equilibriumDistance;
    Node *firstObject;
    Node *secondObject;
    Spring(){}
    Spring(Node &f, Node &s) {
        firstObject = &f;
        secondObject = &s;
        //equilibriumDistance = 30;
        equilibriumDistance = (secondObject->pos - firstObject->pos).mag();
    }
    void Update(float &fElapsedTime) {
        olc::vd2d delta = secondObject->pos - firstObject->pos;
        double deltaLength = delta.mag();
        delta *= (equilibriumDistance - deltaLength) / (equilibriumDistance * 1.0);
        if(!firstObject->locked)
            firstObject->pos -= delta * 0.2f* fElapsedTime;
        if (!secondObject->locked)
            secondObject->pos += delta * 0.2f*fElapsedTime;
    }
    void ApplyDamping(float &dampingAmount, float fElapsedTime) {
        olc::vd2d springDifference = (firstObject->pos - secondObject->pos).norm();
        
        springDifference *= ((firstObject->pos - firstObject->lastPos - (secondObject->pos - secondObject->lastPos)).dot( springDifference))*dampingAmount;
        if (!firstObject->locked)
        firstObject->lastPos += springDifference;
        if (!secondObject->locked)
        secondObject->lastPos -= springDifference;
    }
};

struct Rope {
    std::vector<Node*> nodes;
    std::vector<Spring> springs;
   //uint16_t NUM_NODES = 50;
    uint16_t NUM_NODES = 50; 
    uint16_t MAX_NODES = 100;
    uint16_t MIN_NODES = 3;
    Node* editingNode;
    Rope(){}
    Rope(olc::vd2d startPos, olc::vd2d endPos) {
        nodes.reserve(MAX_NODES);
        springs.reserve(MAX_NODES);
        for (uint16_t i = 0; i < NUM_NODES; i++) {
            nodes.push_back(new Node(startPos+(endPos-startPos)*((double)i/(double)NUM_NODES)));
        }
        for (uint16_t i = 0; i < nodes.size()-1; i++) {
            springs.push_back(Spring(*nodes.at(i), *nodes.at(i+1)));
        }
        nodes.at(0)->Lock();
        nodes.at(nodes.size()-1)->Lock();
    }

    void Increase() {
        if (nodes.size() < MAX_NODES) {
            unsigned int secondIndex = nodes.size()-3;
            if (nodes.size() < 3)
                secondIndex = 1;            
            nodes.push_back(new Node(nodes.at(nodes.size() - 1)->pos + (nodes.at(nodes.size() - 1)->pos - nodes.at(secondIndex)->pos).norm()*springs.at(0).equilibriumDistance));
            springs.push_back(Spring(*nodes.at(nodes.size()-2), *nodes.at(nodes.size()-1)));
        }

    }

    void Decrease() {
        if (nodes.size() > MIN_NODES) {
            Node* toDelete = nodes.at(nodes.size()-1);
            nodes.pop_back();
            springs.pop_back();
            delete toDelete;
        }
    }

    void Delete() {
        //DEAL WITH POINTERS
        uint16_t deletionIndex;
        for (uint16_t i = 0; i < nodes.size(); i++) {
            if (nodes.at(i) == editingNode) {
                deletionIndex = i;
            }
        }
        nodes.erase(nodes.begin() + deletionIndex);
        for (std::vector<Spring>::iterator it = springs.begin(); it != springs.end();) {
            if (it->firstObject == editingNode || it->secondObject == editingNode) {
                springs.erase(it);
            } else {
                it++;
            }
        }
        delete editingNode;
        editingNode = NULL;
    }

    void Insert(olc::vd2d pos, Node *connecting = NULL) {
        nodes.push_back(new Node(pos));
        springs.push_back(Spring(*editingNode, connecting == NULL ? *nodes.at(nodes.size() - 1):*connecting));
        editingNode = nodes.at(nodes.size() - 1);
    }

};
// Override base class with your custom functionality
class RopePhysics : public olc::PixelGameEngine {
public:
    double scale = 1;
    olc::vd2d camera;
    olc::vd2d centre;
    olc::vd2d gravitationalAcceleration;

    //States
    bool drawNodes;
    bool holding;
    bool closestNodeActive;
    bool airResistance;
    bool paused;
    bool editing;
    //Physics Obejcts
    Rope rope;
    Node* closest;
    RopePhysics() {
        sAppName = "Rope Physics";
        centre = olc::vd2d(0, 0);
        camera = olc::vd2d(0, 0);
        gravitationalAcceleration = olc::vd2d(0, 75);
        drawNodes = false;
        holding = false;
        closestNodeActive = false;
        airResistance = true;
        paused = false;
        editing = false;
    }
public:
    bool OnUserCreate() override {
        // Called once at the start, so create things here
        rope = Rope(olc::vd2d(0, -50), olc::vd2d(100, -50)); 
        closest = rope.nodes.at(0);
        return true;
    }
    bool OnUserUpdate(float fElapsedTime) override {
        Clear(olc::Pixel(38, 93, 117));
        //Update Closest to Mouse
        bool close = false;
        double distance = 31;
        if (!holding) {
            for (uint16_t i = 0; i < rope.nodes.size(); i++) {
                if ((olc::vd2d(GetMouseX(), GetMouseY()) - calibrate(rope.nodes.at(i)->pos)).mag() < (olc::vd2d(GetMouseX(), GetMouseY()) - calibrate(closest->pos)).mag()) {
                    closest = rope.nodes.at(i);
                }
            }
        }
        if (IsFocused()) {
            if (GetMouse(0).bPressed) {
                if (editing && (olc::vd2d(GetMouseX(), GetMouseY()) - calibrate(closest->pos)).mag() < 2) {
                    rope.editingNode = closest;
                } else {
                    rope.editingNode = NULL;
                }
            }
            if (GetMouse(0).bHeld) {
                if (!editing) {
                    holding = true;
                    if ((olc::vd2d(GetMouseX(), GetMouseY()) - calibrate(closest->pos)).mag() < 2) {
                        closestNodeActive = true;
                    }
                    if (holding&&closestNodeActive) {
                        closest->pos = ScreenToWorld((olc::vd2d(GetMouseX(), GetMouseY()) - camera) / scale);
                    }
                }
            } else {
                closestNodeActive = false;
                holding = false;
            }

            if (GetMouse(1).bPressed && (olc::vd2d(GetMouseX(), GetMouseY()) - calibrate(closest->pos)).mag() < 10) {
                if(!editing)
                closest->ToggleLock();
            }
            if (GetMouse(1).bPressed && editing) {
                if (rope.editingNode != NULL) {
                    if ((olc::vd2d(GetMouseX(), GetMouseY()) - calibrate(closest->pos)).mag() < 10 && closest != rope.editingNode) {
                        rope.Insert(ScreenToWorld((olc::vd2d(GetMouseX(), GetMouseY()) - camera) / scale), closest);
                    } else {
                        rope.Insert(ScreenToWorld((olc::vd2d(GetMouseX(), GetMouseY()) - camera) / scale), NULL);
                    }
                }
            }

            if (GetKey(olc::Key::B).bPressed) {
                drawNodes = !drawNodes;
            }
            if (GetKey(olc::Key::A).bPressed) {
                airResistance = !airResistance;
            }
            if (GetKey(olc::Key::SPACE).bPressed) {
                paused = !paused;
                if (!paused) editing = false;
            }
            if (GetKey(olc::Key::E).bPressed) {
                editing = !editing;
                paused = !paused;
                if (editing) paused = true;
            }
            if (GetKey(olc::Key::DEL).bPressed) {
                rope.Delete();
            }
            if (GetMouseWheel() > 0) rope.Increase();
            if (GetMouseWheel() < 0) rope.Decrease();

        }
        if(!paused)
        Update(fElapsedTime);

        //Drawing Rope
        for (uint16_t i = 0; i < rope.springs.size(); i++) {
            DrawLine(calibrate(rope.springs.at(i).firstObject->pos), calibrate(rope.springs.at(i).secondObject->pos));
        }
        for (uint16_t i = 0; i < rope.nodes.size(); i++) {
            if (drawNodes)
                FillCircle(calibrate(rope.nodes.at(i)->pos), 2, olc::WHITE);
            if (rope.nodes.at(i)->locked)
                FillCircle(calibrate(rope.nodes.at(i)->pos), 3, olc::RED);
            if (editing && rope.nodes.at(i) == rope.editingNode)
                FillCircle(calibrate(rope.nodes.at(i)->pos), 2, olc::GREEN);
        }

        //HUD
        DrawString(olc::vi2d(10, 0), "Air Resistance: " + std::to_string(airResistance), airResistance ? olc::GREEN : olc::RED);
        DrawString(olc::vi2d(10, 10), "Number of joints: " + std::to_string(rope.nodes.size()), olc::YELLOW);
        DrawString(olc::vi2d(10, 20), "Edit Mode: " + std::to_string(editing), editing ? olc::GREEN : olc::RED);
        DrawString(olc::vi2d((int32_t)(ScreenWidth()*0.8f), 0), paused ? "PAUSED" : "", paused ? olc::RED : olc::WHITE);
        return true;
    }

    olc::vd2d calibrate(olc::vd2d pos) {
        return WorldToScreen((pos - (centre))*scale + camera);
    }
    olc::vd2d WorldToScreen(olc::vd2d c) {
        return olc::vd2d(c.x + ScreenWidth() / 2, c.y + ScreenHeight() / 2);
    }
    olc::vd2d ScreenToWorld(olc::vd2d c) {
        return olc::vd2d(c.x - ScreenWidth() / 2, c.y - ScreenHeight() / 2);
    }

    void CollideWorldEdge() {
        for (uint16_t i = 0; i < rope.nodes.size(); i++) {
            if (calibrate(rope.nodes.at(i)->pos).x >= ScreenWidth()) {
                rope.nodes.at(i)->vel = rope.nodes.at(i)->vel - rope.nodes.at(i)->vel.dot(olc::vd2d(-1, 0))*olc::vd2d(-1, 0);
            }
            if (calibrate(rope.nodes.at(i)->pos).x <= 0) {
                rope.nodes.at(i)->vel = rope.nodes.at(i)->vel - rope.nodes.at(i)->vel.dot(olc::vd2d( 1, 0))*olc::vd2d(-1, 0);
            }
            if (calibrate(rope.nodes.at(i)->pos).y <= 0) {
                rope.nodes.at(i)->vel = rope.nodes.at(i)->vel - rope.nodes.at(i)->vel.dot(olc::vd2d(0, 1))*olc::vd2d(-1, 0);

            }
            if (calibrate(rope.nodes.at(i)->pos).y >= ScreenHeight()) {
                rope.nodes.at(i)->vel = rope.nodes.at(i)->vel - rope.nodes.at(i)->vel.dot(olc::vd2d(0,-1))*olc::vd2d(-1, 0);

            }
        }
    }
    void Update(float fElapsedTime) {
        for (uint16_t i = 0; i < rope.nodes.size(); i++) {
            if (!rope.nodes.at(i)->locked) {
                //Gravity
                rope.nodes.at(i)->ApplyForce(gravitationalAcceleration*rope.nodes.at(i)->mass);
                //Air resistance
                if(airResistance)
                rope.nodes.at(i)->ApplyForce(rope.nodes.at(i)->vel *-0.1f*rope.nodes.at(i)->mass);
            }
            rope.nodes.at(i)->Update(fElapsedTime);
            rope.nodes.at(i)->ResetForce();
        }        

        //Updating springs multiple times for stability
        for (uint16_t k = 0; k < 3; k++) {
            for (uint16_t i = 0; i < 8; i++) {
                for (uint16_t j = 0; j < rope.springs.size(); j++) {
                    rope.springs.at(j).Update(fElapsedTime);
                }
            }
            
            for (uint16_t j = 0; j != rope.springs.size(); j++) {
                //float damping = pow(0.5f, fElapsedTime);
                float damping = (1-pow(0.0f, fElapsedTime))*0.5f;
                rope.springs.at(j).ApplyDamping(damping, fElapsedTime);
            }
            
        }
    }
};
int main() {
    RopePhysics demo;
    if (demo.Construct(256, 240, 4, 4))
        demo.Start();
    return 0;
}