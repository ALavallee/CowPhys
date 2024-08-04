#ifndef COWPHYS_COLLISION_H
#define COWPHYS_COLLISION_H


namespace cp {

class Body;

class Collision {

public:

    Collision(Body *collided) : mCollided(collided), mIsNew(true) {
    }

    void update() {
        mIsNew = false;
    }

    Body *getCollided() {
        return mCollided;
    }

    bool isNew() {
        return mIsNew;
    }

private:
    Body *mCollided;
    bool mIsNew;

};


}

#endif //COWPHYS_COLLISION_H
