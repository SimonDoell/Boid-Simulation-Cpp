#include <SFML/Graphics.hpp>
#include <random>
#include <iostream>
#include <vector>
#include <list>

const int HEIGHT = 1010;
const int WIDTH = 1920;
const int MAX_FRAMES = 60;
const int boids_amount = 500;
//The speed the boids move at (forced)
const float CONSTANT_BOID_SPEED = 3.0f;


//SEPERATION:
//How large the repelling force can be at most
const float MAX_FORCE = 10;
//Amplitude for how strong the repelling force between the boids is
const float FORCE_AMPLITUDE = 250.0f;


//ALIGNMENT:
//Aplitude for how string the multiplier for the inverse square distance is
const float AVAREGE_FORCE_AMPLITUDE = 5.0f;
const float ALIGNMENT_RADIUS = 75.0f;


//COHESION:
//Defines the radius of how many boids to interact with to steer to the center
const float COHESION_RADIUS = 150.0f;
const float COHESION_AMPLIFIER = 10.0f;






sf::Vector2f getRandomVel() {
    float rotation = (rand()%200*M_PI)/100;
    return sf::Vector2f(sin(rotation), cos(rotation));
};


class Boid {
    private:
        float radius;
        float rotation;
        sf::Vector2f velocity;
        sf::Vector2f pos;
        sf::CircleShape obj;

    public:
        Boid(float _radius, sf::Vector2f _pos, sf::Vector2f _velocity, float _rotation) : radius(_radius), pos(_pos), rotation(_rotation), velocity(_velocity) {
            obj.setPointCount(3);
            obj.setOrigin(radius/2, radius/2);
            obj.setFillColor(sf::Color(14, 109+rand()%50, 150+rand()%100, 255));
            obj.setRadius(radius);
            obj.scale(0.6, 1);
        };


        sf::Vector2f getPos() {
            return pos;
        }

        sf::Vector2f getVelocity() {
            return velocity;
        }

        float getRotation() {
            float rot = int(rotation) % 360;
            if (rot < 0) {
                rot += 360;
            }

            return rot;
        }

        void alignRotationWithVel() {
            float angleRadians = atan2(velocity.y, velocity.x);
            float angleDegree = angleRadians * 180 / M_PI;
            rotation = angleDegree+90;
        }

        void translateVel(sf::Vector2f tranlation) {
            velocity += tranlation;
        };

        void changeColor(int r, int g, int b, int a) {
            obj.setFillColor(sf::Color(r, g, b, a));
        }

        void move() {

            float velocityLength = sqrt(velocity.x*velocity.x + velocity.y*velocity.y);
            velocity = sf::Vector2f(velocity.x/velocityLength*CONSTANT_BOID_SPEED, velocity.y/velocityLength*CONSTANT_BOID_SPEED);

            pos += sf::Vector2f(velocity);
            obj.setRotation(rotation);
            if (pos.x > WIDTH) {pos.x = 0;} else if (pos.x < 0) {pos.x = WIDTH;}
            if (pos.y > HEIGHT) {pos.y = 0;} else if (pos.y < 0) {pos.y = HEIGHT;}

            obj.setPosition(pos);
        }

        void moveForward(float steps) {
            sf::Vector2f moveVector = sf::Vector2f(sin(rotation/180*M_PI)*steps, -cos(rotation/180*M_PI)*steps);
            pos += moveVector;
            move();
        }

        void render(sf::RenderWindow& window) {
            window.draw(obj);
        }

};







int main() {
    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "Boids - Simulation", sf::Style::Titlebar | sf::Style::Close);
    window.setFramerateLimit(MAX_FRAMES);
    sf::Event ev;
    sf::ContextSettings settings;
    settings.antialiasingLevel = 2;



    std::vector<Boid> boids;


    for (int i = 0; i < boids_amount; ++i) {
        boids.emplace_back(Boid(10.0f, sf::Vector2f(rand()%WIDTH, rand()%HEIGHT), getRandomVel(), 25*i));
    }





    while (window.isOpen()) {
        //Window Management:
        while (window.pollEvent(ev)) {
            switch(ev.type) {
            case sf::Event::Closed:
                window.close();
                break;
            }
        }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Escape)) window.close();
    

        
        for (int i = 0; i < boids.size(); ++i) {
            //Repelling Force Vars
            sf::Vector2f moveVector(0, 0);
            int neighbours = 0;
            //Aligning Force Vectors
            std::vector<float> neighbourVectorX = {};
            std::vector<float> neighbourVectorY = {};
            float avaregeNeighbourVectorX = 0.0f;
            float avaregeNeighbourVectorY = 0.0f;
            //Cohesion Force Vectors:
            std::vector<sf::Vector2f> cohesionForcePosVector = {};


            for (int ii = 0; ii < boids.size(); ++ii) {
                if (i != ii) {

                    sf::Vector2f selectedBoidPos = boids[ii].getPos();

                    float dx = boids[i].getPos().x - selectedBoidPos.x;
                    float dy = boids[i].getPos().y - selectedBoidPos.y;

                    //Wrap-around correction for x
                    if (abs(dx) > WIDTH/2) {
                        if (dx > 0) {
                            dx -= WIDTH;
                        } else {
                            dx += WIDTH;
                        }
                    }
                    //Wrap-around correction for y
                    if (abs(dy) > HEIGHT/2) {
                        if (dy > 0) {
                            dy -= HEIGHT;
                        } else {
                            dy += HEIGHT;
                        }
                    }

                    float distance = sqrt(dx*dx + dy*dy);

                    
                    
                    if (distance < ALIGNMENT_RADIUS) {
                        float inverseSquareDropoff = 1.0f/distance;
                        sf::Vector2f selectedBoidVel = boids[ii].getVelocity();
                        neighbourVectorX.emplace_back(selectedBoidVel.x * inverseSquareDropoff);
                        neighbourVectorY.emplace_back(selectedBoidVel.y * inverseSquareDropoff);
                    }

                    if (distance < COHESION_RADIUS) {
                        cohesionForcePosVector.emplace_back(selectedBoidPos);
                    }

                    

                    if (distance > 0) {
                        sf::Vector2f posDiff(dx, dy);
                        float posDiffLength = distance;
                        sf::Vector2f posDiffNorm = sf::Vector2f(dx / posDiffLength, dy / posDiffLength);

                        sf::Vector2f separationVector = sf::Vector2f(posDiffNorm.x * (1.0f/distance), posDiffNorm.y * (1.0f/distance));
                        moveVector += separationVector;
                        neighbours++;
                    }
                }
            }



            //Alignment Force Loops:
            float floatXSumAvarege = 0.0f;
            float floatYSumAvarege = 0.0f;

            int vectorXAmount = neighbourVectorX.size();

            if (vectorXAmount > 0) {
                for (int i = 0; i < vectorXAmount; ++i) {
                    floatXSumAvarege += neighbourVectorX[i];
                    floatYSumAvarege += neighbourVectorY[i];
                }

                floatXSumAvarege /= vectorXAmount;
                floatYSumAvarege /= neighbourVectorY.size();
            }


            //Cohesion Force Loops:
            int cohesionForceAmount = cohesionForcePosVector.size();
            
            float cohesionAvaregePosX = 0.0f;
            float cohesionAvaregePosY = 0.0f;
            sf::Vector2f cohesionVector(0, 0);
            
            
            if (cohesionForceAmount > 0) {

                if (cohesionForceAmount > 0) {
                    for (int i = 0; i < cohesionForceAmount; ++i) {
                        sf::Vector2f boidPos = cohesionForcePosVector[i];
                        cohesionAvaregePosX += boidPos.x;
                        cohesionAvaregePosY += boidPos.y;
                    }
                }

                cohesionAvaregePosX /= cohesionForceAmount;
                cohesionAvaregePosY /= cohesionForceAmount;

                cohesionVector = sf::Vector2f(cohesionAvaregePosX-boids[i].getPos().x, cohesionAvaregePosY-boids[i].getPos().y);
                float cohesionVectorLength = sqrt(cohesionVector.x*cohesionVector.x + cohesionVector.y*cohesionVector.y);
                cohesionVector = sf::Vector2f(cohesionVector.x/cohesionVectorLength, cohesionVector.y/cohesionVectorLength);
            }





            if (neighbours > 0) {
                moveVector /= float(neighbours);

                float forceLength = sqrt(moveVector.x*moveVector.x + moveVector.y*moveVector.y);
                if (forceLength > MAX_FORCE) {
                    moveVector = (moveVector / forceLength) * MAX_FORCE;
                }

                moveVector *= FORCE_AMPLITUDE;
                moveVector += sf::Vector2f(floatXSumAvarege*AVAREGE_FORCE_AMPLITUDE, floatYSumAvarege*AVAREGE_FORCE_AMPLITUDE);
                moveVector += cohesionVector /= COHESION_AMPLIFIER;

                boids[i].translateVel(moveVector);
            };
        }



        for (int i = 0; i < boids.size(); ++i) {
            boids[i].alignRotationWithVel();
            boids[i].move();
        }






        window.clear();
        for (int i = 0; i < boids.size(); ++i) {
            boids[i].render(window);
        }
        window.display();

    }




    return 0;
}
