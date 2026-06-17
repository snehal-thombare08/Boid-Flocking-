#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>

const int WIDTH = 1200, HEIGHT = 800;
const int NUM_BOIDS = 300;

const float MAX_SPEED = 200.f;
const float MIN_SPEED = 80.f;
const float SEP_RADIUS = 30.f;
const float ALI_RADIUS = 90.f;
const float COH_RADIUS = 130.f;
const float SEP_FORCE = 2.2f;
const float ALI_FORCE = 1.0f;
const float COH_FORCE = 0.9f;
const float PREDATOR_RADIUS = 150.f;
const float PREDATOR_FORCE = 4.5f;
const float FORCE_SCALE = 28.f;   // tuned: strong enough to flock, not so strong it pins to max speed

struct Vec2 {
    float x, y;
    Vec2(float x=0, float y=0): x(x), y(y){}
    Vec2 operator+(const Vec2& b) const { return {x+b.x, y+b.y}; }
    Vec2 operator-(const Vec2& b) const { return {x-b.x, y-b.y}; }
    Vec2 operator*(float t) const { return {x*t, y*t}; }
    Vec2& operator+=(const Vec2& b) { x+=b.x; y+=b.y; return *this; }
    float len() const { return std::sqrt(x*x+y*y); }
    Vec2 norm() const { float l=len(); return l>0?Vec2{x/l,y/l}:Vec2{0,0}; }
    float dist(const Vec2& b) const { return (*this-b).len(); }
};

float randF(float lo, float hi) {
    return lo + (hi-lo)*(rand()/(float)RAND_MAX);
}

struct Boid {
    Vec2 pos, vel;
    float hue;

    Boid() {
        pos = {randF(0,(float)WIDTH), randF(0,(float)HEIGHT)};
        float angle = randF(0, 6.28f);
        vel = {std::cos(angle)*randF(MIN_SPEED, MAX_SPEED),
               std::sin(angle)*randF(MIN_SPEED, MAX_SPEED)};
        hue = randF(0, 360.f);
    }

    sf::Color getColor(float speed) const {
        float t = (speed - MIN_SPEED) / (MAX_SPEED - MIN_SPEED);
        t = std::max(0.f, std::min(1.f, t));
        // Slow=blue, fast=red/orange
        uint8_t r = (uint8_t)(50 + t*205);
        uint8_t g = (uint8_t)(150 - t*100);
        uint8_t b2 = (uint8_t)(255 - t*200);
        return sf::Color(r, g, b2, 220);
    }
};

std::vector<Boid> boids;

void update(float dt, Vec2 mouse, bool predatorOn, bool scatter) {
    // Snapshot velocities before mutating, so alignment uses last frame's headings consistently
    std::vector<Vec2> oldVel(boids.size());
    for(size_t i=0;i<boids.size();i++) oldVel[i] = boids[i].vel;

    for(size_t i=0;i<boids.size();i++) {
        Boid& b = boids[i];
        Vec2 sep{0,0}, ali{0,0}, coh{0,0};
        int sepCount=0, aliCount=0, cohCount=0;

        for(size_t j=0;j<boids.size();j++) {
            if(i==j) continue;
            Boid& other = boids[j];
            float d = b.pos.dist(other.pos);

            if(d < SEP_RADIUS && d > 0.001f) {
                // Stronger push the closer they are (inverse-distance weighting)
                Vec2 away = (b.pos - other.pos).norm() * ((SEP_RADIUS - d) / SEP_RADIUS);
                sep += away;
                sepCount++;
            }
            if(d < ALI_RADIUS) {
                ali += oldVel[j];
                aliCount++;
            }
            if(d < COH_RADIUS) {
                coh += other.pos;
                cohCount++;
            }
        }

        // Desired heading = priority-weighted blend, not flat addition.
        // Separation matters most (avoid collisions), then alignment (match the group),
        // then cohesion (drift toward the group center) — each as a UNIT direction,
        // weighted, then renormalized. This stops the three forces from cancelling out.
        Vec2 desired{0,0};
        float totalWeight = 0.f;

        if(sepCount > 0) {
            Vec2 sepDir = sep.norm();
            desired += sepDir * SEP_FORCE;
            totalWeight += SEP_FORCE;
        }
        if(aliCount > 0) {
            Vec2 aliDir = (ali * (1.f/aliCount)).norm();
            desired += aliDir * ALI_FORCE;
            totalWeight += ALI_FORCE;
        }
        if(cohCount > 0) {
            Vec2 center = coh * (1.f/cohCount);
            Vec2 cohDir = (center - b.pos).norm();
            desired += cohDir * COH_FORCE;
            totalWeight += COH_FORCE;
        }

        Vec2 steerDir = b.vel.norm(); // fallback: keep current heading
        if(totalWeight > 0.f) steerDir = (desired * (1.f/totalWeight)).norm();

        // Predator overrides everything else when active and close
        if(predatorOn) {
            float d = b.pos.dist(mouse);
            if(d < PREDATOR_RADIUS && d > 0.001f) {
                Vec2 away = (b.pos - mouse).norm();
                float urgency = (PREDATOR_RADIUS - d) / PREDATOR_RADIUS; // 0..1, stronger when closer
                steerDir = (steerDir * (1.f-urgency) + away * urgency).norm();
            }
        }

        if(scatter) {
            steerDir = (steerDir + Vec2{randF(-1.f,1.f), randF(-1.f,1.f)} * 0.6f).norm();
        }

        // Turn rate limited steering: blend current heading toward desired heading
        // gradually instead of snapping, so motion looks fluid not jittery.
        float turnRate = 6.0f; // higher = sharper turns
        Vec2 curDir = b.vel.norm();
        Vec2 blendedDir = (curDir + (steerDir - curDir) * std::min(1.f, turnRate*dt)).norm();

        // Speed varies gently around a cruising speed instead of hard-clamping every frame
        float curSpeed = b.vel.len();
        float targetSpeed = (MIN_SPEED + MAX_SPEED) * 0.5f;
        if(sepCount > 2) targetSpeed = MAX_SPEED;          // crowded -> speed up to escape
        if(predatorOn && b.pos.dist(mouse) < PREDATOR_RADIUS) targetSpeed = MAX_SPEED;
        float newSpeed = curSpeed + (targetSpeed - curSpeed) * std::min(1.f, 3.f*dt);
        newSpeed = std::max(MIN_SPEED, std::min(MAX_SPEED, newSpeed));

        b.vel = blendedDir * newSpeed;
        b.pos += b.vel * dt;

        // Wrap edges
        if(b.pos.x < 0) b.pos.x += WIDTH;
        if(b.pos.x > WIDTH) b.pos.x -= WIDTH;
        if(b.pos.y < 0) b.pos.y += HEIGHT;
        if(b.pos.y > HEIGHT) b.pos.y -= HEIGHT;
    }
}

int main() {
    sf::RenderWindow window(sf::VideoMode({(unsigned)WIDTH,(unsigned)HEIGHT}),
        "Boid Flocking | RMB: Predator | Space: Scatter | +/-: Boids | R: Reset");
    window.setFramerateLimit(60);

    srand(42);
    boids.resize(NUM_BOIDS);

    bool predatorOn = false;
    bool scatter = false;

    sf::Font font;
    bool hasFont = font.openFromFile("C:/Windows/Fonts/arial.ttf");
    sf::Text hud(font);
    hud.setCharacterSize(15);
    hud.setFillColor(sf::Color::White);
    hud.setOutlineColor(sf::Color::Black);
    hud.setOutlineThickness(1.5f);
    hud.setPosition({10.f, 10.f});

    // Trail texture
    sf::RenderTexture trailTex({(unsigned)WIDTH,(unsigned)HEIGHT});
    trailTex.clear(sf::Color(10,10,20));
    sf::Sprite trailSprite(trailTex.getTexture());

    // Fade overlay - higher alpha = faster fade = shorter trails
    sf::RectangleShape fadeRect({(float)WIDTH,(float)HEIGHT});
    fadeRect.setFillColor(sf::Color(10,10,20, 60));

    sf::Clock clock;

    while(window.isOpen()) {
        float dt = std::min(clock.restart().asSeconds(), 0.033f);
        auto mpos = sf::Mouse::getPosition(window);
        Vec2 mouse{(float)mpos.x, (float)mpos.y};

        scatter = false;
        while(auto ev = window.pollEvent()) {
            if(ev->is<sf::Event::Closed>()) window.close();
            if(auto* k = ev->getIf<sf::Event::KeyPressed>()) {
                if(k->code == sf::Keyboard::Key::Escape) window.close();
                if(k->code == sf::Keyboard::Key::R) { boids.clear(); boids.resize(NUM_BOIDS); }
                if(k->code == sf::Keyboard::Key::Space) scatter = true;
                if(k->code == sf::Keyboard::Key::Equal && boids.size()<500) {
                    for(int i=0;i<20;i++) boids.emplace_back();
                }
                if(k->code == sf::Keyboard::Key::Hyphen && boids.size()>20) {
                    for(int i=0;i<20;i++) boids.pop_back();
                }
            }
            if(auto* mb = ev->getIf<sf::Event::MouseButtonPressed>())
                if(mb->button == sf::Mouse::Button::Right) predatorOn = !predatorOn;
        }

        update(dt, mouse, predatorOn, scatter);

        // Draw trails - fade old frame first, no blend mode so it doesn't stack into solid streaks
        trailTex.draw(fadeRect, sf::RenderStates(sf::BlendNone));

        sf::VertexArray triangles(sf::PrimitiveType::Triangles);
        for(auto& b : boids) {
            float spd = b.vel.len();
            sf::Color col = b.getColor(spd);

            Vec2 dir = b.vel.norm();
            Vec2 perp{-dir.y, dir.x};

            float len = 12.f;
            float wid = 5.f;

            Vec2 tip  = b.pos + dir*len;
            Vec2 left = b.pos - dir*(len*0.3f) + perp*wid;
            Vec2 rite = b.pos - dir*(len*0.3f) - perp*wid;

            triangles.append(sf::Vertex{sf::Vector2f(tip.x,tip.y), col});
            triangles.append(sf::Vertex{sf::Vector2f(left.x,left.y), col});
            triangles.append(sf::Vertex{sf::Vector2f(rite.x,rite.y), col});
        }
        trailTex.draw(triangles);
        trailTex.display();

        window.clear(sf::Color(10,10,20));
        window.draw(trailSprite);

        // Predator indicator
        if(predatorOn) {
            sf::CircleShape pred(PREDATOR_RADIUS);
            pred.setOrigin({PREDATOR_RADIUS, PREDATOR_RADIUS});
            pred.setPosition({mouse.x, mouse.y});
            pred.setFillColor(sf::Color(255,0,0,20));
            pred.setOutlineColor(sf::Color(255,50,50,100));
            pred.setOutlineThickness(2.f);
            window.draw(pred);
        }

        if(hasFont) {
            hud.setString(
                "RMB: Predator " + std::string(predatorOn?"(ON) ":"(OFF)") +
                " | Space: Scatter | +/-: Add/Remove Boids | R: Reset | Esc: Quit" +
                "   Boids: " + std::to_string(boids.size())
            );
            window.draw(hud);
        }

        window.display();
    }
    return 0;
}


