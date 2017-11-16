#pragma once
#include <iostream>
#include <minmax.h>
#include <SDL.h>
#include <SDL_image.h>
#include "SDL_SimpleApp.h"
#include "Vector2D.h"
#include "utils.h"
#include "SteeringBehavior.h"
#include "Node.h"
#include "Path.h"
#include <queue> 
#include <functional>
#include <unordered_map>
#include <map>

enum PathfindingType {BREATH_FIRST_SEARCH, DIJKSTRA, GREEDY_BFG, A_STAR};

class Agent
{
	friend class SteeringBehavior;

private:
	SteeringBehavior *steering_behavior;
	Vector2D position;
	Vector2D velocity;
	Vector2D target;

	float mass;
	float orientation;
	float max_force;
	float max_velocity;

	SDL_Color color;

	SDL_Texture *sprite_texture;
	bool draw_sprite;
	int sprite_num_frames;
	int sprite_w;
	int sprite_h;

public:
	Agent();
	~Agent();
	SteeringBehavior *Behavior();
	Vector2D getPosition();
	Vector2D getTarget();
	Vector2D getVelocity();
	float getMaxVelocity();
	void setPosition(Vector2D position);
	void setTarget(Vector2D target);
	void setVelocity(Vector2D velocity);
	void setMass(float mass);
	void setColor(Uint8 r, Uint8 g, Uint8 b, Uint8 a);
	void update(Vector2D steering_force, float dtime, SDL_Event *event);
	void draw();
	Path FindPath(const std::vector<Node*>&, const  Vector2D&, const Vector2D&, const PathfindingType&, std::vector<Vector2D>& floodFillDraw, std::vector<Vector2D>& frontierDraw);
	float heuristic(Node* fromN, Node* toN);
	bool Agent::loadSpriteTexture(char* filename, int num_frames=1);
	
};

class CompareDist
{
public:
	bool operator()(std::pair<int, Node*> n1, std::pair<int, Node*> n2) {
		return n1.first > n2.first;
	}
};
