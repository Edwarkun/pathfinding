#include "Agent.h"

using namespace std;

Agent::Agent() : sprite_texture(0),
                 position(Vector2D(100, 100)),
	             target(Vector2D(1000, 100)),
	             velocity(Vector2D(0,0)),
	             mass(0.1f),
	             max_force(150),
	             max_velocity(200),
	             orientation(0),
	             color({ 255,255,255,255 }),
				 sprite_num_frames(0),
	             sprite_w(0),
	             sprite_h(0),
	             draw_sprite(false)
{
	steering_behavior = new SteeringBehavior;
}

Agent::~Agent()
{
	if (sprite_texture)
		SDL_DestroyTexture(sprite_texture);
	if (steering_behavior)
		delete (steering_behavior);
}

SteeringBehavior * Agent::Behavior()
{
	return steering_behavior;
}

Vector2D Agent::getPosition()
{
	return position;
}

Vector2D Agent::getTarget()
{
	return target;
}

Vector2D Agent::getVelocity()
{
	return velocity;
}

float Agent::getMaxVelocity()
{
	return max_velocity;
}

void Agent::setPosition(Vector2D _position)
{
	position = _position;
}

void Agent::setTarget(Vector2D _target)
{
	target = _target;
}

void Agent::setVelocity(Vector2D _velocity)
{
	velocity = _velocity;
}

void Agent::setMass(float _mass)
{
	mass = _mass;
}

void Agent::setColor(Uint8 r, Uint8 g, Uint8 b, Uint8 a)
{
	color = { r, g, b, a };
}

void Agent::update(Vector2D steering_force, float dtime, SDL_Event *event)
{

	//cout << "agent update:" << endl;

	switch (event->type) {
		/* Keyboard & Mouse events */
	case SDL_KEYDOWN:
		if (event->key.keysym.scancode == SDL_SCANCODE_SPACE)
			draw_sprite = !draw_sprite;
		break;
	default:
		break;
	}


	Vector2D acceleration = steering_force / mass;
	velocity = velocity + acceleration * dtime;
	velocity = velocity.Truncate(max_velocity);

	position = position + velocity * dtime;


	// Update orientation
	if (velocity.Length()>0)
		orientation = (float)(atan2(velocity.y, velocity.x) * RAD2DEG);


	// Trim position values to window size
	if (position.x < 0) position.x = TheApp::Instance()->getWinSize().x;
	if (position.y < 0) position.y = TheApp::Instance()->getWinSize().y;
	if (position.x > TheApp::Instance()->getWinSize().x) position.x = 0;
	if (position.y > TheApp::Instance()->getWinSize().y) position.y = 0;
}

void Agent::draw()
{
	if (draw_sprite)
	{
		Uint32 sprite;
		
		if (velocity.Length() < 5.0)
			sprite = 1;
		else
			sprite = (int)(SDL_GetTicks() / (max_velocity)) % sprite_num_frames;
		
		SDL_Rect srcrect = { (int)sprite * sprite_w, 0, sprite_w, sprite_h };
		SDL_Rect dstrect = { (int)position.x - (sprite_w / 2), (int)position.y - (sprite_h / 2), sprite_w, sprite_h };
		SDL_Point center = { sprite_w / 2, sprite_h / 2 };
		SDL_RenderCopyEx(TheApp::Instance()->getRenderer(), sprite_texture, &srcrect, &dstrect, orientation+90, &center, SDL_FLIP_NONE);
	}
	else 
	{
		draw_circle(TheApp::Instance()->getRenderer(), (int)position.x, (int)position.y, 15, color.r, color.g, color.b, color.a);
		SDL_RenderDrawLine(TheApp::Instance()->getRenderer(), (int)position.x, (int)position.y, (int)(position.x+15*cos(orientation*DEG2RAD)), (int)(position.y+15*sin(orientation*DEG2RAD)));
	}
}

bool Agent::loadSpriteTexture(char* filename, int _num_frames)
{
	if (_num_frames < 1) return false;

	SDL_Surface *image = IMG_Load(filename);
	if (!image) {
		cout << "IMG_Load: " << IMG_GetError() << endl;
		return false;
	}
	sprite_texture = SDL_CreateTextureFromSurface(TheApp::Instance()->getRenderer(), image);

	sprite_num_frames = _num_frames;
	sprite_w = image->w / sprite_num_frames;
	sprite_h = image->h;
	draw_sprite = true;

	if (image)
		SDL_FreeSurface(image);

	return true;
}

Path  Agent::FindPath(const std::vector<Node*>& grid, const Vector2D& startPosition, const Vector2D& finishPosition,const PathfindingType& algorithm, std::vector<Vector2D>& floodFillDraw, std::vector<Vector2D>& frontierDraw) {
	Path path;
	Node* start = nullptr;
	Node* finish = nullptr;

	bool startFound = false;
	bool finishFound = false;
	//In order to prevent an inverted path, we swap the finish and start variables
	//If we don't sawp them, we must use this line of code before returning the path (needs #include <algorithm>):
	//std::reverse(path.points.begin(), path.points.end());
	for (int i = 0; i < grid.size(); i++) {
		if (grid[i]->GetPosition() == startPosition && !startFound) {
			start = grid[i];
			startFound = true;
		}
		if (grid[i]->GetPosition() == finishPosition && !finishFound) {
			finish = grid[i];
			finishFound = true;
		}
	}
	if (!startFound || !finishFound ) {
		return path;
	}

	switch (algorithm) {
		{
		case BREATH_FIRST_SEARCH:
			std::queue<Node*> frontier;
			std::unordered_map<Node*, Node*> cameFrom;
			frontier.push(start);
			cameFrom[start] = nullptr;
			while (frontier.size()) {
				Node* current = frontier.front();

				if (current == finish) {
					while (current != start) {
						path.points.push_back(current->GetPosition());
						current = cameFrom[current];
					}
					path.points.push_back(current->GetPosition());
					std::reverse(path.points.begin(), path.points.end());
					while (frontier.size()) {
						frontierDraw.push_back(frontier.front()->GetPosition());
						frontier.pop();
					}
					return path;
				}

				std::vector<Node*> currentNB = current->GetNB();
				for (int i = 0; i < currentNB.size(); i++) {

					bool visited = false;
					Node* next = currentNB[i];

					for (int j = 0; j < cameFrom.size(); j++) {
						if (cameFrom.find(next) != cameFrom.end()) {
							visited = true;
						}
					}
					if (!visited) {
						cameFrom[next] = current;
						frontier.push(next);
						floodFillDraw.push_back(next->GetPosition());
					}
				}
				frontier.pop();
			}

			break;
		}
		{
		case DIJKSTRA:
			std::priority_queue<std::pair<int, Node*>, std::vector<std::pair<int, Node*>>, CompareDist> frontier;
			std::unordered_map<Node*, Node*> cameFrom;
			std::unordered_map<Node*, int> costSoFar;

			frontier.emplace(std::make_pair(0,start));
			cameFrom[start] = nullptr;
			costSoFar[start] = 0;
			while (frontier.size()) {
				Node* current = frontier.top().second;

				if (current == finish) {
					while (current != start) {
						path.points.push_back(current->GetPosition());
						current = cameFrom[current];
					}
					path.points.push_back(current->GetPosition());
					std::reverse(path.points.begin(), path.points.end());
					while(frontier.size()) {
						frontierDraw.push_back(frontier.top().second->GetPosition());
						frontier.pop();
					}
					return path;
				}

				std::vector<Node*> currentNB = current->GetNB();
				std::vector<std::pair<float, Node*>> newNodes;
				for (int i = 0; i < currentNB.size(); i++) {

					Node* next = currentNB[i];

					int newCost = costSoFar[current] + next->GetCost();
					if (costSoFar.find(next) == costSoFar.end() || newCost < costSoFar[next]) {
						costSoFar[next] = newCost;
						newNodes.push_back(std::make_pair(newCost, next));
						cameFrom[next] = current;
						floodFillDraw.push_back(next->GetPosition());
					}
				}
				frontier.pop();
				for (int i = 0; i < newNodes.size(); i++) {
					frontier.emplace(newNodes[i]);
				}
			}
			break;
		}
		{
		case GREEDY_BFG:
			std::priority_queue<std::pair<int, Node*>, std::vector<std::pair<int, Node*>>, CompareDist> frontier;
			std::unordered_map<Node*, Node*> cameFrom;

			float lowestCost = 0;
			frontier.emplace(lowestCost, start);
			cameFrom[start] = nullptr;


			while (frontier.size()) {

				Node* current = frontier.top().second;

				if (current == finish) {
					while (current != start) {
						path.points.push_back(current->GetPosition());
						current = cameFrom[current];
					}
					path.points.push_back(current->GetPosition());
					std::reverse(path.points.begin(), path.points.end());
					while (frontier.size()) {
						frontierDraw.push_back(frontier.top().second->GetPosition());
						frontier.pop();
					}
					return path;
				}

				std::vector<Node*> currentNB = current->GetNB();
				std::vector<std::pair<float, Node*>> newNodes;
				for (int i = 0; i < currentNB.size(); i++) {

					Node* next = currentNB[i];	

					if (cameFrom.find(next) == cameFrom.end()) {
						float newCost = heuristic(next, finish);
						newNodes.push_back(std::make_pair(newCost, next));
						cameFrom[next] = current;
						floodFillDraw.push_back(next->GetPosition());
					}
				}
				frontier.pop();
				for (int i = 0; i < newNodes.size(); i++) {
					frontier.emplace(newNodes[i]);
				}
			}
			break;
		}
		{
		case A_STAR:
			std::priority_queue<std::pair<int, Node*>, std::vector<std::pair<int, Node*>>, CompareDist> frontier;
			std::unordered_map<Node*, Node*> cameFrom;
			std::unordered_map<Node*, int> costSoFar;

			frontier.emplace(std::make_pair(0, start));
			cameFrom[start] = nullptr;
			costSoFar[start] = 0;
			while (frontier.size()) {
				Node* current = frontier.top().second;

				if (current == finish) {
					while (current != start) {
						path.points.push_back(current->GetPosition());
						current = cameFrom[current];
					}
					path.points.push_back(current->GetPosition());
					std::reverse(path.points.begin(), path.points.end());
					while (frontier.size()) {
						frontierDraw.push_back(frontier.top().second->GetPosition());
						frontier.pop();
					}
					return path;
				}

				std::vector<Node*> currentNB = current->GetNB();
				std::vector<std::pair<float, Node*>> newNodes;
				for (int i = 0; i < currentNB.size(); i++) {

					Node* next = currentNB[i];

					int newCost = costSoFar[current] + next->GetCost();
					if (costSoFar.find(next) == costSoFar.end() || newCost < costSoFar[next]) {
						costSoFar[next] = newCost;
						float priority = newCost + heuristic(finish, next);
						newNodes.push_back(std::make_pair(priority, next));
						cameFrom[next] = current;
						floodFillDraw.push_back(next->GetPosition());
					}
				}

				frontier.pop();
				for (int i = 0; i < newNodes.size(); i++) {
					frontier.emplace(newNodes[i]);
				}
			}
			break;
		}
		default:
			break;
	}
	if (!path.points.size()) {
		int a = 0;
	}
	return path;
}

float Agent::heuristic(Node* fromN, Node* toN) {
	float a = abs(toN->GetPosition().x - fromN->GetPosition().x);
	float b = abs(toN->GetPosition().y - fromN->GetPosition().y);
	float distance = a + b;

	//float distance = sqrt(pow(toN->GetPosition().x - fromN->GetPosition().x,2) + pow(toN->GetPosition().y  - fromN->GetPosition().y,2));
	return distance;
}
