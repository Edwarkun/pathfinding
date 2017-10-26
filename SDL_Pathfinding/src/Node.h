#pragma once
#include "SDL_SimpleApp.h"
#include <vector>
class Node {
private:
	Vector2D position;
	float cost;
	std::vector<Node*> NB;
public:
	Node(const Vector2D& pos, const float& cst) : position(pos), cost(cst) {};
	inline void SetPosition(Vector2D newPos) { position = newPos; }
	inline Vector2D GetPosition() { return position; }
	inline float GetCost() { return cost; }
	inline void SetCost(float newCost) { cost = newCost; }
	inline void AddNB(Node* newNB) { NB.push_back(newNB); }
	inline std::vector<Node*> GetNB() { return NB; }
	void RemoveNB(Node* targetNB) {
		for (int i = 0; i < NB.size(); i++) {
			if (NB[i] == targetNB) {
				NB.erase(NB.begin() + i);
				break;
			}
		}
	}
};