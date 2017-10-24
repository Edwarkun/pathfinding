#pragma once
#include <vector>
#include <time.h>
#include "Scene.h"
#include "Agent.h"
#include "Path.h"
#include <queue>

class ScenePathFinding :
	public Scene
{
public:
	ScenePathFinding();
	~ScenePathFinding();
	void update(float dtime, SDL_Event *event);
	void draw();
	const char* getTitle();
private:
	std::vector<Agent*> agents;

	Vector2D coinPosition;
	Vector2D currentTarget;
	int currentTargetIndex;
	Path path;
	int num_cell_x;
	int num_cell_y;
	bool draw_grid;
	std::vector<SDL_Rect> maze_rects;
	void drawMaze();
	void drawCoin();
	SDL_Texture *background_texture;
	SDL_Texture *coin_texture;
	void initMaze();
	bool loadTextures(char* filename_bg, char* filename_coin);
	std::vector< std::vector<int> > terrain;
	Vector2D cell2pix(Vector2D cell);
	Vector2D pix2cell(Vector2D pix);
	bool isValidCell(Vector2D cell);
	void CreateGrid(const std::vector<std::vector<int>>& maze);

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
		inline void AddNB(Node* newNB) { NB.push_back(newNB);}
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

	std::vector<Node*> grid;
};
