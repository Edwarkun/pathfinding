#pragma once
#include <vector>
#include <time.h>
#include "Scene.h"
#include "Agent.h"
#include "Path.h"
#include <queue>
#include "Node.h"
#include "Text.h"
#include <chrono>

class SceneEnemies :
	public Scene
{
public:
	SceneEnemies();
	~SceneEnemies();
	void update(float dtime, SDL_Event *event);
	void draw();
	const char* getTitle();
private:
	std::vector<Agent*> agents;

	Vector2D coinPosition, enemTarget;
	Vector2D currentTarget, currentTargetEnemies;
	int currentTargetIndex, currentTargetEnemiesIndex;
	Path path, pathEnemies;
	int num_cell_x;
	int num_cell_y;
	bool draw_grid;
	std::vector<SDL_Rect> maze_rects;
	void drawMaze();
	void drawCoin(const Vector2D&);
	SDL_Texture *background_texture;
	SDL_Texture *coin_texture;
	void initMaze();
	bool loadTextures(char* filename_bg, char* filename_coin);
	std::vector< std::vector<int> > terrain;
	Vector2D cell2pix(Vector2D cell);
	Vector2D pix2cell(Vector2D pix);
	bool isValidCell(Vector2D cell);
	void CreateGrid(const std::vector<std::vector<int>>& maze);
	void ModifyGrid(const Vector2D&);


	Text* info1;
	Text* info2;
	Text* info3;
	std::vector<Node*> grid;
	std::vector<Vector2D> floodFill;
	std::vector<Vector2D> frontier;
	std::vector<Vector2D> multipleTargets;
	std::vector<Vector2D> remainingTargets;
	std::vector<Node*> modifyedNodes;
	int numberOfTargets;
	bool newDynamicPath;
};