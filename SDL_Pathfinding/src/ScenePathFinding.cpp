#include "ScenePathFinding.h"

using namespace std;

ScenePathFinding::ScenePathFinding()
{
	draw_grid = false;

	num_cell_x = SRC_WIDTH / CELL_SIZE;
	num_cell_y = SRC_HEIGHT / CELL_SIZE;
	initMaze();
	loadTextures("../res/maze.png", "../res/coin.png");


	srand((unsigned int)time(NULL));

	Agent *agent = new Agent;
	agent->loadSpriteTexture("../res/soldier.png", 4);
	agents.push_back(agent);


	// set agent position coords to the center of a random cell
	Vector2D rand_cell(-1,-1);
	while (!isValidCell(rand_cell)) 
		rand_cell = Vector2D((float)(rand() % num_cell_x), (float)(rand() % num_cell_y));
	agents[0]->setPosition(cell2pix(rand_cell));


	// set the coin in a random cell (but at least 3 cells far from the agent)
	coinPosition = Vector2D(-1, -1);
	while ((!isValidCell(coinPosition)) || (Vector2D::Distance(coinPosition, rand_cell)<3)) 
		coinPosition = Vector2D((float)(rand() % num_cell_x), (float)(rand() % num_cell_y));


	// PathFollowing next Target
	currentTarget = Vector2D(agent->getPosition().x, agent->getPosition().y);
	currentTargetIndex = -1;

	//Execute the pathfinding algorithm
	sceneState = 0;
	newSceneState = sceneState;
	ModifyGrid();
	info1 = new Text("", Vector2D(100, 30), TheApp::Instance()->getRenderer(), 30, false);
	info2 = new Text("", Vector2D(100, 80), TheApp::Instance()->getRenderer(), 30, false);
	info3 = new Text("", Vector2D(SRC_WIDTH / 2 + SRC_WIDTH / 4, 60), TheApp::Instance()->getRenderer(), 50, true);
	timeElapsed = 0;
}

ScenePathFinding::~ScenePathFinding()
{
	if (background_texture)
		SDL_DestroyTexture(background_texture);
	if (coin_texture)
		SDL_DestroyTexture(coin_texture);

	for (int i = 0; i < (int)agents.size(); i++)
	{
		delete agents[i];
	}
	delete info1;
	delete info2;
	delete info3;
}

void ScenePathFinding::update(float dtime, SDL_Event *event)
{
	//Prevent big dt
	if (dtime > 60 / 1000.f) {
		dtime = 60 / 1000.f;
	}
	/* Keyboard & Mouse events */
	switch (event->type) {
	case SDL_KEYDOWN:
		if (event->key.keysym.scancode == SDL_SCANCODE_SPACE)
			draw_grid = !draw_grid;
		if (event->key.keysym.scancode == SDL_SCANCODE_Z) {
			newSceneState = 0;
		}
		if (event->key.keysym.scancode == SDL_SCANCODE_X) {
			newSceneState = 1;
		}
		if (event->key.keysym.scancode == SDL_SCANCODE_C) {
			newSceneState = 2;
		}
		if (event->key.keysym.scancode == SDL_SCANCODE_V) {
			newSceneState = 3;
		}
		if (event->key.keysym.scancode == SDL_SCANCODE_B) {
			newSceneState = 4;
		}
		break;
	case SDL_MOUSEMOTION:
	case SDL_MOUSEBUTTONDOWN:
		/*if (event->button.button == SDL_BUTTON_LEFT)
		
		{
			Vector2D cell = pix2cell(Vector2D((float)(event->button.x), (float)(event->button.y)));
			if (isValidCell(cell))
			{
				if (path.points.size() > 0)
					if (path.points[path.points.size() - 1] == cell2pix(cell))
						break;

				path.points.push_back(cell2pix(cell));
			}
		}*/
		break;
	default:
		break;
	}
	if ((currentTargetIndex == -1) && (path.points.size()>0))
		currentTargetIndex = 0;

	if (currentTargetIndex >= 0)
	{	
		float dist = Vector2D::Distance(agents[0]->getPosition(), path.points[currentTargetIndex]);
		if (dist < path.ARRIVAL_DISTANCE)
		{
			if (currentTargetIndex == path.points.size() - 1)
			{
				if (dist < 3)
				{
					path.points.clear();
					currentTargetIndex = -1;
					agents[0]->setVelocity(Vector2D(0,0));
					// if we have arrived to the coin, replace it in a random cell!
					if (pix2cell(agents[0]->getPosition()) == coinPosition)
					{
						coinPosition = Vector2D(-1, -1);
						while ((!isValidCell(coinPosition)) || (Vector2D::Distance(coinPosition, pix2cell(agents[0]->getPosition()))<3))
							coinPosition = Vector2D((float)(rand() % num_cell_x), (float)(rand() % num_cell_y));
						
					}
				}
				else
				{
					Vector2D steering_force = agents[0]->Behavior()->Arrive(agents[0], currentTarget, path.ARRIVAL_DISTANCE, dtime);
					agents[0]->update(steering_force, dtime, event);
				}
				return;
			}
			currentTargetIndex++;
		}

		currentTarget = path.points[currentTargetIndex];
		Vector2D steering_force = agents[0]->Behavior()->Seek(agents[0], currentTarget, dtime);
		//Creating the "tunnel" effect
		if ((agents[0]->getPosition() - currentTarget).Length() > 1000) {
			steering_force.x = -steering_force.x;
			steering_force.y = -steering_force.y;
		}
		agents[0]->update(steering_force, dtime, event);
	} 
	else
	{
		if (newSceneState != sceneState) {
			ModifyGrid();
			sceneState = newSceneState;
		}
		

		agents[0]->update(Vector2D(0, 0), dtime, event);
		

		floodFill.clear();
		frontier.clear();
		auto start = std::chrono::high_resolution_clock::now();
		switch (sceneState) {
		case 0:
			path = agents[0]->FindPath(grid, currentTarget, cell2pix(coinPosition), BREATH_FIRST_SEARCH, floodFill, frontier);
			info3->SetText("Breadth First Search");
			break;
		case 1:
			path = agents[0]->FindPath(grid, currentTarget, cell2pix(coinPosition), DIJKSTRA, floodFill, frontier);
			info3->SetText("DIJSKTRA");
			break;
		case 2:
			path = agents[0]->FindPath(grid, currentTarget, cell2pix(coinPosition), GREEDY_BFG, floodFill, frontier);
			info3->SetText("GREEDY BFS");
			break;
		case 3:
			path = agents[0]->FindPath(grid, currentTarget, cell2pix(coinPosition), A_STAR, floodFill, frontier);
			info3->SetText("A*");
			break;
		case 4:
			numberOfTargets = 3 + rand() % 5;
			multipleTargets.clear();
			for (int i = 0; i < numberOfTargets; i++) {
				Vector2D newTargetPosition(-1, -1);
				while ((!isValidCell(newTargetPosition)) || (Vector2D::Distance(newTargetPosition, pix2cell(agents[0]->getPosition()))<3))
					newTargetPosition = Vector2D((float)(rand() % num_cell_x), (float)(rand() % num_cell_y));
				multipleTargets.push_back(cell2pix(newTargetPosition));
			}
			path = agents[0]->FindMultiplePath(grid, currentTarget, multipleTargets, floodFill, frontier);
			multipleTargetsDraw = multipleTargets;
			info3->SetText("Multiple Targets A*");
			break;
		}
		auto finish = std::chrono::high_resolution_clock::now();
		std::chrono::duration<float> elapsed = finish - start;

		info1->SetText("Analyzed Nodes: " + std::to_string(floodFill.size() + frontier.size()));
		info2->SetText("Elapsed Time: " + std::to_string(elapsed.count()).substr(0, 6));

		//path = agents[0]->FindPath(grid, currentTarget, cell2pix(coinPosition), BREATH_FIRST_SEARCH, floodFill, frontier);
		//path = agents[0]->FindPath(grid, currentTarget, cell2pix(coinPosition), DIJKSTRA, floodFill, frontier);
		//path = agents[0]->FindPath(grid, currentTarget, cell2pix(coinPosition), GREEDY_BFG, floodFill, frontier);
		//path = agents[0]->FindPath(grid, currentTarget, cell2pix(coinPosition), A_STAR, floodFill, frontier);
		//path = agents[0]->FindMultiplePath(grid, currentTarget, multipleTargets, floodFill, frontier);
	}
	int index = -1;
	for (int i = 0; i < multipleTargetsDraw.size(); i++) {
		Vector2D distance = agents[0]->getPosition() - multipleTargetsDraw[i];
		if (distance.Length() < path.ARRIVAL_DISTANCE) {
			index = i;
		}
	}
	if (index != -1) {
		multipleTargetsDraw.erase(multipleTargetsDraw.begin() + index);
	}
}

void ScenePathFinding::draw()
{
	drawMaze();


	SDL_Rect square;
	for (int i = 0; i < grid.size(); i++) {
		square.x = grid[i]->GetPosition().x - CELL_SIZE / 2;
		square.y = grid[i]->GetPosition().y - CELL_SIZE / 2;
		square.w = square.h = CELL_SIZE;
		SDL_SetRenderDrawColor(TheApp::Instance()->getRenderer(), (255 / 7) * (grid[i]->GetCost() - 1), 0, 0, 255);
		SDL_RenderFillRect(TheApp::Instance()->getRenderer(), &square);
	} 
	if (draw_grid)
	{


		SDL_SetRenderDrawColor(TheApp::Instance()->getRenderer(), 255, 255, 255, 127);
		for (int i = 0; i < SRC_WIDTH; i+=CELL_SIZE)
		{
			SDL_RenderDrawLine(TheApp::Instance()->getRenderer(), i, 0, i, SRC_HEIGHT);
		}
		for (int j = 0; j < SRC_HEIGHT; j = j += CELL_SIZE)
		{
			SDL_RenderDrawLine(TheApp::Instance()->getRenderer(), 0, j, SRC_WIDTH, j);
		}
	}
	/*
	for (int i = 0; i < floodFill.size(); i++) {
		draw_circle(TheApp::Instance()->getRenderer(), floodFill[i].x, floodFill[i].y, 15, 15, 255 , 255, 255);
	}
	for (int i = 0; i < frontier.size(); i++) {
		draw_circle(TheApp::Instance()->getRenderer(), frontier[i].x, frontier[i].y, 15, 225, 15, 225, 255);
	}
	*/
	//Grid drawing
	/*
	for (int i = 0; i < grid.size(); i++) {
		draw_circle(TheApp::Instance()->getRenderer(), grid[i]->GetPosition().x, grid[i]->GetPosition().y, 15, 100, 100, 100, 255);
		std::vector<Node*> nodeNB = grid[i]->GetNB();
		for (int j = 0; j < nodeNB.size(); j++) {
			if (grid[i]->GetPosition().x == nodeNB[j]->GetPosition().x) {
				if(grid[i]->GetPosition().y < nodeNB[j]->GetPosition().y)
					SDL_RenderDrawLine(TheApp::Instance()->getRenderer(), grid[i]->GetPosition().x, grid[i]->GetPosition().y + 3, nodeNB[j]->GetPosition().x, nodeNB[j]->GetPosition().y - 3);
				else
					SDL_RenderDrawLine(TheApp::Instance()->getRenderer(), grid[i]->GetPosition().x, grid[i]->GetPosition().y - 3, nodeNB[j]->GetPosition().x, nodeNB[j]->GetPosition().y + 3);
			}
			else {
				if(grid[i]->GetPosition().x < nodeNB[j]->GetPosition().x)
					SDL_RenderDrawLine(TheApp::Instance()->getRenderer(), grid[i]->GetPosition().x + 3, grid[i]->GetPosition().y, nodeNB[j]->GetPosition().x - 3, nodeNB[j]->GetPosition().y);
				else
					SDL_RenderDrawLine(TheApp::Instance()->getRenderer(), grid[i]->GetPosition().x - 3, grid[i]->GetPosition().y, nodeNB[j]->GetPosition().x + 3, nodeNB[j]->GetPosition().y);

			}

		}
	}
	*/
	//draw_circle(TheApp::Instance()->getRenderer(), grid[3]->GetPosition().x, grid[3]->GetPosition().y, 15, 100, 100, 100, 255);
	/*std::vector<Node*> nodeNB = grid[0]->GetNB();
	for (int j = 0; j < nodeNB.size(); j++) {
		SDL_RenderDrawLine(TheApp::Instance()->getRenderer(), grid[1]->GetPosition().x,	grid[1]->GetPosition().y, nodeNB[j]->GetPosition().x, nodeNB[j]->GetPosition().x);
	}*/


	for (int i = 0; i < (int)path.points.size(); i++)
	{
		draw_circle(TheApp::Instance()->getRenderer(), (int)(path.points[i].x), (int)(path.points[i].y), 15, 255, 255, 0, 255);
		if (i > 0)
			if( (path.points[i - 1] - path.points[i]).Length() < 100)
				SDL_RenderDrawLine(TheApp::Instance()->getRenderer(), (int)(path.points[i - 1].x), (int)(path.points[i - 1].y), (int)(path.points[i].x), (int)(path.points[i].y));
	}

	draw_circle(TheApp::Instance()->getRenderer(), (int)currentTarget.x, (int)currentTarget.y, 15, 255, 0, 0, 255);
	if (sceneState == 4) {

		for (int i = 0; i < multipleTargets.size(); i++) {
			draw_circle(TheApp::Instance()->getRenderer(), multipleTargets[i].x, multipleTargets[i].y, 10, 255, 255, 255, 255);
		}

		for (int i = 0; i < multipleTargetsDraw.size(); i++)
			drawCoin(cell2pix(pix2cell(multipleTargetsDraw[i])));
	}
	else {
		drawCoin(cell2pix(coinPosition));
	}
	agents[0]->draw();
	info1->RenderText();
	info2->RenderText();
	info3->RenderText();
}

const char* ScenePathFinding::getTitle()
{
	return "SDL Steering Behaviors :: PathFinding1 Demo";
}

void ScenePathFinding::drawMaze()
{
	if (draw_grid)
	{

		SDL_SetRenderDrawColor(TheApp::Instance()->getRenderer(), 0, 0, 255, 255);
		for (unsigned int i = 0; i < maze_rects.size(); i++)
			SDL_RenderFillRect(TheApp::Instance()->getRenderer(), &maze_rects[i]);
	}
	else
	{
		SDL_RenderCopy(TheApp::Instance()->getRenderer(), background_texture, NULL, NULL );
	}
}

void ScenePathFinding::drawCoin(const Vector2D& coin_coords)
{
	int offset = CELL_SIZE / 2;
	SDL_Rect dstrect = {(int)coin_coords.x-offset, (int)coin_coords.y - offset, CELL_SIZE, CELL_SIZE};
	SDL_RenderCopy(TheApp::Instance()->getRenderer(), coin_texture, NULL, &dstrect);
}

void ScenePathFinding::initMaze()
{

	// Initialize a list of Rectagles describing the maze geometry (useful for collision avoidance)
	SDL_Rect rect = { 0, 0, 1280, 32 };
	maze_rects.push_back(rect);
	rect = { 608, 32, 64, 32 };
	maze_rects.push_back(rect);
	rect = { 0, 736, 1280, 32 };
	maze_rects.push_back(rect);
	rect = { 608, 512, 64, 224 }; 
	maze_rects.push_back(rect);
	rect = { 0,32,32,288 };
	maze_rects.push_back(rect);
	rect = { 0,416,32,320 };
	maze_rects.push_back(rect);
	rect = { 1248,32,32,288 };
	maze_rects.push_back(rect);
	rect = { 1248,416,32,320 };
	maze_rects.push_back(rect);
	rect = { 128,128,64,32 };
	maze_rects.push_back(rect);
	rect = { 288,128,96,32 };
	maze_rects.push_back(rect);
	rect = { 480,128,64,32 };
	maze_rects.push_back(rect);
	rect = { 736,128,64,32 };
	maze_rects.push_back(rect);
	rect = { 896,128,96,32 };
	maze_rects.push_back(rect);
	rect = { 1088,128,64,32 };
	maze_rects.push_back(rect);
	rect = { 128,256,64,32 };
	maze_rects.push_back(rect);
	rect = { 288,256,96,32 };
	maze_rects.push_back(rect);
	rect = { 480, 256, 320, 32 };
	maze_rects.push_back(rect);
	rect = { 608, 224, 64, 32 }; 
	maze_rects.push_back(rect);
	rect = { 896,256,96,32 };
	maze_rects.push_back(rect);
	rect = { 1088,256,64,32 };
	maze_rects.push_back(rect);
	rect = { 128,384,32,256 };
	maze_rects.push_back(rect);
	rect = { 160,512,352,32 };
	maze_rects.push_back(rect);
	rect = { 1120,384,32,256 };
	maze_rects.push_back(rect);
	rect = { 768,512,352,32 };
	maze_rects.push_back(rect);
	rect = { 256,640,32,96 };
	maze_rects.push_back(rect);
	rect = { 992,640,32,96 };
	maze_rects.push_back(rect);
	rect = { 384,544,32,96 };
	maze_rects.push_back(rect);
	rect = { 480,704,32,32 };
	maze_rects.push_back(rect);
	rect = { 768,704,32,32 };
	maze_rects.push_back(rect);
	rect = { 864,544,32,96 };
	maze_rects.push_back(rect);
	rect = { 320,288,32,128 };
	maze_rects.push_back(rect);
	rect = { 352,384,224,32 };
	maze_rects.push_back(rect);
	rect = { 704,384,224,32 };
	maze_rects.push_back(rect);
	rect = { 928,288,32,128 };
	maze_rects.push_back(rect);

	// Initialize the terrain matrix (for each cell a zero value indicates it's a wall)
	
	// (1st) initialize all cells to 1 by default
	for (int i = 0; i < num_cell_x; i++){
		vector<int> terrain_col(num_cell_y, 1); 
		terrain.push_back(terrain_col);
	}
	// (2nd) set to zero all cells that belong to a wall
	int offset = CELL_SIZE / 2;
	for (int i = 0; i < num_cell_x; i++)
	{
		for (int j = 0; j < num_cell_y; j++)
		{
			Vector2D cell_center ((float)(i*CELL_SIZE + offset), (float)(j*CELL_SIZE + offset));
			for (unsigned int b = 0; b < maze_rects.size(); b++)
			{
				if (Vector2DUtils::IsInsideRect(cell_center, (float)maze_rects[b].x, (float)maze_rects[b].y, (float)maze_rects[b].w, (float)maze_rects[b].h))
				{
					terrain[i][j] = 0;
				    break;
				}  
			}
			
		}
	}
	CreateGrid(terrain);
}

bool ScenePathFinding::loadTextures(char* filename_bg, char* filename_coin)
{
	SDL_Surface *image = IMG_Load(filename_bg);
	if (!image) {
		cout << "IMG_Load: " << IMG_GetError() << endl;
		return false;
	}
	background_texture = SDL_CreateTextureFromSurface(TheApp::Instance()->getRenderer(), image);

	if (image)
		SDL_FreeSurface(image);

	image = IMG_Load(filename_coin);
	if (!image) {
		cout << "IMG_Load: " << IMG_GetError() << endl;
		return false;
	}
	coin_texture = SDL_CreateTextureFromSurface(TheApp::Instance()->getRenderer(), image);

	if (image)
		SDL_FreeSurface(image);

	return true;
}

Vector2D ScenePathFinding::cell2pix(Vector2D cell)
{
	int offset = CELL_SIZE / 2;
	return Vector2D(cell.x*CELL_SIZE + offset, cell.y*CELL_SIZE + offset);
}

Vector2D ScenePathFinding::pix2cell(Vector2D pix)
{
	return Vector2D((float)((int)pix.x/CELL_SIZE), (float)((int)pix.y / CELL_SIZE));
}

bool ScenePathFinding::isValidCell(Vector2D cell)
{
	if ((cell.x < 0) || (cell.y < 0) || (cell.x >= terrain.size()) || (cell.y >= terrain[0].size()) )
		return false;
	return !(terrain[(unsigned int)cell.x][(unsigned int)cell.y] == 0);
}

void ScenePathFinding::CreateGrid(const std::vector<std::vector<int>>& maze) {
	int offset = CELL_SIZE / 2;
	bool* leftWall = new bool[maze[0].size()];
	
	std::queue<Node*> leftNodes;
	bool topWall = true;
	for (int i = 0; i < maze.size(); i++) { // Columns
		for (int j = 0; j < maze[i].size(); j++) { // Rows
			if (maze[i][j] == 0) { // that position is actually a wall, we won't create a node
				if (!leftWall[j]) {//we prevent diagonal linking
					leftNodes.pop();
				}
				topWall = true;
				leftWall[j] = true;
			}
			else {
				Node* newNode = new Node(Vector2D(offset + i * CELL_SIZE, offset + j * CELL_SIZE), maze[i][j]);

				if (leftWall[j]) { // if the left node was a wall we store the actual one it into a queue
					leftWall[j] = false;
					if(i != maze.size() - 1)
						leftNodes.push(newNode);
				}
				else { // if the left node wasn't a wall
					newNode->AddNB(leftNodes.front());
					leftNodes.front()->AddNB(newNode);
					leftNodes.pop();
					if (i != maze.size() - 1)
						leftNodes.push(newNode);
				}
				
				if (!topWall) {
					newNode->AddNB(grid.back());
					grid.back()->AddNB(newNode);
				}
				grid.push_back(newNode);
				topWall = false;
			}
		}
	}
	//Build the "tunnel"
	grid[0]->AddNB(grid[grid.size() - 3]);
	grid[grid.size() - 3]->AddNB(grid[0]);

	grid[1]->AddNB(grid[grid.size() - 2]);
	grid[grid.size() - 2]->AddNB(grid[1]);

	grid[2]->AddNB(grid[grid.size() - 1]);
	grid[grid.size() - 1]->AddNB(grid[2]);

	delete[] leftWall;
}

void ScenePathFinding::ModifyGrid() {
	if (modifyedNodes.size()) {
		for (int i = 0; i < modifyedNodes.size(); i++) {
			modifyedNodes[i]->SetCost(1);
		}
		modifyedNodes.clear();
	}

	for (int i = 0; i < 8; i++) {
		int randomNode = rand() % grid.size();
		grid[randomNode]->SetCost(8);
		std::vector<Node*> NB1 = grid[randomNode]->GetNB();
		modifyedNodes.push_back(grid[randomNode]);

		for (int j = 0; j < NB1.size(); j++) {
			NB1[j]->SetCost(6);
			std::vector<Node*> NB2 = NB1[j]->GetNB();
			modifyedNodes.push_back(NB1[j]);

			for (int k = 0; k < NB2.size(); k++) {
				if (NB2[k]->GetCost() == 1)
					NB2[k]->SetCost(4);
				std::vector<Node*> NB3 = NB2[k]->GetNB();
				modifyedNodes.push_back(NB2[k]);

				for (int w = 0; w < NB3.size(); w++) {
					if(NB3[w]->GetCost() == 1)
						NB3[w]->SetCost(2);
					std::vector<Node*> NB4 = NB3[w]->GetNB();
					modifyedNodes.push_back(NB3[w]);
				}
			}
		}
	}
}