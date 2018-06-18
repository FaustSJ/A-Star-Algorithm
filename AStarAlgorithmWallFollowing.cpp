//This is a RECURSIVE function that implements pathfinding through A*
bool Agent::findPath(GridNode* currentNode, GridNode* goalNode, int costSoFar)
{
	//set up
	std::vector<GridNode*> adjList; //the list of the current node's neighbors
	std::vector<GridNode*> toTraverse; //the list of nodes to visit next	
	currentNode->setVisited(); //mark the current node as visited
	int minimum = -2;
	int nodeIndex = 0;
	GridNode* bestNode = NULL;
	bool toReturn = false;

	std::cout<< "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" <<std::endl;
	std::cout<< "currently at node "<< currentNode->getRow() << "," << currentNode->getColumn() <<std::endl;
	std::cout<< "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" <<std::endl;

	//STOP CONDITION 1 : if the goal node is reached
	if(currentNode->getID() == goalNode->getID()) //compare IDs
	{
		std::cout<< "Path found!" <<std::endl;
		if((this->totalDH > (currentNode->getHeuristic() + costSoFar))||(this->totalDH == -1)) 
		{   
			std::cout<< "This is currently the shortest path." <<std::endl;
			setWayPoints(currentNode);
			this->totalDH = currentNode->getHeuristic() + costSoFar;
		}
		else
		{   std::cout<< "...however, it's not the shortest path." <<std::endl;}
		agentGrid->softReset();
		return true; 
	}
	//STOP CONDITION 2 : if this path's d+h surpasses the totalDH (set by another path)
	if((this->totalDH < (currentNode->getHeuristic() + costSoFar))&&(this->totalDH != -1))
	{   
		std::cout<< "Path halted, shorter path found." <<std::endl;
		std::cout<< "totalDH = "<<totalDH << std::endl;
		std::cout<< "this node's cost total: "<<currentNode->getHeuristic()<<" + "<<costSoFar<<std::endl;
		agentGrid->softReset();
		return true; 
	}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//Otherwise, find the next node to traverse to
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	//compiles a list of adjacent, non-Null node. Their costs have been set.
	adjList = agentGrid->getWalkableAdjacentNodes(currentNode);//grab neighbors

	int counter = -1; //for index tracking
	for(GridNode* adjNode : adjList)
	{   
		counter++; //for index tracking
		if(adjNode != NULL)
		{
			//update each neighbor's g+h
			if(!adjNode->isHeuristicSet())
			{   adjNode->setHeuristic(agentGrid->getDistance(adjNode, goalNode));}
			

			//only check the visitable nodes
			if(adjNode->notVisited())//&&(adjNode->isClear()))
			{
				//have the first visitable node's heuristic set the minimum
				if(minimum == -2)
				{   minimum = adjNode->getCost() + adjNode->getHeuristic();}

				//is the node's g+h < the minimum? Is the node also NOT the previous node nor the start node?
				if(((adjNode->getCost() + adjNode->getHeuristic()) <= minimum)&&((currentNode->getPrevNode()==NULL)||(adjNode->getID() != currentNode->getPrevNode()->getID()))&&(adjNode->getID()!=startNodeID))
				{
					minimum = adjNode->getCost() + adjNode->getHeuristic();
					bestNode = adjNode;
					nodeIndex = counter;
					std::cout<< "best node has a Row,Col of " << bestNode->getRow() << "," << bestNode->getColumn() <<std::endl;
				}			
			}
		}
//		else{   std::cout<<"null node found."<<std::endl;}
	}
	
	if(bestNode==NULL)
	{
		std::cout<<"ERROR: NO NEXT NODE WAS PICKED"<<std::endl;
	}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//if a wall is reached, initiate wall-following
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	if((bestNode!=NULL)&&(!bestNode->isClear()))
	{
		//we start by checking the directly-adjacent nodes
		std::cout<< "bestNode is a wall. Initiate wall-following." <<std::endl;
		//clockwise node
		if(nodeIndex==7)
		{
			if((adjList[0]!=NULL)&&(adjList[0]->isClear())&&(adjList[0]->notVisited()))
			{   
				//make sure it isn't the previous node or start node
				if(((currentNode->getPrevNode()==NULL)||(adjList[0]->getID() != currentNode->getPrevNode()->getID()))&&(adjList[0]->getID()!=startNodeID))
				{
					std::cout<< "directly-adjacent node "<< adjList[0]->getRow() << "," << adjList[0]->getColumn() << " is walkable." <<std::endl;
					toTraverse.push_back(adjList[0]);
		}	}	}	
		else
		{
			if((adjList[nodeIndex+1]!=NULL)&&(adjList[nodeIndex+1]->isClear())&&(adjList[nodeIndex+1]->notVisited()))
			{   
				//make sure it isn't the previous node or start node
				if(((currentNode->getPrevNode()==NULL)||(adjList[nodeIndex+1]->getID()!=currentNode->getPrevNode()->getID()))&&(adjList[nodeIndex+1]->getID()!=startNodeID))
				{
					std::cout<< "directly-adjacent node "<< adjList[nodeIndex+1]->getRow() << "," << adjList[nodeIndex+1]->getColumn() << " is walkable." <<std::endl;
					toTraverse.push_back(adjList[nodeIndex+1]);
		}	}	}	
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//counter-clockwise node
		if(nodeIndex==0)
		{
			if((adjList[7]!=NULL)&&(adjList[7]->isClear())&&(adjList[7]->notVisited()))
			{   
				//make sure it isn't the previous node or start node
				if(((currentNode->getPrevNode()==NULL)||(adjList[7]->getID() != currentNode->getPrevNode()->getID()))&&(adjList[7]->getID()!=startNodeID))
				{
					std::cout<< "directly-adjacent node "<< adjList[7]->getRow() << "," << adjList[7]->getColumn() << " is walkable." <<std::endl;
					toTraverse.push_back(adjList[7]);
		}	}	}	
		else
		{
			if((adjList[nodeIndex-1]!=NULL)&&(adjList[nodeIndex-1]->isClear())&&(adjList[nodeIndex-1]->notVisited()))
			{   
				//make sure it isn't the previous node or start node
				if(((currentNode->getPrevNode()==NULL)||(adjList[nodeIndex-1]->getID() != currentNode->getPrevNode()->getID()))&&(adjList[nodeIndex-1]->getID()!=startNodeID))
				{
					std::cout<< "directly-adjacent node "<< adjList[nodeIndex-1]->getRow() << "," << adjList[nodeIndex-1]->getColumn() << " is walkable." <<std::endl;
					toTraverse.push_back(adjList[nodeIndex-1]);
		}	}	}	
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//if neither direct-adjacent nodes are clear, check direct-diagonal nodes
		if(toTraverse.empty())
		{
			//clockwise node
			if(nodeIndex==7)
			{
				if((adjList[1]!=NULL)&&(adjList[1]->isClear())&&(adjList[1]->notVisited()))
				{   
					//make sure it isn't the previous node or start node
					if(((currentNode->getPrevNode()==NULL)||(adjList[1]->getID() != currentNode->getPrevNode()->getID()))&&(adjList[1]->getID()!=startNodeID))
					{
						//if the adjacent node's aren't "wall", then don't follow that path.
						if(((adjList[0]!=NULL)&&(!adjList[0]->isClear()))||((adjList[2]!=NULL)&&(!adjList[2]->isClear())))
						{
							std::cout<< "directly-diagonal node "<< adjList[1]->getRow() << "," << adjList[1]->getColumn() << " is walkable." <<std::endl;
							toTraverse.push_back(adjList[1]);
			}	}	}	}
			else if(nodeIndex==6)
			{
				if((adjList[0]!=NULL)&&(adjList[0]->isClear())&&(adjList[0]->notVisited()))
				{   
					//make sure it isn't the previous node or start node
					if(((currentNode->getPrevNode()==NULL)||(adjList[0]->getID() != currentNode->getPrevNode()->getID()))&&(adjList[0]->getID()!=startNodeID))
					{
						//if the adjacent node's aren't "wall", then don't follow that path.
						if(((adjList[7]!=NULL)&&(!adjList[7]->isClear()))||((adjList[1]!=NULL)&&(!adjList[1]->isClear())))
						{
							std::cout<< "directly-diagonal node "<< adjList[0]->getRow() << "," << adjList[0]->getColumn() << " is walkable." <<std::endl;
							toTraverse.push_back(adjList[0]);
			}	}	}	}
			else
			{
				if((adjList[nodeIndex+2]!=NULL)&&(adjList[nodeIndex+2]->isClear())&&(adjList[nodeIndex+2]->notVisited()))
				{   
					//make sure it isn't the previous node or start node
					if(((currentNode->getPrevNode()==NULL)||(adjList[nodeIndex+2]->getID() != currentNode->getPrevNode()->getID()))&&(adjList[nodeIndex+2]->getID()!=startNodeID))
					{
						//if the adjacent node's aren't "wall", then don't follow that path.
						if(nodeIndex==5)
						{
							if(((adjList[nodeIndex+1]!=NULL)&&(!adjList[nodeIndex+1]->isClear()))||((adjList[0]!=NULL)&&(!adjList[0]->isClear())))
							{
								std::cout<< "directly-diagonal node "<< adjList[nodeIndex+2]->getRow() << "," << adjList[nodeIndex+2]->getColumn() << " is walkable." <<std::endl;
								toTraverse.push_back(adjList[nodeIndex+2]);
							}
						}
						else if(((adjList[nodeIndex+1]!=NULL)&&(!adjList[nodeIndex+1]->isClear()))||((adjList[nodeIndex+3]!=NULL)&&(!adjList[nodeIndex+3]->isClear())))
						{
							std::cout<< "directly-diagonal node "<< adjList[nodeIndex+2]->getRow() << "," << adjList[nodeIndex+2]->getColumn() << " is walkable." <<std::endl;
							toTraverse.push_back(adjList[nodeIndex+2]);
			}	}	}	}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
			//counter-clockwise node
			if(nodeIndex==0)
			{
				if((adjList[6]!=NULL)&&(adjList[6]->isClear())&&(adjList[6]->notVisited()))
				{   
					//make sure it isn't the previous node or start node
					if(((currentNode->getPrevNode()==NULL)||(adjList[6]->getID() != currentNode->getPrevNode()->getID()))&&(adjList[6]->getID()!=startNodeID))
					{
						if(((adjList[7]!=NULL)&&(!adjList[7]->isClear()))||((adjList[5]!=NULL)&&(!adjList[5]->isClear())))
						{
							std::cout<< "directly-diagonal node "<< adjList[6]->getRow() << "," << adjList[6]->getColumn() << " is walkable." <<std::endl;
							toTraverse.push_back(adjList[6]);
			}	}	}	}	
			else if(nodeIndex==1)
			{
				if((adjList[7]!=NULL)&&(adjList[7]->isClear())&&(adjList[7]->notVisited()))
				{  
					//make sure it isn't the previous node or start node
					if(((currentNode->getPrevNode()==NULL)||(adjList[7]->getID() != currentNode->getPrevNode()->getID()))&&(adjList[7]->getID()!=startNodeID))
					{
						if(((adjList[0]!=NULL)&&(!adjList[0]->isClear()))||((adjList[6]!=NULL)&&(!adjList[6]->isClear())))
						{
							std::cout<< "directly-diagonal node "<< adjList[7]->getRow() << "," << adjList[7]->getColumn() << " is walkable." <<std::endl;
							toTraverse.push_back(adjList[7]);
			}	}	}	}
			else
			{
				if((adjList[nodeIndex-2]!=NULL)&&(adjList[nodeIndex-2]->isClear())&&(adjList[nodeIndex-2]->notVisited()))
				{   
					//make sure it isn't the previous node or start node
					if(((currentNode->getPrevNode()==NULL)||(adjList[nodeIndex-2]->getID() != currentNode->getPrevNode()->getID()))&&(adjList[nodeIndex-2]->getID()!=startNodeID))
					{
						//if the adjacent node's aren't "wall", then don't follow that path.
						if(nodeIndex==2)
						{
							if(((adjList[nodeIndex-1]!=NULL)&&(!adjList[nodeIndex-1]->isClear()))||((adjList[7]!=NULL)&&(!adjList[7]->isClear())))
							{
								std::cout<< "directly-diagonal node "<< adjList[nodeIndex-2]->getRow() << "," << adjList[nodeIndex-2]->getColumn() << " is walkable." <<std::endl;
								toTraverse.push_back(adjList[nodeIndex-2]);
							}
						}
						else if(((adjList[nodeIndex-1]!=NULL)&&(!adjList[nodeIndex-1]->isClear()))||((adjList[nodeIndex-3]!=NULL)&&(!adjList[nodeIndex-3]->isClear())))
						{
							std::cout<< "directly-diagonal node "<< adjList[nodeIndex-2]->getRow() << "," << adjList[nodeIndex-2]->getColumn() << " is walkable." <<std::endl;
							toTraverse.push_back(adjList[nodeIndex-2]);
		}	}	}	}	}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//if there are no directly-adjacent nor direct-diagonal nodes, then check adjacent-opposite nodes
		if(toTraverse.empty())
		{
			//clockwise node
			if(nodeIndex==7)
			{
				if((adjList[2]!=NULL)&&(adjList[2]->isClear())&&(adjList[2]->notVisited()))
				{   
					//make sure it isn't the previous node or start node
					if(((currentNode->getPrevNode()==NULL)||(adjList[2]->getID() != currentNode->getPrevNode()->getID()))&&(adjList[2]->getID()!=startNodeID))
					{
						if(((adjList[1]!=NULL)&&(!adjList[1]->isClear()))||((adjList[3]!=NULL)&&(!adjList[3]->isClear())))
						{
							std::cout<< "adjacent-opposite node "<< adjList[2]->getRow() << "," << adjList[2]->getColumn() << " is walkable." <<std::endl;
							toTraverse.push_back(adjList[2]);
			}	}	}	}
			else if(nodeIndex==6)
			{
				if((adjList[1]!=NULL)&&(adjList[1]->isClear())&&(adjList[1]->notVisited()))
				{   
					//make sure it isn't the previous node or start node
					if(((currentNode->getPrevNode()==NULL)||(adjList[1]->getID() != currentNode->getPrevNode()->getID()))&&(adjList[1]->getID()!=startNodeID))
					{
						if(((adjList[0]!=NULL)&&(!adjList[0]->isClear()))||((adjList[2]!=NULL)&&(!adjList[2]->isClear())))
						{
							std::cout<< "adjacent-opposite node "<< adjList[1]->getRow() << "," << adjList[1]->getColumn() << " is walkable." <<std::endl;
							toTraverse.push_back(adjList[1]);
			}	}	}	}
			else if(nodeIndex==5)
			{
				if((adjList[0]!=NULL)&&(adjList[0]->isClear())&&(adjList[0]->notVisited()))
				{   
					//make sure it isn't the previous node or start node
					if(((currentNode->getPrevNode()==NULL)||(adjList[0]->getID() != currentNode->getPrevNode()->getID()))&&(adjList[0]->getID()!=startNodeID))
					{
						if(((adjList[7]!=NULL)&&(!adjList[7]->isClear()))||((adjList[1]!=NULL)&&(!adjList[1]->isClear())))
						{
							std::cout<< "adjacent-opposite node "<< adjList[0]->getRow() << "," << adjList[0]->getColumn() << " is walkable." <<std::endl;
							toTraverse.push_back(adjList[0]);
			}	}	}	}
			else
			{
				if((adjList[nodeIndex+3]!=NULL)&&(adjList[nodeIndex+3]->isClear())&&(adjList[nodeIndex+3]->notVisited()))
				{   
					//make sure it isn't the previous node or start node
					if(((currentNode->getPrevNode()==NULL)||(adjList[nodeIndex+3]->getID() != currentNode->getPrevNode()->getID()))&&(adjList[nodeIndex+3]->getID()!=startNodeID))
					{
						//if the adjacent node's aren't "wall", then don't follow that path.
						if(nodeIndex==4)
						{
							if(((adjList[nodeIndex+2]!=NULL)&&(!adjList[nodeIndex+2]->isClear()))||((adjList[0]!=NULL)&&(!adjList[0]->isClear())))
							{
								std::cout<< "adjacent-opposite node "<< adjList[nodeIndex+3]->getRow() << "," << adjList[nodeIndex+3]->getColumn() << " is walkable." <<std::endl;
								toTraverse.push_back(adjList[nodeIndex+3]);
							}
						}
						else if(((adjList[nodeIndex+2]!=NULL)&&(!adjList[nodeIndex+2]->isClear()))||((adjList[nodeIndex+4]!=NULL)&&(!adjList[nodeIndex+4]->isClear())))
						{
							std::cout<< "adjacent-opposite node "<< adjList[nodeIndex+3]->getRow() << "," << adjList[nodeIndex+3]->getColumn() << " is walkable." <<std::endl;
							toTraverse.push_back(adjList[nodeIndex+3]);
			}	}	}	}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
			//counter-clockwise node
			if(nodeIndex==0)
			{
				if((adjList[5]!=NULL)&&(adjList[5]->isClear())&&(adjList[5]->notVisited()))
				{   
					//make sure it isn't the previous node or start node
					if(((currentNode->getPrevNode()==NULL)||(adjList[5]->getID() != currentNode->getPrevNode()->getID()))&&(adjList[5]->getID()!=startNodeID))
					{
						if(((adjList[6]!=NULL)&&(!adjList[6]->isClear()))||((adjList[4]!=NULL)&&(!adjList[4]->isClear())))
						{
							std::cout<< "adjacent-opposite node "<< adjList[5]->getRow() << "," << adjList[5]->getColumn() << " is walkable." <<std::endl;
							toTraverse.push_back(adjList[5]);
			}	}	}	}
			else if(nodeIndex==1)
			{
				if((adjList[6]!=NULL)&&(adjList[6]->isClear())&&(adjList[6]->notVisited()))
				{  
					//make sure it isn't the previous node or start node
					if(((currentNode->getPrevNode()==NULL)||(adjList[6]->getID() != currentNode->getPrevNode()->getID()))&&(adjList[6]->getID()!=startNodeID))
					{
						if(((adjList[7]!=NULL)&&(!adjList[7]->isClear()))||((adjList[5]!=NULL)&&(!adjList[5]->isClear())))
						{
							std::cout<< "adjacent-opposite node "<< adjList[6]->getRow() << "," << adjList[6]->getColumn() << " is walkable." <<std::endl;
							toTraverse.push_back(adjList[6]);
			}	}	}	}
			else if(nodeIndex==2)
			{
				if((adjList[7]!=NULL)&&(adjList[7]->isClear())&&(adjList[7]->notVisited()))
				{  
					//make sure it isn't the previous node or start node
					if(((currentNode->getPrevNode()==NULL)||(adjList[7]->getID() != currentNode->getPrevNode()->getID()))&&(adjList[7]->getID()!=startNodeID))
					{
						if(((adjList[0]!=NULL)&&(!adjList[0]->isClear()))||((adjList[6]!=NULL)&&(!adjList[6]->isClear())))
						{
							std::cout<< "adjacent-opposite node "<< adjList[7]->getRow() << "," << adjList[7]->getColumn() << " is walkable." <<std::endl;
							toTraverse.push_back(adjList[7]);
			}	}	}	}
			else
			{
				if((adjList[nodeIndex-3]!=NULL)&&(adjList[nodeIndex-3]->isClear())&&(adjList[nodeIndex-3]->notVisited()))
				{   
					//make sure it isn't the previous node or start node
					if(((currentNode->getPrevNode()==NULL)||(adjList[nodeIndex-3]->getID() != currentNode->getPrevNode()->getID()))&&(adjList[nodeIndex-3]->getID()!=startNodeID))
					{
						//if the adjacent node's aren't "wall", then don't follow that path.
						if(nodeIndex==3)
						{
							if(((adjList[nodeIndex-2]!=NULL)&&(!adjList[nodeIndex-2]->isClear()))||((adjList[7]!=NULL)&&(!adjList[7]->isClear())))
							{
								std::cout<< "adjacent-opposite node "<< adjList[nodeIndex-3]->getRow() << "," << adjList[nodeIndex-3]->getColumn() << " is walkable." <<std::endl;
								toTraverse.push_back(adjList[nodeIndex-3]);
							}
						}
						else if(((adjList[nodeIndex-2]!=NULL)&&(!adjList[nodeIndex-2]->isClear()))||((adjList[nodeIndex-4]!=NULL)&&(!adjList[nodeIndex-4]->isClear())))
						{
							std::cout<< "adjacent-opposite node "<< adjList[nodeIndex-3]->getRow() << "," << adjList[nodeIndex-3]->getColumn() << " is walkable." <<std::endl;
							toTraverse.push_back(adjList[nodeIndex-3]);
		}	}	}	}	}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//finally, if no other node was picked, try the direct-accross node.
		if(toTraverse.empty())
		{
			if(nodeIndex==0)
			{
				if((adjList[4]!=NULL)&&(adjList[4]->isClear())&&(adjList[4]->notVisited()))
				{ 
					//make sure it isn't the previous node or start node
					if(((currentNode->getPrevNode()==NULL)||(adjList[4]->getID() != currentNode->getPrevNode()->getID()))&&(adjList[4]->getID()!=startNodeID))
					{
						//make sure it's adjacent to a wall
						if(((adjList[3]!=NULL)&&(!adjList[3]->isClear()))||((adjList[5]!=NULL)&&(!adjList[5]->isClear())))
						{
							std::cout<< "direct-accross node "<< adjList[4]->getRow() << "," << adjList[4]->getColumn() << " is walkable." <<std::endl;
							toTraverse.push_back(adjList[4]);
			}	}	}	}
			else if(nodeIndex==1)
			{
				if((adjList[5]!=NULL)&&(adjList[5]->isClear())&&(adjList[5]->notVisited()))
				{ 
					//make sure it isn't the previous node or start node
					if(((currentNode->getPrevNode()==NULL)||(adjList[5]->getID() != currentNode->getPrevNode()->getID()))&&(adjList[5]->getID()!=startNodeID))
					{
						//make sure it's adjacent to a wall
						if(((adjList[4]!=NULL)&&(!adjList[4]->isClear()))||((adjList[6]!=NULL)&&(!adjList[6]->isClear())))
						{
							std::cout<< "direct-accross node "<< adjList[5]->getRow() << "," << adjList[5]->getColumn() << " is walkable." <<std::endl;
							toTraverse.push_back(adjList[5]);
			}	}	}	}
			else if(nodeIndex==2)
			{
				if((adjList[6]!=NULL)&&(adjList[6]->isClear())&&(adjList[6]->notVisited()))
				{ 
					//make sure it isn't the previous node or start node
					if(((currentNode->getPrevNode()==NULL)||(adjList[6]->getID() != currentNode->getPrevNode()->getID()))&&(adjList[6]->getID()!=startNodeID))
					{
						//make sure it's adjacent to a wall
						if(((adjList[5]!=NULL)&&(!adjList[5]->isClear()))||((adjList[7]!=NULL)&&(!adjList[7]->isClear())))
						{
							std::cout<< "direct-accross node "<< adjList[6]->getRow() << "," << adjList[6]->getColumn() << " is walkable." <<std::endl;
							toTraverse.push_back(adjList[6]);
			}	}	}	}
			else if(nodeIndex==3)
			{
				if((adjList[7]!=NULL)&&(adjList[7]->isClear())&&(adjList[7]->notVisited()))
				{ 
					//make sure it isn't the previous node or start node
					if(((currentNode->getPrevNode()==NULL)||(adjList[7]->getID() != currentNode->getPrevNode()->getID()))&&(adjList[7]->getID()!=startNodeID))
					{
						//make sure it's adjacent to a wall
						if(((adjList[0]!=NULL)&&(!adjList[0]->isClear()))||((adjList[6]!=NULL)&&(!adjList[6]->isClear())))
						{
							std::cout<< "direct-accross node "<< adjList[7]->getRow() << "," << adjList[7]->getColumn() << " is walkable." <<std::endl;
							toTraverse.push_back(adjList[7]);
			}	}	}	}
			else
			{
				if((adjList[nodeIndex-4]!=NULL)&&(adjList[nodeIndex-4]->isClear())&&(adjList[nodeIndex-4]->notVisited()))
				{
					if(nodeIndex==4)
					{
						//make sure it's adjacent to a wall
						if(((adjList[1]!=NULL)&&(!adjList[1]->isClear()))||((adjList[7]!=NULL)&&(!adjList[7]->isClear())))
						{
							std::cout<< "direct-accross node "<< adjList[0]->getRow() << "," << adjList[0]->getColumn() << " is walkable." <<std::endl;
							toTraverse.push_back(adjList[0]);
						}
					}
					else if(((adjList[nodeIndex-3]!=NULL)&&(!adjList[nodeIndex-3]->isClear()))||((adjList[nodeIndex-5]!=NULL)&&(!adjList[nodeIndex-5]->isClear())))
					{
						std::cout<< "direct-accross node "<< adjList[nodeIndex-4]->getRow() << "," << adjList[nodeIndex-4]->getColumn() << " is walkable." <<std::endl;
						toTraverse.push_back(adjList[nodeIndex-4]);
	}	}	}	}	}
	else if(bestNode!=NULL)
	{   
		std::cout<< "bestNode is walkable." <<std::endl;
		toTraverse.push_back(bestNode);
	}//add to list of nodes to traverse
	else
	{
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//have we reached a dead end? start backtracking
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		std::cout<< "A dead end was reached." <<std::endl;
		//if we have backtracked to the starter node, there is no path to the goal
		if(currentNode->getPrevNode() == NULL)
		{   
			std::cout<< "No path was found!" <<std::endl;
			agentGrid->softReset();
			return false;
		}
		std::cout<< "Turning around..." <<std::endl;
		//otherwise, backup and pick another direction
		toReturn = findPath(currentNode->getPrevNode(), goalNode, (costSoFar - currentNode->getCost()));
	}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//Start following the chosen path(s)!
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	for(GridNode* toVisit : toTraverse)//for each node in list of nodes to traverse
	{
		toVisit->setPrevNode(currentNode);
		std::cout<< "now at node "<< currentNode->getRow() << "," << currentNode->getColumn() <<std::endl;
		std::cout<< "about to visit "<< toVisit->getRow() << "," << toVisit->getColumn() <<std::endl;
		toReturn = findPath(toVisit, goalNode, (costSoFar + currentNode->getCost()));			
	}
	std::cout<<"...backtracking to an unexplored branch... "<<std::endl;
	currentNode->reset(); //sets prevNode to NULL to avoid complications

	//don't forget to return whether or not a path has been found
	return toReturn;
}