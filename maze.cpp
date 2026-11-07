#include <iostream>
#include <cmath>
using namespace std;

const int MAX_SIZE = 20;            
const int INF = 1000000;            

struct Position { int row; int col; };

struct SearchResult
{
    bool found = false;
    int pathCost = 0;
    int expandedNodes = 0;
    int generatedNodes = 0;
    int pathLength = 0;
    Position path[MAX_SIZE * MAX_SIZE];
};

int absValue(int x) { return x < 0 ? -x : x; }

class Maze
{
private:
    int rows = 0, cols = 0;
    char cells[MAX_SIZE][MAX_SIZE];
    Position goal = {-1, -1};

public:
   
    bool inputMaze()
    {
        cout << "Enter rows and columns: ";
        cin >> rows >> cols;
        if (rows < 1 || rows > MAX_SIZE || cols < 1 || cols > MAX_SIZE)
        {
            cout << "Invalid size!\n";
            return false;
        }

        cout << "Enter the maze (F free, W wall, L flooded, X food):\n";
        int goalCount = 0;
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
            {
                cin >> cells[i][j];
                if (cells[i][j] == 'X') { goal.row = i; goal.col = j; goalCount++; }
            }

        if (goalCount != 1)
        {
            cout << "Maze must have exactly one X!\n";
            return false;
        }
        return true;
    }

    void display(const SearchResult *result = nullptr)
    {
        char canvas[MAX_SIZE][MAX_SIZE];
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                canvas[i][j] = cells[i][j];

        if (result != nullptr)
        {
            for (int k = 0; k < result->pathLength; k++)
            {
                Position p = result->path[k];
                if (canvas[p.row][p.col] == 'F' ||
    canvas[p.row][p.col] == 'L')
{
    canvas[p.row][p.col] = '*';
}
            }
            canvas[result->path[0].row][result->path[0].col] = 'S';
        }

        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++) cout << canvas[i][j] << " ";
            cout << endl;
        }
    }

    bool isValidMove(int row, int col)
    {
        return row >= 0 && row < rows && col >= 0 && col < cols && cells[row][col] != 'W';
    }

    bool isGoal(int row, int col) { return cells[row][col] == 'X'; }
    
    bool isValidStart(int row, int col)
{
    if (row < 0 || row >= rows || col < 0 || col >= cols)
        return false;

    if (cells[row][col] == 'W' || cells[row][col] == 'X')
        return false;

    return true;
}
    
    Position getGoal() { return goal; }
   
    int getRows() { return rows; }
   
    int getCols() { return cols; }

    int getNeighbors(int row, int col, Position neighbors[])
    {
        int dRow[4] = {-1, 1, 0, 0};
        int dCol[4] = {0, 0, 1, -1};
        int count = 0;
        for (int i = 0; i < 4; i++)
            if (isValidMove(row + dRow[i], col + dCol[i]))
            {
                neighbors[count].row = row + dRow[i];
                neighbors[count].col = col + dCol[i];
                count++;
            }
        return count;
    }

};


class Search
{
private:
    double euclidean(Position a, Position b)      // h1
    {
        int dr = a.row - b.row, dc = a.col - b.col;
        return sqrt((double)(dr * dr + dc * dc));
    }

    double manhattan(Position a, Position b)      // h2
    {
        return absValue(a.row - b.row) + absValue(a.col - b.col);
    }

    SearchResult runSearch(Maze &maze, Position start, int heuristicType, bool useAStar)
    {
        SearchResult result;
        Position goal = maze.getGoal();

        int g[MAX_SIZE][MAX_SIZE];                  
        int parentRow[MAX_SIZE][MAX_SIZE];
        int parentCol[MAX_SIZE][MAX_SIZE];
        bool open[MAX_SIZE][MAX_SIZE];              
        bool closed[MAX_SIZE][MAX_SIZE];            
        for (int i = 0; i < MAX_SIZE; i++)
            for (int j = 0; j < MAX_SIZE; j++)
            {
                g[i][j] = INF;
                parentRow[i][j] = parentCol[i][j] = -1;
                open[i][j] = closed[i][j] = false;
            }

        g[start.row][start.col] = 0;
        open[start.row][start.col] = true;
        result.generatedNodes = 1;

        while (true)
        {
            int curRow = -1, curCol = -1;
            double bestF = INF, bestH = INF;

            for (int i = 0; i < maze.getRows(); i++)
                for (int j = 0; j < maze.getCols(); j++)
                    if (open[i][j])
                    {
                        Position p = {i, j};
                        double h = (heuristicType == 1) ? euclidean(p, goal) : manhattan(p, goal);
                        double f = useAStar ? g[i][j] + h : h;

                        if (f < bestF || (f == bestF && h < bestH))
                        {
                            bestF = f;
                            bestH = h;
                            curRow = i;
                            curCol = j;
                        }
                    }

            if (curRow == -1) return result;       
            open[curRow][curCol] = false;
            closed[curRow][curCol] = true;
            result.expandedNodes++;

            if (maze.isGoal(curRow, curCol))
            {
                result.found = true;
                result.pathCost = g[curRow][curCol];

                int length = result.pathCost + 1;
                result.pathLength = length;
                int r = curRow, c = curCol;
                for (int k = length - 1; k >= 0; k--)
                {
                    result.path[k].row = r;
                    result.path[k].col = c;
                    int pr = parentRow[r][c];
                    c = parentCol[r][c];
                    r = pr;
                }
                return result;
            }

            Position neighbors[4];
            int count = maze.getNeighbors(curRow, curCol, neighbors);
            for (int i = 0; i < count; i++)
            {
                int r = neighbors[i].row, c = neighbors[i].col;
                if (closed[r][c]) continue;

                int newG = g[curRow][curCol] + 1;   
                if (!open[r][c] || newG < g[r][c])
                {
                    if (!open[r][c]) result.generatedNodes++;
                    open[r][c] = true;
                    g[r][c] = newG;
                    parentRow[r][c] = curRow;
                    parentCol[r][c] = curCol;
                }
            }
        }
    }

public:
    double heuristic(Position p, Position goal, int type)
    {
        return (type == 1) ? euclidean(p, goal) : manhattan(p, goal);
    }
    SearchResult greedySearch(Maze &maze, Position start, int type) { return runSearch(maze, start, type, false); }
    SearchResult aStarSearch(Maze &maze, Position start, int type)  { return runSearch(maze, start, type, true); }
};

void printResult(const char *name, SearchResult &r, Maze &maze)
{
    cout << name ;
    if (!r.found) { cout << "No path found\n"; return; }

    cout << "Cost: " << r.pathCost << "   Expanded: " << r.expandedNodes
         << "   Generated: " << r.generatedNodes << "\nPath: ";
    for (int i = 0; i < r.pathLength; i++)
    {
        cout << "(" << r.path[i].row << "," << r.path[i].col << ")";
        if (i < r.pathLength - 1) cout << " -> ";
    }
    cout << endl;
    maze.display(&r);
}


int main()
{
    Maze maze;
    Search search;

    if (!maze.inputMaze())
        return 0;

    cout << "\n Maze:\n";
    maze.display();


 Position start;

cout << "\nEnter starting row: ";
cin >> start.row;

cout << "Enter starting column: ";
cin >> start.col;

if (!cin)
{
    cout << "Invalid input!\n";
    return 0;
}

while (!maze.isValidStart(start.row, start.col))
{
    cout << "Invalid position! Try again.\n";

    cout << "Enter starting row: ";
    cin >> start.row;

    cout << "Enter starting column: ";
    cin >> start.col;
}
    SearchResult results[4];

    results[0] = search.greedySearch(maze, start, 1);
    results[1] = search.greedySearch(maze, start, 2);
    results[2] = search.aStarSearch(maze, start, 1);
    results[3] = search.aStarSearch(maze, start, 2);

    const char *names[4] =
    {
        "Greedy Search h1",
        "Greedy Search h2",
        "A* Search h1",
        "A* Search h2"
    };



    for (int i = 0; i < 4; i++)
    {
        cout << "\n" << names[i] << "\n";
        printResult("", results[i], maze);
    }

    return 0;
}
