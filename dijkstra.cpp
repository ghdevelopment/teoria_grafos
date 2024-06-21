#include <iostream>
#include <fstream>
#include <vector>
#include <limits>
#include <algorithm>
#include <stack> // Para utilizar stack (pilha)

using namespace std;

struct VertexInfo
{
    int distance;
    int predecessor;
};

void initializeSingleSource(int s, vector<VertexInfo> &vertexInfo)
{
    int V = vertexInfo.size();
    for (int v = 0; v < V; ++v)
    {
        vertexInfo[v].distance = numeric_limits<int>::max();
        vertexInfo[v].predecessor = -1;
    }
    vertexInfo[s].distance = 0;
}

void relax(int u, int v, int weight, vector<VertexInfo> &vertexInfo)
{
    if (vertexInfo[v].distance > vertexInfo[u].distance + weight)
    {
        vertexInfo[v].distance = vertexInfo[u].distance + weight;
        vertexInfo[v].predecessor = u;
    }
}

bool readGraphFromFile(const string &filename, vector<vector<pair<int, int>>> &G, int &numVertices, int &numEdges)
{
    ifstream file(filename);
    if (!file.is_open())
    {
        cerr << "Erro ao abrir o arquivo: " << filename << endl;
        return false;
    }

    file >> numVertices >> numEdges;
    G.resize(numVertices);

    int u, v, weight;
    bool hasNegativeWeight = false;

    while (file >> u >> v)
    {
        if (file.peek() == '\n' || file.peek() == '\r' || file.peek() == EOF)
        {
            weight = 1;
        }
        else
        {
            file >> weight;
        }

        if (weight < 0)
        {
            cerr << "Erro: Encontrado peso negativo (" << weight << ") em uma das arestas." << endl;
            hasNegativeWeight = true;
            break;
        }

        G[u].emplace_back(v, weight);
    }

    file.close();

    return !hasNegativeWeight;
}

void dijkstra(vector<vector<pair<int, int>>> &G, int s, vector<VertexInfo> &vertexInfo)
{
    int V = G.size();
    vector<bool> visited(V, false); // Para controlar se um vértice já foi visitado
    initializeSingleSource(s, vertexInfo);

    for (int i = 0; i < V - 1; ++i)
    {
        int u = -1;
        for (int j = 0; j < V; ++j)
        {
            if (!visited[j] && (u == -1 || vertexInfo[j].distance < vertexInfo[u].distance))
            {
                u = j;
            }
        }

        if (vertexInfo[u].distance == numeric_limits<int>::max())
        {
            break; // Se u é inalcançável, podemos sair do loop
        }

        visited[u] = true;

        for (auto &neighbor : G[u])
        {
            int v = neighbor.first;
            int weight = neighbor.second;
            relax(u, v, weight, vertexInfo);
        }
    }
}

void printShortestPaths(int sourceVertex, const vector<VertexInfo> &vertexInfo)
{
    int V = vertexInfo.size();
    cout << "Caminhos mínimos a partir do vértice " << sourceVertex << ":" << endl;

    for (int i = 0; i < V; ++i)
    {
        cout << "Para o vértice " << i << ": ";
        if (vertexInfo[i].distance == numeric_limits<int>::max())
        {
            cout << "Não há caminho alcançável." << endl;
        }
        else
        {
            cout << "Distância = " << vertexInfo[i].distance << ", Predecessor = ";
            if (vertexInfo[i].predecessor == -1)
            {
                cout << "Nulo";
            }
            else
            {
                cout << vertexInfo[i].predecessor;
            }

            cout << ", Caminho = ";

            // Recuperar o caminho mínimo usando uma pilha
            stack<int> path;
            int p = i;
            while (p != -1)
            {
                path.push(p);
                p = vertexInfo[p].predecessor;
            }

            while (!path.empty())
            {
                cout << path.top();
                path.pop();
                if (!path.empty())
                {
                    cout << " -> ";
                }
            }

            cout << endl;
        }
    }
}

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        cerr << "Uso: " << argv[0] << " <nome_do_arquivo>" << endl;
        return 1;
    }

    string filename = argv[1];
    int numVertices, numEdges;
    vector<vector<pair<int, int>>> graph;

    if (!readGraphFromFile(filename, graph, numVertices, numEdges))
    {
        return 1;
    }

    int sourceVertex = 0;
    vector<VertexInfo> vertexInfo(numVertices);

    dijkstra(graph, sourceVertex, vertexInfo);
    printShortestPaths(sourceVertex, vertexInfo);

    return 0;
}
