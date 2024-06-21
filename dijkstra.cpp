#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <limits>    // Para usar std::numeric_limits
#include <algorithm> // Para usar std::remove_if

using namespace std;

// Estrutura para armazenar informações de distância e predecessor
struct VertexInfo
{
    int distance;    // Distância mínima do vértice de origem
    int predecessor; // Vértice predecessor no caminho mínimo
};

// Função para inicializar as distâncias e predecessores
void initializeSingleSource(int s, vector<VertexInfo> &vertexInfo)
{
    int V = vertexInfo.size(); // Número de vértices

    for (int v = 0; v < V; ++v)
    {
        vertexInfo[v].distance = numeric_limits<int>::max(); // Inicializa todas as distâncias com infinito
        vertexInfo[v].predecessor = -1;                      // Inicializa todos os predecessores como -1 (NULL)
    }

    vertexInfo[s].distance = 0; // Distância do vértice de origem s para ele mesmo é 0
}

// Função para relaxamento de uma aresta
void relax(int u, int v, int weight, vector<VertexInfo> &vertexInfo)
{
    if (vertexInfo[v].distance > vertexInfo[u].distance + weight)
    {
        vertexInfo[v].distance = vertexInfo[u].distance + weight;
        vertexInfo[v].predecessor = u;
    }
}

// Função para ler o grafo de um arquivo txt
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
        // Verifica se foi fornecido um peso, senão atribui 1 como peso padrão
        if (file.peek() == '\n' || file.peek() == '\r' || file.peek() == EOF)
        {
            weight = 1;
        }
        else
        {
            file >> weight;
        }

        // Verifica se há peso negativo
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

// Algoritmo de Dijkstra
// Algoritmo de Dijkstra
// Algoritmo de Dijkstra
void dijkstra(vector<vector<pair<int, int>>> &G, int s, vector<VertexInfo> &vertexInfo)
{
    int V = G.size(); // Número de vértices
    vector<int> Q(V); // Fila de prioridade inicialmente contendo todos os vértices

    for (int i = 0; i < V; ++i)
    {
        vertexInfo[i].distance = numeric_limits<int>::max(); // Inicializa todas as distâncias com infinito
        vertexInfo[i].predecessor = -1;                      // Inicializa todos os predecessores como -1 (NULL)
        Q[i] = i;                                            // Todos os vértices estão em Q inicialmente
    }

    vertexInfo[s].distance = 0; // Distância do vértice de origem s para ele mesmo é 0

    while (!Q.empty())
    {
        // Encontra o vértice u com a menor distância em d dentro de Q
        int u = *min_element(Q.begin(), Q.end(), [&](int a, int b) {
            return vertexInfo[a].distance < vertexInfo[b].distance;
        });

        // Remove u de Q
        Q.erase(remove(Q.begin(), Q.end(), u), Q.end());

        // Para cada vértice v adjacente a u
        for (auto &neighbor : G[u])
        {
            int v = neighbor.first;
            int weight = neighbor.second;
            // Realiza o relaxamento de u para v
            if (vertexInfo[v].distance > vertexInfo[u].distance + weight)
            {
                vertexInfo[v].distance = vertexInfo[u].distance + weight;
                vertexInfo[v].predecessor = u;
            }
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

    string filename = argv[1]; // Nome do arquivo de entrada
    int numVertices, numEdges;
    vector<vector<pair<int, int>>> graph; // Grafo como lista de adjacências de pares (vértice, peso)

    // Lê o grafo do arquivo
    if (!readGraphFromFile(filename, graph, numVertices, numEdges))
    {
        return 1; // Aborta se houver erro na leitura do arquivo
    }

    int sourceVertex = 0; // Vértice de origem

    // Vetor para armazenar informações de cada vértice
    vector<VertexInfo> vertexInfo(numVertices);

    // Chamada do algoritmo de Dijkstra para encontrar caminhos mínimos
    dijkstra(graph, sourceVertex, vertexInfo);

    // Exibindo as distâncias mínimas calculadas a partir do vértice de origem
    cout << "Distâncias mínimas a partir do vértice " << sourceVertex << ":" << endl;
    for (int i = 0; i < numVertices; ++i)
    {
        cout << "Vértice " << i << ": ";
        if (vertexInfo[i].distance == numeric_limits<int>::max())
        {
            cout << "Distância = Infinito, Predecessor = Nulo" << endl;
        }
        else
        {
            cout << "Distância = " << vertexInfo[i].distance << ", Predecessor = ";
            if (vertexInfo[i].predecessor == -1)
            {
                cout << "Nulo" << endl;
            }
            else
            {
                cout << vertexInfo[i].predecessor << endl;
            }
        }
    }

    return 0;
}
