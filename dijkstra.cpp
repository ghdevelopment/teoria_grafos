#include <iostream>
#include <fstream>
#include <vector>
#include <limits>
#include <algorithm>
#include <stack> // Para utilizar stack (pilha)

using namespace std;

// Estrutura para armazenar informações de cada vértice durante o algoritmo de Dijkstra
struct VertexInfo
{
    int distance;    // Distância mínima até o vértice
    int predecessor; // Vértice predecessor no caminho mínimo
};

// Inicializa as distâncias e predecessores de todos os vértices, exceto o vértice origem 's'
void initializeSingleSource(int s, vector<VertexInfo> &vertexInfo)
{
    int V = vertexInfo.size(); // Número total de vértices no grafo
    for (int v = 0; v < V; ++v)
    {
        vertexInfo[v].distance = numeric_limits<int>::max(); // Inicializa todas as distâncias como infinito
        vertexInfo[v].predecessor = -1;                      // Define todos os predecessores como -1 (nulo)
    }
    vertexInfo[s].distance = 0; // Define a distância do vértice origem como 0
}

// Relaxamento da aresta (u, v) com peso 'weight'
void relax(int u, int v, int weight, vector<VertexInfo> &vertexInfo)
{
    // Se encontrarmos um caminho mais curto até 'v' através de 'u', atualizamos as informações
    if (vertexInfo[v].distance > vertexInfo[u].distance + weight)
    {
        vertexInfo[v].distance = vertexInfo[u].distance + weight; // Atualiza a distância mínima até 'v'
        vertexInfo[v].predecessor = u;                            // Atualiza o predecessor de 'v' no caminho mínimo
    }
}

// Lê o grafo de um arquivo e armazena em uma estrutura de dados adequada
bool readGraphFromFile(const string &filename, vector<vector<pair<int, int>>> &G, int &numVertices, int &numEdges)
{
    ifstream file(filename); // Abre o arquivo
    if (!file.is_open())
    {
        cerr << "Erro ao abrir o arquivo: " << filename << endl; // Exibe mensagem de erro se não conseguir abrir o arquivo
        return false;
    }

    file >> numVertices >> numEdges; // Lê o número de vértices e arestas do arquivo
    G.resize(numVertices);           // Redimensiona o vetor de adjacências para o número de vértices

    int u, v, weight;
    bool hasNegativeWeight = false;

    while (file >> u >> v)
    {
        if (file.peek() == '\n' || file.peek() == '\r' || file.peek() == EOF)
        {
            weight = 1; // Define o peso da aresta como 1 se não houver peso especificado
        }
        else
        {
            file >> weight; // Lê o peso da aresta do arquivo
        }

        if (weight < 0)
        {
            cerr << "Erro: Encontrado peso negativo (" << weight << ") em uma das arestas." << endl; // Exibe mensagem de erro se houver peso negativo
            hasNegativeWeight = true;
            break;
        }

        G[u].emplace_back(v, weight); // Adiciona a aresta ao grafo direcionado
    }

    file.close(); // Fecha o arquivo após a leitura

    return !hasNegativeWeight; // Retorna verdadeiro se não houver pesos negativos no grafo
}

// Algoritmo de Dijkstra para encontrar os caminhos mínimos a partir de um vértice origem 's'
void dijkstra(vector<vector<pair<int, int>>> &G, int s, vector<VertexInfo> &vertexInfo)
{
    int V = G.size();                      // Número total de vértices no grafo
    vector<bool> visited(V, false);        // Vetor para controlar se um vértice já foi visitado durante o algoritmo
    initializeSingleSource(s, vertexInfo); // Inicializa as distâncias e predecessores dos vértices

    for (int i = 0; i < V - 1; ++i)
    {
        int u = -1;
        // Encontra o vértice não visitado com a menor distância mínima atual
        for (int j = 0; j < V; ++j)
        {
            if (!visited[j] && (u == -1 || vertexInfo[j].distance < vertexInfo[u].distance))
            {
                u = j;
            }
        }

        if (vertexInfo[u].distance == numeric_limits<int>::max())
        {
            break; // Se 'u' é inalcançável, interrompe o loop
        }

        visited[u] = true; // Marca o vértice 'u' como visitado

        // Relaxa todas as arestas saindo de 'u'
        for (auto &neighbor : G[u])
        {
            int v = neighbor.first;          // Vértice adjacente
            int weight = neighbor.second;    // Peso da aresta (u, v)
            relax(u, v, weight, vertexInfo); // Aplica o relaxamento da aresta (u, v)
        }
    }
}

// Imprime os caminhos mínimos a partir de um vértice origem 'sourceVertex'
void printShortestPaths(int sourceVertex, const vector<VertexInfo> &vertexInfo)
{
    int V = vertexInfo.size(); // Número total de vértices no grafo
    cout << "Caminhos mínimos a partir do vértice " << sourceVertex << ":" << endl;

    for (int i = 0; i < V; ++i)
    {
        cout << "Para o vértice " << i << ": ";
        if (vertexInfo[i].distance == numeric_limits<int>::max())
        {
            cout << "Não há caminho alcançável." << endl; // Exibe mensagem se não houver caminho até o vértice 'i'
        }
        else
        {
            cout << "Distância = " << vertexInfo[i].distance << ", Predecessor = ";
            if (vertexInfo[i].predecessor == -1)
            {
                cout << "Nulo"; // Exibe "Nulo" se o vértice 'i' não tiver predecessor
            }
            else
            {
                cout << vertexInfo[i].predecessor; // Exibe o vértice predecessor de 'i'
            }

            cout << ", Caminho = ";

            // Recupera o caminho mínimo usando uma pilha
            stack<int> path;
            int p = i;
            while (p != -1)
            {
                path.push(p);
                p = vertexInfo[p].predecessor; // Retrocede no caminho até o vértice origem 'sourceVertex'
            }

            while (!path.empty())
            {
                cout << path.top();
                path.pop();
                if (!path.empty())
                {
                    cout << " -> "; // Exibe seta entre os vértices no caminho
                }
            }

            cout << endl;
        }
    }
}

// Função principal que lê o grafo de um arquivo e executa o algoritmo de Dijkstra
int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        cerr << "Uso: " << argv[0] << " <nome_do_arquivo>" << endl; // Exibe mensagem de uso correto se o nome do arquivo não for fornecido
        return 1;
    }

    string filename = argv[1];            // Nome do arquivo passado como argumento
    int numVertices, numEdges;            // Número de vértices e arestas no grafo
    vector<vector<pair<int, int>>> graph; // Estrutura de dados para armazenar o grafo

    if (!readGraphFromFile(filename, graph, numVertices, numEdges))
    {
        return 1; // Retorna 1 se houver erro ao ler o arquivo
    }

    int sourceVertex = 0;                       // Vértice origem para o algoritmo de Dijkstra
    vector<VertexInfo> vertexInfo(numVertices); // Estrutura para armazenar as informações dos vértices

    dijkstra(graph, sourceVertex, vertexInfo);    // Executa o algoritmo de Dijkstra para encontrar os caminhos mínimos
    printShortestPaths(sourceVertex, vertexInfo); // Imprime os caminhos mínimos a partir do vértice origem

    return 0; // Retorna 0 indicando que o programa foi executado com sucesso
}