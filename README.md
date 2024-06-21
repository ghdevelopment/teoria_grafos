# teoria_grafos
Repositório criado para colocar o algoritmo de Dijkstra realizado para a matéria Teoria de Grafos
ministrada pelo professor André da Cunha Ribeiro.

Aluno: Guilherme Honório Caetano
2022102201940254

*AVISOS*

-O código foi implementado para utilização de lista de adjacência.

-Além do código, deixei alguns dos arquivos de teste que utilizei para testar o algoritmo.
-Para executar o algoritmo, basta colocar os "arquivos.txt" e o "arquivo.cpp" com o código na mesma pasta,
acessar o terminal e gerar o executável.

$ g++ dijkstra.cpp -o dijkstra

em seguida, você pode criar novos arquivos com diferentes grafos.

o padrão de leitura é:
na primeira linha você coloca a quantidade de vértices seguido da quantidade de arestas: 

<exemplo>
6 7 

nas demais linhas serão representadas as arestas entre o vértice de origem e vértice de destino, seguido do peso (todos em números).
-em caso de peso negativo o algoritmo será abortado.
-em caso de representações sem peso, será atribuído o valor 1 por padrão.

<exemplo>
0 1 10
0 2 25
.
.
.
Ao final a representação do arquivo será:

6 7
0 1 10
0 2 25
.
.
.

para executar o algoritmo após já ter criado o executável e preparado o arquivo de texto:

$ ./dijkstra.exe nome_arquivo.txt
