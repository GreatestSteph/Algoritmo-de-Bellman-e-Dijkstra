import sys
# Algoritmos de Dijkstra e Bellman-Ford

# Grafo e algoritmos de bellman e dijkstra
class Grafo:
    def __init__(self, vertices):
        self.V = vertices  # Número de vértices no grafo
        self.grafo = [[] for _ in range(vertices)]  # Lista para armazenar as arestas do grafo


    def adiciona_aresta(self, origem, destino, peso):
        # Adiciona uma aresta ao grafo com os vértices origem, destino e o peso
        self.grafo[origem].append((destino, peso))


    def bellman_ford(self, origem):
        # Inicializa as distâncias com infinito, exceto para a origem que é 0
        distancia = [float('inf')] * self.V
        distancia[origem] = 0

        # Relaxamento das arestas repetidas
        for _ in range(self.V - 1):
            # Percorre todas as arestas do grafo
            for u in range(self.V):
                # Para cada vértice de origem, percorre todas as arestas conectadas a ele
                for v, w in self.grafo[u]:
                    # Verifica se a distância atual até o destino é infinita (não foi alcançado ainda)
                    # ou se uma rota mais curta é encontrada
                    if distancia[u] != float('inf') and distancia[u] + w < distancia[v]:
                        # Atualiza a distância se encontrar um caminho mais curto
                        distancia[v] = distancia[u] + w

        # Verifica ciclos negativos
        for u in range(self.V):
            # Percorre novamente todas as arestas do grafo
            for v, w in self.grafo[u]:
                # Verifica se há uma rota mais curta após a percorrer
                if distancia[u] != float('inf') and distancia[u] + w < distancia[v]:
                    # Se sim, então mostra a mensagem que o grafo contém um ciclo negativo e encerra a função
                    print("O grafo contém um ciclo negativo.")
                    return

        # Se não houver ciclos negativos, então mostra os resultados
        self.imprime_resultados(distancia)


    def dijkstra(self, origem):
        # Inicia uma lista para rastrear quais vértices foram visitados durante o processo
        visitados = [False] * self.V

        # Inicia uma lista para armazenar as distâncias mínimas de um vértice da origem para cada vértice no grafo
        distancia = [float('inf')] * self.V

        # Define a distância do vértice da origem para ele mesmo como 0, pois não tem custo para alcançar o próprio vértice
        distancia[origem] = 0

        # Encontra o caminho mínimo para todos os vértices
        for _ in range(self.V):
            u = self.menor_distancia(distancia, visitados)
            visitados[u] = True

            for v, w in self.grafo[u]:
                # Atualiza distância se uma rota mais curta for encontrada
                if not visitados[v] and distancia[u] + w < distancia[v]:
                    distancia[v] = distancia[u] + w

        self.imprime_resultados(distancia)


    def menor_distancia(self, distancia, visitados):
        # Variável para armazena a menor distância encontrada como infinito
        minimo = float('inf')

        # O índice do vértice com a menor distância recebe 0
        minimo_index = 0

        # Percorre todos os vértices no grafo
        for v in range(self.V):
            # Verifica se a distância do vértice atual é menor que o mínimo e se o vértice não foi visitado
            if distancia[v] < minimo and not visitados[v]:
                # Atualiza o mínimo e o índice do vértice com a menor distância
                minimo = distancia[v]
                minimo_index = v

        # Retorna o índice do vértice com a menor distância não visitado
        return minimo_index

    def imprime_resultados(self, distancia):
        # Imprime as distâncias mínimas a partir do vértice 0
        print("Distâncias mínimas a partir do vértice 0:")
        for i in range(self.V):
            print(f"Vértice {i}: {distancia[i] if distancia[i] != float('inf') else 'inf'}")


# Função principal para a entrada do usuário e execução dos algoritmos
def main():
    # Obtém o número de vértices no grafo
    vertices = int(input("Digite o número de vértices no grafo: "))
    grafo = Grafo(vertices)

    # Obtém o número de arestas e suas informações
    arestas = int(input("Digite o número de arestas no grafo: "))

    for i in range(arestas):
        try:
            # Obtém a entrada do usuário para uma aresta (origem, destino, peso)
            origem, destino, peso = map(int, input(
                f"Digite os valores sobre a aresta {i + 1}, separando com espaço (origem, destino, peso): ").split())
            # Adiciona a aresta ao grafo
            grafo.adiciona_aresta(origem, destino, peso)
        except ValueError:
            # Trata erros de entrada inválida
            print("Entrada inválida. Certifique-se de fornecer três valores inteiros separados por espaço.")
            sys.exit(1)

    # Escolhe o algoritmo a ser executado
    algoritmo = input("Escolha o algoritmo (digite bellman ou dijkstra): ").lower()

    # Executa o algoritmo selecionado
    if algoritmo == "bellman":
        origem = 0
        grafo.bellman_ford(origem)
    elif algoritmo == "dijkstra":
        origem = 0
        grafo.dijkstra(origem)
    else:
        print("Digite corretamente! Escolha entre bellman e dijkstra.")


# Verifica se o código está sendo executado diretamente
if __name__ == "__main__":
    main()
