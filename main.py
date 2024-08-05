from algoritmos import (
    eh_conexo, eh_bipartido, possui_caminho_euleriano,
    possui_ciclo, componentes_conexas, componentes_fortemente_conexas,
    caminho_euleriano, caminho_hamiltoniano, pontos_de_articulacao, arestas_ponte,
    arvore_profundidade, arvore_largura, arvore_geradora_minima,
    ordenacao_topologica, caminho_minimo, fluxo_maximo, fecho_transitivo
)
from utilidades import ler_grafo_do_exemplo, imprimir_resultados

import networkx as nx
import matplotlib.pyplot as plt

def desenhar_grafo(grafo):
    """
    Função para desenhar o grafo utilizando a biblioteca NetworkX.
    Se o grafo for direcionado, desenha setas; se houver pesos nas arestas, eles são exibidos.
    """
    G = nx.DiGraph() if grafo.direcionado else nx.Graph()

    for u in grafo.lista_adjacencia:
        for v in grafo.lista_adjacencia[u]:
            G.add_edge(u, v, weight=grafo.pesos_arestas.get((u, v), 1))

    pos = nx.spring_layout(G)
    labels = nx.get_edge_attributes(G, 'weight')
    nx.draw(G, pos, with_labels=True, node_color='skyblue', node_size=700, font_size=12, font_weight='bold', arrows=grafo.direcionado)
    nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)
    plt.show()

def perguntar_funcoes_ao_usuario():
    """
    Função que pergunta ao usuário quais funções deseja testar.
    O usuário pode escolher funções específicas ou testar todas.
    """
    entrada_funcoes = input("Quais funções vão ser testadas? (use números separados por espaço ou 'todas' para testar todas): ").strip().lower()
    
    funcoes_validas = list(range(1, 18))  # Números válidos de funções (1 a 17)

    if entrada_funcoes == "todas":
        return funcoes_validas
    
    try:
        numeros_funcoes = list(map(int, entrada_funcoes.split()))
        if any(num not in funcoes_validas for num in numeros_funcoes):
            print(f"Erro: Insira apenas números válidos entre 1 e {len(funcoes_validas)}. Exemplo: 1 3 5")
            return None
    except ValueError:
        print("Erro: Por favor, insira números inteiros separados por espaço ou 'todas'. Exemplo: 1 3 5 ou todas")
        return None

    return numeros_funcoes

def executar_funcoes(grafo, funcoes):
    """
    Função que executa as funções escolhidas pelo usuário no grafo.
    Retorna um dicionário com os resultados de cada função.
    """
    resultados = {}
    direcionado = grafo.direcionado

    for funcao in funcoes:
        if funcao == 1:
            resultados["1 - Conexo"] = 1 if eh_conexo(grafo) else 0
        elif funcao == 2:
            resultados["2 - Bipartido"] = 1 if eh_bipartido(grafo) else 0
        elif funcao == 3:
            resultados["3 - Euleriano"] = 1 if possui_caminho_euleriano(grafo) else 0
        elif funcao == 4:
            resultados["4 - Possui ciclo"] = 1 if possui_ciclo(grafo) else 0
        elif funcao == 5:
            componentes = componentes_conexas(grafo)
            resultados["5 - Componentes conexas"] = " ".join(" ".join(map(str, sorted(componente))) for componente in componentes)
        elif funcao == 6:
            resultados["6 - Componentes fortemente conexas"] = " ".join(map(str, componentes_fortemente_conexas(grafo))) if direcionado else "-1"
        elif funcao == 7:
            caminho_eulerian = caminho_euleriano(grafo)
            resultados["7 - Caminho Euleriano"] = " ".join(map(str, caminho_eulerian)) if isinstance(caminho_eulerian, list) else "-1"
        elif funcao == 8:
            caminho_hamiltonian = caminho_hamiltoniano(grafo)
            resultados["8 - Caminho Hamiltoniano"] = " ".join(map(str, caminho_hamiltonian)) if caminho_hamiltonian else "-1"
        elif funcao == 9:
            pontos_articulacao = pontos_de_articulacao(grafo)
            resultados["9 - Vértices de articulação"] = " ".join(map(str, pontos_articulacao)) if isinstance(pontos_articulacao, list) else "-1"
        elif funcao == 10:
            arestas_pont = arestas_ponte(grafo)
            resultados["10 - Arestas ponte"] = " ".join([f"{u},{v}" for u, v in arestas_pont]) if isinstance(arestas_pont, list) else "-1"
        elif funcao == 11:
            resultado_arvore = arvore_profundidade(grafo)
            resultados["11 - Árvore de profundidade"] = " ".join([f"{u},{v}" for u, v in resultado_arvore]) if isinstance(resultado_arvore, list) else "-1"
        elif funcao == 12:
            resultado_arvore_largura = arvore_largura(grafo)
            resultados["12 - Árvore de largura"] = " ".join([f"{u},{v}" for u, v in resultado_arvore_largura]) if isinstance(resultado_arvore_largura, list) else "-1"
        elif funcao == 13:
            arvore_mst = arvore_geradora_minima(grafo)
            resultados["13 - Árvore geradora mínima"] = " ".join([f"{u},{v}" for u, v in arvore_mst]) if isinstance(arvore_mst, list) else "-1"
        elif funcao == 14:
            ordenacao = ordenacao_topologica(grafo) if direcionado else "-1"
            resultados["14 - Ordem topológica"] = " ".join(map(str, ordenacao)) if isinstance(ordenacao, list) else "-1"
        elif funcao == 15:
            caminho_minimo_valor = caminho_minimo(grafo, 0, grafo.vertices-1)
            resultados["15 - Valor do caminho mínimo (0 -> (n-1))"] = caminho_minimo_valor
        elif funcao == 16:
            fluxo = fluxo_maximo(grafo, 0, grafo.vertices-1) if direcionado else "-1"
            resultados["16 - Valor do fluxo máximo (0 -> (n-1))"] = fluxo
        elif funcao == 17:
            fecho_transitivo_resultado = fecho_transitivo(grafo) if direcionado else "-1"
            resultados["17 - Fecho transitivo (0)"] = fecho_transitivo_resultado

    return resultados

def main():
    """
    Função principal que controla o fluxo do programa:
    1. Lê o grafo do exemplo.
    2. Desenha o grafo.
    3. Pergunta ao usuário quais funções deseja testar.
    4. Executa as funções escolhidas e exibe os resultados.
    """
    grafo = ler_grafo_do_exemplo()
    
    # Verifica se o grafo é None, o que indica que a entrada foi interrompida
    if grafo is None:
        print("Execução interrompida pelo usuário. Nenhum grafo foi criado.")
        return

    print("Grafo:", grafo.lista_adjacencia)

    # Desenhar o grafo
    desenhar_grafo(grafo)

    # Perguntar ao usuário quais funções testar
    funcoes = perguntar_funcoes_ao_usuario()
    if funcoes is None:
        return

    # Executar as funções selecionadas
    resultados = executar_funcoes(grafo, funcoes)
    imprimir_resultados(resultados)

if __name__ == "__main__":
    main()

def imprimir_resultados(resultados):
    """
    Função que imprime os resultados das funções executadas.
    """
    for chave, valor in resultados.items():
        print(f"{chave}: {valor}")
