import numpy as np
import heapq
from collections import defaultdict
from queue import *


class Graph_l:   
    def __init__(self, v, a, d): #(número de vértices, arestas, direcionado(bool))
        self.aqruivo_saida = [] #Lista que vai receber os resultados desejados
        self.v = v
        self.a = a
        self.d = d 
        self.num_arestas = len(self.a) 
        self.neg = False
        self.lista = [[] for i in range(self.v)] #Não sabemos quantos elementos tem em cada linha (quantos vizinhos cada vértice tem)
        self.residual = [[] for i in range(self.v)]
        self.pesos = [[] for i in range(self.v)]  #lista que contém os pesos 
        if self.d==False:
            for i in range(self.num_arestas):
                u=self.a[i][0] #Pega o primeiro vértice do par de arestas
                v= self.a[i][1] #Pega o segundo vértice do par de arestas
                p= self.a[i][2] #Pega o peso da aresta
                if p < 0: self.neg = True
                self.lista[u-1]+= [[v ,p, ]] #Adiciona o vértice v na linha u (o índice é u-1, pois i se inicia no 0)
                self.lista[v-1]+= [[u, p]] #Adiciona o vértice u na linha v
                self.pesos[u-1]+= [p]
                self.pesos[v-1] += [p]
        else:
            for i in range(self.num_arestas):
                u=self.a[i][0] #Pega o primeiro vértice do par de arestas
                v= self.a[i][1] #Pega o segundo vértice do par de arestas
                p= self.a[i][2]
                if p < 0: self.neg = True
                self.lista[u-1]+= [[v, p]] #Adiciona o vértice v na linha u (o índice é u-1, pois i se inicia no 0)
                self.residual[u-1] += [[v, p, True]] #Aresta original
                self.residual[v-1] += [[u, 0.0, False]] #Aresta residual
                self.pesos[u-1]+= [p]   

    def mostra_lista(self): #Retorna a lista de adjacência
        return self.lista 
    
    def BFS_ff(self, vi):
        self.marcados = np.zeros(self.v,dtype=int) #Lista com os nós que já foram explorados
        self.Q = Queue() #Cria uma fila vazia
        pai = np.array([-1] * self.v, dtype = int) #inicia vetor com os pais
        nivel = np.array([-1] * self.v, dtype = int) #inicia vetor dos níveis
        nivel[vi -1] = 0
        self.marcados[vi-1]=1#Marca vi, que é o nó em que começamos a busca
        self.Q.add(vi) #Adciona o nó na fila
        pai[vi-1] = 0 #Indica que é raiz 
        while self.Q.is_empty() == False: #A busca continua até a lista ficar vazia(Até todos os vértices serem explorados)
            v = self.Q.pop() #Remove o último vértice da fila, que corresponde ao vértice mais antigo
            for w in self.residual[v-1]: #Visita os vizinhos de v
                #print(f'bfs:\n {w}\n{self.residual[v-1]}')
                if w[2] == False: pass
                else:
                    if self.marcados[w[0]-1]==0: 
                        pai[w[0]-1]= v #Se w não tiver sido marcado, o vértice que o descobriu(seu pai) foi v
                        self.marcados[w[0]-1]=1 #Marca w se não foi descoberto ainda
                        self.Q.add(w[0]) #Adiciona w na primeira poição da fila
        return pai

    def BFS(self,vi,p): #Implementação da busca em largura
        self.marcados = np.zeros(self.v,dtype=int) #Lista com os nós que já foram explorados
        self.Q = Queue() #Cria uma fila vazia
        pai = np.array([-1] * self.v, dtype = int) #inicia vetor com os pais
        nivel = np.array([-1] * self.v, dtype = int) #inicia vetor dos níveis
        nivel[vi -1] = 0
        self.marcados[vi-1]=1#Marca vi, que é o nó em que começamos a busca
        self.Q.add(vi) #Adciona o nó na fila
        pai[vi-1] = 0 #Indica que é raiz 
        while self.Q.is_empty() == False: #A busca continua até a lista ficar vazia(Até todos os vértices serem explorados)
            v = self.Q.pop() #Remove o último vértice da fila, que corresponde ao vértice mais antigo
            for w in self.lista[v-1]: #Visita os vizinhos de v
                if self.marcados[w[0]-1]==0: 
                    pai[w[0]-1]= v #Se w não tiver sido marcado, o vértice que o descobriu(seu pai) foi v
                    nivel[w[0]-1] = nivel[v-1]+1 #Muda o nível atual para o nível de w
                    self.marcados[w[0]-1]=1 #Marca w se não foi descoberto ainda
                    self.Q.add(w[0]) #Adiciona w na primeira poição da fila

        self.maxlevel = np.max(nivel) #Pega o nível máximo

        self.BFStree = [pai, nivel, self.maxlevel] 
        if p==1:
            print(f'\nBusca BFS:\nPais: {pai}\nNíveis: {nivel}')
        return (self.BFStree)

    
    def Dijkstra(self,vi, p):  #vi é o vértice inicial e p decide se o resultado precisa ser printado 
        if self.neg == True:
            d = 'Não é possível'
            if p ==1: 
                print(f'\nDijkstra a partir de {vi} : d ')
        else:  
            pai = np.array([-1] * self.v, dtype = int) #inicia vetor com os pais
            nivel = np.array([-1] * self.v, dtype = int) #inicia vetor dos níveis
            S = np.zeros(self.v) #Definir S como vazio (conjunto dos vértices que já achamos a menor distância)
            len_S = 0 #variável auxiliar para não ter que verificar a cada iteração a lista S
            dist = np.full(self.v, np.inf) #distâncias começam infinitas
            dist_aux = np.full(self.v, np.inf) #usado para não ter que achar o mínimo em dist e ainda verificar se está em S, os vértices já adicionados em S estrão como inf.
            dist_aux[vi-1]=0 
            pai [vi-1] = 0
            nivel [vi-1] = 0 
            while len_S != self.v : #Enquanto S != V
                p_u = np.min(dist_aux) #Selecione u em V-S, tal que dist[u] é mínima (peso da aresta mínima)
                u = np.argmin(dist_aux) #u é o vértice que tem a distância mínima
                if p_u == np.inf :   #todas as distâncias são infinitas e S!= V, então grafo não é conexo
                    break
                dist[u]=dist_aux[u] #atualizamos a distância defiitiva 
                dist_aux[u]= np.inf  #usado para que não alteremos mais a distância de quem já foi adicionado a S
                S[u]= 1 #Adicione u em S
                len_S += 1 #atualizando a quantidade de vértices em S
                for viz in range(len(self.lista[u])): #Para cada vizinho de u faça
                    v = self.lista[u][viz][0] #v é o vértice  vizinho a u a ser analisado
                    if S[v-1] ==0: #se ainda não está em S 
                        if dist_aux[v-1] > dist[u] + self.lista[u][viz][1] :  #Se dist[v] > dist[u] + w(u,v) então
                            dist_aux[v-1] = dist[u] + self.lista[u][viz][1] #dist[v] = dist[u] + w(u,v), atualizando a distância no auxiliar 
                            pai[v-1] = u+1 #atualiza o pai 
                            nivel[v-1] = nivel[u] + 1 #atualiza o nível 
            d = [dist, pai, nivel] #retorna as distâncias, os pais e s níveis.
            if p ==1: 
                print(f'\nDijkstra a partir de {vi} : {d[0]}, árvore (pais): {d[1]}, árvore(níveis): {d[2]} ')
        return d
    
    def DijkstraHeap(self, vi,p ) :
        if self.neg == True:
            d = 'Não é possível'
            if p ==1: 
                print(f'\nDijkstra a partir de {vi} : {d} ')
        else:
            pais = np.array([-1] * self.v, dtype = int) #inicia vetor com os pais
            pais[vi-1] = 0
            dist = np.full(self.v, np.inf) #define um vetor de custos/distancias e pais
            dist[vi-1] = 0 #define a distancia do primeiro vertice como 0 e pai vi
            h = [(0, vi)]   #pq - priority queue, adiciona o primeiro vértice no heap com distância 0
            while len(h) > 0:
                dist_atual, u =  heapq.heappop(h) #pega o primeiro valor da fila de prioridade(menor dist)
                for vizinhos in self.lista[u-1]:
                    viz = vizinhos[0]
                    peso = vizinhos[1]
                    d = dist_atual + peso
                    if d < dist[viz-1]:
                        dist[viz-1] = d
                        heapq.heappush(h, (d, viz))
                        pais[viz-1] = u
            if p==1 : 
                print ('Distância com heap: ', dist)
        return (dist, pais)

    def caminho_heap(self, v1, v2, p):
        pais = self.DijkstraHeap(v2, 0)[1]  #fazemos a árvore a partir de v2, para ir encontrando os pais de v1 até chegar em v2
        caminho = [v1] #caminho começa já com o vétice de partida
        v= v1  #começamos em v1
        if self.DijkstraHeap(v2, 0)[1][v1-1] == -1: 
            caminho = "Não há caminho, v2 e v1 não estão na mesma cc"
        else:
            while v != v2: #enquanto vértice analisado não for vértice destino 
                v = pais [v-1]  #próximo vértice do caminho é o pai do que último adicionado  
                caminho.append(v)   #adicionamos esse pai ao caminho
        if p==1:
            print(f'\nCaminho entre  {v1} e {v2}: {caminho} ')
        return caminho 
    
    
    def caminho(self,v1,v2,p): #v1 é o vértice de partida, v2 o de chegada e p é para imprimir 
        pais = self.Dijkstra(v2, 0)[1]  #fazemos a árvore a partir de v2, para ir encontrando os pais de v1 até chegar em v2
        caminho = [v1] #caminho começa já com o vétice de partida
        v= v1  #começamos em v1
        if self.Dijkstra(v2, 0)[2][v1-1] == -1: 
            caminho = "Não há caminho, v2 e v1 não estão na mesma cc"
        else:
            while v != v2: #enquanto vértice analisado não for vértice destino 
                v = pais [v-1]  #próximo vértice do caminho é o pai do que último adicionado  
                caminho.append(v)   #adicionamos esse pai ao caminho
        if p==1:
            print(f'\nCaminho entre  {v1} e {v2}: {caminho} ')
        return caminho 
    
    def distancia(self, v1, v2, p): #Retorna a distâncoa entre dois vértices
        D= self.Dijkstra(v1, 0)[0][v2-1]
        if p ==1:
            print (D)
        return D

    def get_gargalo(self, caminho):
        aux = []
        gargalo = 0.0
        for v in caminho[1:]:
            for viz in self.residual[v-1]:
                if viz[0] in caminho and viz[2] == True:
                    aux += [viz[1]]
                    gargalo = np.min(aux)
        return gargalo
    
    def get_caminho(self, s, t):
        fluxo = 0
        pais = self.BFS_ff(s)
        if -1 in pais: return 0.0, fluxo
        a = pais[t-1]
        caminho = [t]
        while a != s:
            caminho.append(a)
            a = pais[a-1]
        caminho.append(s)
        gargalo = self.get_gargalo(caminho)
        fluxo += gargalo
        for v in caminho:
            for viz in self.residual[v-1]:
                if viz[0] in caminho:
                    if viz[2] == False: #aresta residual
                        viz[1] += gargalo
                    else: #aresta normal
                        viz[1] -= gargalo
                    if viz[1] == 0.0:
                        self.residual[v-1].remove(viz)
        return gargalo, fluxo

    def FF(self, s, t):
        fluxoMax = 0.0
        gargalo, fluxo = (-1,-1)
        while gargalo != 0.0:
            gargalo, fluxo = self.get_caminho(s,t)
            fluxoMax += fluxo
        return fluxoMax


