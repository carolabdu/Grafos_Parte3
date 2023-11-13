#Código usado para fazer os estudos de caso

from TP3 import *

with open('grafo_teste.txt', 'r') as arquivo:  #mudar o número do grafo de acordo com o teste 
        texto = [vertice for vertice in arquivo.read().split()  ] #Cria uma lista com o conteúdo presente no arquivo de teste
        vertices = int(texto[0]) #Pega o primeiro elemento da lista acima, que correnspode ao número de vértices do grafo
        arestas = [] #Lista vazia que vai conter todos os pares de arestas do grafo
        for i in range(len(texto[1:])//3):
            arestas += [[int(texto[3*i+1])]+[int(texto[3*i+2])]+ [float(texto[3*i+3])]] 

grafo = Graph_l(vertices, arestas)
print(grafo.mostra_lista())