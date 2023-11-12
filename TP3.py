import numpy as np
import heapq
from collections import defaultdict


class Graph_l:   
    def __init__(self, v, a): #(número de vértices, arestas)
        self.v = v 
        self.a = a
        self.num_arestas = len(self.a)
        self.neg = False
        self.lista = [[] for i in range(self.v)] #Não sabemos quantos elementos tem em cada linha (quantos vizinhos cada vértice tem)
        self.pesos = [[] for i in range(self.v)]  #lista que contém apenas os pesos 
        for i in range(self.num_arestas):
            u=self.a[i][0] #Pega o primeiro vértice do par de arestas
            v= self.a[i][1] #Pega o segundo vértice do par de arestas
            p= self.a[i][2] #Pega o peso da aresta
            if p < 0: self.neg = True
            self.lista[u-1]+= [(v,p)] #Adiciona o vértice v na linha u (o índice é u-1, pois i se inicia no 0)
            self.pesos[u-1]+= [p]
           

    def mostra_lista(self): #Retorna a lista de adjacência
        return self.lista 