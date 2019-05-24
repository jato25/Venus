#!/usr/bin/python
import sys, rospy, time
from pylab import *
import numpy as np
from master_msgs_iele3338.srv import mapaInicio, posInicio, posInicioResponse
from master_msgs_iele3338.msg import Obstacle
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import *

tamanoCua = 60
robot = 130/tamanoCua
pub = rospy.Publisher('PosRobotSiguiente', Float32MultiArray, queue_size=10)

#Metodo que crea la cuadricula que representa el mapa de V-rep
def crearCuadricula(obstacles):
	global matNod, matriz
	matriz = [[0  for i in range(int(2500/tamanoCua))]for j in range(int(2500/tamanoCua))]
	for obstaculo in obstacles:
		x = obstaculo[0]/tamanoCua
		y = obstaculo[1]/tamanoCua
		r = obstaculo[2]/(tamanoCua)
		matriz[int(x)][int(y)] = 1
		for j in range(int(x-r-robot)-1, int(x+r+robot)+1):
			for k in range(int(y-r-robot)-1, int(y+r+robot)+1):
				dist = np.linalg.norm(np.array([x,y]) - np.array([j+0.5,k+0.5]))
				if dist <= (r + robot):
					matriz[j][k] = 1
	#Se agregan las celdas vecinas con la posicion actual del robot 				
	for nod in matNod:
		for nod2 in nod:
			posN = nod2.pos
			if (matriz[posN[0]][posN[1]] == 0):
				for indice in range(posN[0] - 1, posN[0] + 2 ,1):
					for ja in range(posN[1] - 1, posN[1] + 2 ,1):
						if (indice >= 0 and indice <= int(2500/tamanoCua)-1 and ja >= 0 and ja <= int(2500/tamanoCua)-1):
							if (matriz[indice][ja] == 0 and not ((abs(indice - posN[0]) + abs(ja - posN[1])) == 0)):
								nod2.agregarVecinos(matNod[ja][indice])

class Nodo:
	def __init__(self, pos):
		self.pos = pos
		self.coord = [tamanoCua/2 + (tamanoCua*pos[0]) , (2500 + (tamanoCua/2)) - (tamanoCua*pos[1])]
		self.costo = 1000000000
		self.vecinos = []
		self.objetivo = False
		self.visitado = False
		self.padre = None
		self.h = 100000000
	#Metodo que define la heuristica con la posicion que entra por parametro	
	def defHeu(self, pos_f):
		self.h = math.sqrt((self.coord[0]-pos_f[0])**2 + (self.coord[1]-pos_f[1])**2) 
		return self.h
	#Metodo que asigna el nodo padre del nodo que llama este metodo	
	def asignarPadre(self, padre):
		self.padre = padre
	#Metodo que cambia el costo del nodo 
	def cambiarCosto(self, costo):
		self.costo = costo
	#Metodo que agrega a la lista de vecinnos el nodo que llame al metodo
	def agregarVecinos(self, vecino):
		self.vecinos.append(vecino)
	#Metodo que define que el nodo que llama al metodo es el actual	
	def esActual(self, visitado):
		self.visitado = visitado
	#Metodo que define que el nodo que llama al metodo es el objetivo	
	def esObjetivo(self, objetivo):
		self.objetivo = objetivo

#Metodo que busca el nodo en la matriz de nodo con las posciones que le entran por paramtero
def buscarNodo(x,y):
	global matNod
	a = int(round((x-tamanoCua/2)/tamanoCua))
	b = int(round(((2500+(tamanoCua/2))-y)/tamanoCua))
	return matNod[b][a]

#Metodo que busca un mejor nodo que el que llama al metodo partiendo del costo actual devuelve el mejor nodo				
def buscarMejor(nodos):
	cost = 10000000000
	best = None
	for nod in nodos:
		if(nod.costo < cost):
			cost = nod.costo
			best = nod
	return best

#Metodo para el algoritmo de A estrella		
def Astar(inicio, destino):
	global matNod
	for fil in matNod:
		for nod in fil:
			nod.esActual(False)
	pos_f = [destino[0],destino[1]]
	goal = buscarNodo(destino[0],destino[1])
	goal.esObjetivo(True)
	explorados = []
	actual = buscarNodo(inicio[0],inicio[1])
	actual.asignarPadre(None)
	explorados.append(actual)
	actual.cambiarCosto(0)
	actual.esActual(True)
	costos = {actual : 0}
	rutax = []
	rutay = []
	while not len(explorados) == 0:
		actual = buscarMejor(explorados)
		explorados.remove(actual)
		actual.esActual(True)
		if actual.objetivo:
			break
		vecinos = actual.vecinos
		for vecino in vecinos:
			costo = costos[actual] + actual.defHeu(vecino.coord)
			if (vecino not in explorados and not vecino.visitado) or costo < costos[vecino]:
				costos[vecino] = costo
				vecino.cambiarCosto(costo + 3*vecino.defHeu(pos_f))
				vecino.asignarPadre(actual)
				explorados.append(vecino)
	rutax = []
	rutay = []
	while actual.padre != None:
		coord2 = actual.coord
		rutax.append(coord2[0])
		rutay.append(coord2[1])
		actual = actual.padre
	return rutax, rutay

def posInfo(info):
	global start
	s.shutdown()
	return posInicioResponse(start)

def posActual(data):
	global venusPos
	venusPos = data.data

if __name__ == '__main__':
	global matNod, matriz, start, venusPos
	venusPos = [0,0,0]
	matNod = [[Nodo([i , j]) for i in range(int(2500/tamanoCua))]for j in range(int(2500/tamanoCua))]
	rospy.init_node('mapa')
	rospy.wait_for_service('mapa_inicio')
	met = rospy.ServiceProxy('mapa_inicio', mapaInicio)
	req = met()
	start = [req.start.position.x, req.start.position.y, req.start.orientation.w]
	goal = np.array([req.goal.position.x, req.goal.position.y, req.goal.orientation.w])
	obstacles = []
	n_obstacles = req.n_obstacles
	for i in range(req.n_obstacles):
		obstacles.append(np.array([req.obstacles[i].position.position.x , req.obstacles[i].position.position.y, req.obstacles[i].radius]))
	s = rospy.Service('pos_inicio', posInicio,  posInfo)
	s.spin()	
	start = np.array(start)
	rospy.Subscriber('venus_position',Float32MultiArray, posActual)
	crearCuadricula(obstacles)
	rospy.loginfo('Mapa listo')
	tasa = rospy.Rate(50)
	x_list,y_list = Astar(start, goal)
	x_list.reverse()
	y_list.reverse()
	x_list.append(goal[0])
	y_list.append(goal[1])
	rospy.loginfo('Ruta lista')
	'''for i in range(len(x_list)):
		#print((x_list[i],y_list[i]))
		matriz[x_list[i]][y_list[i]] = '*'
		if i == 0:
			matriz[x_list[i]][y_list[i]] = 'I'
	for i in range(int(2500/tamanoCua)):
		for j in range(int(2500/tamanoCua)):
			print matriz[i][j],
		print("")'''
	while not rospy.is_shutdown():
		for i in range(len(x_list)):
			rho = 50000000
			while (rho > 30):
				rho = math.sqrt((x_list[i]-venusPos[0])**2 + (y_list[i]-venusPos[1])**2)
				pub.publish(data = [x_list[i],y_list[i],math.pi])
				tasa.sleep()
			rho = 50000000
		x_list = []
		y_list = []
		tasa.sleep()

	
