from instance import Instance
import pandas as pd
import cplex

EPS = 0.1


######## Funciones auxiliares, por si les sirve ############

def eval_travel_time(inst, sol):
	#''' Dada una instancia y una solucion (siguiendo el formato especificado), calcula la suma de los tiempos 
	#de viaje'''
	ret = 0
	for vehicle, pax in sol:
		ret += inst.get_travel_time(vehicle, pax)
	return ret


def eval_arrival_time(inst, sol):
	#''' Dada una instancia y una solucion (siguiendo el formato especificado), calcula la suma de los tiempos 
	#de respuesta'''
	ret = 0
	for vehicle, pax in sol:
		ret += inst.get_arrival_time(vehicle, pax)
	return ret


def eval_over_time(inst, sol, target_time): # Programacion lineal 
	#''' Dada una instancia y una solucion (siguiendo el formato especificado), calcula la cantidad de asignaciones
	#cuyo tiempo de respuetas es mayor que target_time'''
	ret = 0
	for vehicle, pax in sol:
		if inst.get_arrival_time(vehicle,pax) > target_time:
			ret += 1
	return ret

def eval_over_time_travel_time(inst, sol, target_time): #Greedy y PL
	#''' Dada una instancia y una solucion (siguiendo el formato especificado), calcula la cantidad de asignaciones
	#cuyo tiempo de viaje es mayor que target_time. Mismo esquema que la funcion anterior, pero varia la funcion
	#a considerar.'''
	ret = 0
	for vehicle, pax in sol:
		if inst.get_travel_time(vehicle,pax) > target_time:
			ret += 1
	return ret


def check_feasibility(inst, sol):
	#''' Valida que la solucion sea una asignacion valida'''
	veh = [0]*inst.n
	pax = [0]*inst.n
	for v,p in sol:
		veh[v] = veh[v] + 1
		pax[p] = pax[p] + 1

	for i in range(inst.n):
		if veh[i] > 1 or pax[i] > 1:
			return False
	return True


################## Solucion FCFS greedy ######################
def get_best_vehicle(inst, vehicle_used, pax):
	#''' Funcion auxiliar. Puede ayudar a resolver la siguiente funcion. No es obligatorio usarla.
	#Recibe una instancia, una lista con los taxis ya usados, y un paciente que se busca asignar.'''
	
	#Guardamos el indice de la cantidad de vehiculos
	vehiculos_idx = list(range(0, inst.n))
	#nos quedamos solo con aquellos vehiculos disponibles (ASUMIMOS QUE veicle_used es una lista de indices de vehiculos que ya se usaron)
	vehiculos_disponibles_idx = [x for x in vehiculos_idx if x not in vehicle_used]
	
	#Guardamos como tiempo minimo el primer vehiculo disponible
	min_tiempo_viaje = inst.time_matrix[vehiculos_disponibles_idx[0]][pax]
	min_tiempo_idx = vehiculos_disponibles_idx[0]
	for i in vehiculos_disponibles_idx[1:]:
		
		if  inst.time_matrix[i][pax] < min_tiempo_viaje:
			min_tiempo_viaje = inst.time_matrix[i][pax]
			min_tiempo_idx = i
											   
	return min_tiempo_idx

def solve_instance_greedy_by_pax(inst):
	#'''Heurística FCFS. Retorna valor de funcion objetivo y solucion.'''
	lista_pacientes = list(range(0,inst.n ))
	solucion = []
	vehiculos_usados = []
	
	
	for j in lista_pacientes:
		min_tiempo_vehicle_idx = get_best_vehicle(inst,
										 vehiculos_usados,
										 j)
		solucion.append((min_tiempo_vehicle_idx , j))
		vehiculos_usados.append(min_tiempo_vehicle_idx)
	
	
	f_obj = 0
	for i,j in solucion:
		f_obj = f_obj + inst.cost[i][j]
	
	return f_obj , solucion 


###############################################################


def generate_variables(inst, myprob):
	#''' Genera las variables del modelo. No es obligatorio usarla.'''
	n_vars_tot = inst.n * inst.n
	obj = [0] * n_vars_tot
	lb = [0] * n_vars_tot
	names = []
	
	#var_cnt va a ser el valor que va llevando la cuenta de cuantas variables agregamos hasta el momento
	var_cnt = 0
	#generamos los indices
	
	for i in range(inst.n): #recorro la matriz 
		for j in range(inst.n):
			#tenemos los dos for anidados porque necesitamos las combinaciones de (i,j) , i = 1,.... , n  y j = 1,.....,n
			inst.var_idx[(i,j)] = var_cnt
			obj[inst.var_idx[(i,j)]] = inst.cost[i][j]
			names.append('x_' + str(i) + '_' + str(j))
			var_cnt +=1 
			
	myprob.variables.add(obj = obj , lb  = lb , names = names)
	
			

def generate_constraints(inst, myprob):
	#''' Genera las restricciones para la estrategia de matching'''
	
	
	vals = [1.0] * inst.n
	
	#Restricciones a nivel PACIENTE
	for j in range(inst.n):
		#Recorre para cada columna(paciente) todas las filas (ambulacias)
		ind = []
		for i in range(inst.n):
			ind.append(inst.var_idx[(i,j)])   #Genera una lista dejando fijo el paciente y considerando todos los vehiculos
		
		
		#Generamos lado izquierdo de la restriccion poniendo un 1 para cada vehiculo y el paciente J
		row  = [ind , vals]

		#Agregamos la restriccion por igualdad y que tiene que ser igual a 1
		myprob.linear_constraints.add(lin_expr = [row],
								senses = ['E'],
								rhs = [1])
	
	
	
	
	#Restricciones a nivel Vehiculo
	for i in range(inst.n):
		#Recorre para cada fila(vehiculo) todas las columnas (pacientes)
		ind = []
		for j in range(inst.n):
			ind.append(inst.var_idx[(i,j)])   #Genera una lista dejando fijo el vehiculo y considerando todos los pacientes
		
		
		#Generamos lado izquierdo de la restriccion poniendo un 1 para cada paciente y el vehiculo i
		row  = [ind , vals]
		
		#Agregamos la restriccion por igualdad y que tiene que ser igual a 1
		myprob.linear_constraints.add(lin_expr = [row] ,
								senses = ['E'],
								rhs = [1])


def populate_by_row(inst, myprob):
	''' Genera el modelo para la estrategia de matching (variables y restricciones)'''
	
	#Generamos las variables
	generate_variables(inst, myprob)
	
	# Seteamos el problema de minimizacion.
	myprob.objective.set_sense(myprob.objective.sense.minimize)
	
	#Generamos las restricciones
	generate_constraints(inst, myprob)
	
	


def solve_instance_lp(inst):
	#''' Resuelve el modelo usando CPLEX. Retorna funcion objetivo y solucion. '''
	    # Resolvemos el LP.
	
	#genaramos el solver
	myprob = cplex.Cplex()
	
	#Generamos las vairables del modelo , el problema a trabajar y las restricciones
	
	populate_by_row(inst , myprob)
	
	myprob.solve()
	
	f_obj = myprob.solution.get_objective_value()
		
	lista_variables = [(i,j)for i in range(inst.n) for j in range(inst.n)]
	
	filtro = [ x == 1 for x in myprob.solution.get_values()]
	
	solucion = []
	
	for i ,  tupla in enumerate(lista_variables):
		if filtro[i]:
			solucion.append(tupla)
		 
	
	return f_obj , solucion








def main():

	# Setup para item 5.
	#inst_types = ['small', 'medium', 'large']
	n_inst = ['0','1','2','3','4','5','6','7','8','9']
	#path_base = 'input/basic/'

	# Setup para item 6 y 7. Descomentar las siguientes lineas.
	target_time = 20
    #Solo se debe usar una opción de path_base a la vez
	path_base = 'input/small_low/'
	#path_base = 'input/small_moderate/'
	#path_base = 'input/small_high/'
	inst_types = ['small']
	
	columna_inst_type = []
	columna_n_inst = []
	columna_eval_travel_time_greedy = []
	columna_eval_travel_time_lp = []
	columna_arrival_time_lp = []
	
	
	
	##### Punto 5 y 6
	
	columna_eval_over_time_travel_time_greedy = []
	columna_eval_over_time_travel_time_lp = []
	columna_eval_over_time_arrival_time_lp = []
	
	

	for t in inst_types:
		for n in n_inst:

			# Leemos el archivo y cargamos la instancia.
			inst_file = path_base + t + '_' + n + '.csv'
			inst = Instance(inst_file)

			# Seteamos la funcion objetivo que queremos usar en la matriz de costos.
			inst.set_cost_arrival_time()

			# Solucion greedy.
			f_greedy, x_greedy = solve_instance_greedy_by_pax(inst)

			# Solucion lp
			f_lp, x_lp = solve_instance_lp(inst)

			# Mostramos informacion basica. 
			# Cambiar y exportar los datos y formato que el grupo considere mejor.
			
			sum_travel_time_greedy = eval_travel_time(inst, x_greedy)
			sum_travel_time_lp = eval_travel_time(inst, x_lp)
			sum_arrival_time_lp = eval_arrival_time(inst, x_lp)
			
			
			#punto 5 y 6 
			sum_over_time_travel_time_greedy = eval_over_time_travel_time(inst , x_greedy , target_time )
			sum_over_time_travel_time_linear = eval_over_time_travel_time(inst , x_lp , target_time )

			sum_over_time_lp = eval_over_time(inst, x_lp , target_time)
			 
			
			
			print(inst_file," greedy: ", sum_travel_time_greedy, " lp: ", sum_arrival_time_lp)
			
			columna_inst_type.append(t)
			columna_n_inst.append(n)
			columna_eval_travel_time_greedy.append(sum_travel_time_greedy)
			columna_eval_travel_time_lp.append(sum_travel_time_lp)
			columna_arrival_time_lp.append(sum_arrival_time_lp)
			
			
			##### Punto 5 y 6 
			columna_eval_over_time_travel_time_greedy.append(sum_over_time_travel_time_greedy)
			columna_eval_over_time_travel_time_lp.append(sum_over_time_travel_time_linear)
			columna_eval_over_time_arrival_time_lp.append(sum_over_time_lp)
			
		
		data_frame  = pd.DataFrame({'ins_type' : columna_inst_type,
							         'n_inst' : columna_n_inst,
									 'greedy_travel_time' : columna_eval_travel_time_greedy,
									 'greddy_over_time_travel_time' : columna_eval_over_time_travel_time_greedy, 
									 'lp_travel_time' : columna_eval_travel_time_lp, 
									 'lp_arrival_time' : columna_arrival_time_lp,
									 'lp_over_time_time_travel' : columna_eval_over_time_travel_time_lp ,
									 'lp_over_time_arrival' : columna_eval_over_time_arrival_time_lp})
		
		data_frame.to_csv('datos_para_analisis_punto_5_small_low.csv')
			


if __name__ == '__main__':
	main()