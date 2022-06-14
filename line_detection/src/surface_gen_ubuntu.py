"""
TE3002B Implementacion de robotica inteligente
Equipo 1
    Diego Reyna Reyes A01657387
    Samantha Barron Martinez A01652135
    Jorge Antonio Hoyo Garcia A01658142
Control Difuso
Ciudad de Mexico, 10/06/2022
"""

import numpy as np
import skfuzzy as sk
import csv

def evaluar(fx,x,v):
    """
    Evaluate the function fx for a value v
    
    Parameters
    ----------
    fx :    numpy.array(float[])
        The array that contains the output values of the functions
    x :     numpy.array(float[])
        The array that contains all the inputs of the function that define the outputs given in fx
    v :     float
        The value of x we want to obtain it's value in fx
    """
    idx = np.where(x==v)
    #In case it's empty
    if idx[0].shape == (0,):
        closest = 0 
        for i in x:
            if abs(v-i) < abs(v-closest):
                closest = i
        idx = np.where(x==closest)
    #Send the value of fx
    return fx[idx[0][0]]
    
def intersectar(a,ax,va,b,bx,vb):
    """
    Intersects 2 membership functions and outputs the minumum \mu of each of the two evaluations

    Parameters
    ----------
    a :     numpy.array(float[])
        The array that contains the output values of the first membership function
    ax :    numpy.array(float[])
        The array that contains all the inputs of the function that define the outputs given in a
    va :     float
        The value of ax we want to obtain it's value in a
    b :     numpy.array(float[])
        The array that contains the output values of the second membership function
    bx :    numpy.array(float[])
        The array that contains all the inputs of the function that define the outputs given in b
    vb :    numpy.array(float[])
        The value of bx we want to obtain it's value in b
    """
    d = [evaluar(a,ax,va),evaluar(b,bx,vb)]
    return min(d)
def calcular(inputs,possible_values,entrada,ls_qual_ra,ls_qual,output_case):
    """
    Calculates the discrete output of our fuzzy control system, using discrete inputs

    Parameters
    ----------
    inputs : numpy.array(float[])
        An array that contains n rows, a row per input parameter. Each of these rows contains all 
        the possible values of the membership function of the linguistic variables used to describe that input
    possible_values : numpy.array(float[])
        An array containing every possible value for the input, these values define the range of the membership functions given on "inputs"
    entrada : float[]
        A list containing the discrete values of the input, this list size has to coincide with the number of rows on the "inputs" array
    ls_qual_ra : numpy.array(float[])
        An array that contains all the possible values of the membership function of the linguistic 
        variables used to describe the output
    ls_qual : numpy.array(float[])
        An array containing every possible value for the output
    output_case : float[]
        A list containing the relationship between the instersections and the output
    """
    minimos = []
    #For each input parameter
    for param_idx in range(len(inputs)-1):
        param = inputs[param_idx]
        my_x = possible_values[param_idx]
        #For each possible membership function of the parameter
        for ran_idx in range(len(param)):
            t = []
            ran = param[ran_idx]
            #For each parameter not intersected with this one
            for other_idx in range(param_idx+1,len(inputs)):
                other_x = possible_values[other_idx]
                #For each membership function of the other paramter
                for other_ran in inputs[other_idx]:
                    #Intersect
                    t.append(intersectar(ran,my_x,entrada[param_idx],other_ran,other_x,entrada[other_idx]))
            minimos.append(t)
    
    outputs = np.zeros((1,len(output_case)))
    #Obtain tha maximum per each output case
    for i in range(len(output_case)):
        for j in range(len(output_case[i])):
            if minimos[i][j] > outputs[0][output_case[i][j]]:
                outputs[0][output_case[i][j]] = minimos[i][j]
    #Limit the otuput of each membership function to the maximum obtained
    salidas = []
    idx = 0
    for ran in ls_qual_ra:
        t = []
        for v in ran:
            if v > outputs[0][idx]:
                t.append(outputs[0][idx])
            else:
                t.append(v)
        salidas.append(t)
        idx+=1
        
    
    #Obtain a single array with the contour of the resulting topped membership function
    resultado = []
    #Para cada x
    for i in range(len(ls_qual)):
        maxi = 0
        #Evaluar el valor de cada salida en esa x
        for j in range(len(salidas)):
            if maxi < salidas[j][i]:
                maxi = salidas[j][i]
        resultado.append(maxi)
    #Obtain the discrete output value (z*)
    num = 0
    idx = 0
    for i in resultado:
        num += i * ls_qual[idx]
        idx +=1
    z_estrella = num/np.sum(resultado)
    return resultado,z_estrella
def superficie(inputs,possible_values,ls_qual_ra,ls_qual,output_case,names_variables):
    """
    Creates the control surface fo every possible value

    Parameters
    ----------
    inputs : numpy.array(float[])
        An array that contains n rows, a row per input parameter. Each of these rows contains all 
        the possible values of the membership function of the linguistic variables used to describe that input
    possible_values : numpy.array(float[])
        An array containing every possible value for the input, these values define the range of the membership functions given on "inputs"
    ls_qual_ra : numpy.array(float[])
        An array that contains all the possible values of the membership function of the linguistic 
        variables used to describe the output
    ls_qual : numpy.array(float[])
        An array containing every possible value for the output
    names_variables : str[]
        A list containg the names for the x,y and z axis
    """
    Z = []
    #Calculate for every possible value of the inputs (Only two inputs are considered here)
    for i in possible_values[1]:
        t = []
        for j in possible_values[0]:
            _,z_estrella = calcular(inputs,possible_values,[j,i],ls_qual_ra,ls_qual,output_case)
            t.append(z_estrella)
        Z.append(t)
        #print(i)
    #Create the 3D graph
    X,Y = np.meshgrid(possible_values[0],possible_values[1])
    Z = np.array(Z)
    
    #Save the files
    file_name = names_variables[2] + "z.csv"
    with open(file_name, "w") as f:
        wr = csv.writer(f)
        wr.writerows(Z)
    file_name = "x.csv"
    with open(file_name, "w") as f:
        wr = csv.writer(f)
        wr.writerows(X)
    file_name = "y.csv"
    with open(file_name, "w") as f:
        wr = csv.writer(f)
        wr.writerows(Y)
if __name__ == "__main__":
    #Create every single membership
    #1280*720
    #5/6 y 7/20-13/20
    #Distancia
    #-85,-40 --> Vuelta
    #-40,-25 --> Desalineado
    #-25,25  -->Centrado
    ld_qual = np.arange(-85, 85.1,1)#Distancia del centro de la imagen
    ld_names = ["Lejos izq","Media izq","Centro","Media Der","Lejos Der"]
    #Pendiente
    #-10,-5 Recta neg
    #-5,-1 Curva nega
    #-1,1 Horizontal
    #1,5 Curva pos
    #5,10 Recta pos
    p_qual = np.arange(-10, 10.1,0.5)#Pendiente
    ad_names = ["Recta negativa","Curva negativa","Horizontal","Curva positiva","Recta positiva"]
    ls_qual = np.arange(0, 0.105,0.005)#Linear speed
    ls_names = ["baja","semi-baja","media","semi-alta","alta"]
    as_qual = np.arange(-0.15, 0.16,0.01)#Angular speed
    as_names = ["negativa alta","negativa media","baja","positiva media","positiva alta"]

    #Linear distance to the objective
    ld_qual_ra = []
    ld_qual_ra.append(sk.gaussmf(ld_qual, -85, 10))
    ld_qual_ra.append(sk.gaussmf(ld_qual, -50, 12))
    ld_qual_ra.append(sk.gaussmf(ld_qual, 0, 12))
    ld_qual_ra.append(sk.gaussmf(ld_qual, 50, 12))
    ld_qual_ra.append(sk.gaussmf(ld_qual, 85, 10))

    #Angular distance to the objective
    p_qual_ra = []
    p_qual_ra.append(sk.gaussmf(p_qual, -10, 1))
    p_qual_ra.append(sk.gaussmf(p_qual, -5, 1.2))
    p_qual_ra.append(sk.trimf(p_qual, [-2,0,2]))
    p_qual_ra.append(sk.gaussmf(p_qual, 5, 1.2))
    p_qual_ra.append(sk.gaussmf(p_qual, 10, 1))

    #Save the inputs
    inputs = [ld_qual_ra,p_qual_ra]
    possible_values = [ld_qual,p_qual]

    #Linear speed
    ls_qual_ra = []
    ls_qual_ra.append(sk.trapmf(ls_qual, [0, 0, 0.005 , 0.006]))
    ls_qual_ra.append(sk.trimf(ls_qual, [0.006,0.025,0.049]))
    ls_qual_ra.append(sk.trimf(ls_qual, [0.03,0.05,0.07]))
    ls_qual_ra.append(sk.trimf(ls_qual, [0.051,0.075,0.094]))
    ls_qual_ra.append(sk.trapmf(ls_qual, [0.094, 0.095, 0.1 , 0.1]))

    #Angular speed
    as_qual_ra = []
    as_qual_ra.append(sk.trimf(as_qual, [-0.15,-0.15,-0.1]))
    as_qual_ra.append(sk.trimf(as_qual, [-0.15,-0.08,-0.01]))
    as_qual_ra.append(sk.trimf(as_qual, [-0.02,0,0.02]))
    as_qual_ra.append(sk.trimf(as_qual, [0.01,0.08,0.15]))
    as_qual_ra.append(sk.trimf(as_qual, [0.1,0.15,0.16]))

    output_case_1 = [[2,2,0,0,2],[3,2,0,2,3],[4,2,0,2,4],[3,2,0,2,3],[2,2,0,2,2]]
    output_case_2 = [[3,2,3,4,1],[3,1,2,3,1],[2,1,2,3,2],[3,1,2,3,1],[3,0,1,2,1]]
    names_variables = ["Distancia","Pendiente","l"]
    superficie(inputs,possible_values,ls_qual_ra,ls_qual,output_case_1,names_variables)
    names_variables = ["Distancia","Pendiente","a"]
    output_case = [[0,1,2,3,4],[0,1,2,3,4],[1,1,2,3,3],[1,1,2,3,3],[1,1,2,3,3]]
    superficie(inputs,possible_values,as_qual_ra,as_qual,output_case_2,names_variables)
    