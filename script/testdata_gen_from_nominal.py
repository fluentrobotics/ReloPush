#import numpy as np
import random as rn
import os

type_delim = "!"
header_delim = ":"
object_delim = ";"
elem_delilm = ","

class movableObject:
    def __init__(self, x, y, th, name, n_side) -> None:
        self.name = name
        self.x = x
        self.y = y
        self.th = th
        self.n_side = n_side

    def add_randomness(self, min_val=-0.05, max_val=0.05):
        x_rn = self.x + rn.uniform(min_val,max_val)
        if(x_rn < 0):
            x_rn = self.x + abs(rn.uniform(min_val,max_val))

        y_rn = self.y + rn.uniform(min_val,max_val)
        if(y_rn < 0):
            y_rn = self.y = abs(rn.uniform(min_val,max_val))
        # todo: do different method for yaw
        th_rn = self.th

        return movableObject(x_rn,y_rn,th_rn,self.name,self.n_side)


    def to_str(self):
        out_str = self.name + elem_delilm + str(self.x) + elem_delilm + str(self.y) + elem_delilm + str(self.th) + elem_delilm + str(self.n_side)

        return out_str


class pose2D:
    def __init__(self, x, y, yaw) -> None:
        self.x = x
        self.y = y
        self.yaw = yaw

    def add_randomness(self, min_val=-0.05, max_val=0.05):
        x_rn = self.x + rn.uniform(min_val,max_val)
        if(x_rn < 0):
            x_rn = self.x + abs(rn.uniform(min_val,max_val))

        y_rn = self.y + rn.uniform(min_val,max_val)
        if(y_rn < 0):
            y_rn = self.y = abs(rn.uniform(min_val,max_val))

        # todo: do different method for yaw
        yaw_rn = self.yaw

        return pose2D(x_rn,y_rn,yaw_rn)


    def to_str(self):
        out_str = str(self.x) + elem_delilm + str(self.y) + elem_delilm + str(self.yaw)

        return out_str
    

def serialize_list(list_to_serialize:list, header:str):
    out_str = header + header_delim

    for o in range(len(list_to_serialize)):
        obj = list_to_serialize[o]
        out_str += obj.to_str()

        if(o < len(list_to_serialize)-1):
            out_str += object_delim

    return out_str


class nominal_dataset:
    def __init__(self, dataset_name:str) -> None:
        self.name = dataset_name
        self.mo_list = []
        self.robot = []
        self.d_list = []
        self.d_table = {}

    def add_randomness(self):
        dataset_rn = nominal_dataset(self.name)
        
        # mo list
        for mo in self.mo_list:
            dataset_rn.mo_list.append(mo.add_randomness())
        
        # robot
        for rb in self.robot:
            dataset_rn.robot.append(rb.add_randomness())

        # delivery
        for d in self.d_list:
            dataset_rn.d_list.append(d.add_randomness())

        # todo: add randomness in assignment
        dataset_rn.d_table = self.d_table

        return dataset_rn



    def serialize(self):
        out_str = ""

        # serialize mo
        out_str += serialize_list(self.mo_list, "mo")
        out_str += type_delim

        # append robot
        out_str += serialize_list(self.robot, "robot")
        out_str += type_delim

        # append delivery
        out_str += serialize_list(self.d_list, "goal")
        out_str += type_delim

        # append assign
        out_str += "assign"
        out_str += header_delim
        out_str += ';'.join([f"{key},{value}" for key, value in self.d_table.items()])

        return out_str
    
    def write_to_file(self):
        # list of random data
        dataset_rn_list = []
        for i in range(100):
            dataset_rn_list.append(self.add_randomness().serialize())

        # Specify the file name
        file_name = self.name + '.txt'
        # Get the directory of the current script
        current_directory = os.path.dirname(__file__)
        # Construct the full file path
        file_path = os.path.join(current_directory+"/../testdata/", file_name)

        with open(file_path, 'w') as file:
            file.write('\n'.join(dataset_rn_list))


        


# 2obj 1relo test instance
dataset_one = nominal_dataset("data_2o1r")
dataset_one.mo_list = [movableObject(2.2,3.5,0,"b1",4), movableObject(1,3.5,0,"b3",4)] ## objects
dataset_one.robot = [pose2D(2, 2.5, 1.5708)] ## robot
dataset_one.d_list = [movableObject(0,0,0,"d1",4), movableObject(3,3.5,0,"d3",4)] ## delivery
dataset_one.d_table = {"b1":"d1", "b3":"d3"} ## assign

# 2obj 1relo test instance
dataset_two = nominal_dataset("data_2o1r_2")
dataset_two.mo_list = [movableObject(2.2,3.5,0,"b1",4), movableObject(1,3.5,0,"b3",4)] ## objects
dataset_two.robot = [pose2D(2, 2.5, 1.5708)] ## robot
dataset_two.d_list = [movableObject(0,0,0,"d1",4), movableObject(3,3.5,0,"d3",4)] ## delivery
dataset_two.d_table = {"b1":"d3", "b3":"d1"} ## assign

# 3obj 0relo test instance
dataset_three = nominal_dataset("data_3o")
dataset_three.mo_list = [movableObject(1,1.5,0,"b1",4),
    movableObject(1.4,2.3,0,"b2",4),
    movableObject(1.2,3.1,0,"b3",4)]
dataset_three.robot = [pose2D(2, 2.5, 1.5708)]
dataset_three.d_list = [movableObject(3.6,1.5,0,"d1",4),
    movableObject(3.8,2.3,0,"d2",4),
    movableObject(3.4,3.1,0,"d3",4)]
dataset_three.d_table = {"b1":"d1", "b3":"d3", "b2":"d2"} 

# 3obj 0relo test instance
dataset_four = nominal_dataset("data_3o_2")
dataset_four.mo_list = [movableObject(1,1.5,0,"b1",4),
    movableObject(1.4,2.3,0,"b2",4),
    movableObject(1.2,3.1,0,"b3",4)]
dataset_four.robot = [pose2D(2, 2.5, 1.5708)]
dataset_four.d_list = [movableObject(3.6,1.5,0,"d1",4),
    movableObject(3.8,2.3,0,"d2",4),
    movableObject(3.4,3.1,0,"d3",4)]
dataset_four.d_table = {"b1":"d2", "b3":"d1", "b2":"d3"} 


# 4obj
dataset_five = nominal_dataset("data_4o")
dataset_five.mo_list = [
    movableObject(2.9, 2.13,0,"b1",4),
    movableObject(0.58, 4.04,0,"b2",4),
    movableObject(2.9, 3.37,0,"b3",4),
    movableObject(2.09, 0.8,0,"b4",4)]
dataset_five.robot = [pose2D(2.35, 2.43, 1.5708)]
dataset_five.d_list = [
    movableObject(1.47, 4.39,0,"d1",4),
    movableObject(0.78, 2.91,0,"d2",4),
    movableObject(1.12, 1.88,0,"d3",4),
    movableObject(0.63, 0.76,0,"d4",4)]
dataset_five.d_table = {"b1":"d2", "b3":"d1", "b2":"d3", "b4":"d4"}

# 5obj
dataset_five_obj = nominal_dataset("data_5o")
dataset_five_obj.mo_list = [
    movableObject(1, 1.5,0,"b1",4),
    movableObject(1.4, 2.3,0,"b2",4),
    #movableObject(2, 4,0,"b3",4),
    movableObject(1.35, 4,0,"b3",4),
    #movableObject(3.4, 2.3,0,"b4",4),
    movableObject(3, 2.3,0,"b4",4),
    movableObject(1.58,4.7,0,"b5",4)
]
dataset_five_obj.d_list = [
    movableObject(3.6,1.5,0,"d1",4),
    movableObject(3.8,2.3,0,"d2",4),
    movableObject(3.4,3.1,0,"d3",4),
    movableObject(2.9,4,3,"d4",4),
    movableObject(3.8,5,0,"d5",4)
]
dataset_five_obj.robot = [pose2D(2.35, 2.43, 1.5708)]
dataset_five_obj.d_table = {"b1":"d1", "b2":"d2", "b3":"d3", "b4":"d4", "b5":"d5"}

# 6obj
dataset_six_obj = nominal_dataset("data_6o")
dataset_six_obj.mo_list = [
    movableObject(2.535403340155436, 2.3734257276055715,0,"b1",4),
    movableObject(2.0667146712201987, 0.8197366685520656,0,"b2",4), 
    movableObject(0.97, 2.56,0,"b3",4), 
    movableObject(2.549343365221235, 3.891071067604316,0,"b4",4), 
    movableObject(1.227864392727091, 3.4392290840385407,0,"b5",4), 
    movableObject(1.4124319235785139, 2.0168052994577845,0,"b6",4)
]
dataset_six_obj.d_list = [
    movableObject(0.9265007250020825, 0.8014723733004034,0,"d1",4), 
    movableObject(1.7172613891475708, 4.141671923375169,0,"d2",4), 
    movableObject(2, 2.56,0,"d3",4), 
    movableObject(3.02991373378123, 3.2185004448812062,0,"d4",4), 
    movableObject(0.9, 4.3,0,"d5",4), 
    movableObject(3.4734931508421695, 0.4606829184492558,0,"d6",4)
]
dataset_six_obj.robot = [pose2D(3.5, 3.5, 3.14)] ## robot
dataset_six_obj.d_table = {"b1":"d1", "b2":"d2", "b3":"d3", "b4":"d4", "b5":"d5", "b6":"d6"}



#dataset_one.write_to_file()
#dataset_two.write_to_file()
#dataset_three.write_to_file()
#dataset_four.write_to_file()
#dataset_five.write_to_file()
dataset_five_obj.write_to_file()
dataset_six_obj.write_to_file()

#print(dataset_one.add_randomness().serialize())
print('done')