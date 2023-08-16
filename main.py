from CONFIG import *
from Ultrasonic import Ultrasonic
from ShaftEncoder import ShaftEncoder
import numpy as np
from Multiprocessing import Process, Value, Array

def main():

  ultra_arr = Array('i',[0,0,0]) # 'i' for signed integer, 'd' for double prec float

  TH_General = Process(target = )
  TH_Ultrasonic = Process(target = Ultrasonic, args=(ultra_arr))
  TH_ShaftEncoder = Process(target = ShaftEncoder)

  TH_General.start()
  TH_Ultrasonic.start()
  TH_ShaftEncoder.start()
