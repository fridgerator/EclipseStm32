import time
import random

def main():
  outfile = open('/home/klemen/temp/out.txt', 'a')
  #Get a random number.
  num1 = random.randint(20380000, 20390000)
  num2 = random.randint(20380000, 20390000)
  num3 = random.randint(20380000, 20390000)
  #Write 12 random intergers in the range of 1-100 on one line
  #to the file.
  outfile.write(str(num1) + ' ' + str(num2) + ' ' + str(num3) + '\n')
  outfile.close()

while 1==1:
  main()
  time.sleep(0.1)
  