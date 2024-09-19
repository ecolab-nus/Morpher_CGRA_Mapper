import os
import sys
from pathlib import Path

assert(len(sys.argv)  == 6 ) 

json_arch = str(sys.argv[1])
mem_alloc_file = str(sys.argv[2])
bank_size = int(sys.argv[3])
bank_number = int(sys.argv[4])
mem_json_arch = sys.argv[5]
mem_alloc = {}


def get_mem_alloc():
  f = open(mem_alloc_file, 'r')
  lines = f.readlines()
  
  for i in range(1, len(lines)):
    result = [x.strip() for x in lines[i].split(',')]
    assert(len(result) == 2)
    mem_alloc[result[0]] = int(result[1])
  return mem_alloc

def generate_mem_alloc():
  mem_lines = []
  for i in range(bank_number):
    mem_lines.append("\n\"SPM_B" + str(i) + "_WRAPPER\" : {")
    mem_lines.append("\n  \"TSOCKETS\" : [")
    mem_lines.append("\n    \"MEMPORT_P0\", \"MEMPORT_P1\"")
    mem_lines.append("\n  ],")
    mem_lines.append("\n  \"SUBMODS\" : {")
    mem_lines.append("\n    \"SPM\" : [{\"name\" : \"SPM0\"}]")
    mem_lines.append("\n  },")
    mem_lines.append("\n  \"DATA_LAYOUT\" :{")
    temp_lines = []
    for var,start in mem_alloc.items():
      if start >= i * bank_size and start < (i+1) * bank_size:
        temp_lines.append("\n    \""+var+"\":"+str(start))
    for j in range(0, len(temp_lines)-1):
      temp_lines[j] =  temp_lines[j] + ","
    for line in temp_lines:
      mem_lines.append(line) 
    mem_lines.append("\n  },")
    mem_lines.append("\n  \"INIT_FILE\" : \"../json/test_data.json\",")
    mem_lines.append("\n  \"CONNECTIONS\" : {")
    mem_lines.append("\n    \"SPM0.PORT0\" : \"THIS.MEMPORT_P0\",")
    mem_lines.append("\n    \"SPM0.PORT1\" : \"THIS.MEMPORT_P1\"")
    mem_lines.append("\n  }")
    mem_lines.append("\n},")
    mem_lines.append("\n")
  return mem_lines

get_mem_alloc()
f = open(json_arch, 'r')
new_lines= []
lines = f.readlines()
for line in lines:
  if ("\"CGRA\" :{" in line):
    mem_lines = generate_mem_alloc()
    for mem_line in mem_lines:
      new_lines.append(mem_line)
  new_lines.append(line)



f = open(mem_json_arch, 'w')
for line in new_lines:
  f.write(line)

f.close()
  