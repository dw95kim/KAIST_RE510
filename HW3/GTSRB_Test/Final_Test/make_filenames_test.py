from os import walk
from os.path import isfile, join
import csv 

mypath = "./Images"
csv_lines = []
text_lines = []

with open('/workspace/GTSRB_Test/Final_Test/Images/GT-final_test.csv', 'r') as gt_file:
    reader = csv.reader(gt_file, delimiter=';') # change contents to floats
    for row in reader: # each row is a list
        csv_lines.append(row)

(dir_path, dir_names, file_names) = walk(mypath).next()
for filename in file_names:
    for row in csv_lines:
        if row[0] == filename:
            class_number = row[7]
            text_lines.append(dir_path+ '/' + filename + ' ' + class_number)
            break
    gt_file.close()
   
with open('test.txt', 'w') as f:
    for item in text_lines:
        f.write("%s\n" % item)
