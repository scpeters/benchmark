import csv
import numpy as np

# function for exact matching or partial string matching
# used for selecting specific test cases based on test parameters
def match(a,b):
    if a == b:
        return True
    else:
        try:
            if a in b:
                return True
        except:
            return False
    return False

# open data file as csv and construct dictionary of arrays
def makeCsvDictOfArrays(filename):
    with open(filename, 'rt') as csvfile:
        spamreader = csv.DictReader(csvfile)
        csvDict = {}
        for field in spamreader.fieldnames:
            csvDict[field] = []
        csvDict[None] = []
        for row in spamreader:
            for k in row.keys():
                try:
                    csvDict[k] = np.append(csvDict[k], float(row[k]))
                except:
                    csvDict[k].append(row[k])
        return csvDict

# query a dictionary of arrays for indices of matching parameters
def query(d, params):
    result = []
    for k in params:
        result.append(set([i for i,x in enumerate(d[k]) if match(params[k], x)]))
    return sorted(set.intersection(*result))
