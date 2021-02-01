from os import listdir
from os.path import join, isfile

def find_all_files(path='bags/'):
    return [join(path, f) for f in listdir(path) 
            if isfile(join(path, f))]

def clean_activations(s):
    s = s.replace('[', '')
    s = s.replace(']', '')
    return list(map(float, s.split()))