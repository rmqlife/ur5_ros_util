import numpy as np
import json
import sys

if __name__=="__main__":
    if len(sys.argv) < 2:
        print("Usage: python npz_to_json.py <npz_filename>")
        sys.exit(1)

    npz_filename = sys.argv[1]
    data = np.load(npz_filename)
    data = {"T_c2g":data['T_c2g'].tolist()}
    print(data)
    json_filename = npz_filename.replace('.npz', '.json')
    with open(json_filename, 'w') as f:
        json.dump(data, f)
        
