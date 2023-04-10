import json 
qtable = {}
l =  '01'
c =  '012'
rc = '01'
r =  '0123'
action_list = [0,0,0,0]

for i in l:
    for j in c:
        for k in rc:
            for m in r:
                qtable[i+j+k+m] = action_list

json_object = json.dumps(qtable, indent=4)

with open("src/stingray_sim/src/qtable.json", "w") as outfile:
    outfile.write(json_object)