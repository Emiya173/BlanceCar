import re

with open('source.html', 'r') as f:
    t = f.readlines()
    src = []
    for i in range(0,len(t),2):
        src.append(t[i] + t[i+1])
    res = open('res.txt', 'a+')
    pt = r'<code class.*?>(.*?)</code>'
    for row in src:
        row = re.findall(pt, row)
        res.write(''.join(row)+'\n')
    res.close()
    
        
        